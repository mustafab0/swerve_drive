#include "motor.h"
#include <stdbool.h>

// Constants
#define ACCEL_TICK_HZ       5000
#define DIR_BLANK_TICKS     5       // ≈1 ms at 5 kHz
#define SPS_MIN_DELTA       1       // don't rewrite timer unless SPS changed by ≥1
#define SPS_STOP_EPSILON    10      // only force stop if target within ±10 SPS of zero
#define MAX_ACCEL_SPS       32500   // max acceleration to prevent Q16.16 overflow

// Static function prototypes
static void update_direction(Motor* m, uint8_t new_dir);
static void update_stepgen_sps(Motor* m);
static void handle_velocity_mode(Motor* m);
static void handle_position_mode(Motor* m);
static int32_t abs_i32(int32_t value);

void motor_init(Motor* m, const MotorHW* hw)
{
    if (!m || !hw) return;
    
    // Copy hardware configuration
    m->hw = *hw;
    
    // Initialize state
    m->sps_now_q16 = 0;
    m->sps_tgt_q16 = 0;
    m->a_max_q16 = 0;
    
    m->dir_sign = 0;
    m->dir_pending = 0;
    m->dir_blank = 0;
    m->sps_final_q16 = 0;
    
    m->pos_steps = 0;
    m->remaining_steps = 0;
    m->planned_remaining_q16 = 0;
    m->last_cnt = 0;
    
    m->last_sps_i = 0;
    m->state = M_IDLE;
    
    // Initialize DIR pin to low
    if (m->hw.dir_port && m->hw.dir_pin)
    {
        HAL_GPIO_WritePin(m->hw.dir_port, m->hw.dir_pin, GPIO_PIN_RESET);
    }
    
    // Initialize and start pulse counter if available
    if (m->hw.pulse_cnt_htim)
    {
        __HAL_TIM_SET_COUNTER(m->hw.pulse_cnt_htim, 0);
        m->last_cnt = 0;
        
        // Start the counter timer in external clock mode
        HAL_TIM_Base_Start(m->hw.pulse_cnt_htim);
    }
}

void motor_set_velocity(Motor* m, int32_t sps_target, int32_t a_max)
{
    if (!m) return;
    
    // Clamp to max SPS
    if (abs_i32(sps_target) > (int32_t)m->hw.sps_max)
    {
        sps_target = (sps_target >= 0) ? (int32_t)m->hw.sps_max : -(int32_t)m->hw.sps_max;
    }
    
    m->sps_tgt_q16 = SPS_TO_Q16(abs_i32(sps_target));  // Store magnitude
    
    // Clamp acceleration to prevent Q16.16 overflow
    if (a_max > MAX_ACCEL_SPS) a_max = MAX_ACCEL_SPS;
    m->a_max_q16 = SPS_TO_Q16(a_max);
    m->state = M_VEL;
    m->sps_final_q16 = 0;  // Clear any pending final target
    
    // Handle direction with epsilon logic
    uint8_t target_dir = (sps_target >= 0) ? 0 : 1;
    if (target_dir != m->dir_sign)
    {
        if (Q16_TO_SPS(m->sps_now_q16) == 0)
        {
            // Can change direction immediately when stopped
            update_direction(m, target_dir);
        }
        else
        {
            // Check if target is close to zero (within epsilon)
            if (abs_i32(sps_target) <= SPS_STOP_EPSILON)
            {
                // Target is close to zero, just stop
                m->sps_tgt_q16 = 0;
            }
            else
            {
                // Target is significant, go through zero then to target
                m->dir_pending = target_dir;
                m->sps_final_q16 = SPS_TO_Q16(abs_i32(sps_target)); // Store final target
                // First decelerate to zero, then direction will flip
                m->sps_tgt_q16 = 0;
            }
        }
    }
}

void motor_move_steps(Motor* m, int32_t steps, int32_t v_max_sps, int32_t a_max)
{
    if (!m || steps == 0) return;
    
    // Clamp velocity to max SPS
    if (v_max_sps > (int32_t)m->hw.sps_max)
    {
        v_max_sps = (int32_t)m->hw.sps_max;
    }
    
    m->remaining_steps = abs_i32(steps);
    m->planned_remaining_q16 = SPS_TO_Q16(abs_i32(steps));  // Initialize planner remaining
    m->sps_tgt_q16 = SPS_TO_Q16(v_max_sps);
    
    // Clamp acceleration to prevent Q16.16 overflow
    if (a_max > MAX_ACCEL_SPS) a_max = MAX_ACCEL_SPS;
    m->a_max_q16 = SPS_TO_Q16(a_max);
    m->state = M_POS_ACCEL;
    
    // Handle direction
    uint8_t target_dir = (steps >= 0) ? 0 : 1;
    if (target_dir != m->dir_sign)
    {
        if (Q16_TO_SPS(m->sps_now_q16) == 0)
        {
            // Can change direction immediately when stopped
            update_direction(m, target_dir);
        }
        else
        {
            // Request direction change when stopped
            m->dir_pending = target_dir;
            m->sps_tgt_q16 = 0; // First decelerate to stop
        }
    }
}


void motor_stop_smooth(Motor* m)
{
    if (!m) return;
    
    m->sps_tgt_q16 = 0;
    m->state = M_VEL;
    m->remaining_steps = 0;
    m->planned_remaining_q16 = 0;  // Clear planner tracking
}

bool motor_is_busy(const Motor* m)
{
    if (!m) return false;
    return (m->state != M_IDLE) || (Q16_TO_SPS(m->sps_now_q16) != 0);
}

int32_t motor_get_position_steps(const Motor* m)
{
    if (!m) return 0;
    return m->pos_steps;
}

int32_t motor_get_velocity_sps(const Motor* m)
{
    if (!m) return 0;
    int32_t sps = Q16_TO_SPS(m->sps_now_q16);
    return m->dir_sign ? -sps : sps;
}

void motor_tick_accel(Motor* m)
{
    if (!m) return;
    
    // Handle direction blanking period
    if (m->dir_blank > 0)
    {
        m->dir_blank--;
        return;
    }
    
    // Handle direction change request
    if (m->dir_pending != m->dir_sign && Q16_TO_SPS(m->sps_now_q16) == 0)
    {
        update_direction(m, m->dir_pending);
        
        // After direction change, restore the final target if we were doing a two-phase change
        if (m->sps_final_q16 > 0)
        {
            m->sps_tgt_q16 = m->sps_final_q16;
            m->sps_final_q16 = 0; // Clear the pending final target
        }
    }
    
    // State machine
    switch (m->state)
    {
        case M_IDLE:
            // Nothing to do
            break;
            
        case M_VEL:
            handle_velocity_mode(m);
            break;
            
        case M_POS_ACCEL:
        case M_POS_CRUISE:
        case M_POS_DECEL:
            handle_position_mode(m);
            break;
    }
    
    // Update stepgen if SPS changed significantly
    update_stepgen_sps(m);
}

void motor_poll_counter(Motor* m)
{
    if (!m || !m->hw.pulse_cnt_htim) return;
    
    uint32_t cnt_now = __HAL_TIM_GET_COUNTER(m->hw.pulse_cnt_htim);
    int32_t delta = (int32_t)(cnt_now - m->last_cnt);
    
    // Handle counter overflow - determine if 16-bit or 32-bit timer
    bool is_32bit = (m->hw.pulse_cnt_htim->Instance == TIM2 || m->hw.pulse_cnt_htim->Instance == TIM5);
    
    if (is_32bit)
    {
        // 32-bit timer - no overflow handling needed for reasonable polling rates
        // (would need 4.3 billion counts to overflow)
    }
    else
    {
        // 16-bit timer - handle overflow
        if (delta > 32767)
        {
            delta -= 65536;
        }
        else if (delta < -32767)
        {
            delta += 65536;
        }
    }
    
    // Update position
    if (m->dir_sign)
    {
        m->pos_steps -= delta;  // Negative direction
    }
    else
    {
        m->pos_steps += delta;  // Positive direction
    }
    
    // Update remaining steps for position moves
    if (m->state >= M_POS_ACCEL && m->state <= M_POS_DECEL)
    {
        m->remaining_steps -= abs_i32(delta);
        if (m->remaining_steps < 0)
        {
            m->remaining_steps = 0;
        }
    }
    
    m->last_cnt = cnt_now;
}

// Static helper functions
static void update_direction(Motor* m, uint8_t new_dir)
{
    if (m->dir_sign != new_dir)
    {
        m->dir_sign = new_dir;
        m->dir_pending = new_dir;
        
        if (m->hw.dir_port && m->hw.dir_pin)
        {
            GPIO_PinState pin_state = new_dir ? GPIO_PIN_SET : GPIO_PIN_RESET;
            HAL_GPIO_WritePin(m->hw.dir_port, m->hw.dir_pin, pin_state);
        }
        
        // Start blanking period
        m->dir_blank = DIR_BLANK_TICKS;
    }
}

static void update_stepgen_sps(Motor* m)
{
    int32_t sps_i = Q16_TO_SPS(m->sps_now_q16);
    
    if (abs_i32(sps_i - m->last_sps_i) >= SPS_MIN_DELTA)
    {
        m->last_sps_i = sps_i;
        if (m->hw.stepgen)
        {
            stepgen_set_sps(m->hw.stepgen, (uint32_t)sps_i);
        }
    }
}

static void handle_velocity_mode(Motor* m)
{
    int32_t delta_sps_q16 = m->a_max_q16 / ACCEL_TICK_HZ;
    
    if (m->sps_now_q16 < m->sps_tgt_q16)
    {
        // Accelerate
        m->sps_now_q16 += delta_sps_q16;
        if (m->sps_now_q16 > m->sps_tgt_q16)
        {
            m->sps_now_q16 = m->sps_tgt_q16;
        }
    }
    else if (m->sps_now_q16 > m->sps_tgt_q16)
    {
        // Decelerate
        m->sps_now_q16 -= delta_sps_q16;
        if (m->sps_now_q16 < m->sps_tgt_q16)
        {
            m->sps_now_q16 = m->sps_tgt_q16;
        }
    }
    
    // Check if stopped and idle
    if (m->sps_tgt_q16 == 0 && m->sps_now_q16 == 0)
    {
        m->state = M_IDLE;
    }
}

static void handle_position_mode(Motor* m)
{
    // Check hardware counter for precise stop (ground truth at 100Hz from polling)
    if (m->remaining_steps <= 0)
    {
        // Hardware says we've reached target - STOP immediately
        m->sps_now_q16 = 0;
        m->sps_tgt_q16 = 0;
        m->planned_remaining_q16 = 0;
        m->state = M_IDLE;
        return;
    }
    
    int32_t delta_sps_q16 = m->a_max_q16 / ACCEL_TICK_HZ;
    
    // Update planner remaining at 5kHz (for smooth decel timing)
    // Decrement by current velocity each tick
    if (m->sps_now_q16 > 0)
    {
        int32_t steps_this_tick_q16 = m->sps_now_q16 / ACCEL_TICK_HZ;
        m->planned_remaining_q16 -= steps_this_tick_q16;
        
        // Don't let planner go negative
        if (m->planned_remaining_q16 < 0)
        {
            m->planned_remaining_q16 = 0;
        }
    }
    
    // Calculate deceleration distance from current speed
    // decel_distance = v² / (2*a)
    int32_t decel_distance_q16 = 0;
    if (m->a_max_q16 > 0)
    {
        decel_distance_q16 = (m->sps_now_q16 * m->sps_now_q16) / (2 * m->a_max_q16);
    }
    
    // Use planner remaining for smooth decel timing decisions
    int32_t planned_remaining_steps = Q16_TO_SPS(m->planned_remaining_q16);
    
    switch (m->state)
    {
        case M_POS_ACCEL:
            // Check if we need to start decelerating (using planner remaining)
            if (decel_distance_q16 >= m->planned_remaining_q16)
            {
                // Start deceleration
                m->state = M_POS_DECEL;
            }
            else if (m->sps_now_q16 >= m->sps_tgt_q16)
            {
                // Reached cruise velocity
                m->state = M_POS_CRUISE;
            }
            else
            {
                // Continue accelerating
                m->sps_now_q16 += delta_sps_q16;
                if (m->sps_now_q16 > m->sps_tgt_q16)
                {
                    m->sps_now_q16 = m->sps_tgt_q16;
                }
            }
            break;
            
        case M_POS_CRUISE:
            // Check if we need to start decelerating (using planner remaining)
            if (decel_distance_q16 >= m->planned_remaining_q16)
            {
                // Start deceleration
                m->state = M_POS_DECEL;
            }
            // Maintain cruise speed (sps_now_q16 stays at sps_tgt_q16)
            break;
            
        case M_POS_DECEL:
            // Decelerate toward zero
            if (m->sps_now_q16 > delta_sps_q16)
            {
                m->sps_now_q16 -= delta_sps_q16;
            }
            else
            {
                // Close to zero - let hardware counter handle precise stop
                m->sps_now_q16 = 0;
            }
            break;
            
        default:
            break;
    }
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}
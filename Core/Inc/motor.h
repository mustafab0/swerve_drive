#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "stepgen.h"
#include <stdint.h>
#include <stdbool.h>

// Motor hardware description (owned by each motor)
typedef struct {
    StepGen*           stepgen;        // STEP PWM driver (from Phase 1)
    GPIO_TypeDef*      dir_port;       // DIR GPIO port
    uint16_t           dir_pin;        // DIR GPIO pin
    TIM_HandleTypeDef* pulse_cnt_htim; // Timer in External Clock Mode counting STEP pulses
    uint32_t           sps_max;        // clamp
    uint32_t           sps_start;      // optional safe start SPS when leaving zero (0=off)
} MotorHW;

typedef enum { 
    M_IDLE, 
    M_VEL, 
    M_POS_ACCEL, 
    M_POS_CRUISE, 
    M_POS_DECEL 
} MotorState;

typedef struct {
    MotorHW  hw;

    // speeds/accels as Q16.16 magnitudes
    int32_t  sps_now_q16;
    int32_t  sps_tgt_q16;
    int32_t  a_max_q16;

    // direction & gating
    uint8_t  dir_sign;     // 0/1 current DIR level
    uint8_t  dir_pending;  // request to flip dir when stopped
    uint16_t dir_blank;    // short blank time after DIR change (e.g., 5 ticks)
    int32_t  sps_final_q16; // final target after direction change (0 if no pending change)

    // position accounting
    int32_t  pos_steps;        // absolute position from pulse-counter deltas
    int32_t  remaining_steps;  // for position moves, counts down to 0 (from HW counter @ 100Hz)
    int32_t  planned_remaining_q16; // planner remaining in Q16.16 (for decel timing @ 5kHz)
    uint32_t last_cnt;         // last read of HW counter

    // cache
    int32_t  last_sps_i;       // last integer SPS sent to stepgen

    MotorState state;
} Motor;

// Public API
void    motor_init(Motor* m, const MotorHW* hw);
void    motor_set_velocity(Motor* m, int32_t sps_target, int32_t a_max);
void    motor_move_steps(Motor* m, int32_t steps, int32_t v_max_sps, int32_t a_max);
void    motor_stop_smooth(Motor* m);
bool    motor_is_busy(const Motor* m);
int32_t motor_get_position_steps(const Motor* m);
int32_t motor_get_velocity_sps(const Motor* m);

// Called by TIM6 ISR @ 5 kHz
void    motor_tick_accel(Motor* m);

// Called by control loop @ ~100–200 Hz
void    motor_poll_counter(Motor* m);

// Q16 conversion helpers
#define SPS_TO_Q16(sps)     ((int32_t)(sps) << 16)
#define Q16_TO_SPS(q16)     ((int32_t)((q16) >> 16))
#define Q16_FRAC_PART(q16)  ((q16) & 0xFFFF)

#endif
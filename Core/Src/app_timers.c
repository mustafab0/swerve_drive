#include "app_timers.h"

extern TIM_HandleTypeDef htim6;

// Enable DWT for cycle counting (for precise ISR timing)
static void enable_dwt_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable cycle counter
    DWT->CYCCNT = 0;                                 // Reset counter
}

// Diagnostics data
static TimerDiagnostics diag = {0};

// Moving average filter for ISR duration (simple exponential)
#define ALPHA_8BIT 32  // α = 32/256 ≈ 0.125 for moving average

void app_timers_init(void)
{
    // Enable DWT cycle counter for precise ISR timing
    enable_dwt_cycle_counter();
    
    // Reset diagnostics
    app_timers_reset_diagnostics();
    
    // Start TIM6 with interrupt at 5kHz
    HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        // Mark ISR start time for performance measurement (use DWT cycle counter)
        uint32_t start_cycles = DWT->CYCCNT;
        
        // Call the motor tick function (implemented in main.c)
        motor_tick_all_5k();
        
        // Update diagnostics
        uint32_t end_cycles = DWT->CYCCNT;
        uint32_t duration_cycles = end_cycles - start_cycles;
        
        // Convert cycles to microseconds (assuming 216MHz system clock)
        uint32_t duration_us = duration_cycles / 216;
        
        diag.tick_count++;
        
        // Update max duration
        if (duration_us > diag.max_duration_us)
        {
            diag.max_duration_us = duration_us;
        }
        
        // Update moving average (exponential filter)
        // avg = (1-α) * avg + α * new_sample
        diag.avg_duration_us = ((256 - ALPHA_8BIT) * diag.avg_duration_us + ALPHA_8BIT * duration_us) >> 8;
        
        // Detect overruns (ISR took longer than 200us = 1/5000s)
        if (duration_us > 200)
        {
            diag.overrun_count++;
        }
    }
}

const TimerDiagnostics* app_timers_get_diagnostics(void)
{
    return &diag;
}

void app_timers_reset_diagnostics(void)
{
    diag.tick_count = 0;
    diag.max_duration_us = 0;
    diag.avg_duration_us = 0;
    diag.overrun_count = 0;
}
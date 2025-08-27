#ifndef APP_TIMERS_H
#define APP_TIMERS_H

#include "main.h"
#include <stdint.h>

// Initialize all system timers
void app_timers_init(void);

// External function to be implemented by application layer
// This will be called at 5kHz from TIM6 ISR
void motor_tick_all_5k(void);

// Diagnostics - ISR performance metrics
typedef struct {
    uint32_t tick_count;           // Total ticks since start
    uint32_t max_duration_us;      // Peak ISR duration in microseconds  
    uint32_t avg_duration_us;      // Moving average duration
    uint32_t overrun_count;        // Count of ISR overruns (if detected)
} TimerDiagnostics;

// Get timer diagnostics (for CLI reporting)
const TimerDiagnostics* app_timers_get_diagnostics(void);

// Reset diagnostics counters
void app_timers_reset_diagnostics(void);

#endif
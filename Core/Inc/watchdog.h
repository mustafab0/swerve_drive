#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Watchdog health monitoring flags
typedef struct {
    bool motor_polling_ok;          // All 6 motors polled
    bool swerve_update_ok;          // All 3 modules updated
    bool kinematics_ok;             // Kinematics computed and sent
    bool tim6_heartbeat_ok;         // 5kHz TIM6 ISR alive
} WatchdogHealth;

// Initialize IWDG with ~500ms timeout
void watchdog_init(void);

// Reset all health flags (call at start of each 100Hz loop)
void watchdog_reset_health_flags(void);

// Set individual health flags
void watchdog_set_motor_polling_ok(void);
void watchdog_set_swerve_update_ok(void);
void watchdog_set_kinematics_ok(void);
void watchdog_set_tim6_heartbeat_ok(void);

// Check if all health conditions are met and kick watchdog if so
void watchdog_conditional_kick(void);

// Check for watchdog timeout and reset if needed (call from main loop)
void watchdog_check_timeout(void);

// Get current health status (for debugging/telemetry)
const WatchdogHealth* watchdog_get_health_status(void);

// Manual watchdog kick (use sparingly)
void watchdog_kick(void);

// Called from TIM6 5kHz ISR to update heartbeat
void watchdog_tim6_heartbeat(void);

#ifdef __cplusplus
}
#endif

#endif // WATCHDOG_H
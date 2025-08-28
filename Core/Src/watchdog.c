#include "watchdog.h"

// Health monitoring state
static WatchdogHealth health_status = {0};

// Software watchdog state
static uint32_t watchdog_last_kick_time = 0;
static const uint32_t WATCHDOG_TIMEOUT_MS = 500;  // 500ms timeout
static bool watchdog_enabled = false;

// TIM6 heartbeat monitoring
static volatile uint32_t tim6_last_tick = 0;

void watchdog_init(void)
{
    // Initialize software watchdog
    watchdog_last_kick_time = HAL_GetTick();
    watchdog_enabled = true;
    
    // Reset health flags
    watchdog_reset_health_flags();
    
    // Initialize TIM6 heartbeat tracking
    tim6_last_tick = HAL_GetTick();
}

void watchdog_reset_health_flags(void)
{
    health_status.motor_polling_ok = false;
    health_status.swerve_update_ok = false;
    health_status.kinematics_ok = false;
    health_status.tim6_heartbeat_ok = false;
}

void watchdog_set_motor_polling_ok(void)
{
    health_status.motor_polling_ok = true;
}

void watchdog_set_swerve_update_ok(void)
{
    health_status.swerve_update_ok = true;
}

void watchdog_set_kinematics_ok(void)
{
    health_status.kinematics_ok = true;
}

void watchdog_set_tim6_heartbeat_ok(void)
{
    health_status.tim6_heartbeat_ok = true;
}

// Call this from TIM6 ISR to update heartbeat
void watchdog_tim6_heartbeat(void)
{
    tim6_last_tick = HAL_GetTick();
}

void watchdog_conditional_kick(void)
{
    // Check TIM6 heartbeat (should have been updated recently)
    uint32_t current_tick = HAL_GetTick();
    
    // TIM6 runs at 5kHz, so we expect updates every 0.2ms
    // Allow up to 5ms tolerance for checking
    if ((current_tick - tim6_last_tick) <= 5) {
        health_status.tim6_heartbeat_ok = true;
    } else {
        health_status.tim6_heartbeat_ok = false;
    }
    
    // Only kick if ALL health conditions are met
    if (health_status.motor_polling_ok &&
        health_status.swerve_update_ok &&
        health_status.kinematics_ok &&
        health_status.tim6_heartbeat_ok) {
        
        // All systems healthy - kick the software watchdog
        watchdog_kick();
    }
    
    // If any health check failed, do NOT kick -> system will reset in ~500ms
}

// Check if watchdog has timed out (call from main loop)
void watchdog_check_timeout(void)
{
    if (!watchdog_enabled) return;
    
    uint32_t current_tick = HAL_GetTick();
    if ((current_tick - watchdog_last_kick_time) >= WATCHDOG_TIMEOUT_MS) {
        // Watchdog timeout - force system reset
        NVIC_SystemReset();
    }
}

const WatchdogHealth* watchdog_get_health_status(void)
{
    return &health_status;
}

void watchdog_kick(void)
{
    if (watchdog_enabled) {
        watchdog_last_kick_time = HAL_GetTick();
    }
}
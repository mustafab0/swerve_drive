#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H

#include "motor.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// swerve_module.h — steer+drive wrapper with shortest-path, drive inversion, and ±150° steer limit

// Steer soft limit (mechanical) ±150°
#define SWERVE_STEER_LIMIT_RAD (150.0f * 3.14159265358979323846f / 180.0f)

// Angle tolerance to consider "at setpoint" (adjust to taste)
#define SWERVE_ANGLE_TOL_RAD (0.010f) // ~0.57°

// Physical constants
#define SWERVE_STEER_GEAR_RATIO    2.7f
#define SWERVE_DRIVE_GEAR_RATIO    3.0f
#define SWERVE_WHEEL_RADIUS_M      0.15f

// ---------- Data types ----------
typedef struct {
    // Motors
    Motor* steer; // steer (position/relative moves)
    Motor* drive; // drive (velocity)

    // Calibration inputs from user board (required at init)
    // Motor steps-per-rev (fullsteps * microsteps) at the MOTOR shaft
    float steer_steps_per_rev_motor;
    float drive_steps_per_rev_motor;

    // Derived conversion factors (computed in init)
    float steer_steps_per_rad; // (+) steps per rad at the steering JOINT
    float drive_steps_per_m; // (+) steps per meter of wheel travel

    // Motion limits (steps/s and steps/s^2 at MOTOR; set from your motor caps)
    int32_t steer_vmax_sps;
    int32_t steer_amax_sps2;
    int32_t drive_vmax_sps;
    int32_t drive_amax_sps2;

    // Homing / zeroing
    bool is_homed;
    int32_t steer_zero_offset_steps; // absolute steps corresponding to 0 rad

    // Runtime
    float angle_cmd_rad; // last commanded absolute angle (wrapped)
    uint8_t drive_inverted; // 0/1 inversion flag applied to drive velocity
} SwerveModule;

// ---------- API ----------
void swerve_module_init(
    SwerveModule* m,
    Motor* steer, Motor* drive,
    float steer_steps_per_rev_motor,
    float drive_steps_per_rev_motor,
    int32_t steer_vmax_sps, int32_t steer_amax_sps2,
    int32_t drive_vmax_sps, int32_t drive_amax_sps2
);

// Set current mechanical angle = 0 rad (call after your homing procedure reaches the zero reference)
static inline void swerve_module_zero_here(SwerveModule* m) {
    // absolute steps now become the zero reference
    int32_t pos = motor_get_position_steps(m->steer);
    m->steer_zero_offset_steps = -pos; // so abs_steps = 0 at this pose
    m->is_homed = true;
}

static inline bool swerve_module_is_homed(const SwerveModule* m) { 
    return m->is_homed; 
}

// High-level commands
void swerve_module_set_angle_abs(SwerveModule* m, float angle_rad);
void swerve_module_set_wheel_speed(SwerveModule* m, float v_mps);

// Utility
float swerve_module_get_angle_abs(const SwerveModule* m); // returns wrapped (-pi,pi]
float swerve_module_get_current_velocity(const SwerveModule* m); // returns current velocity in m/s

// Legacy compatibility functions for existing CLI
static inline void swerve_module_home(SwerveModule* m, int32_t zero_offset_steps) {
    m->steer_zero_offset_steps = zero_offset_steps;
    m->is_homed = true;
}

static inline float swerve_module_get_current_angle(const SwerveModule* m) {
    return swerve_module_get_angle_abs(m);
}

static inline void swerve_module_set_command(SwerveModule* m, float angle_rad, float velocity_mps) {
    swerve_module_set_angle_abs(m, angle_rad);
    swerve_module_set_wheel_speed(m, velocity_mps);
}

// Additional compatibility functions for CLI
static inline bool swerve_module_is_at_angle(const SwerveModule* m) {
    if (!m->is_homed) return false;
    float current = swerve_module_get_angle_abs(m);
    float error = fabsf(m->angle_cmd_rad - current);
    return error <= SWERVE_ANGLE_TOL_RAD;
}

static inline void swerve_module_set_calibration(SwerveModule* m, float steer_spr, float drive_spm, float wheel_radius) {
    // Update calibration values
    m->steer_steps_per_rad = steer_spr;
    m->drive_steps_per_m = drive_spm;
}

// Helper functions for CLI
static inline float angle_difference(float target_rad, float current_rad) {
    float diff = target_rad - current_rad;
    // Normalize to [-π, π]
    while (diff > 3.14159f) diff -= 2.0f * 3.14159f;
    while (diff < -3.14159f) diff += 2.0f * 3.14159f;
    return diff;
}

static inline int32_t angle_to_steps(float angle_rad, float steps_per_rad, int32_t zero_offset) {
    return zero_offset + (int32_t)(angle_rad * steps_per_rad);
}

#endif
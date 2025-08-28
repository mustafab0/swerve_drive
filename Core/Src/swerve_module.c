#include "swerve_module.h"
#include "motor.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Constants from the header file - remove redefinition to avoid warning


static inline float wrap_pi(float x){
// wrap to (-pi, pi]
while (x <= -M_PI) x += 2.0f*(float)M_PI;
while (x > M_PI) x -= 2.0f*(float)M_PI;
return x;
}


static inline float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }
static inline float sgnf(float v){ return (v < 0.0f) ? -1.0f : 1.0f; }


void swerve_module_init(
SwerveModule* m,
Motor* steer, Motor* drive,
float steer_steps_per_rev_motor,
float drive_steps_per_rev_motor,
int32_t steer_vmax_sps, int32_t steer_amax_sps2,
int32_t drive_vmax_sps, int32_t drive_amax_sps2
){
m->steer = steer;
m->drive = drive;
m->steer_steps_per_rev_motor = steer_steps_per_rev_motor;
m->drive_steps_per_rev_motor = drive_steps_per_rev_motor;


// Convert motor steps to joint/wheel space via gear ratios & wheel radius
// steer: motor_steps_per_rev * GR / (2*pi)
m->steer_steps_per_rad = (steer_steps_per_rev_motor * SWERVE_STEER_GEAR_RATIO) / (2.0f*(float)M_PI);
// drive: motor_steps_per_rev * GR / (2*pi*R)
m->drive_steps_per_m = (drive_steps_per_rev_motor * SWERVE_DRIVE_GEAR_RATIO) / (2.0f*(float)M_PI * SWERVE_WHEEL_RADIUS_M);


m->steer_vmax_sps = steer_vmax_sps;
m->steer_amax_sps2 = steer_amax_sps2;
m->drive_vmax_sps = drive_vmax_sps;
m->drive_amax_sps2 = drive_amax_sps2;


m->is_homed = false;
m->steer_zero_offset_steps = 0;
m->angle_cmd_rad = 0.0f;
m->drive_inverted = 0;
}


float swerve_module_get_angle_abs(const SwerveModule* m){
// absolute steps at joint = zero_offset + motor position steps
int32_t abs_steps = m->steer_zero_offset_steps + motor_get_position_steps(m->steer);
float ang = (float)abs_steps / m->steer_steps_per_rad; // rad
return wrap_pi(ang);
}


void swerve_module_set_wheel_speed(SwerveModule* m, float v_mps){
// convert linear m/s to motor steps/s and apply inversion if needed
float sps_f = v_mps * m->drive_steps_per_m;
if (m->drive_inverted) sps_f = -sps_f;
// clamp to drive limits
if (sps_f > (float)m->drive_vmax_sps) sps_f = (float)m->drive_vmax_sps;
if (sps_f < -(float)m->drive_vmax_sps) sps_f = -(float)m->drive_vmax_sps;
motor_set_velocity(m->drive, (int32_t)(sps_f + (sps_f >= 0 ? 0.5f : -0.5f)), m->drive_amax_sps2);
}


void swerve_module_set_angle_abs(SwerveModule* m, float angle_rad){
    if (!m->is_homed) {
        // ignore until homed; optionally queue or log
        return;
    }
    const float limit = SWERVE_STEER_LIMIT_RAD; // ±150° in rad

    // Wrap desired and read current
    float target = wrap_pi(angle_rad);
    float current = swerve_module_get_angle_abs(m);

    // Calculate both possible targets (normal and flipped)
    float target_normal = target;
    float target_flipped = wrap_pi(target + (float)M_PI);
    
    // Calculate deltas for both options
    float delta_normal = wrap_pi(target_normal - current);
    float delta_flipped = wrap_pi(target_flipped - current);
    
    // Check which targets are within physical limits
    bool normal_valid = (target_normal >= -limit && target_normal <= limit);
    bool flipped_valid = (target_flipped >= -limit && target_flipped <= limit);
    
    // Choose the best option considering both limits and shortest path
    float final_target;
    uint8_t drive_inverted;
    
    if (normal_valid && flipped_valid) {
        // Both options valid, choose shortest path
        if (fabsf(delta_normal) <= fabsf(delta_flipped)) {
            final_target = target_normal;
            drive_inverted = 0;
        } else {
            final_target = target_flipped;
            drive_inverted = 1;
        }
    } else if (normal_valid) {
        // Only normal target is valid
        final_target = target_normal;
        drive_inverted = 0;
    } else if (flipped_valid) {
        // Only flipped target is valid
        final_target = target_flipped;
        drive_inverted = 1;
    } else {
        // Neither target is valid, clamp normal target to limits
        final_target = clampf(target_normal, -limit, limit);
        drive_inverted = 0;
    }
    
    m->drive_inverted = drive_inverted;
    m->angle_cmd_rad = final_target;
    
    // Convert to motor steps and move
    int32_t target_steps = m->steer_zero_offset_steps + (int32_t)(final_target * m->steer_steps_per_rad);
    int32_t current_steps = m->steer_zero_offset_steps + motor_get_position_steps(m->steer);
    int32_t steps_to_move = target_steps - current_steps;
    
    // Only move if above threshold
    int32_t threshold = (int32_t)(SWERVE_ANGLE_TOL_RAD * m->steer_steps_per_rad);
    if (abs(steps_to_move) > threshold) {
        motor_move_steps(m->steer, steps_to_move, m->steer_vmax_sps, m->steer_amax_sps2);
    }
}

float swerve_module_get_current_velocity(const SwerveModule* m) {
    if (!m) return 0.0f;
    
    // Get drive motor velocity in steps/s
    int32_t drive_sps = motor_get_velocity_sps(m->drive);
    
    // Convert to m/s
    float velocity_mps = (float)drive_sps / m->drive_steps_per_m;
    
    // Account for drive inversion
    if (m->drive_inverted) {
        velocity_mps = -velocity_mps;
    }
    
    return velocity_mps;
}
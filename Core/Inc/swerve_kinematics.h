// swerve_kinematics.h — 3‑module swerve kinematics with slew limiting and scaling
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
// Positions of each module in the robot body frame (meters)
// (+x forward, +y left). Index order is up to you (e.g., 0=FR, 1=FL, 2=R).
float x[3];
float y[3];

// Chassis limits
float vx_max;          // Max forward/backward speed (m/s)
float vy_max;          // Max left/right speed (m/s)  
float wz_max;          // Max yaw rate (rad/s)
float v_module_max;    // Max individual wheel speed (m/s)

// Acceleration limits (per 0.01s tick at 100Hz)
float dvx_max;         // Max vx change per tick (m/s per tick)
float dvy_max;         // Max vy change per tick (m/s per tick)
float dwz_max;         // Max wz change per tick (rad/s per tick)
} SKM_Config;


// State for slew limiting
typedef struct {
SKM_Config cfg;
// User-commanded targets
float vx_cmd, vy_cmd, wz_cmd;
// Slew-limited actual values used for computation
float vx, vy, wz;
} SKM_State;


// Outputs for one kinematics evaluation
typedef struct {
float angle_rad[3]; // desired wheel angles in (−pi, pi]
float speed_mps[3]; // desired wheel linear speeds (>=0)
float scale_factor; // scaling applied if any wheel exceeded limits
} SKM_Out;


// Initialize with module positions and limits
void skm_init(SKM_State* state, const SKM_Config* cfg);

// Set user twist commands
static inline void skm_set_twist(SKM_State* state, float vx, float vy, float wz) {
state->vx_cmd = vx;
state->vy_cmd = vy; 
state->wz_cmd = wz;
}

// Update at 100Hz - applies slew limiting and computes wheel targets
void skm_update_100Hz(SKM_State* state, SKM_Out* out);


#ifdef __cplusplus
}
#endif
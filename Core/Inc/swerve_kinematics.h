// swerve_kinematics.h — 3‑module swerve kinematics with scaling
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
float v_module_max;    // Max individual wheel speed (m/s)
} SKM_Config;


// State for kinematics
typedef struct {
SKM_Config cfg;
// User-commanded twist values
float vx_cmd, vy_cmd, wz_cmd;
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

// Update kinematics - computes wheel targets from twist commands
void skm_update_100Hz(SKM_State* state, SKM_Out* out);


#ifdef __cplusplus
}
#endif
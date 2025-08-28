// swerve_kinematics.c — 3-module swerve kinematics with scaling
#include "../Inc/swerve_kinematics.h"
#include <math.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


static inline float wrap_pi(float a){
while(a<=-M_PI) a+=2.0f*(float)M_PI;
while(a> M_PI) a-=2.0f*(float)M_PI;
return a;
}

static inline float clampf(float v, float lo, float hi) {
return v < lo ? lo : (v > hi ? hi : v);
}

static inline float hypot2f(float x,float y){ return sqrtf(x*x + y*y); }


void skm_init(SKM_State* state, const SKM_Config* cfg) {
// Copy config
state->cfg = *cfg;

// Initialize commands to zero
state->vx_cmd = state->vy_cmd = state->wz_cmd = 0.0f;
}


void skm_update_100Hz(SKM_State* state, SKM_Out* out) {
const SKM_Config* cfg = &state->cfg;

// Use commanded values directly (no slew limiting)
float vx = state->vx_cmd;
float vy = state->vy_cmd; 
float wz = state->wz_cmd;

// Compute per-module kinematics
float max_speed = 0.0f;

for (int i = 0; i < 3; i++) {
    // Tangential velocity from body yaw rate at module position  
    float tx = wz * cfg->y[i];
    float ty = wz * cfg->x[i];
    float Vx = vx + tx;
    float Vy = vy + ty;
    
    out->angle_rad[i] = wrap_pi(atan2f(Vy, Vx)); // desired wheel heading
    out->speed_mps[i] = hypot2f(Vx, Vy); // wheel linear speed (>=0)
    
    // Track maximum speed across all modules
    if (out->speed_mps[i] > max_speed) {
        max_speed = out->speed_mps[i];
    }
}

// Uniform scaling: if any wheel exceeds max, scale all down proportionally
out->scale_factor = 1.0f;
if (max_speed > cfg->v_module_max) {
    out->scale_factor = cfg->v_module_max / max_speed;
    for (int i = 0; i < 3; i++) {
        out->speed_mps[i] *= out->scale_factor;
    }
}
}
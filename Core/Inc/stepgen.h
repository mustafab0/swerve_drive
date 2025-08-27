#ifndef STEPGEN_H
#define STEPGEN_H

#include "main.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;           // e.g., TIM_CHANNEL_1
    uint32_t timer_clk_hz;      // exact timer clock (pass from main)
    uint32_t sps_now;           // cached current SPS
    uint8_t  running;           // 0/1
} StepGen;

void stepgen_init(StepGen* sg, TIM_HandleTypeDef* htim, uint32_t channel, uint32_t timer_clk_hz);
void stepgen_start(StepGen* sg);
void stepgen_stop(StepGen* sg);
void stepgen_set_sps(StepGen* sg, uint32_t sps);   // 0 = stop
uint32_t stepgen_get_sps(const StepGen* sg);

#endif
#include "stepgen.h"

static void set_timer_values(StepGen* sg, uint32_t arr_val, uint32_t ccr_val);

void stepgen_init(StepGen* sg, TIM_HandleTypeDef* htim, uint32_t channel, uint32_t timer_clk_hz)
{
    if (!sg || !htim) return;
    
    sg->htim = htim;
    sg->channel = channel;
    sg->timer_clk_hz = timer_clk_hz;
    sg->sps_now = 0;
    sg->running = 0;
    
    // Enable ARR preload (auto-reload preload)
    htim->Instance->CR1 |= TIM_CR1_ARPE;
    
    // Enable CCR preload for the specific channel
    switch(channel)
    {
        case TIM_CHANNEL_1:
            htim->Instance->CCMR1 |= TIM_CCMR1_OC1PE;
            break;
        case TIM_CHANNEL_2:
            htim->Instance->CCMR1 |= TIM_CCMR1_OC2PE;
            break;
        case TIM_CHANNEL_3:
            htim->Instance->CCMR2 |= TIM_CCMR2_OC3PE;
            break;
        case TIM_CHANNEL_4:
            htim->Instance->CCMR2 |= TIM_CCMR2_OC4PE;
            break;
    }
}

void stepgen_start(StepGen* sg)
{
    if (!sg || !sg->htim) return;
    
    if (!sg->running && sg->sps_now > 0)
    {
        HAL_TIM_PWM_Start(sg->htim, sg->channel);
        HAL_TIM_Base_Start(sg->htim);
        sg->running = 1;
    }
}

void stepgen_stop(StepGen* sg)
{
    if (!sg || !sg->htim) return;
    
    if (sg->running)
    {
        HAL_TIM_PWM_Stop(sg->htim, sg->channel);
        // Note: Don't stop base timer as other channels might be using it
        sg->running = 0;
    }
}

void stepgen_set_sps(StepGen* sg, uint32_t sps)
{
    if (!sg || !sg->htim) return;
    
    // Stop if sps is 0
    if (sps == 0)
    {
        stepgen_stop(sg);
        sg->sps_now = 0;
        return;
    }
    
    // Only update if SPS actually changed (idempotent)
    if (sg->sps_now == sps) return;
    
    // Calculate new ARR and CCR values
    // ARR = (timer_clk_hz / sps) - 1
    // Guard against divide-by-zero (already checked above, but safety)
    if (sps == 0) return;
    
    uint32_t period_ticks = sg->timer_clk_hz / sps;
    if (period_ticks == 0) period_ticks = 1; // Minimum period
    
    uint32_t arr_val = period_ticks - 1;
    uint32_t ccr_val = period_ticks / 2;  // 50% duty cycle
    
    // Ensure CCR doesn't exceed ARR
    if (ccr_val > arr_val) ccr_val = arr_val;
    
    // Update timer values
    set_timer_values(sg, arr_val, ccr_val);
    
    sg->sps_now = sps;
    
    // Start if not already running
    if (!sg->running)
    {
        stepgen_start(sg);
    }
}

uint32_t stepgen_get_sps(const StepGen* sg)
{
    if (!sg) return 0;
    return sg->sps_now;
}


// Helper function to set ARR and CCR values with preload
static void set_timer_values(StepGen* sg, uint32_t arr_val, uint32_t ccr_val)
{
    if (!sg || !sg->htim) return;
    
    TIM_HandleTypeDef* htim = sg->htim;
    
    // Set ARR (period)
    __HAL_TIM_SET_AUTORELOAD(htim, arr_val);
    
    // Set CCR (compare value) for the specific channel
    switch(sg->channel)
    {
        case TIM_CHANNEL_1:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, ccr_val);
            break;
        case TIM_CHANNEL_2:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, ccr_val);
            break;
        case TIM_CHANNEL_3:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, ccr_val);
            break;
        case TIM_CHANNEL_4:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, ccr_val);
            break;
    }
    
    // Generate update event to latch the preloaded values
    htim->Instance->EGR = TIM_EGR_UG;
}
/**
 * @file drv8251.c
 * @brief DRV8251 motor driver implementation
 */

#include "drv8251.h"

#include <math.h>

// -- PRIVATE FUNCTION PROTOTYPES ---------------------------------------------



// -- FUNCTION DEFINITIONS ----------------------------------------------------

void DRV8251_Init(drv8251_config_t *config) {
    config->tim_autoreload = __HAL_TIM_GET_AUTORELOAD(config->in1_tim);

    // Start PWM on both channels with 0% duty cycle
    HAL_TIM_PWM_Start(config->in1_tim, config->in1_tim_channel);
    HAL_TIM_PWM_Start(config->in2_tim, config->in2_tim_channel);
    __HAL_TIM_SET_COMPARE(config->in1_tim, config->in1_tim_channel, 0);
    __HAL_TIM_SET_COMPARE(config->in2_tim, config->in2_tim_channel, 0);

    // Set initial state to coast
    config->state = DRV8251_STATE_COAST;
}

void DRV8251_SetSpeed(drv8251_config_t *config, float speed) {
    // TODO: check if this is set up correctly
    // Clamp speed to [-1.0, 1.0]
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;
    config->speed = speed;

    if (config->dir_inverted) speed = -speed;

    float duty = fabsf(speed);

    if (duty < DRV8251_MIN_DUTY_CYCLE) {
        DRV8251_Coast(config);
        return;
    } 

    uint32_t autoreload = config->tim_autoreload;
    
    if (speed > 0.0f) {
        // Forward: set IN1 to 100% duty cycle and IN2 duty cycle based on speed
        __HAL_TIM_SET_COMPARE(config->in1_tim, config->in1_tim_channel,
            autoreload); // IN1=1 (100% duty cycle)
        __HAL_TIM_SET_COMPARE(config->in2_tim, config->in2_tim_channel,
            autoreload*(1.0f - duty)); // IN2=PWM
        config->state = DRV8251_STATE_FORWARD;
    } else {
        // Reverse: set IN2 to 100% duty cycle and IN1 duty cycle based on speed
        __HAL_TIM_SET_COMPARE(config->in1_tim, config->in1_tim_channel,
            autoreload*(1.0f - duty)); // IN1=PWM
        __HAL_TIM_SET_COMPARE(config->in2_tim, config->in2_tim_channel,
            autoreload); // IN2=1 (100% duty cycle)
        config->state = DRV8251_STATE_REVERSE;
    }
}

void DRV8251_Brake(drv8251_config_t *config) {
    // Set both channels to 100% duty cycle
    __HAL_TIM_SET_COMPARE(
        config->in1_tim, config->in1_tim_channel, config->tim_autoreload);
    __HAL_TIM_SET_COMPARE(
        config->in2_tim, config->in2_tim_channel, config->tim_autoreload);

    config->state = DRV8251_STATE_BRAKE;
    config->speed = 0.0f;
}

void DRV8251_Coast(drv8251_config_t *config) {
    // Set both channels to 0% duty cycle
    __HAL_TIM_SET_COMPARE(config->in1_tim, config->in1_tim_channel, 0);
    __HAL_TIM_SET_COMPARE(config->in2_tim, config->in2_tim_channel, 0);
    config->state = DRV8251_STATE_COAST;
}


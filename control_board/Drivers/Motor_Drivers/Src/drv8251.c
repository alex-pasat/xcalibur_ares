/**
 * @file drv8251.c
 * @brief DRV8251 motor driver implementation
 */

#include "drv8251.h"
#include "motor_utils.h"

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
}

void DRV8251_SetDuty(drv8251_config_t *config, float duty) {
  if (duty >  1.0f) duty =  1.0f;
  if (duty < -1.0f) duty = -1.0f;
  config->duty = duty;

  if (config->dir_inverted) duty = -duty;

  float mag = fabsf(duty);

  if (mag > 0 && mag < config->MIN_DUTY_CYCLE) {
    mag = config->MIN_DUTY_CYCLE;
  }

  uint32_t ar = config->tim_autoreload;
  uint32_t compare = (uint32_t)(mag * ar);

  if (duty > 0.0f) {
    // Forward: IN1=0, IN2=PWM
    __HAL_TIM_SET_COMPARE(config->in1_tim, config->in1_tim_channel, 0);
    __HAL_TIM_SET_COMPARE(config->in2_tim, config->in2_tim_channel, compare);
  } else {
    // Reverse: IN1=PWM, IN2=0
    __HAL_TIM_SET_COMPARE(config->in1_tim, config->in1_tim_channel, compare);
    __HAL_TIM_SET_COMPARE(config->in2_tim, config->in2_tim_channel, 0);
  }
}

void DRV8251_SetSpeed(drv8251_config_t *config, float speed) {
  if (speed > config->MAX_RPS) speed = config->MAX_RPS;

  float duty = speed / config->MAX_RPS;
  DRV8251_SetDuty(config, duty);
  config->speed_rps = speed;
}

void DRV8251_Brake(drv8251_config_t *config) {
  // Set both channels to 100% duty cycle
  __HAL_TIM_SET_COMPARE(config->in1_tim, config->in1_tim_channel, config->tim_autoreload);
  __HAL_TIM_SET_COMPARE(config->in2_tim, config->in2_tim_channel, config->tim_autoreload);
  config->duty = 0;
  config->speed_rps = 0.0f;

}

void DRV8251_Coast(drv8251_config_t *config) {
    // Set both channels to 0% duty cycle
    __HAL_TIM_SET_COMPARE(config->in1_tim, config->in1_tim_channel, 0);
    __HAL_TIM_SET_COMPARE(config->in2_tim, config->in2_tim_channel, 0);
    config->duty = 0;
    config->speed_rps = 0.0f;
}


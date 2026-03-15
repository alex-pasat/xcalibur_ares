#include "drv88xx.h"

#include <math.h>

// Internal helper functions

// TODO: use this timer
static uint32_t get_tim_us(drv88xx_config_t *config) {
  return (HAL_GetTick() * 1000) + __HAL_TIM_GET_COUNTER(config->tim);
}

static int32_t dist_to_go(drv88xx_config_t *config)
{
    return config->target_pos - config->current_pos;
}

static float compute_new_speed(drv88xx_config_t *config)
{
  int32_t dist_to = dist_to_go(config);
  int32_t steps_to_stop = (int32_t)((config->speed * config->speed) / (2.0f * config->acceleration));

  if (dist_to == 0 && steps_to_stop <= 1)
  {
    config->step_interval_us = 0;
    config->speed = 0.0f;
    config->n = 0;
    return 0;
  }

  if (dist_to > 0)
  {
    // We are anticlockwise from the target
    // Need to go clockwise from here, maybe decelerate now
    if (config->n > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((steps_to_stop >= dist_to) || config->direction == DIRECTION_CCW)
        config->n = -steps_to_stop; // Start deceleration
    }
    else if (config->n < 0)
    {
      // Currently decelerating, need to accel again?
      if ((steps_to_stop < dist_to) && config->direction == DIRECTION_CW)
        config->n = -config->n; // Start accceleration
    }
  }
  else if (dist_to < 0)
  {
    // We are clockwise from the target
    // Need to go anticlockwise from here, maybe decelerate
    if (config->n > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((steps_to_stop >= -dist_to) || config->direction == DIRECTION_CW)
      config->n = -steps_to_stop; // Start deceleration
    }
    else if (config->n < 0)
    {
      // Currently decelerating, need to accel again?
      if ((steps_to_stop < -dist_to) && config->direction == DIRECTION_CCW)
      config->n = -config->n; // Start accceleration
    }
  }

  // need to acclelerate or decelerate
  if (config->n == 0)
  {
      // first step from stopped
      config->cn = config->c0;
      config->direction = (dist_to > 0) ? DIRECTION_CW : DIRECTION_CCW;
  }
  else
  {
      // subsequent step, accelerate or decelerate
      config->cn = config->cn - ((2.0f * config->cn) / ((4.0f * config->n) + 1));
      config->cn = (config->cn < config->cmin) ? config->cmin : config->cn;
  }

  config->n++;
  config->step_interval_us = (uint32_t)config->cn;
  config->speed = 1000000.0f / (config->cn);
  config->speed = (config->direction == DIRECTION_CW) ? config->speed : -config->speed;

  return config->step_interval_us;
}

// Public API functions

void DRV88xx_Init(drv88xx_config_t *config, float max_speed, float acceleration)
{
    // Set default values
    config->current_pos = 0;
    config->target_pos = 0;
    config->speed = 0.0f;
    config->last_step_time = 0;

    // Initialize GPIO pins
    HAL_GPIO_WritePin(config->dir_port, config->dir_pin, config->dir_inverted ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(config->step_port, config->step_pin, GPIO_PIN_RESET); // Ensure step pin is low
    // disable the motor by default
    if (config->en_port != NULL)
        HAL_GPIO_WritePin(config->en_port, config->en_pin,
            config->en_inverted ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Recompute accelerations constants based on max speed and acceleration
    DRV88xx_SetAcceleration(config, acceleration);
    DRV88xx_SetMaxSpeed(config, max_speed);

    if (config->speed == 0.0f) DRV88xx_SetSpeed(config, DRV88xx_DEFAULT_SPEED);
}

void DRV88xx_SetCurrentPosition(drv88xx_config_t *config, int32_t position)
{
    config->current_pos = position;
    config->target_pos = position;
    config->n = 0;
    config->step_interval_us = 0;
    config->speed = 0.0f;
}

void DRV88xx_SetSpeed(drv88xx_config_t *config, float speed)
{
  if (speed == config->speed)
    return;
  speed = (speed > config->max_speed) ? config->max_speed : ((speed < -config->max_speed) ? -config->max_speed : speed);
  if (speed == 0.0f)
    config->step_interval_us = 0;
  else
  {
    config->step_interval_us = (uint32_t)fabsf(1000000.0f / speed);
    config->direction = (speed > 0.0f) ? DIRECTION_CW : DIRECTION_CCW;
  }
  config->speed = speed;
}

void DRV88xx_SetMaxSpeed(drv88xx_config_t *config, float max_speed)
{
    if (max_speed <= 0.0f)
        return; // Invalid max speed, ignore
    else if (config->max_speed != max_speed)
    {
        config->max_speed = max_speed;
        config->cmin = 1000000.0f / max_speed;
        // recompute n from current speed and adjust speed if accelerating or cruising
        if (config->n > 0)
        {
            config->n = (int32_t)((config->speed * config->speed) / (2.0f * config->acceleration)); // Equation 16
            compute_new_speed(config);
        }
    }
}

void DRV88xx_SetAcceleration(drv88xx_config_t *config, float acceleration)
{
    if (acceleration <= 0.0f)
        return; // Invalid acceleration, ignore

    if (config->acceleration != acceleration)
    {
        config->n = (int32_t)((float)config->n * (config->acceleration / acceleration));
        config->c0 = 0.676f * sqrtf(2.0f / acceleration) * 1000000.0f;
        config->acceleration = acceleration;
        compute_new_speed(config);
    }
}

void DRV88xx_MoveTo(drv88xx_config_t *config, int32_t target_pos)
{
  int32_t target_usteps = target_pos;
  if (target_usteps != config->target_pos)
  {
      config->target_pos = target_usteps;
      compute_new_speed(config);
  }
}

void DRV88xx_Move(drv88xx_config_t *config, int32_t relative)
{
    DRV88xx_MoveTo(config, config->current_pos + relative);
}

bool DRV88xx_Run(drv88xx_config_t *config)
{
    if (DRV88xx_RunSpeed(config))
        compute_new_speed(config);
    return (config->speed != 0.0f) || dist_to_go(config) != 0;
}

bool DRV88xx_RunSpeed(drv88xx_config_t *config)
{
  if (!config->step_interval_us) return false; 

  uint32_t time_since_last_step = get_tim_us(config) - config->last_step_time;
  if (time_since_last_step < config->step_interval_us)
    return false; // Not time for the next step yet

  // Set direction pin, considering inversion
  HAL_GPIO_WritePin(
    config->dir_port, 
    config->dir_pin, 
    (config->direction == DIRECTION_CW) ^ config->dir_inverted ? GPIO_PIN_SET : GPIO_PIN_RESET
  );
  
  // Time to step
  if (config->direction == DIRECTION_CW)
    config->current_pos++;
  else
    config->current_pos--;

  // Pulse the step pin
  HAL_GPIO_WritePin(config->step_port, config->step_pin, GPIO_PIN_SET);
  uint32_t pulse_start = get_tim_us(config);
  while (get_tim_us(config) - pulse_start < DRV88xx_MIN_PULSE_WIDTH_US); // wait for minimum pulse width
  HAL_GPIO_WritePin(config->step_port, config->step_pin, GPIO_PIN_RESET);

  config->last_step_time = get_tim_us(config);
  return true;
}

void DRV88xx_RunToPosition(drv88xx_config_t *config)
{
  while (DRV88xx_Run(config));
}

bool DRV88xx_RunSpeedToPosition(drv88xx_config_t *config)
{
    if (config->target_pos == config->current_pos)
        return false; // Already at position
    if (config->target_pos > config->current_pos)
      config->direction = DIRECTION_CW;
    else
        config->direction = DIRECTION_CCW;
    return DRV88xx_RunSpeed(config);
}

void DRV88xx_RunToNewPosition(drv88xx_config_t *config, int32_t position)
{
    DRV88xx_MoveTo(config, position);
    DRV88xx_RunToPosition(config);
}

void DRV88xx_Stop(drv88xx_config_t *config)
{
    if (config->speed != 0.0f)
    {
        int32_t steps_to_stop = (int32_t)((config->speed * config->speed) / (2.0f * config->acceleration)) + 1;
        if (config->speed > 0)
            DRV88xx_Move(config, steps_to_stop);
        else
            DRV88xx_Move(config, -steps_to_stop);
    }
}

void DRV88xx_DisableOutputs(drv88xx_config_t *config)
{
    if (config->en_port != NULL)
        HAL_GPIO_WritePin(config->en_port, config->en_pin, config->en_inverted ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void DRV88xx_EnableOutputs(drv88xx_config_t *config)
{
    if (config->en_port != NULL)
        HAL_GPIO_WritePin(config->en_port, config->en_pin, config->en_inverted ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
#include "drv8825.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>


// Internal helper functions

static uint32_t distTo(drv8825_config_t *config)
{
    return config->target_pos - config->current_pos;
}

static float computeNewSpeed(drv8825_config_t *config)
{
    uint32_t dist_to = distTo(config);
    uint32_t steps_to_stop = (uint32_t)((config->speed * config->speed) / (2.0f * config->acceleration));

    if (dist_to == 0 && steps_to_stop <= 1)
    {
        // We are at the target and it's time to stop
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

    if (config->n == 0) {
        // first step from stopped
        config->cn = config->c0;
        config->direction = (dist_to > 0) ? DIRECTION_CW : DIRECTION_CCW;
    } else {
        // subsequent step, accelerate or decelerate
        config->cn = config->cn - ((2.0f * config->cn) / ((4.0f * config->n) + 1));
        config->cn = (config->cn < config->cmin) ? config->cmin : config->cn;
    }

    config->n++;
    config->step_interval = (uint32_t)config->cn;
    config->speed = 1000000.0f / config->cn;
    config->speed = (config->direction == DIRECTION_CW) ? config->speed : -config->speed;

    return config->step_interval;
}

// Public API functions

void DRV8825_Test(drv8825_config_t *config)
{
    // Step 100 steps in one direction
    DRV8825_Move(config, 100);
    while (DRV8825_Run(config)); // Wait until movement is complete

    // Delay for a bit
    HAL_Delay(1000);

    // Step 100 steps in the other direction
    DRV8825_Move(config, -200);
    while (DRV8825_Run(config)); // Wait until movement is complete{
    
}

void DRV8825_Init(drv8825_config_t *config,
                  GPIO_TypeDef *step_port, uint32_t step_pin,
                  GPIO_TypeDef *dir_port, uint32_t dir_pin,
                  GPIO_TypeDef *en_port, uint32_t en_pin,
                  GPIO_TypeDef *nfault_port,
                  uint32_t nfault_pin)
{

    config->step_port = step_port;
    config->step_pin = step_pin;
    config->dir_port = dir_port;
    config->dir_pin = dir_pin;
    config->en_port = en_port;
    config->en_pin = en_pin;
    config->nfault_port = nfault_port;
    config->nfault_pin = nfault_pin;

    // Set default values
    config->current_pos = 0;
    config->target_pos = 0;
    config->speed = 0.0f;
    config->max_speed = 0;
    config->acceleration = 0;
    config->last_step_time = 0;
    config->min_pulse_width = 1; // Minimum pulse width in microseconds
    config->dir_inverted = false;
    config->en_inverted = false;

    // Initialize GPIO pins
    HAL_GPIO_WritePin(config->en_port, config->en_pin, GPIO_PIN_SET); // Disable motor by default
    HAL_GPIO_WritePin(config->dir_port, config->dir_pin, GPIO_PIN_RESET); // Default direction
    HAL_GPIO_WritePin(config->step_port, config->step_pin, GPIO_PIN_RESET); // Ensure step pin is low
    if (config->nfault_port != NULL)
        HAL_GPIO_WritePin(config->nfault_port, config->nfault_pin, GPIO_PIN_RESET); // Clear fault pin if used

    DRV8825_SetAcceleration(config, 1.0f); // Set some reasonable default acceleration
    DRV8825_SetMaxSpeed(config, 1000.0f); // Set some reasonable default max speed
}

void DRV8825_SetCurrentPosition(drv8825_config_t *config, int32_t position)
{
    config->current_pos = position;
    config->target_pos = position;
    config->n = 0;
    config->step_interval = 0;
    config->speed = 0.0f;
}

void DRV8825_SetSpeed(drv8825_config_t *config, float speed)
{
    if (speed == config->speed)
        return;
    speed = (speed > config->max_speed) ? config->max_speed : ((speed < -config->max_speed) ? -config->max_speed : speed);
    if (speed == 0.0f)
        config->step_interval = 0;
    else
    {
        config->step_interval = (uint32_t)fabsf(1000000.0f / speed);
        config->direction = (speed > 0.0f) ? DIRECTION_CW : DIRECTION_CCW;
    }
    config->speed = speed;
}

void DRV8825_SetMaxSpeed(drv8825_config_t *config, float max_speed)
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
            config->n = (uint32_t)((config->speed * config->speed) / (2.0f * config->acceleration)); // Equation 16
            computeNewSpeed(config);
        }
    }

}

void DRV8825_SetAcceleration(drv8825_config_t *config, float acceleration)
{
    if (acceleration <= 0.0f)
        return; // Invalid acceleration, ignore
    else if (config->acceleration != acceleration)
    {
        config->n = config->n * (config->acceleration / acceleration);
        config->c0 = 0.676f * sqrtf(2.0f / acceleration) * 1000000.0f; // Equation 15
        config->acceleration = acceleration;
        computeNewSpeed(config);
    }
}

void DRV8825_MoveTo(drv8825_config_t *config, int32_t target_pos)
{
    if (target_pos != config->target_pos) {
        config->target_pos = target_pos;
        computeNewSpeed(config);
    }
}

void DRV8825_Move(drv8825_config_t *config, int32_t relative)
{
    DRV8825_MoveTo(config, config->current_pos + relative);
}

bool DRV8825_Run(drv8825_config_t *config)
{
    if (DRV8825_RunSpeed(config))
        computeNewSpeed(config);
    return (config->speed != 0.0f) || distTo(config) != 0;
}

bool DRV8825_RunSpeed(drv8825_config_t *config)
{
    if (config->step_interval == 0)
        return false; // No movement required

    uint32_t now = HAL_GetTick() * 1000; // Get current time in microseconds
    uint32_t time_since_last_step = now - config->last_step_time;
    if (time_since_last_step < config->step_interval)
        return false; // Not time for the next step yet

    // Time to step
    if (config->direction == DIRECTION_CCW)
        config->current_pos++;
    else
        config->current_pos--;

    // Pulse the step pin
    HAL_GPIO_WritePin(config->step_port, config->step_pin, GPIO_PIN_SET);
    HAL_Delay(config->min_pulse_width / 1000); // Delay for minimum pulse width
    HAL_GPIO_WritePin(config->step_port, config->step_pin, GPIO_PIN_RESET);

    config->last_step_time = now;
    return true;
}

void DRV8825_RunToPosition(drv8825_config_t *config)
{
    while (DRV8825_Run(config));
}

bool DRV8825_RunSpeedToPosition(drv8825_config_t *config)
{
    if (config->target_pos == config->current_pos)
        return false; // Already at position
    if (config->target_pos > config->current_pos)
        config->direction = DIRECTION_CCW;
    else
        config->direction = DIRECTION_CCW;
    return DRV8825_RunSpeed(config);
}

void DRV8825_RunToNewPosition(drv8825_config_t *config, int32_t position)
{
    DRV8825_MoveTo(config, position);
    DRV8825_RunToPosition(config);
}

void DRV8825_Stop(drv8825_config_t *config)
{
    if (config->speed != 0.0f)
    {
        uint32_t steps_to_stop = (uint32_t)((config->speed * config->speed) / (2.0f * config->acceleration)) + 1;
        if (config->speed > 0)
            DRV8825_Move(config, steps_to_stop);
        else
            DRV8825_Move(config, -steps_to_stop);
    }
}

void DRV8825_DisableOutputs(drv8825_config_t *config)
{
    if (config->en_port != NULL)
        HAL_GPIO_WritePin(config->en_port, config->en_pin, config->en_inverted ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void DRV8825_EnableOutputs(drv8825_config_t *config)
{
    if (config->en_port != NULL)
        HAL_GPIO_WritePin(config->en_port, config->en_pin, config->en_inverted ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
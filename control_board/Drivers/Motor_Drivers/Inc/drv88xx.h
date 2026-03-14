/**
 * @file drv88xx.h
 * @brief Header file for DRV88xx stepper motor driver. This file defines the
 * configuration struct and function prototypes for controlling stepper motors using
 * the DRV88xx family of drivers (eg DRV8825, DRV8834).
 */
#ifndef DRV88xx_H
#define DRV88xx_H

// -- INCLUDES ----------------------------------------------------------------
#include "stm32g491xx.h"
#include "stm32g4xx.h"
#include "motor_utils.h"

#include <stdbool.h>
#include <stdint.h>

// -- DEFINES -----------------------------------------------------------------

// Minimum pulse width in microseconds for the step pin.
#define DRV88xx_MIN_PULSE_WIDTH_US 2

#define DRV88xx_MAX_SPEED 1000.0f // Maximum speed in steps per second
#define DRV88xx_ACCELERATION 500.0f // Acceleration in steps per second^2

#define DRV88xx_DEFAULT_SPEED 100.0f // Default speed in steps per second if not set explicitly

// -- TYPE DEFINITIONS --------------------------------------------------------
typedef struct
{
    // Step pin. A pulse on this pin will cause the motor to step one increment.
    // The direction of the step is determined by the state of the direction pin.
    GPIO_TypeDef *step_port;
    uint32_t step_pin;

    // Direction pin. Controls the direction of the motor.
    // High for CW, Low for CCW (or vice versa depending on wiring)
    GPIO_TypeDef *dir_port;
    uint32_t dir_pin;
    bool dir_inverted;

    // Enable pin for stepper driver, or 0xFF if unused.
    GPIO_TypeDef *en_port; // GPIO port for enable pin, or NULL if not used
    uint32_t en_pin; // GPIO pin for enable, or 0xFF if not used
    bool en_inverted; // true if enable pin is active LOW, false if active HIGH

    GPIO_TypeDef *nfault_port; // GPIO port for fault pin, or NULL if not used
    uint32_t nfault_pin; // GPIO pin for fault, or 0xFF if not used

    TIM_HandleTypeDef *tim; // Timer used for step timing

    uint8_t MICROSTEPS;

    // absolute current position in steps
    int32_t current_pos; // Steps

    // target position in steps
    int32_t target_pos; // Steps

    // current speed in steps per second
    // #TODO: positive is CW?
    float speed; // Steps per second

    // maximum speed in steps per second
    // Must be > 0.
    float max_speed; // Steps per second

    // acceleration for motor. must be > 0.
    float acceleration; // Steps per second^2

    // The last step time in microseconds
    uint32_t last_step_time;


    uint32_t n;     // The step counter for speed calculations
    float c0;       // Initial step size in microseconds
    float cn;       // Last step size in microseconds
    float cmin;     // Min step size in microseconds based on maxSpeed

    bool direction; // Current direction of the motor
    uint32_t step_interval_us; // Current interval between steps in microseconds
} drv88xx_config_t;

// -- FUNCTION PROTOTYPES -----------------------------------------------------
void DRV88xx_Init(drv88xx_config_t *config, float max_speed, float acceleration);

/**
 * @brief Set the current position of the motor. This will set both the current
 * position and the target position to the specified value, and reset the speed
 * and step interval. This is useful for setting the current position to a known
 * value, such as after homing the motor.
 * @param config Pointer to the DRV88xx configuration struct
 * @param position The current position in steps. Negative is anticlockwise from
 * the 0 position.
 */
void DRV88xx_SetCurrentPosition(drv88xx_config_t *config, int32_t position);

/**
 * @brief Sets the desired speed of the motor to use with DRV88xx_RunSpeed()
 * and DRV88xx_RunSpeedToPosition().
 * @param config Pointer to the DRV88xx configuration struct
 * @param speed The desired speed in steps per second. Positive is clockwise.
 * Speeds of more than 1000 steps per second are unreliable. Very slow speeds
 * may be set (eg 0.00027777 for once per hour, approximately). Speed accuracy
 * depends on the crystal. Jitter depends on how frequently you call the
 * DRV88xx_RunSpeed() function. The speed will be limited by the current value
 * of max_speed.
 */
void DRV88xx_SetSpeed(drv88xx_config_t *config, float speed);

/**
 * @brief Sets the maximum permitted speed. The DRV88xx_Run() function will
 * accelerate up to the speed set by this function.
 * @param config Pointer to the DRV88xx configuration struct
 * @param max_speed The desired maximum speed in steps per second. Must be > 0
 * @warning Speeds that exceed the maximum speed supported by the processor may
 * Result in non-linear accelerations and decelerations.
 */
void DRV88xx_SetMaxSpeed(drv88xx_config_t *config, float max_speed);

/**
 * @brief Sets the acceleration/deceleration rate.
 * @param config Pointer to the DRV88xx configuration struct
 * @param acceleration The desired acceleration in steps per second per second.
 * Must be > 0.0. This is an expensive call since it requires a square root
 * to be calculated. Dont call more often than needed.
 */
void DRV88xx_SetAcceleration(drv88xx_config_t *config, float acceleration);

/**
 * @brief set the target position. The DRV88xx_Run() function will try to
 * move the motor from the current position to the target position, taking
 * into account the max speed and acceleration.
 * @attention DRV88xx_MoveTo() also recalculates the speed for the next step.
 * If you are trying to use constant speed movements, you should call
 * DRV88xx_SetSpeed() after calling DRV88xx_MoveTo().
 * @param config Pointer to the DRV88xx configuration struct
 * @param target_pos The desired absolute position in steps. Negative is
 * anticlockwise from the 0 position.
 */
void DRV88xx_MoveTo(drv88xx_config_t *config, int32_t target_pos);

/**
 * @brief Set the target position relative to the current position.
 * @param config Pointer to the DRV88xx configuration struct
 * @param relative The desired position relative to the current position.
 * Negative is anticlockwise from the current position.
 */
void DRV88xx_Move(drv88xx_config_t *config, int32_t relative);

/**
 * @brief Poll the motor and step it if a step is due, implementing
 * accelerations and decelerations to achieve the target position.
 * @attention You must call this as frequently as possible, but at least once
 * per minimum step time interval, preferably in your main loop. Note that each
 * call to run() will make at most one step, and then only when a step is due,
 * based on the current speed and the time since the last step.
 * @param config Pointer to the DRV88xx configuration struct
 * @return true if the motor is still running to the target position.
 */
bool DRV88xx_Run(drv88xx_config_t *config);

/**
 * @brief Poll the motor and step it if a step is due, implementing a constant
 * speed as set by the most recent call to DRV88xx_SetSpeed(). You must call
 * this as frequently as possible, but at least once per step interval.
 * @param config Pointer to the DRV88xx configuration struct
 * @return true if the motor was stepped.
 */
bool DRV88xx_RunSpeed(drv88xx_config_t *config);

/**
 * @brief Moves the motor (with acceleration/deceleration) to the target
 * position and blocks until it is at position. 
 * @warning Don't use this in event loops, since it blocks until the motor is at
 * position.
 */
void DRV88xx_RunToPosition(drv88xx_config_t *config);

/**
 * @brief Moves the motor (with acceleration/deceleration) to the new target
 * position and blocks until it is at position.
 * @warning Don't use this in event loops, since it blocks until the motor is at
 * position.
 * @param config Pointer to the DRV88xx configuration struct
 * @param position The new target position in steps. Negative is anticlockwise
 * from the 0 position.
 * @return true if it stepped
 */
bool DRV88xx_RunSpeedToPosition(drv88xx_config_t *config);

/**
 * @brief Moves the motor (with acceleration/deceleration) to the new target
 * position and blocks until it is at position.
 * @warning Don't use this in event loops, since it blocks until the motor is at
 * position.
 * @param config Pointer to the DRV88xx configuration struct
 * @param position The new target position in steps. Negative is anticlockwise
 * from the 0 position.
 */
void DRV88xx_RunToNewPosition(drv88xx_config_t *config, int32_t position);

/**
 * @brief Sets a new target position that causes the stepper to stop as quickly
 * as possible, using the current speed and acceleration parameters.
 * @param config Pointer to the DRV88xx configuration struct
 */
void DRV88xx_Stop(drv88xx_config_t *config);

/**
 * @brief Disable the stepper motor outputs. This will typically disable the
 * power to the motor coils, saving power. This is useful to support low power
 * modes: disable the outputs when you dont need to hold position or move the
 * motor, and enable the outputs when you need to hold position or move the motor.
 * @param config Pointer to the DRV88xx configuration struct
 */
void DRV88xx_DisableOutputs(drv88xx_config_t *config);
/**
 * @brief Enable the stepper motor outputs. This will typically enable the
 * power to the motor coils. You should call this before trying to hold position
 * or move the motor, and call DRV88xx_DisableOutputs() when you dont need to
 * hold position or move the motor to save power.
 * @param config Pointer to the DRV88xx configuration struct
 */
void DRV88xx_EnableOutputs(drv88xx_config_t *config);

/**
 * @brief Test function to verify that the driver is working. This will step the
 * motor 100 steps in one direction, then 100 steps in the other direction,
 * with a delay in between. You can use this to verify that the motor is
 * connected correctly and that the driver is working.
 * @param config Pointer to the DRV88xx configuration struct
 */
void DRV88xx_Test(drv88xx_config_t *config);

#endif /* DRV88xx_H */
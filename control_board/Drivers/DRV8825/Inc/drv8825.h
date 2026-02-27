#ifndef DRV8825_H
#define DRV8825_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx.h"

#define DRV8825_NUM_STEPPERS 2
#define STEPPER_MICROSTEP_ENABLED false

typedef enum
{
    DIRECTION_CCW = GPIO_PIN_RESET,
    DIRECTION_CW = GPIO_PIN_SET
} drv8825_direction_e;

#if STEPPER_MICROSTEP_ENABLED
typedef enum
{
    MS_FULL = 0,
    MS_HALF = 1,
    MS_1_4 = 2,
    MS_1_8 = 3,
    MS_1_16 = 4,
    MS_1_32 = 5 // DRV8825 supports up to 1/32 microstepping
} microstep_mode_e;
#endif

#if STEPPER_MICROSTEP_ENABLED
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

    // absolute current position in steps
    uint32_t current_pos; // Steps

    // target position in steps
    uint32_t target_pos; // Steps

    // current speed in steps per second
    float speed; // Steps per second

    // maximum speed in steps per second
    // Must be > 0.
    float max_speed; // Steps per second

    // acceleration for motor. must be > 0.
    float acceleration; // Steps per second^2
    float sqrt_twoa;    // Precomputed sqrt(2*acceleration)

    // The last step time in microseconds
    unsigned long last_step_time;

    // The minimum allowed pulse width in microseconds
    unsigned int min_pulse_width;

    // Enable pin for stepper driver, or 0xFF if unused.
    GPIO_TypeDef *en_port;
    uint32_t en_pin;
    bool en_inverted;

    // #TODO: Add microstep control pins (M0, M1, M2) if needed for microstepping configuration

    struct
    {
        uint32_t n;     // The step counter for speed calculations
        float c0;       // Initial step size in microseconds
        float cn;       // Last step size in microseconds
        float cmin;     // Min step size in microseconds based on maxSpeed
        bool direction; // Current direction of the motor
    } state;
} drv8825_config_t;
#else

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

    // The minimum allowed pulse width in microseconds
    uint32_t min_pulse_width;

    // Enable pin for stepper driver, or 0xFF if unused.
    GPIO_TypeDef *en_port; // GPIO port for enable pin, or NULL if not used
    uint32_t en_pin;
    bool en_inverted;

    // Fault pin for stepper driver, or 0xFF if unused. Active LOW.
    GPIO_TypeDef *nfault_port;
    uint32_t nfault_pin;

    uint32_t n;     // The step counter for speed calculations
    float c0;       // Initial step size in microseconds
    float cn;       // Last step size in microseconds
    float cmin;     // Min step size in microseconds based on maxSpeed

    bool direction; // Current direction of the motor
    uint32_t step_interval; // Current interval between steps in microseconds
} drv8825_config_t;

#endif

void DRV8825_Init(
    drv8825_config_t *config, 
    GPIO_TypeDef *step_port, 
    uint32_t step_pin, GPIO_TypeDef 
    *dir_port, uint32_t dir_pin, 
    GPIO_TypeDef *en_port, 
    uint32_t en_pin,
    GPIO_TypeDef *nfault_port,
    uint32_t nfault_pin
);

/**
 * @brief Set the current position of the motor. This will set both the current
 * position and the target position to the specified value, and reset the speed
 * and step interval. This is useful for setting the current position to a known
 * value, such as after homing the motor.
 * @param config Pointer to the DRV8825 configuration struct
 * @param position The current position in steps. Negative is anticlockwise from
 * the 0 position.
 */
void DRV8825_SetCurrentPosition(drv8825_config_t *config, int32_t position);

/**
 * @brief Sets the desired speed of the motor to use with DRV8825_RunSpeed()
 * and DRV8825_RunSpeedToPosition().
 * @param config Pointer to the DRV8825 configuration struct
 * @param speed The desired speed in steps per second. Positive is clockwise.
 * Speeds of more than 1000 steps per second are unreliable. Very slow speeds
 * may be set (eg 0.00027777 for once per hour, approximately). Speed accuracy
 * depends on the crystal. Jitter depends on how frequently you call the
 * DRV8825_RunSpeed() function. The speed will be limited by the current value
 * of max_speed.
 */
void DRV8825_SetSpeed(drv8825_config_t *config, float speed);

/**
 * @brief Sets the maximum permitted speed. The DRV8825_Run() function will
 * accelerate up to the speed set by this function.
 * @param config Pointer to the DRV8825 configuration struct
 * @param max_speed The desired maximum speed in steps per second. Must be > 0
 * @warning Speeds that exceed the maximum speed supported by the processor may
 * Result in non-linear accelerations and decelerations.
 */
void DRV8825_SetMaxSpeed(drv8825_config_t *config, float max_speed);

/**
 * @brief Sets the acceleration/deceleration rate.
 * @param config Pointer to the DRV8825 configuration struct
 * @param acceleration The desired acceleration in steps per second per second.
 * Must be > 0.0. This is an expensive call since it requires a square root
 * to be calculated. Dont call more often than needed.
 */
void DRV8825_SetAcceleration(drv8825_config_t *config, float acceleration);

/**
 * @brief set the target position. The DRV8825_Run() function will try to
 * move the motor from the current position to the target position, taking
 * into account the max speed and acceleration.
 * @attention DRV8825_MoveTo() also recalculates the speed for the next step.
 * If you are trying to use constant speed movements, you should call
 * DRV8825_SetSpeed() after calling DRV8825_MoveTo().
 * @param config Pointer to the DRV8825 configuration struct
 * @param target_pos The desired absolute position in steps. Negative is
 * anticlockwise from the 0 position.
 */
void DRV8825_MoveTo(drv8825_config_t *config, int32_t target_pos);

/**
 * @brief Set the target position relative to the current position.
 * @param config Pointer to the DRV8825 configuration struct
 * @param relative The desired position relative to the current position.
 * Negative is anticlockwise from the current position.
 */
void DRV8825_Move(drv8825_config_t *config, int32_t relative);

/**
 * @brief Poll the motor and step it if a step is due, implementing
 * accelerations and decelerations to achieve the target position.
 * @attention You must call this as frequently as possible, but at least once
 * per minimum step time interval, preferably in your main loop. Note that each
 * call to run() will make at most one step, and then only when a step is due,
 * based on the current speed and the time since the last step.
 * @param config Pointer to the DRV8825 configuration struct
 * @return true if the motor is still running to the target position.
 */
bool DRV8825_Run(drv8825_config_t *config);

/**
 * @brief Poll the motor and step it if a step is due, implementing a constant
 * speed as set by the most recent call to DRV8825_SetSpeed(). You must call
 * this as frequently as possible, but at least once per step interval.
 * @param config Pointer to the DRV8825 configuration struct
 * @return true if the motor was stepped.
 */
bool DRV8825_RunSpeed(drv8825_config_t *config);

/**
 * @brief Moves the motor (with acceleration/deceleration) to the target
 * position and blocks until it is at position. 
 * @warning Don't use this in event loops, since it blocks until the motor is at
 * position.
 */
void DRV8825_RunToPosition(drv8825_config_t *config);

/**
 * @brief Moves the motor (with acceleration/deceleration) to the new target
 * position and blocks until it is at position.
 * @warning Don't use this in event loops, since it blocks until the motor is at
 * position.
 * @param config Pointer to the DRV8825 configuration struct
 * @param position The new target position in steps. Negative is anticlockwise
 * from the 0 position.
 * @return true if it stepped
 */
bool DRV8825_RunSpeedToPosition(drv8825_config_t *config);

/**
 * @brief Moves the motor (with acceleration/deceleration) to the new target
 * position and blocks until it is at position.
 * @warning Don't use this in event loops, since it blocks until the motor is at
 * position.
 * @param config Pointer to the DRV8825 configuration struct
 * @param position The new target position in steps. Negative is anticlockwise
 * from the 0 position.
 */
void DRV8825_RunToNewPosition(drv8825_config_t *config, int32_t position);

/**
 * @brief Sets a new target position that causes the stepper to stop as quickly
 * as possible, using the current speed and acceleration parameters.
 * @param config Pointer to the DRV8825 configuration struct
 */
void DRV8825_Stop(drv8825_config_t *config);

/**
 * @brief Disable the stepper motor outputs. This will typically disable the
 * power to the motor coils, saving power. This is useful to support low power
 * modes: disable the outputs when you dont need to hold position or move the
 * motor, and enable the outputs when you need to hold position or move the motor.
 * @param config Pointer to the DRV8825 configuration struct
 */
void DRV8825_DisableOutputs(drv8825_config_t *config);
/**
 * @brief Enable the stepper motor outputs. This will typically enable the
 * power to the motor coils. You should call this before trying to hold position
 * or move the motor, and call DRV8825_DisableOutputs() when you dont need to
 * hold position or move the motor to save power.
 * @param config Pointer to the DRV8825 configuration struct
 */
void DRV8825_EnableOutputs(drv8825_config_t *config);

/**
 * @brief Test function to verify that the driver is working. This will step the
 * motor 100 steps in one direction, then 100 steps in the other direction,
 * with a delay in between. You can use this to verify that the motor is
 * connected correctly and that the driver is working.
 * @param config Pointer to the DRV8825 configuration struct
 */
void DRV8825_Test(drv8825_config_t *config);

#endif /* DRV8825_H */
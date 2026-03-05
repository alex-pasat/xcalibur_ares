/**
 * @file drv8251.h
 * @brief DRV8251 motor driver header. Implements control of a single DC motor
 * using the DRV8251DDAR driver chip. There is no feedback control in this
 * driver, just open-loop PWM control of speed and direction.
 */

#ifndef DRV8251_H
#define DRV8251_H

// -- INCLUDES ----------------------------------------------------------------
#include "stm32g4xx.h"

#include <stdbool.h>
#include <stdint.h>

// -- DEFINES -----------------------------------------------------------------
#define DRV8251_MAX_PWM 200000 // Max Frequency 200kHz

#define DRV8251_MIN_DUTY_CYCLE 0.001f // Minimum duty cycle to overcome static friction

// -- TYPE DEFINITIONS --------------------------------------------------------
typedef enum {
    DRV8251_STATE_COAST = 0,   // IN1=0, IN2=0 — high-Z / sleep
    DRV8251_STATE_FORWARD,     // IN1=PWM, IN2=1
    DRV8251_STATE_REVERSE,     // IN1=1, IN2=PWM
    DRV8251_STATE_BRAKE,       // IN1=1, IN2=1
    DRV8251_STATE_FAULT,       // nFAULT asserted by chip
} drv8251_state_t;

typedef struct {
    // -- IN1 pin --
    GPIO_TypeDef *in1_port;
    uint32_t in1_pin;
    TIM_HandleTypeDef *in1_tim;
    uint32_t in1_tim_channel;

    // -- IN2 pin --
    GPIO_TypeDef *in2_port;
    uint32_t in2_pin;
    TIM_HandleTypeDef *in2_tim;
    uint32_t in2_tim_channel;

    drv8251_state_t state; // current state of the motor

    // -- Speed: -1.0 (full reverse) to 1.0 (full forward) --
    float speed; // desired speed [-1.0, 1.0]

    // -- PWM config --
    bool dir_inverted;
    uint32_t tim_autoreload;

} drv8251_config_t;

// -- FUNCTION PROTOTYPES -----------------------------------------------------

/**
 * @brief Initialise the driver. Call after CubeMX-generated MX_TIMx_Init().
 * Starts PWM on both channels and puts the motor in coast/sleep mode.
 */
void DRV8251_Init(drv8251_config_t *config);

/**
 * @brief Set motor speed and direction.
 * @param speed  -1.0 = full reverse, 0.0 = coast, +1.0 = full forward.
 *               Values are clamped to [-1.0, 1.0].
 */
void DRV8251_SetSpeed(drv8251_config_t *config, float speed);

/**
 * @brief Actively brake the motor. Both outputs driven HIGH.
 * Motor will resist movement. Call DRV8251_SetSpeed() to resume.
 */
void DRV8251_Brake(drv8251_config_t *config);

/**
 * @brief Coast the motor. Both outputs LOW. Motor freewheels.
 * Device will enter sleep mode after tSLEEP (~1ms).
 */
void DRV8251_Coast(drv8251_config_t *config);

#endif /* DRV8251_H */
#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "drv8251.h"
#include "drv88xx.h"
#include "encoder.h"
#include "current_sense.h"
#include "qpid.h"
#include "stm32g491xx.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  KNIFETYPE_CHEF,
  KNIFETYPE_PARING,
  KNIFETYPE_GYOTO,
  KNIFETYPE_JAP_UTIL,
  N_KNIFE_TYPES,
} knife_type_t;

typedef struct {
  float target_bevel_angle_deg;
  // etc..
} sharpening_parameters_t;

typedef struct {
  GPIO_TypeDef *port;
  uint32_t pin;
  uint8_t threshold;
  bool last_state;
  bool state;
  uint8_t debounce_count;
} gpio_sensor_t;

typedef struct {
  drv88xx_config_t *config;
  gpio_sensor_t *limit_sw;
  bool limit_triggered;
  uint8_t MICROSTEPS;
} stepper_ctrl_t;

typedef struct {
  drv8251_config_t *drv;
  enc_config_t *enc;
  qPID_controller_t pid;

  gpio_sensor_t *limit_sw;

  gpio_sensor_t *hall_effect;

  GPIO_TypeDef *adc_port; // Optional ADC port for current sensing
  uint32_t adc_pin;   // Optional ADC pin
  current_sense_config_t curr_config; // Configuration for current sensing

  bool enabled;
  float target_rps; // Desired speed (rev/s)
  float dt;         // Control loop period (seconds)

  // cached values
  float current_rps; // Current speed (rev/s)
  uint32_t current_ma; // Current in milliamps
  float current_angle_deg; // Current angle (degrees)
  bool limit_triggered; // Whether the limit switch is currently triggered
  bool hall_triggered; // Whether the hall effect sensor is currently triggered
} motor_ctrl_t;

// -- Knife Parameters --------------------------------------------------------

void Ctrl_SetKnifeType(knife_type_t type);

// -- Stepper Control API -----------------------------------------------------

/**
 * @brief Initialize the stepper motor controller
 * @param ctrl Pointer to the stepper control structure
 * @param drv Pointer to the driver configuration
 */
void StepperCtrl_Init(stepper_ctrl_t *ctrl, drv88xx_config_t *drv);

/**
 * @brief Move the stepper motor to a specific position
 * @param ctrl Pointer to the stepper control structure
 * @param steps Number of steps to move (signed integer)
 */
void StepperCtrl_SetTarget(stepper_ctrl_t *ctrl, int32_t steps);

/**
 * @brief Stop the stepper motor
 * @param ctrl Pointer to the stepper control structure
 */
void StepperCtrl_Stop(stepper_ctrl_t *ctrl);

/**
 * @brief Sets position as home (0 steps) and clears state
 * @param ctrl Pointer to the stepper control structure
 */
void StepperCtrl_SetHome(stepper_ctrl_t *ctrl);

/**
 * @brief Run the stepper motor at a specific speed
 * @param ctrl Pointer to the stepper control structure
 * @param speed Speed in steps per second (positive for one direction, negative for the other)
 */
bool StepperCtrl_Run(stepper_ctrl_t *ctrl);

// -- DC Motor Control API ----------------------------------------------------

void MotorCtrl_Init(motor_ctrl_t *ctrl, drv8251_config_t *drv,
                    enc_config_t *enc, qPID_controller_t *pid,
                    qPID_Gains_t pid_gains, float dt);

void MotorCtrl_SetTarget(motor_ctrl_t *ctrl, float target_rps);

void MotorCtrl_Enable(motor_ctrl_t *ctrl);

void MotorCtrl_Disable(motor_ctrl_t *ctrl);

void MotorCtrl_Update(motor_ctrl_t *ctrl);

#endif // ROBOT_CONTROL_H
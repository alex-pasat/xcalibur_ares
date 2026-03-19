#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "drv8251.h"
#include "drv88xx.h"
#include "encoder.h"
#include "current_sense.h"
#include "qpid.h"
#include "stm32g491xx.h"
#include "stm32g4xx_hal_tim.h"

#include <stdint.h>
#include <stdbool.h>

#define KICKSTART_DURATION_MS 10
#define BRAKE_DURATION_MS 100

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

  uint16_t GEAR_RATIO;

  float target_rps; // Desired speed (rev/s)
  float last_target_rps; // Last target speed (rev/s)
  float dt_s;         // Control loop period (seconds)

  // cached values
  float current_rps; // Current speed (rev/s)
  uint32_t current_ma; // Current in milliamps
  float current_angle_deg; // Current angle (degrees)
  bool limit_triggered; // Whether the limit switch is currently triggered
  bool hall_triggered; // Whether the hall effect sensor is currently triggered

  // QoL var
  bool braking;
  uint32_t cuur_brake_ms;
  uint32_t curr_kickstart_ms;
} motor_ctrl_t;

typedef struct {
  GPIO_TypeDef *port;
  uint32_t pin;

  TIM_HandleTypeDef *tim;
  uint32_t tim_channel;

  float duty_cycle;
  float duty_step;
  bool increasing;
} led_pulse_ctrl_t;

typedef struct {
  GPIO_TypeDef *port;
  uint32_t pin;

  TIM_HandleTypeDef *tim;
  uint32_t tim_channel;
} fan_ctrl_t;

typedef struct {
  GPIO_TypeDef *port;
  uint32_t pin;

  TIM_HandleTypeDef *tim;
  uint32_t tim_channel;

  current_sense_config_t current;
  uint32_t MAX_CURRENT_mA;
} pump_ctrl_t;

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

void MotorCtrl_Stop(motor_ctrl_t *ctrl);

void MotorCtrl_Disable(motor_ctrl_t *ctrl);

void MotorCtrl_Update(motor_ctrl_t *ctrl);

// -- Other Control API -------------------------------------------------------

void LED_SetDuty(led_pulse_ctrl_t *ctrl, float duty_cycle);

/**
 * @brief Update the LED pulse control with a new duty cycle
 * @param ctrl Pointer to the LED pulse control structure
 */
void LED_PulseUpdate(led_pulse_ctrl_t *ctrl);

void Fan_SetDuty(fan_ctrl_t *ctrl, float duty_cycle);

// TODO: maybe add flow rate control
void Pump_SetDuty(pump_ctrl_t *ctrl, float duty_cycle);

#endif // ROBOT_CONTROL_H
#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "drv8251.h"
#include "drv88xx.h"
#include "encoder.h"
#include "qpid.h"
#include "stm32g491xx.h"
#include "stm32g4xx.h"


typedef struct {
  drv88xx_config_t *config;

  GPIO_TypeDef *limit_port; // Optional limit switch port
  uint32_t limit_pin;       // Optional limit switch pin
  // TODO: implement limit switch handling in the state machine

} stepper_ctrl_t;

typedef struct {
  drv8251_config_t *drv; // PWM driver
  enc_config_t *enc;     // Encoder
  qPID_controller_t pid; // PID controller

  GPIO_TypeDef *limit_port; // Optional limit switch port
  uint32_t limit_pin;       // Optional limit switch pin
  bool limit_active_high;
  bool limit_triggered;

  GPIO_TypeDef *hall_port; // Optional hall effect sensor port
  uint32_t hall_pin;       // Optional hall effect sensor pin
  // TODO: We want to hall effect to act as a zero position reference
  // so we can home the motor to a known position on startup.
  // When the hall sensor is triggered, we will reset the encoder count to 0

  GPIO_TypeDef *adc_port; // Optional ADC port for current sensing
  uint32_t adc_pin;   // Optional ADC pin
  uint8_t shunt_resistor_mohm; // Shunt resistor value in milliohms for current calculation

  float target_rps; // Desired speed (rev/s)
  float dt;         // Control loop period (seconds)
  bool enabled;
  
} motor_ctrl_t;

void MotorCtrl_Init(motor_ctrl_t *ctrl, drv8251_config_t *drv,
                    enc_config_t *enc, qPID_controller_t *pid,
                    qPID_Gains_t pid_gains, float dt);

void MotorCtrl_SetTarget(motor_ctrl_t *ctrl, float target_rps);

void MotorCtrl_Enable(motor_ctrl_t *ctrl);

void MotorCtrl_Disable(motor_ctrl_t *ctrl);

void MotorCtrl_Update(motor_ctrl_t *ctrl);

#endif // ROBOT_CONTROL_H
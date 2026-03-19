#include "robot_control.h"
#include "drv8251.h"
#include "drv88xx.h"
#include "encoder.h"
#include "current_sense.h"

#include <stdint.h>

// -- Helper Functions --------------------------------------------------------


static bool debounce_sensor(gpio_sensor_t *sensor) {
  if (sensor->port == NULL) return false; // No sensor connected

  // Read the raw state of the sensor
  bool raw = HAL_GPIO_ReadPin(sensor->port, sensor->pin) == GPIO_PIN_SET;
  // Check if the state has changed since the last reading
  if (raw != sensor->last_state) {
    // State has changed, reset debounce counter
    sensor->debounce_count = 0;
    sensor->last_state = raw;
  } else {
    // State is the same, increment debounce counter
    if (sensor->debounce_count < sensor->threshold) {
      sensor->debounce_count++;
    } else {
      // Debounce threshold reached, return stable state
      sensor->state = raw;
    }
  }
  // Debouncing, return previous stable state
  return sensor->state;
}

// -- Stepper Control ---------------------------------------------------------

#define DRV88xx_MAX_SPD 200.0f
#define DRV88xx_ACCEL 50.0f

void StepperCtrl_Init(stepper_ctrl_t *ctrl, drv88xx_config_t *drv) {
  DRV88xx_Init(
    drv,
    DRV88xx_MAX_SPD * ctrl->MICROSTEPS,
    DRV88xx_ACCEL * ctrl->MICROSTEPS
  );
  ctrl->config = drv;
}

void StepperCtrl_SetTarget(stepper_ctrl_t *ctrl, int32_t target_pos) {
  DRV88xx_MoveTo(ctrl->config, target_pos * ctrl->MICROSTEPS);
}

void StepperCtrl_Stop(stepper_ctrl_t *ctrl) {
  DRV88xx_Stop(ctrl->config);
}

void StepperCtrl_SetHome(stepper_ctrl_t *ctrl) {
  DRV88xx_SetCurrentPosition(ctrl->config, 0);
}

bool StepperCtrl_Run(stepper_ctrl_t *ctrl) {
  ctrl->limit_triggered = debounce_sensor(ctrl->limit_sw);

  // TODO: uncomment this when we have limit switches wired up
  // if (ctrl->limit_triggered) {
  //   DRV88xx_Stop(ctrl->config);
  //   return true;
  // }

  return DRV88xx_Run(ctrl->config);
}

// -- Motor Control -----------------------------------------------------------

void MotorCtrl_Init(motor_ctrl_t *ctrl, drv8251_config_t *drv,
                    enc_config_t *enc, qPID_controller_t *pid,
                    qPID_Gains_t pid_gains, float dt) {
  ctrl->drv = drv;
  ctrl->enc = enc;
  ctrl->target_rps = 0.0f;
  ctrl->last_target_rps = 0.0f;
  ctrl->dt = dt;

  qPID_Setup(&ctrl->pid, pid_gains.Kc, pid_gains.Ki, pid_gains.Kd, dt);

  qPID_SetSaturation(&ctrl->pid, ctrl->drv->MIN_DUTY_CYCLE, 1.0f);

  DRV8251_Init(drv); // Initialize the motor driver
  if (enc != NULL) Encoder_Init(enc); // Initialize the encoder
}

void MotorCtrl_SetTarget(motor_ctrl_t *ctrl, float target_rps) {
  ctrl->target_rps = target_rps;
  if (ctrl->target_rps == 0.0f) {
    qPID_Reset(&ctrl->pid);
    DRV8251_Coast(ctrl->drv);
  }
}

void MotorCtrl_Stop(motor_ctrl_t *ctrl) {
  DRV8251_Coast(ctrl->drv);
  qPID_Reset(&ctrl->pid);
}

void MotorCtrl_Disable(motor_ctrl_t *ctrl) {
  DRV8251_Coast(ctrl->drv);
  qPID_Reset(&ctrl->pid);
}

void MotorCtrl_Update(motor_ctrl_t *ctrl) {
  ctrl->current_rps = Encoder_ComputeVelocity(ctrl->enc, ctrl->dt);
  ctrl->current_angle_deg = Encoder_GetAngleDeg(ctrl->enc);

  if (ctrl->target_rps == 0.0f) {
    // If target is zero, just coast and reset PID
    qPID_Reset(&ctrl->pid);
    DRV8251_Coast(ctrl->drv);
    return;
  }

  // Update saturation and reset integrator on direction change
  bool going_fwd = ctrl->target_rps >= 0.0f;
  bool was_fwd   = ctrl->last_target_rps >= 0.0f;

  if (going_fwd != was_fwd) {
    qPID_Reset(&ctrl->pid);
    if (going_fwd)
      qPID_SetSaturation(&ctrl->pid,  ctrl->drv->MIN_DUTY_CYCLE, 1.0f);
    else
      qPID_SetSaturation(&ctrl->pid, -1.0f, -ctrl->drv->MIN_DUTY_CYCLE);
  }

  float duty = qPID_Control(&ctrl->pid, ctrl->target_rps, ctrl->current_rps);
  DRV8251_SetDuty(ctrl->drv, duty);

  ctrl->current_ma      = CurrentSense_GetCurrentmA(&ctrl->curr_config);
  ctrl->limit_triggered = debounce_sensor(ctrl->limit_sw);
  ctrl->hall_triggered  = debounce_sensor(ctrl->hall_effect);
}
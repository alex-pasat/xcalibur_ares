#include "robot_control.h"
#include "drv8251.h"
#include "drv88xx.h"
#include "encoder.h"
#include "current_sense.h"

#include <stdint.h>

// TODO: add target bevel angle received from SPI (type of knife)

const sharpening_parameters_t sharpening_params[N_KNIFE_TYPES] = {
  [KNIFETYPE_CHEF] = {.target_bevel_angle_deg = 20.0f},
  [KNIFETYPE_PARING] = {.target_bevel_angle_deg = 15.0f},
  [KNIFETYPE_GYOTO] = {.target_bevel_angle_deg = 18.0f},
  [KNIFETYPE_JAP_UTIL] = {.target_bevel_angle_deg = 10.0f},
  // TODO: set thse to actual sharpening params
};

static knife_type_t current_knife_type = KNIFETYPE_CHEF;

// -- Knife Parameters --------------------------------------------------------

void Ctrl_SetKnifeType(knife_type_t type) {
  current_knife_type = type;
}

// -- Stepper Control ---------------------------------------------------------

// TODO: set these to reasonable values.. we probably won't need to be going 
// very fast since we want precise control over the movement
#define DRV88xx_MAX_SPD 1000.0f
#define DRV88xx_ACCEL 50.0f

void StepperCtrl_Init(stepper_ctrl_t *ctrl, drv88xx_config_t *drv) {
  DRV88xx_Init(drv);

  DRV88xx_SetMaxSpeed(drv, DRV88xx_MAX_SPD);
  DRV88xx_SetAcceleration(drv, DRV88xx_ACCEL);

  ctrl->config = drv;
}

void StepperCtrl_SetTarget(stepper_ctrl_t *ctrl, int32_t target_pos) {
  ctrl->config->target_pos = target_pos;
}

void StepperCtrl_SetHome(stepper_ctrl_t *ctrl) {
  DRV88xx_SetCurrentPosition(ctrl->config, 0);
}

bool StepperCtrl_Run(stepper_ctrl_t *ctrl) {
  if (ctrl->limit_sw->port != NULL) {
    // TODO: debounce
    if(0) {
      DRV88xx_Stop(ctrl->config);
      return false;
    }
  }

  return DRV88xx_Run(ctrl->config);
}

// -- Motor Control -----------------------------------------------------------

void MotorCtrl_Init(motor_ctrl_t *ctrl, drv8251_config_t *drv,
                    enc_config_t *enc, qPID_controller_t *pid,
                    qPID_Gains_t pid_gains, float dt) {
  ctrl->drv = drv;
  ctrl->enc = enc;
  ctrl->target_rps = 0.0f;
  ctrl->dt = dt;
  ctrl->enabled = false;

  qPID_Setup(&ctrl->pid, pid_gains.Kc, pid_gains.Ki, pid_gains.Kd, dt);
  DRV8251_Init(drv); // Initialize the motor driver
  if (enc != NULL) Encoder_Init(enc); // Initialize the encoder
}

void MotorCtrl_SetTarget(motor_ctrl_t *ctrl, float target_rps) {
  ctrl->target_rps = target_rps;
}

void MotorCtrl_Enable(motor_ctrl_t *ctrl) { ctrl->enabled = true; }

void MotorCtrl_Disable(motor_ctrl_t *ctrl) {
  ctrl->enabled = false;
  DRV8251_Coast(ctrl->drv);
  qPID_Reset(&ctrl->pid);
}

void MotorCtrl_SetKnifeType(knife_type_t type) {
  // TODO: Implement knife type setting logic
}

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

void MotorCtrl_Update(motor_ctrl_t *ctrl) {
  if (!ctrl->enabled) return;

  
  // Read current speed from encoder
  float current_rps = Encoder_ComputeVelocity(ctrl->enc, ctrl->dt);
  
  // Compute control action using PID controller
  float control_signal = qPID_Control(
    &ctrl->pid, ctrl->target_rps, current_rps);
  
  // Set motor speed based on control signal
  DRV8251_SetSpeed(ctrl->drv, control_signal);

  ctrl->current_ma = CurrentSense_GetCurrentmA(&ctrl->curr_config);

  ctrl->current_angle_deg = Encoder_GetAngleDeg(ctrl->enc);

  ctrl->limit_triggered = debounce_sensor(ctrl->limit_sw);
  ctrl->hall_triggered = debounce_sensor(ctrl->hall_effect);
}
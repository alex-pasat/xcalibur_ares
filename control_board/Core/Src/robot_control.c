#include "robot_control.h"
#include "drv8251.h"
#include "encoder.h"
#include "current_sense.h"

#include <stdint.h>

// TODO: add target bevel angle received from SPI (type of knife)

const sharpening_parameters_t sharpening_params[N_KNIFE_TYPES] = {
  // TODO: set these to the actual target bevel angles for each knife type
    [KNIFETYPE_GERMAN] = {.target_bevel_angle_deg = 20.0f},
    [KNIFETYPE_JAPANESE] = {.target_bevel_angle_deg = 15.0f},
};


void MotorCtrl_Init(motor_ctrl_t *ctrl, drv8251_config_t *drv,
                    enc_config_t *enc, qPID_controller_t *pid,
                    qPID_Gains_t pid_gains, float dt) {
  ctrl->drv = drv;
  ctrl->enc = enc;
  ctrl->target_rps = 0.0f;
  ctrl->dt = dt;
  ctrl->enabled = false;

  qPID_Setup(&ctrl->pid, pid_gains.Kc, pid_gains.Ki, pid_gains.Kd, dt);
  DRV8251_Init(drv); // Initialize the motor driverFF
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

float MotorCtrl_ReadCurrentMA(motor_ctrl_t *ctrl) {
  if (ctrl->adc_port == NULL) return 0; // ADC not configured

  // Read current value from ADC
  return CurrentSense_GetCurrentmA(&ctrl->curr_config);
}

void MotorCtrl_SetKnifeType(knife_type_t type) {
  // TODO: Implement knife type setting logic
}

void MotorCtrl_Update(motor_ctrl_t *ctrl) {
  // check if encoder is used
  if (ctrl->enc == NULL)
    // lowk this should not happen
    return;

  // Only update if enabled
  if (!ctrl->enabled)
    return;

  // TODO: compute delta_ticks or call from fixed time interrupt to compute
  // velocity

  // Read current speed from encoder
  float current_rps = Encoder_ComputeVelocity(ctrl->enc, ctrl->dt);

  // Compute control action using PID controller
  float control_signal = qPID_Control(
    &ctrl->pid, ctrl->target_rps, current_rps);

  // Set motor speed based on control signal
  DRV8251_SetSpeed(ctrl->drv, control_signal);
}
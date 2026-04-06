#include "robot_control.h"
#include "drv8251.h"
#include "drv88xx.h"
#include "encoder.h"
#include "current_sense.h"
#include "qpid.h"

#include <stdint.h>

// -- Helper Functions --------------------------------------------------------

static bool debounce_sensor(gpio_sensor_t *sensor) {
  if (sensor->port == NULL) return false; // No sensor connected

  // Read the raw state of the sensor
  bool raw = HAL_GPIO_ReadPin(sensor->port, sensor->pin) == GPIO_PIN_SET;
  if (sensor->active_low) {
    raw = !raw; // Invert if active low
  }
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
  ctrl->limit_triggered = false;
  ctrl->backing_off = false;
}

void StepperCtrl_SetTarget(stepper_ctrl_t *ctrl, int32_t target_pos) {
  DRV88xx_MoveTo(ctrl->config, target_pos * ctrl->MICROSTEPS);
}

void StepperCtrl_SetTarget_m(stepper_ctrl_t *ctrl, float target_m) {
  int32_t target_steps = (int32_t)(target_m * ctrl->uSTEPS_PER_M);
  DRV88xx_MoveTo(ctrl->config, target_steps);
}

void StepperCtrl_Stop(stepper_ctrl_t *ctrl) {
  DRV88xx_Stop(ctrl->config);
}

#define HOMING_DIST 1000000 
void StepperCtrl_StartHoming(stepper_ctrl_t *ctrl) {
  ctrl->backing_off = false;
  int32_t target_pos = ctrl->homing_direction ? HOMING_DIST : -HOMING_DIST;
  DRV88xx_MoveTo(ctrl->config, target_pos * ctrl->MICROSTEPS);
}

void StepperCtrl_SetHome(stepper_ctrl_t *ctrl) {
  DRV88xx_SetCurrentPosition(ctrl->config, 0);
}

bool StepperCtrl_Run(stepper_ctrl_t *ctrl) {
  ctrl->limit_triggered = debounce_sensor(ctrl->limit_sw);

  if (ctrl->limit_triggered && !ctrl->backing_off) {
    DRV88xx_Stop(ctrl->config);
    // Start backoff in opposite direction
    int32_t backoff = ctrl->homing_direction
                    ? -ctrl->BACKOFF_STEPS
                    : ctrl->BACKOFF_STEPS;
    DRV88xx_MoveTo(ctrl->config,
      ctrl->config->current_pos + backoff * ctrl->MICROSTEPS);
    ctrl->backing_off = true;
  }

  if (ctrl->backing_off) {
    if (!DRV88xx_Run(ctrl->config) && !ctrl->limit_triggered) {
      ctrl->backing_off    = false;
      return false;
    } 
    return true;
  }

  return DRV88xx_Run(ctrl->config);
}

// -- Motor Control -----------------------------------------------------------

void MotorCtrl_Init(motor_ctrl_t *ctrl, drv8251_config_t *drv,
                    enc_config_t *enc, qPID_Gains_t pid_gains, float dt) {
  ctrl->drv = drv;
  ctrl->enc = enc;
  ctrl->target_rps = 0.0f;
  ctrl->last_target_rps = 0.0f;
  ctrl->dt_s = dt;
  
  ctrl->curr_brake_ms = 0;
  ctrl->braking = false;
  ctrl->curr_kickstart_ms = 0;

  if (ctrl->limit_sw != NULL) {
    ctrl->limit_sw->debounce_count = 0;
    ctrl->limit_sw->last_state = false;
  }

  if (ctrl->hall_effect != NULL) {
    ctrl->hall_effect->debounce_count = 0;
    ctrl->hall_effect->last_state = false;
  }

  qPID_Setup(&ctrl->pid, pid_gains.Kc, pid_gains.Ki, pid_gains.Kd, dt);

  qPID_SetSaturation(&ctrl->pid, -1.0f, 1.0f);

  // prevent integral windup
  qPID_SetExtraGains(&ctrl->pid, 0.1f, 0.0f); 

  DRV8251_Init(drv); // Initialize the motor driver
  if (enc != NULL) Encoder_Init(enc); // Initialize the encoder
}

void MotorCtrl_SetHome(motor_ctrl_t *ctrl) {
  if (ctrl->enc != NULL) {
    Encoder_Reset(ctrl->enc); 
  }
}

void MotorCtrl_SetTarget(motor_ctrl_t *ctrl, float target_rps) {
  ctrl->target_rps = target_rps;
  if (ctrl->target_rps == 0.0f) {
    qPID_Reset(&ctrl->pid);
    DRV8251_Coast(ctrl->drv);
  }
}

void MotorCtrl_StartHoming(motor_ctrl_t *ctrl) {
  ctrl->angle_limiting = false; // Disable angle limiting during homing
  MotorCtrl_SetTarget(ctrl, ctrl->homing_speed_rps);
}

void MotorCtrl_ReEnableLimits(motor_ctrl_t *ctrl) {
  ctrl->angle_limiting = true;
}

float MotorCtrl_GetCurrentAngleDeg(motor_ctrl_t *ctrl) {
  return ctrl->current_angle_deg;
}

void MotorCtrl_Stop(motor_ctrl_t *ctrl) {
  DRV8251_Brake(ctrl->drv);
  ctrl->target_rps = 0.0f;
  ctrl->braking = true;
  ctrl->curr_brake_ms = 0;
}

void MotorCtrl_Disable(motor_ctrl_t *ctrl) {
  DRV8251_Coast(ctrl->drv);
  ctrl->target_rps = 0.0f;
  ctrl->braking = false;
  qPID_Reset(&ctrl->pid);
}

void MotorCtrl_Update(motor_ctrl_t *ctrl) {
  // Update sensor readings
  ctrl->current_rps = Encoder_ComputeVelocityRPS(ctrl->enc, ctrl->dt_s);
  ctrl->current_angle_deg = Encoder_GetAngleDegContinuous(ctrl->enc);
  ctrl->current_ma      = CurrentSense_GetCurrentmA(&ctrl->curr_config);
  ctrl->limit_triggered = debounce_sensor(ctrl->limit_sw);
  ctrl->hall_triggered  = debounce_sensor(ctrl->hall_effect);

  // check angle limits
  if (ctrl->angle_limiting) {
    bool at_min_limit = ctrl->current_angle_deg <= ctrl->ANGLE_MIN_DEG;
    bool at_max_limit = ctrl->current_angle_deg >= ctrl->ANGLE_MAX_DEG;
    if ((at_min_limit && ctrl->target_rps < 0.0f) || 
        (at_max_limit && ctrl->target_rps > 0.0f)) {
      // TODO: set a flag
      MotorCtrl_Stop(ctrl);
      return;
    }
  }

  // Compute control output
  if (ctrl->target_rps == 0.0f) {
    qPID_Reset(&ctrl->pid);

    ctrl->curr_kickstart_ms = 0;
    ctrl->last_target_rps = 0.0f;

    if (ctrl->braking) {
      if (ctrl->curr_brake_ms < BRAKE_DURATION_MS) {
        DRV8251_Brake(ctrl->drv);
        ctrl->curr_brake_ms += (uint32_t)(ctrl->dt_s * 1000);
        return;
      } else {
        ctrl->braking = false;
      }
    }
    
    DRV8251_Coast(ctrl->drv);
    return;
  }

  ctrl->braking = false;

  bool direction_changed = (ctrl->last_target_rps * ctrl->target_rps < 0.0f);
  if (direction_changed || ctrl->last_target_rps == 0.0f) {
    ctrl->curr_kickstart_ms = 0;
  }

  if (ctrl->curr_kickstart_ms < KICKSTART_DURATION_MS) {
    float duty = (ctrl->target_rps > 0.0f) ? ctrl->drv->STALL_DUTY_CYCLE : -ctrl->drv->STALL_DUTY_CYCLE;
    DRV8251_SetDuty(ctrl->drv, duty);
    ctrl->curr_kickstart_ms += (uint32_t)(ctrl->dt_s * 1000);
    ctrl->last_target_rps = ctrl->target_rps;
    return;
  }

  ctrl->last_target_rps = ctrl->target_rps;

  float duty = qPID_Control(&ctrl->pid, ctrl->target_rps, ctrl->current_rps);
  DRV8251_SetDuty(ctrl->drv, duty);
}

// -- Other Control Implementations -------------------------------------------

void LED_SetDuty(led_pulse_ctrl_t *ctrl, float duty_cycle) {
  __HAL_TIM_SET_COMPARE(
    ctrl->tim, ctrl->tim_channel, 
    (uint32_t)(duty_cycle * __HAL_TIM_GET_AUTORELOAD(ctrl->tim))
  );
}

void LED_PulseUpdate(led_pulse_ctrl_t *ctrl) {
  if (ctrl->increasing) {
    ctrl->duty_cycle += ctrl->duty_step;
    if (ctrl->duty_cycle >= 1.0f) {
      ctrl->duty_cycle = 1.0f;
      ctrl->increasing = false;
    }
  } else {
    ctrl->duty_cycle -= ctrl->duty_step;
    if (ctrl->duty_cycle <= 0.0f) {
      ctrl->duty_cycle = 0.0f;
      ctrl->increasing = true;
    }
  }  

  // Apply a non-linear transformation to make the pulse more visually appealing
  float adjusted_duty = ctrl->duty_cycle * ctrl->duty_cycle;
  LED_SetDuty(ctrl, adjusted_duty);
}

void Fan_SetDuty(fan_ctrl_t *ctrl, float duty_cycle) {
  __HAL_TIM_SET_COMPARE(
    ctrl->tim, ctrl->tim_channel, 
    (uint32_t)(duty_cycle * __HAL_TIM_GET_AUTORELOAD(ctrl->tim))
  );
}

void Pump_SetDuty(pump_ctrl_t *ctrl, float duty_cycle) {
  uint32_t current_ma = CurrentSense_GetCurrentmA(&ctrl->current);

  if (current_ma >= ctrl->MAX_CURRENT_mA) {
    // Overcurrent, shut off pump
    __HAL_TIM_SET_COMPARE(ctrl->tim, ctrl->tim_channel, 0);
    return;
  }

  __HAL_TIM_SET_COMPARE(
    ctrl->tim, ctrl->tim_channel, 
    (uint32_t)(duty_cycle * __HAL_TIM_GET_AUTORELOAD(ctrl->tim))
  );
}

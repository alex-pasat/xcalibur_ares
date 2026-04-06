#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "motor_utils.h"
#include "robot_control.h"

// -- Motor Configuration -----------------------------------------------------

// TODO: set these to the actual current limits
#define CURRENT_THRESHOLD_KNIFECLAMP_MA 500
#define CURRENT_SENSE_THRESHOLD_PITCH_MA 300

#define TIMER_FREQ_HZ 170000000 // SYSCLK Frequency
#define TIMER_PSC 0
#define TIMER_PWM_FREQ_HZ 50000 // Desired PWM frequency for DRV8251

// time step for control loop updates
#define CONTROL_TIME_STEP_S 0.001f

#define SPI_BUF_SIZE 4

#define ADC_NUM_CHANNELS 3

#define DRV8834_MICROSTEPS 16

// -- Motor Handles ------------------------------------------------------------

#define STEPPER_LIST(X) \
  X(stepper_underpass)

#define DC_LIST(X) \
  X(dc_pitch) \
  X(dc_roll) \
  X(dc_yaw) \
  X(dc_clamp) \
  X(dc_extra)

#define X(name) extern stepper_ctrl_t name;
STEPPER_LIST(X)
#undef X

#define X(name) extern motor_ctrl_t name;
DC_LIST(X)
#undef X

#define AS_COUNT(name) +1
#define NUM_STEPPER_MOTORS (0 STEPPER_LIST(AS_COUNT))
#define NUM_DC_MOTORS (0 DC_LIST(AS_COUNT))

extern stepper_ctrl_t *stepper_motors[];
extern motor_ctrl_t *dc_motors[];

extern led_pulse_ctrl_t led_strip;
extern fan_ctrl_t fan;
extern pump_ctrl_t pump;

// -- Function Prototypes -----------------------------------------------------

void RobotConfig_Init(void);


#endif /* ROBOT_CONFIG_H */
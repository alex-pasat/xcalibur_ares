#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "motor_utils.h"
#include "robot_control.h"

// -- Motor Configuration -----------------------------------------------------

// TODO: set these to the actual current limits
#define CURRENT_THRESHOLD_KNIFECLAMP_MA 300
#define CURRENT_SENSE_THRESHOLD_PITCH_MA 300

#define DRV88xx_MAX_SPD 1000.0f
#define DRV88xx_ACCEL 50.0f

#define TIMER_FREQ_HZ 170000000 // SYSCLK Frequency
#define TIMER_PSC 0
#define TIMER_PWM_FREQ_HZ 50000 // Desired PWM frequency for DRV8251

// TODO: should this be hardcoded or computed based on the timer configuration?
#define CONTROL_TIME_STEP_S 0.01f // 10 ms control loop period

// Gear Ratios TODO: set these to the actual gear ratios
#define GEAR_RATIO_ROLL 50.0f

#define SPI_BUF_SIZE 4
#define USB_BUF_SIZE 1024

// size of word for ADC DMA buffer
#define ADC_BUFFER_SIZE 7

// -- Motor Handles ------------------------------------------------------------

// DRV88xx stepper motor configurations
extern stepper_ctrl_t stepper_spool;
extern stepper_ctrl_t stepper_raise1;
extern stepper_ctrl_t stepper_raise2;

// DRV8834 stepper motor configurations
extern stepper_ctrl_t stepper_underpass;
extern stepper_ctrl_t stepper_bevel;

// DRV8251DDAR DC motor configurations
extern motor_ctrl_t dc_pitch;
extern motor_ctrl_t dc_roll;
extern motor_ctrl_t dc_yaw;
extern motor_ctrl_t clamp;
extern motor_ctrl_t tension;
extern motor_ctrl_t sclamp1;
extern motor_ctrl_t sclamp2;

// TODO add fan and pump control structs

// -- Function Prototypes -----------------------------------------------------

void RobotConfig_Init(void);


#endif /* ROBOT_CONFIG_H */
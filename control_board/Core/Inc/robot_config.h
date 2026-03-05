#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx.h"
#include "motor_utils.h"
#include "drv88xx.h"
#include "robot_control.h"

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
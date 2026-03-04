#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx.h"
#include "motor_utils.h"
#include "drv88xx.h"

// -- Motor Handles ------------------------------------------------------------

// DRV88xx stepper motor configurations
extern drv88xx_config_t stepper_spool;
extern drv88xx_config_t stepper_raise1;
extern drv88xx_config_t stepper_raise2;

// DRV8834 stepper motor configurations
extern drv88xx_config_t stepper_underpass;
extern drv88xx_config_t stepper_bevel;
// DRV8251DDAR DC motor configurations
// TODO : implement DRV8251DDAR driver and add config structs here



#endif /* ROBOT_CONFIG_H */
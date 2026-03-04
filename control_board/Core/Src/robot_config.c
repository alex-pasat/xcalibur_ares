/**
 * @file    robot_config.c
 * @brief   Robot configuration definitions. This file defines the motor config
 * structs for each motor, and any other global configuration variables for the
 * robot.
 */
#include "robot_config.h"

#define DRV88xx_MAX_SPD 1000.0f
#define DRV88xx_ACCEL 50.0f

drv88xx_config_t stepper_spool = {
    .step_port      = GPIOE, .step_pin = GPIO_PIN_0,
    .dir_port       = GPIOE, .dir_pin  = GPIO_PIN_1,
    .dir_inverted   = false, // TODO: check wiring and set this correctly
    .max_speed      = DRV88xx_MAX_SPD,
    .acceleration   = DRV88xx_ACCEL,
    .en_port        = NULL, .en_pin = 0xFF, // not used
    .en_inverted    = false, // not used
    .nfault_port    = NULL, .nfault_pin = 0xFF, // not used
};

drv88xx_config_t stepper_raise1 = {
    .step_port      = GPIOE, .step_pin = GPIO_PIN_5, 
    .dir_port       = GPIOE, .dir_pin  = GPIO_PIN_6,
    .dir_inverted   = false, // TODO: check wiring and set this correctly
    .max_speed      = DRV88xx_MAX_SPD,
    .acceleration   = DRV88xx_ACCEL,
    .en_port        = NULL, .en_pin = 0xFF, // not used
    .en_inverted    = false, // not used
    .nfault_port    = GPIOC, .nfault_pin = GPIO_PIN_13,
};

drv88xx_config_t stepper_raise2 = {
    .step_port      = GPIOA, .step_pin = GPIO_PIN_15, 
    .dir_port       = GPIOC, .dir_pin  = GPIO_PIN_10,
    .dir_inverted   = false, // TODO: check wiring and set this correctly
    .max_speed      = DRV88xx_MAX_SPD,
    .acceleration   = DRV88xx_ACCEL,
    .en_port        = NULL, .en_pin = 0xFF, // not used
    .en_inverted    = false, // not used
    .nfault_port    = GPIOC, .nfault_pin = GPIO_PIN_11,
};

drv88xx_config_t stepper_underpass = {
    .step_port      = GPIOD, .step_pin = GPIO_PIN_2, 
    .dir_port       = GPIOD, .dir_pin  = GPIO_PIN_1,
    .dir_inverted   = false, // TODO: check wiring and set this correctly
    .max_speed      = DRV88xx_MAX_SPD,
    .acceleration   = DRV88xx_ACCEL,
    .en_port        = NULL, .en_pin = 0xFF, // not used
    .en_inverted    = false, // not used
    .nfault_port    = NULL, .nfault_pin = 0xFF, // not used
};

drv88xx_config_t stepper_bevel = {
    .step_port      = GPIOD, .step_pin = GPIO_PIN_0, 
    .dir_port       = GPIOC, .dir_pin  = GPIO_PIN_12,
    .dir_inverted   = false, // TODO: check wiring and set this correctly
    .max_speed      = DRV88xx_MAX_SPD,
    .acceleration   = DRV88xx_ACCEL,
    .en_port        = NULL, .en_pin = 0xFF, // not used
    .en_inverted    = false, // not used
    .nfault_port    = NULL, .nfault_pin = 0xFF, // not used
};
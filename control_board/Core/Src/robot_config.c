/**
 * @file    robot_config.c
 * @brief   Robot configuration definitions. This file defines the motor config
 * structs for each motor, and any other global configuration variables for the
 * robot.
 */
#include "robot_config.h"
#include "drv8251.h"
#include "encoder.h"
#include "qpid.h"
#include "robot_control.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal_tim.h"

#define DRV88xx_MAX_SPD 1000.0f
#define DRV88xx_ACCEL 50.0f

#define TIMER_FREQ_HZ 170000000 // SYSCLK Frequency
#define TIMER_PSC 0
#define TIMER_PWM_FREQ_HZ 50000 // Desired PWM frequency for DRV8251

// TODO: should this be hardcoded or computed based on the timer configuration?
#define CONTROL_TIME_STEP_S 0.001f // 1 ms control loop time step

// Stepper motor configurations

stepper_ctrl_t stepper_spool = {
    .config =
        &(drv88xx_config_t){
            .step_port = GPIOE,
            .step_pin = GPIO_PIN_0,
            .dir_port = GPIOE,
            .dir_pin = GPIO_PIN_1,
            .dir_inverted = false, // TODO: check wiring and set this correctly
            .max_speed = DRV88xx_MAX_SPD,
            .acceleration = DRV88xx_ACCEL,
            .en_port = NULL,
            .en_pin = 0xFF,       // not used
            .en_inverted = false, // not used
            .nfault_port = NULL,
            .nfault_pin = 0xFF, // not used
        },
    .limit_port = NULL,
    .limit_pin = 0xFF, // not used
};

stepper_ctrl_t stepper_raise1 = {
    .config =
        &(drv88xx_config_t){
            .step_port = GPIOE,
            .step_pin = GPIO_PIN_5,
            .dir_port = GPIOE,
            .dir_pin = GPIO_PIN_6,
            .dir_inverted = false, // TODO: check wiring and set this correctly
            .max_speed = DRV88xx_MAX_SPD,
            .acceleration = DRV88xx_ACCEL,
            .en_port = NULL,
            .en_pin = 0xFF,       // not used
            .en_inverted = false, // not used
            .nfault_port = GPIOC,
            .nfault_pin = GPIO_PIN_13,
        },
    .limit_port = NULL,
    .limit_pin = 0xFF, // not used
};

stepper_ctrl_t stepper_raise2 = {
    .config =
        &(drv88xx_config_t){
            .step_port = GPIOA,
            .step_pin = GPIO_PIN_15,
            .dir_port = GPIOC,
            .dir_pin = GPIO_PIN_10,
            .dir_inverted = false, // TODO: check wiring and set this correctly
            .max_speed = DRV88xx_MAX_SPD,
            .acceleration = DRV88xx_ACCEL,
            .en_port = NULL,
            .en_pin = 0xFF,       // not used
            .en_inverted = false, // not used
            .nfault_port = GPIOC,
            .nfault_pin = GPIO_PIN_11,
        },
    .limit_port = NULL,
    .limit_pin = 0xFF, // not used
};

stepper_ctrl_t stepper_underpass = {
    .config =
        &(drv88xx_config_t){
            .step_port = GPIOD,
            .step_pin = GPIO_PIN_2,
            .dir_port = GPIOD,
            .dir_pin = GPIO_PIN_1,
            .dir_inverted = false, // TODO: check wiring and set this correctly
            .max_speed = DRV88xx_MAX_SPD,
            .acceleration = DRV88xx_ACCEL,
            .en_port = NULL,
            .en_pin = 0xFF,       // not used
            .en_inverted = false, // not used
            .nfault_port = NULL,
            .nfault_pin = 0xFF, // not used
        },
    .limit_port = GPIOE,
    .limit_pin = GPIO_PIN_9,
};

stepper_ctrl_t stepper_bevel = {
    .config =
        &(drv88xx_config_t){
            .step_port = GPIOD,
            .step_pin = GPIO_PIN_0,
            .dir_port = GPIOC,
            .dir_pin = GPIO_PIN_12,
            .dir_inverted = false, // TODO: check wiring and set this correctly
            .max_speed = DRV88xx_MAX_SPD,
            .acceleration = DRV88xx_ACCEL,
            .en_port = NULL,
            .en_pin = 0xFF,       // not used
            .en_inverted = false, // not used
            .nfault_port = NULL,
            .nfault_pin = 0xFF, // not used
        },
    .limit_port = GPIOE,
    .limit_pin = GPIO_PIN_11,
};

// DC motor control structs

// TODO: set PID gains for each motor based on tuning
// Set ki=0, kd=0, increase kp until the motor reaches target speed but
// oscillates Back kp off to ~60% of that value Increase ki slowly until
// steady-state error disappears Add a small kd only if you need faster
// disturbance rejection
qPID_Gains_t pid_gains_pitch = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};
qPID_Gains_t pid_gains_roll = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};
qPID_Gains_t pid_gains_yaw = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};
qPID_Gains_t pid_gains_clamp = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};
qPID_Gains_t pid_gains_tension = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};
qPID_Gains_t pid_gains_sclamp1 = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};
qPID_Gains_t pid_gains_sclamp2 = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

drv8251_config_t dc_pitch_drv = {
    .in1_port = GPIOD,
    .in1_pin = GPIO_PIN_3,
    .in1_tim = &htim2,
    .in1_tim_channel = TIM_CHANNEL_1,
    .in2_port = GPIOD,
    .in2_pin = GPIO_PIN_4,
    .in2_tim = &htim2,
    .in2_tim_channel = TIM_CHANNEL_2,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

drv8251_config_t dc_roll_drv = {
    .in1_port = GPIOB,
    .in1_pin = GPIO_PIN_9,
    .in1_tim = &htim3,
    .in1_tim_channel = TIM_CHANNEL_4,
    .in2_port = GPIOB,
    .in2_pin = GPIO_PIN_8,
    .in2_tim = &htim3,
    .in2_tim_channel = TIM_CHANNEL_3,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

drv8251_config_t dc_yaw_drv = {
    .in1_port = GPIOC,
    .in1_pin = GPIO_PIN_3,
    .in1_tim = &htim1,
    .in1_tim_channel = TIM_CHANNEL_4,
    .in2_port = GPIOC,
    .in2_pin = GPIO_PIN_2,
    .in2_tim = &htim1,
    .in2_tim_channel = TIM_CHANNEL_3,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

drv8251_config_t clamp_drv = {
    .in1_port = GPIOF,
    .in1_pin = GPIO_PIN_10,
    .in1_tim = &htim15,
    .in1_tim_channel = TIM_CHANNEL_1,
    .in2_port = GPIOF,
    .in2_pin = GPIO_PIN_9,
    .in2_tim = &htim15,
    .in2_tim_channel = TIM_CHANNEL_2,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

drv8251_config_t tension_drv = {
    .in1_port = GPIOB,
    .in1_pin = GPIO_PIN_7,
    .in1_tim = &htim4,
    .in1_tim_channel = TIM_CHANNEL_2,
    .in2_port = GPIOB,
    .in2_pin = GPIO_PIN_6,
    .in2_tim = &htim4,
    .in2_tim_channel = TIM_CHANNEL_1,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

drv8251_config_t sclamp1_drv = {
    .in1_port = GPIOB,
    .in1_pin = GPIO_PIN_5,
    .in1_tim = &htim16,
    .in1_tim_channel = TIM_CHANNEL_1,
    .in2_port = GPIOB,
    .in2_pin = GPIO_PIN_4,
    .in2_tim = &htim17,
    .in2_tim_channel = TIM_CHANNEL_1,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

// Note: SCLAMP2 shares the same timer channels as PITCH, so avoid using them at
// the same time
drv8251_config_t sclamp2_drv = {
    .in1_port = GPIOD,
    .in1_pin = GPIO_PIN_7,
    .in1_tim = &htim2,
    .in1_tim_channel = TIM_CHANNEL_4,
    .in2_port = GPIOD,
    .in2_pin = GPIO_PIN_6,
    .in2_tim = &htim2,
    .in2_tim_channel = TIM_CHANNEL_3,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

// Encoder configurations

// SOME OF THESE WILL NEED TO BE IGNORED OR POLLED INSTEAD OF USING EXTI
enc_config_t enc_pitch = {
    .enc_a_port = GPIOC,
    .enc_a_pin = GPIO_PIN_7,
    .enc_b_port = GPIOC,
    .enc_b_pin = GPIO_PIN_6,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

enc_config_t enc_roll = {
    .enc_a_port = GPIOB,
    .enc_a_pin = GPIO_PIN_15,
    .enc_b_port = GPIOB,
    .enc_b_pin = GPIO_PIN_14,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

enc_config_t enc_yaw = {
    .enc_a_port = GPIOC,
    .enc_a_pin = GPIO_PIN_5,
    .enc_b_port = GPIOC,
    .enc_b_pin = GPIO_PIN_4,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

enc_config_t enc_clamp = {
    .enc_a_port = GPIOD,
    .enc_a_pin = GPIO_PIN_15,
    .enc_b_port = GPIOD,
    .enc_b_pin = GPIO_PIN_14,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

enc_config_t enc_tension = {
    .enc_a_port = GPIOB,
    .enc_a_pin = GPIO_PIN_15,
    .enc_b_port = GPIOB,
    .enc_b_pin = GPIO_PIN_14,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

enc_config_t enc_sclamp1 = {
    .enc_a_port = GPIOC,
    .enc_a_pin = GPIO_PIN_9,
    .enc_b_port = GPIOC,
    .enc_b_pin = GPIO_PIN_8,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

enc_config_t enc_sclamp2 = {
    .enc_a_port = GPIOD,
    .enc_a_pin = GPIO_PIN_15,
    .enc_b_port = GPIOD,
    .enc_b_pin = GPIO_PIN_14,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

motor_ctrl_t dc_pitch = {
    .drv = &dc_pitch_drv,
    .enc = &enc_pitch,
    .pid = {0},
    .enabled = false,
    .hall_port = GPIOE,
    .hall_pin = GPIO_PIN_13,
};

motor_ctrl_t dc_roll = {
    .drv = &dc_roll_drv,
    .enc = &enc_roll,
    .pid = {0},
    .enabled = false,
    .hall_port = GPIOE,
    .hall_pin = GPIO_PIN_14,
};

motor_ctrl_t dc_yaw = {
    .drv = &dc_yaw_drv,
    .enc = &enc_yaw,
    .pid = {0},
    .enabled = false,
    .hall_port = GPIOE,
    .hall_pin = GPIO_PIN_15,
};

motor_ctrl_t clamp = {
    .drv = &clamp_drv,
    .enc = &enc_clamp,
    .pid = {0},
    .enabled = false,
};

motor_ctrl_t tension = {
    .drv = &tension_drv,
    .enc = &enc_tension,
    .pid = {0},
    .enabled = false,
};

motor_ctrl_t sclamp1 = {
    .drv = &sclamp1_drv,
    .enc = &enc_sclamp1,
    .pid = {0},
    .enabled = false,
};

motor_ctrl_t sclamp2 = {
    .drv = &sclamp2_drv,
    .enc = &enc_sclamp2,
    .pid = {0},
    .enabled = false,
};

void RobotConfig_Init(void) {
  // Initialize stepper motor configurations
  DRV88xx_Init(stepper_spool.config);
  DRV88xx_Init(stepper_raise1.config);
  DRV88xx_Init(stepper_raise2.config);
  DRV88xx_Init(stepper_underpass.config);
  DRV88xx_Init(stepper_bevel.config);

  DRV8251_Init(&dc_pitch_drv);
  DRV8251_Init(&dc_roll_drv);
  DRV8251_Init(&dc_yaw_drv);
  DRV8251_Init(&clamp_drv);
  DRV8251_Init(&tension_drv);
  DRV8251_Init(&sclamp1_drv);
  DRV8251_Init(&sclamp2_drv);

  Encoder_Init(&enc_pitch);
  Encoder_Init(&enc_roll);
  Encoder_Init(&enc_yaw);
  Encoder_Init(&enc_clamp);
  Encoder_Init(&enc_tension);
  Encoder_Init(&enc_sclamp1);
  Encoder_Init(&enc_sclamp2);

  qPID_Setup(&dc_pitch.pid, pid_gains_pitch.Kc, pid_gains_pitch.Ki,
             pid_gains_pitch.Kd, CONTROL_TIME_STEP_S);
  qPID_Setup(&dc_roll.pid, pid_gains_roll.Kc, pid_gains_roll.Ki,
             pid_gains_roll.Kd, CONTROL_TIME_STEP_S);
  qPID_Setup(&dc_yaw.pid, pid_gains_yaw.Kc, pid_gains_yaw.Ki, pid_gains_yaw.Kd,
             CONTROL_TIME_STEP_S);
  qPID_Setup(&clamp.pid, pid_gains_clamp.Kc, pid_gains_clamp.Ki,
             pid_gains_clamp.Kd, CONTROL_TIME_STEP_S);
  qPID_Setup(&tension.pid, pid_gains_tension.Kc, pid_gains_tension.Ki,
             pid_gains_tension.Kd, CONTROL_TIME_STEP_S);
  qPID_Setup(&sclamp1.pid, pid_gains_sclamp1.Kc, pid_gains_sclamp1.Ki,
             pid_gains_sclamp1.Kd, CONTROL_TIME_STEP_S);
  qPID_Setup(&sclamp2.pid, pid_gains_sclamp2.Kc, pid_gains_sclamp2.Ki,
             pid_gains_sclamp2.Kd, CONTROL_TIME_STEP_S);
}
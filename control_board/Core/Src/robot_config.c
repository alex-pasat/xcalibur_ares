/**
 * @file    robot_config.c
 * @brief   Robot configuration definitions. This file defines the motor config
 * structs for each motor, and any other global configuration variables for the
 * robot.
 */
 #include "robot_config.h"
 #include "main.h"
#include "drv8251.h"
#include "encoder.h"
#include "qpid.h"
#include "robot_control.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_tim.h"

#define DRV88xx_MAX_SPD 1000.0f
#define DRV88xx_ACCEL 50.0f

#define TIMER_FREQ_HZ 170000000 // SYSCLK Frequency
#define TIMER_PSC 0
#define TIMER_PWM_FREQ_HZ 50000 // Desired PWM frequency for DRV8251

// TODO: should this be hardcoded or computed based on the timer configuration?
#define CONTROL_TIME_STEP_S 0.01f // 10 ms control loop period

// Gear Ratios TODO: set these to the actual gear ratios
#define GEAR_RATIO_ROLL 50.0f

// size of word for ADC DMA buffer
#define ADC_BUFFER_SIZE 7
volatile uint16_t adc_dma_buf[ADC_BUFFER_SIZE] = {0};

// Stepper motor configurations
#if 0
stepper_ctrl_t stepper_spool = {
    .config =
        &(drv88xx_config_t){
            .step_port = SPOOL_STEP_GPIO_Port,
            .step_pin = SPOOL_STEP_Pin,
            .dir_port = SPOOL_DIR_GPIO_Port,
            .dir_pin = SPOOL_DIR_Pin,
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
            .step_port = RAISE1_STEP_GPIO_Port,
            .step_pin = RAISE1_STEP_Pin,
            .dir_port = RAISE1_DIR_GPIO_Port,
            .dir_pin = RAISE1_DIR_Pin,
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
            .step_port = RAISE2_STEP_GPIO_Port,
            .step_pin = RAISE2_STEP_Pin,
            .dir_port = RAISE2_DIR_GPIO_Port,
            .dir_pin = RAISE2_DIR_Pin,
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
#endif

stepper_ctrl_t stepper_underpass = {
    .config =
        &(drv88xx_config_t){
            .step_port = UNDERPASS_STEP_GPIO_Port,
            .step_pin = UNDERPASS_STEP_Pin,
            .dir_port = UNDERPASS_DIR_GPIO_Port,
            .dir_pin = UNDERPASS_DIR_Pin,
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

#if 0
stepper_ctrl_t stepper_bevel = {
    .config =
        &(drv88xx_config_t){
            .step_port = BEVEL_STEP_GPIO_Port,
            .step_pin = BEVEL_STEP_Pin,
            .dir_port = BEVEL_DIR_GPIO_Port,
            .dir_pin = BEVEL_DIR_Pin,
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
#endif

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
#if 0
qPID_Gains_t pid_gains_tension = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};
qPID_Gains_t pid_gains_sclamp1 = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};
qPID_Gains_t pid_gains_sclamp2 = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};
#endif

qPID_AutoTuning_t at_pitch;
qPID_AutoTuning_t at_roll;
qPID_AutoTuning_t at_yaw;
qPID_AutoTuning_t at_clamp;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

drv8251_config_t dc_pitch_drv = {
    .in1_port = PITCH_M_IN_B_GPIO_Port,
    .in1_pin = PITCH_M_IN_B_Pin,
    .in1_tim = &htim2,
    .in1_tim_channel = TIM_CHANNEL_1,
    .in2_port = PITCH_M_IN_A_GPIO_Port,
    .in2_pin = PITCH_M_IN_A_Pin,
    .in2_tim = &htim2,
    .in2_tim_channel = TIM_CHANNEL_2,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

drv8251_config_t dc_roll_drv = {
    .in1_port = ROLL_M_IN_B_GPIO_Port,
    .in1_pin = ROLL_M_IN_B_Pin,
    .in1_tim = &htim4,
    .in1_tim_channel = TIM_CHANNEL_3,
    .in2_port = ROLL_M_IN_A_GPIO_Port,
    .in2_pin = ROLL_M_IN_A_Pin,
    .in2_tim = &htim4,
    .in2_tim_channel = TIM_CHANNEL_4,
    .dir_inverted = false, // TODO: check wiring and set this correctly
    .MIN_RPM = 39,
    .MAX_RPM = 1500,
};

drv8251_config_t dc_yaw_drv = {
    .in1_port = YAW_M_IN_B_GPIO_Port,
    .in1_pin = YAW_M_IN_B_Pin,
    .in1_tim = &htim1,
    .in1_tim_channel = TIM_CHANNEL_3,
    .in2_port = YAW_M_IN_A_GPIO_Port,
    .in2_pin = YAW_M_IN_A_Pin,
    .in2_tim = &htim1,
    .in2_tim_channel = TIM_CHANNEL_4,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

drv8251_config_t clamp_drv = {
    .in1_port = KNIFECLAMP_M_IN_B_GPIO_Port,
    .in1_pin = KNIFECLAMP_M_IN_B_Pin,
    .in1_tim = &htim15,
    .in1_tim_channel = TIM_CHANNEL_1,
    .in2_port = KNIFECLAMP_M_IN_A_GPIO_Port,
    .in2_pin = KNIFECLAMP_M_IN_A_Pin,
    .in2_tim = &htim15,
    .in2_tim_channel = TIM_CHANNEL_2,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

#if 0
drv8251_config_t tension_drv = {
    .in1_port = TENSION_M_IN_B_GPIO_Port,
    .in1_pin = TENSION_M_IN_B_Pin,
    .in1_tim = &htim4,
    .in1_tim_channel = TIM_CHANNEL_2,
    .in2_port = TENSION_M_IN_A_GPIO_Port,
    .in2_pin = TENSION_M_IN_A_Pin,
    .in2_tim = &htim4,
    .in2_tim_channel = TIM_CHANNEL_1,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

drv8251_config_t sclamp1_drv = {
    .in1_port = SCLAMP1_M_IN_B_GPIO_Port,
    .in1_pin = SCLAMP1_M_IN_B_Pin,
    .in1_tim = &htim16,
    .in1_tim_channel = TIM_CHANNEL_1,
    .in2_port = SCLAMP1_M_IN_A_GPIO_Port,
    .in2_pin = SCLAMP1_M_IN_A_Pin,
    .in2_tim = &htim17,
    .in2_tim_channel = TIM_CHANNEL_1,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};

// Note: SCLAMP2 shares the same timer channels as PITCH, so avoid using them at
// the same time
drv8251_config_t sclamp2_drv = {
    .in1_port = SCLAMP2_M_IN_B_GPIO_Port,
    .in1_pin = SCLAMP2_M_IN_B_Pin,
    .in1_tim = &htim2,
    .in1_tim_channel = TIM_CHANNEL_4,
    .in2_port = SCLAMP2_M_IN_A_GPIO_Port,
    .in2_pin = SCLAMP2_M_IN_A_Pin,
    .in2_tim = &htim2,
    .in2_tim_channel = TIM_CHANNEL_3,
    .dir_inverted = false, // TODO: check wiring and set this correctly
};
#endif

// Encoder configurations
enc_config_t enc_pitch = {
    .enc_a_port = PITCH_ENC_A_GPIO_Port,
    .enc_a_pin = PITCH_ENC_A_Pin,
    .enc_b_port = PITCH_ENC_B_GPIO_Port,
    .enc_b_pin = PITCH_ENC_B_Pin,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

enc_config_t enc_roll = {
    .enc_a_port = ROLL_ENC_A_GPIO_Port,
    .enc_a_pin = ROLL_ENC_A_Pin,
    .enc_b_port = ROLL_ENC_B_GPIO_Port,
    .enc_b_pin = ROLL_ENC_B_Pin,
    .counts_per_rev = 28 * GEAR_RATIO_ROLL, // 28*gear ratio of the roll motor
};

enc_config_t enc_yaw = {
    .enc_a_port = YAW_ENC_A_GPIO_Port,
    .enc_a_pin = YAW_ENC_A_Pin,
    .enc_b_port = YAW_ENC_B_GPIO_Port,
    .enc_b_pin = YAW_ENC_B_Pin,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

enc_config_t enc_clamp = {
    .enc_a_port = KNIFECLAMP_ENC_A_GPIO_Port,
    .enc_a_pin = KNIFECLAMP_ENC_A_Pin,
    .enc_b_port = KNIFECLAMP_ENC_B_GPIO_Port,
    .enc_b_pin = KNIFECLAMP_ENC_B_Pin,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

#if 0
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
#endif

extern ADC_HandleTypeDef hadc1;

motor_ctrl_t dc_pitch = {
    .drv = &dc_pitch_drv,
    .enc = &enc_pitch,
    .pid = {0},
    .enabled = false,
    .hall_port = GPIOE,
    .hall_pin = GPIO_PIN_13,
    .adc_port = ADC_PITCH_GPIO_Port,
    .adc_pin = ADC_PITCH_Pin,
    .curr_config = {
        .adc_instance = &hadc1,
        .adc_channel = ADC_CHANNEL_6,
        .shunt_resistor_mohm = 24,
    },
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
    .adc_port = ADC_KNIFECLAMP_GPIO_Port,
    .adc_pin = ADC_KNIFECLAMP_Pin,
    .curr_config = {
        .adc_instance = &hadc1,
        .adc_channel = ADC_CHANNEL_1,
        .shunt_resistor_mohm = 82,
    },
};

#if 0
motor_ctrl_t tension = {
    .drv = &tension_drv,
    // .enc = &enc_tension,
    .pid = {0},
    .enabled = false,
};

motor_ctrl_t sclamp1 = {
    .drv = &sclamp1_drv,
    // .enc = &enc_sclamp1,
    .pid = {0},
    .enabled = false,
};

motor_ctrl_t sclamp2 = {
    .drv = &sclamp2_drv,
    // .enc = &enc_sclamp2,
    .pid = {0},
    .enabled = false,
};
#endif

void RobotConfig_Init(void) {
  // Init DMA
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(uint16_t*)adc_dma_buf, ADC_BUFFER_SIZE);

  // Initialize stepper motor configurations
#if 0
  DRV88xx_Init(stepper_spool.config);
  DRV88xx_Init(stepper_raise1.config);
  DRV88xx_Init(stepper_raise2.config);
  DRV88xx_Init(stepper_bevel.config);
  #endif
  DRV88xx_Init(stepper_underpass.config);

  Encoder_Init(&enc_pitch);
  Encoder_Init(&enc_roll);
  Encoder_Init(&enc_yaw);
  Encoder_Init(&enc_clamp);
  #if 0
  Encoder_Init(&enc_tension);
  Encoder_Init(&enc_sclamp1);
  Encoder_Init(&enc_sclamp2);
  #endif
  // qPID_BindAutoTuning(&dc_roll.pid, &at_pitch);
  // qPID_BindAutoTuning(&dc_roll.pid, &at_roll);
  // qPID_BindAutoTuning(&dc_yaw.pid, &at_yaw);
  // qPID_BindAutoTuning(&clamp.pid, &at_clamp);

  // qPID_EnableAutoTuning(&dc_roll.pid, 5000);

  MotorCtrl_Init(&dc_pitch, &dc_pitch_drv, &enc_pitch, &dc_pitch.pid,
      pid_gains_pitch, CONTROL_TIME_STEP_S);
  MotorCtrl_Init(&dc_roll, &dc_roll_drv, &enc_roll, &dc_roll.pid,
      pid_gains_roll, CONTROL_TIME_STEP_S);
  MotorCtrl_Init(&dc_yaw, &dc_yaw_drv, &enc_yaw, &dc_yaw.pid,
      pid_gains_yaw, CONTROL_TIME_STEP_S);
  MotorCtrl_Init(&clamp, &clamp_drv, &enc_clamp, &clamp.pid,
      pid_gains_clamp, CONTROL_TIME_STEP_S);
#if 0
  MotorCtrl_Init(&tension, &tension_drv, NULL, &tension.pid,
      pid_gains_tension, CONTROL_TIME_STEP_S);
  MotorCtrl_Init(&sclamp1, &sclamp1_drv, NULL,
      &sclamp1.pid, pid_gains_sclamp1, CONTROL_TIME_STEP_S);
  MotorCtrl_Init(&sclamp2, &sclamp2_drv, NULL,
      &sclamp2.pid, pid_gains_sclamp2, CONTROL_TIME_STEP_S);
#endif
}
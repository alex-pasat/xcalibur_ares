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
#include "robot_control.h"

#include "qpid.h"
#include "stm32g491xx.h"
#include "tiny_ring_buffer.h"

#include <string.h>

// -- Defines -----------------------------------------------------------------

#define PITCH_M_CPR 64.0f
#define PITCH_M_GEAR_RATIO 150.0f
#define PITCH_M_COUNTS_PER_REV 7600.0f

#define ROLL_M_CPR 12.0f
#define ROLL_M_GEAR_RATIO 380.0f
#define ROLL_M_COUNTS_PER_REV (ROLL_M_CPR * ROLL_M_GEAR_RATIO * 4.0f)

#define YAW_M_CPR 12.0f
#define YAW_M_GEAR_RATIO 380.0f
#define YAW_M_COUNTS_PER_REV 10600.0f // measured

// -- PV Definitions ----------------------------------------------------------

volatile uint16_t adc_dma_buf[ADC_NUM_CHANNELS];

// -- Stepper Configurations --------------------------------------------------

extern TIM_HandleTypeDef htim7;

stepper_ctrl_t stepper_underpass = {
    .config =
        &(drv88xx_config_t){
            .step_port = UNDERPASS_STEP_GPIO_Port,
            .step_pin = UNDERPASS_STEP_Pin,
            .dir_port = UNDERPASS_DIR_GPIO_Port,
            .dir_pin = UNDERPASS_DIR_Pin,
            .dir_inverted = true,
            .en_port = NULL,
            .en_pin = 0xFF,       // not used
            .en_inverted = false, // not used
            .nfault_port = NULL,
            .nfault_pin = 0xFF, // not used
            
            .tim = &htim7,

            // .steps_per_rev = 100,
            .max_speed = 1000.0f, // TODO: set this to the actual max speed
            .acceleration = 500.0f, // TODO: set this to the actual acceleration
        },
    .limit_sw = &(gpio_sensor_t){
        .port = GPIOE,
        .pin = GPIO_PIN_9,
        .threshold = 5,
        .last_state = false,
        .debounce_count = 0,
    },
    .MICROSTEPS = DRV8834_MICROSTEPS,
};

// -- DC Motor Configurations -------------------------------------------------

qPID_Gains_t pid_gains_pitch = {.Kc = 0.1f, .Ki = 0.0f, .Kd = 0.0f};
qPID_Gains_t pid_gains_roll = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};
qPID_Gains_t pid_gains_yaw = {.Kc = 6.7f, .Ki = 0.67f, .Kd = 0.0f};
qPID_Gains_t pid_gains_clamp = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim20;

drv8251_config_t dc_pitch_drv = {
    .in1_port = PITCH_M_IN_B_GPIO_Port,
    .in1_pin = PITCH_M_IN_B_Pin,
    .in1_tim = &htim2,
    .in1_tim_channel = TIM_CHANNEL_1,
    .in2_port = PITCH_M_IN_A_GPIO_Port,
    .in2_pin = PITCH_M_IN_A_Pin,
    .in2_tim = &htim2,
    .in2_tim_channel = TIM_CHANNEL_2,
    .dir_inverted = true, // TODO: check wiring and set this correctly
    .MIN_DUTY_CYCLE = 0.11f,
    .MAX_RPS = RPM_TO_RPS(1500),
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
    .MIN_DUTY_CYCLE = 0.01f, // TODO: set this based on testing to overcome static friction
    .MAX_RPS = RPM_TO_RPS(1500),
};

drv8251_config_t dc_yaw_drv = {
    .in1_port = YAW_M_IN_B_GPIO_Port,
    .in1_pin = YAW_M_IN_B_Pin,
    .in1_tim = &htim17,
    .in1_tim_channel = TIM_CHANNEL_1,
    .in2_port = YAW_M_IN_A_GPIO_Port,
    .in2_pin = YAW_M_IN_A_Pin,
    .in2_tim = &htim16,
    .in2_tim_channel = TIM_CHANNEL_1,
    .dir_inverted = false,
    .MIN_DUTY_CYCLE = 0.60f,
    .MAX_RPS = RPM_TO_RPS(1500),
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
    .MIN_DUTY_CYCLE = 0.01f, // TODO: set this based on testing to overcome static friction
    .MAX_RPS = RPM_TO_RPS(1500),
};

// -- Encoder Configurations --------------------------------------------------

enc_config_t enc_pitch = {
    .enc_a_port = PITCH_ENC_A_GPIO_Port,
    .enc_a_pin = PITCH_ENC_A_Pin,
    .enc_b_port = PITCH_ENC_B_GPIO_Port,
    .enc_b_pin = PITCH_ENC_B_Pin,
    .counts_per_rev = PITCH_M_COUNTS_PER_REV,
};

enc_config_t enc_roll = {
    .enc_a_port = ROLL_ENC_A_GPIO_Port,
    .enc_a_pin = ROLL_ENC_A_Pin,
    .enc_b_port = ROLL_ENC_B_GPIO_Port,
    .enc_b_pin = ROLL_ENC_B_Pin,
    .counts_per_rev = ROLL_M_CPR * 4,
    .gear_ratio = ROLL_M_GEAR_RATIO,
};

enc_config_t enc_yaw = {
    .enc_a_port = YAW_ENC_A_GPIO_Port,
    .enc_a_pin = YAW_ENC_A_Pin,
    .enc_b_port = YAW_ENC_B_GPIO_Port,
    .enc_b_pin = YAW_ENC_B_Pin,
    .counts_per_rev = 10600,
    .gear_ratio = YAW_M_GEAR_RATIO,
};

enc_config_t enc_clamp = {
    .enc_a_port = KNIFECLAMP_ENC_A_GPIO_Port,
    .enc_a_pin = KNIFECLAMP_ENC_A_Pin,
    .enc_b_port = KNIFECLAMP_ENC_B_GPIO_Port,
    .enc_b_pin = KNIFECLAMP_ENC_B_Pin,
    .counts_per_rev = 1024, // TODO: set this to the actual CPR of your encoder
};

// -- Motor Control Structs ---------------------------------------------------

extern ADC_HandleTypeDef hadc1;

motor_ctrl_t dc_pitch = {
    .drv = &dc_pitch_drv,
    .enc = &enc_pitch,
    .pid = {0},
    .limit_sw = &(gpio_sensor_t){
      .port = NULL, // no limit switch
      .pin = 0xFF,
    },
    .hall_effect = &(gpio_sensor_t){
      .port = GPIOE,
      .pin = GPIO_PIN_13,
      .threshold = 5,
      .last_state = false,
      .debounce_count = 0,
    },
    .adc_port = ADC_PITCH_GPIO_Port,
    .adc_pin = ADC_PITCH_Pin,
    .curr_config = {
        .adc_instance = &hadc1,
        .adc_index = 2, // ADC RANK 3
        .shunt_resistor_mohm = 24,
    },
};

motor_ctrl_t dc_roll = {
    .drv = &dc_roll_drv,
    .enc = &enc_roll,
    .pid = {0},
    .limit_sw = &(gpio_sensor_t){
      .port = NULL, // no limit switch
      .pin = 0xFF,
    },
    .hall_effect = &(gpio_sensor_t) {
      .port = GPIOE,
      .pin = GPIO_PIN_14,
      .threshold = 5,
      .last_state = false,
      .debounce_count = 0,
    },
    .adc_port = NULL,
    .adc_pin = 0xFF,
};

motor_ctrl_t dc_yaw = {
    .drv = &dc_yaw_drv,
    .enc = &enc_yaw,
    .pid = {0},
    .limit_sw = &(gpio_sensor_t){
      .port = NULL, // no limit switch
      .pin = 0xFF,
    },
    .hall_effect = &(gpio_sensor_t) {
      .port = GPIOE,
      .pin = GPIO_PIN_15,
      .threshold = 5,
      .last_state = false,
      .debounce_count = 0,
    },
    .adc_port = NULL,
    .adc_pin = 0xFF,
};

motor_ctrl_t clamp = {
    .drv = &clamp_drv,
    .enc = &enc_clamp,
    .pid = {0},
    .limit_sw = &(gpio_sensor_t){
      .port = NULL, // no limit switch
      .pin = 0xFF,
    },
    .hall_effect = &(gpio_sensor_t){
      .port = EXTRA_HALL_GPIO_Port,
      .pin = EXTRA_HALL_Pin,
      .threshold = 5,
      .last_state = false,
      .debounce_count = 0,
    },
    .adc_port = ADC_KNIFECLAMP_GPIO_Port,
    .adc_pin = ADC_KNIFECLAMP_Pin,
    .curr_config = {
        .adc_instance = &hadc1,
        .adc_index = 0, // ADC RANK 1
        .shunt_resistor_mohm = 82,
    },
};

// -- MISC Control Structs ----------------------------------------------------

led_pulse_ctrl_t led_strip = {
    .port = LED_STRIP_GPIO_Port,
    .pin = LED_STRIP_Pin,

    .tim = &htim20,
    .tim_channel = TIM_CHANNEL_1,

    // start on
    .duty_cycle = 1.0f,
    .duty_step = 0.005f, 
    .increasing = false,
};

fan_ctrl_t fan = {
    .port = FAN1_GPIO_GPIO_Port,
    .pin = FAN1_GPIO_Pin,
    .tim = &htim3,
    .tim_channel = TIM_CHANNEL_2,
};

pump_ctrl_t pump = {
    .port = PUMP_M_IN_GPIO_Port,
    .pin = PUMP_M_IN_Pin,
    .tim = &htim3,
    .tim_channel = TIM_CHANNEL_3,
    .current = {
        .adc_instance = &hadc1,
        .adc_index = 1, // ADC RANK 2
        .shunt_resistor_mohm = 75,
    },
    .MAX_CURRENT_mA = 800,
};

// -- Function Definitions ----------------------------------------------------

void RobotConfig_Init(void) {
  // Init DMA
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(uint16_t*)adc_dma_buf, ADC_NUM_CHANNELS);

  // Initialize stepper motor configurations

  StepperCtrl_Init(&stepper_underpass, stepper_underpass.config);

  MotorCtrl_Init(&dc_pitch, &dc_pitch_drv, &enc_pitch, &dc_pitch.pid,
      pid_gains_pitch, CONTROL_TIME_STEP_S);
  MotorCtrl_Init(&dc_roll, &dc_roll_drv, &enc_roll, &dc_roll.pid,
      pid_gains_roll, CONTROL_TIME_STEP_S);
  MotorCtrl_Init(&dc_yaw, &dc_yaw_drv, &enc_yaw, &dc_yaw.pid,
      pid_gains_yaw, CONTROL_TIME_STEP_S);
  MotorCtrl_Init(&clamp, &clamp_drv, &enc_clamp, &clamp.pid,
      pid_gains_clamp, CONTROL_TIME_STEP_S);

  HAL_TIM_PWM_Start(led_strip.tim, led_strip.tim_channel);
  HAL_TIM_PWM_Start(fan.tim, fan.tim_channel);
  HAL_TIM_PWM_Start(pump.tim, pump.tim_channel);
}
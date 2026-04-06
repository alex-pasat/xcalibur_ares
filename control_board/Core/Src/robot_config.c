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
#define PITCH_M_COUNTS_PER_REV 9600.0f

#define ROLL_M_CPR 7.0f
#define ROLL_M_GEAR_RATIO 380.0f
#define ROLL_M_COUNTS_PER_REV (ROLL_M_CPR * ROLL_M_GEAR_RATIO * 4.0f)

#define YAW_M_CPR 7.0f
#define YAW_M_GEAR_RATIO 380.0f
#define YAW_M_COUNTS_PER_REV (YAW_M_CPR * YAW_M_GEAR_RATIO * 4.0f)

#define CLAMP_M_CPR 7.0f
#define CLAMP_M_GEAR_RATIO 380.0f
#define CLAMP_M_COUNTS_PER_REV (CLAMP_M_CPR * CLAMP_M_GEAR_RATIO * 4.0f)

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

            .max_speed = 1000.0f,
            .acceleration = 500.0f,
        },
    .limit_sw = &(gpio_sensor_t){
        .port = UNDERPASS_LIMIT_GPIO_Port,
        .pin = UNDERPASS_LIMIT_Pin,
        .threshold = 0,
    },
    .homing_direction = false,
    .BACKOFF_STEPS = 50,
    .MICROSTEPS = DRV8834_MICROSTEPS,
    .uSTEPS_PER_M = 27027.0f
};

stepper_ctrl_t *stepper_motors[] = {&stepper_underpass};

// -- DC Motor Configurations -------------------------------------------------

const qPID_Gains_t pid_gains_pitch = {.Kc = 1.2f, .Ki = 1.0f, .Kd = 0.0f};
const qPID_Gains_t pid_gains_roll = {.Kc = 8.0f, .Ki = 0.67f, .Kd = 0.0f};
const qPID_Gains_t pid_gains_yaw = {.Kc = 10.0f, .Ki = 0.67f, .Kd = 0.0f};
const qPID_Gains_t pid_gains_clamp = {.Kc = 8.0f, .Ki = 0.67f, .Kd = 0.0f};
const qPID_Gains_t pid_gains_extra = {.Kc = 1.0f, .Ki = 0.0f, .Kd = 0.0f};

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
    .dir_inverted = true,
    .STALL_DUTY_CYCLE = 0.12f,
    .MIN_DUTY_CYCLE = 0.03f,
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
    .dir_inverted = true,
    .STALL_DUTY_CYCLE = 0.65f,
    .MIN_DUTY_CYCLE = 0.61f,
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
    .dir_inverted = true,
    .STALL_DUTY_CYCLE = 0.65f,
    .MIN_DUTY_CYCLE = 0.61f,
    .MAX_RPS = RPM_TO_RPS(1500),
};

drv8251_config_t dc_clamp_drv = {
    .in1_port = KNIFECLAMP_M_IN_B_GPIO_Port,
    .in1_pin = KNIFECLAMP_M_IN_B_Pin,
    .in1_tim = &htim15,
    .in1_tim_channel = TIM_CHANNEL_1,
    .in2_port = KNIFECLAMP_M_IN_A_GPIO_Port,
    .in2_pin = KNIFECLAMP_M_IN_A_Pin,
    .in2_tim = &htim15,
    .in2_tim_channel = TIM_CHANNEL_2,
    .dir_inverted = true,
    .STALL_DUTY_CYCLE = 0.65f,
    .MIN_DUTY_CYCLE = 0.61f,
    .MAX_RPS = RPM_TO_RPS(1500),
};

drv8251_config_t dc_extra_drv = {
    .in1_port = MOTOR_DRIVER_IN_B_GPIO_Port,
    .in1_pin = MOTOR_DRIVER_IN_B_Pin,
    .in1_tim = &htim2,
    .in1_tim_channel = TIM_CHANNEL_3,
    .in2_port = MOTOR_DRIVER_IN_A_GPIO_Port,
    .in2_pin = MOTOR_DRIVER_IN_A_Pin,
    .in2_tim = &htim2,
    .in2_tim_channel = TIM_CHANNEL_4,
    .dir_inverted = false,
    .STALL_DUTY_CYCLE = 0.0f,
    .MIN_DUTY_CYCLE = 0.0f,
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
    .counts_per_rev = ROLL_M_COUNTS_PER_REV,
};

enc_config_t enc_yaw = {
    .enc_a_port = YAW_ENC_A_GPIO_Port,
    .enc_a_pin = YAW_ENC_A_Pin,
    .enc_b_port = YAW_ENC_B_GPIO_Port,
    .enc_b_pin = YAW_ENC_B_Pin,
    .counts_per_rev = YAW_M_COUNTS_PER_REV,
};

enc_config_t enc_clamp = {
    .enc_a_port = KNIFECLAMP_ENC_A_GPIO_Port,
    .enc_a_pin = KNIFECLAMP_ENC_A_Pin,
    .enc_b_port = KNIFECLAMP_ENC_B_GPIO_Port,
    .enc_b_pin = KNIFECLAMP_ENC_B_Pin,
    .counts_per_rev = CLAMP_M_COUNTS_PER_REV,
};

// -- Motor Control Structs ---------------------------------------------------

extern ADC_HandleTypeDef hadc1;

motor_ctrl_t dc_pitch = {
    .drv = &dc_pitch_drv,
    .enc = &enc_pitch,
    .pid = {0},
    .pid_gains = pid_gains_pitch,
    .limit_sw = &(gpio_sensor_t){
      .port = NULL, 
      .pin = 0xFF,
    },
    .hall_effect = &(gpio_sensor_t){
      .port = GPIOE,
      .pin = GPIO_PIN_13,
      .threshold = 2,
      .active_low = true,
    },
    // home clockwise until hall effect triggered
    .homing_speed_rps = 0.2f,
    .adc_port = ADC_PITCH_GPIO_Port,
    .adc_pin = ADC_PITCH_Pin,
    .curr_config = {
        .adc_instance = &hadc1,
        .adc_index = 2, // ADC RANK 3
        .shunt_resistor_mohm = 24,
    },
    .angle_limiting = true,
    .ANGLE_MIN_DEG = -40.0f,
    .ANGLE_MAX_DEG = 30.0f,
};

motor_ctrl_t dc_roll = {
    .drv = &dc_roll_drv,
    .enc = &enc_roll,
    .pid = {0},
    .pid_gains = pid_gains_roll,
    .limit_sw = &(gpio_sensor_t){
      .port = NULL, 
      .pin = 0xFF,
    },
    .hall_effect = &(gpio_sensor_t) {
      .port = GPIOE,
      .pin = GPIO_PIN_14,
      .threshold = 0,
      .active_low = true,
    },
    .homing_speed_rps = 0.2f,
    .adc_port = NULL,
    .adc_pin = 0xFF,
    .angle_limiting = false,
};

motor_ctrl_t dc_yaw = {
    .drv = &dc_yaw_drv,
    .enc = &enc_yaw,
    .pid = {0},
    .pid_gains = pid_gains_yaw,
    .limit_sw = &(gpio_sensor_t){
      .port = NULL,
      .pin = 0xFF,
    },
    // TODO: set homing speed and direction
    .hall_effect = &(gpio_sensor_t) {
      .port = GPIOE,
      .pin = GPIO_PIN_15,
      .threshold = 0,
      .active_low = true,
    },
    .homing_speed_rps = -0.5f,
    .adc_port = NULL,
    .adc_pin = 0xFF,
    .angle_limiting = true,
    .ANGLE_MIN_DEG = 0.0f,
    .ANGLE_MAX_DEG = 0.0f,
};

motor_ctrl_t dc_clamp = {
    .drv = &dc_clamp_drv,
    .enc = &enc_clamp,
    .pid = {0},
    .pid_gains = pid_gains_clamp,
    .limit_sw = &(gpio_sensor_t){
      .port = NULL, 
      .pin = 0xFF,
    },
    // TODO: set homing speed and direction
    .hall_effect = &(gpio_sensor_t){
      .port = EXTRA_HALL_GPIO_Port,
      .pin = EXTRA_HALL_Pin,
      .threshold = 0,
      .active_low = true,
    },
    .homing_speed_rps = 0.5f,
    .adc_port = ADC_KNIFECLAMP_GPIO_Port,
    .adc_pin = ADC_KNIFECLAMP_Pin,
    .curr_config = {
        .adc_instance = &hadc1,
        .adc_index = 0, // ADC RANK 1
        .shunt_resistor_mohm = 82,
    },
};

motor_ctrl_t dc_extra = {
    .drv = &dc_extra_drv,
    .enc = NULL,
    .pid = {0},
    .pid_gains = pid_gains_extra,
    .limit_sw = &(gpio_sensor_t){
      .port = NULL,
      .pin = 0xFF,
    },
    .hall_effect = &(gpio_sensor_t){
      .port = NULL,
      .pin = 0xFF,
      .threshold = 0,
      .active_low = false,
    },
    .adc_port = NULL,
    .adc_pin = 0xFF,
};

motor_ctrl_t *dc_motors[] = {&dc_pitch, &dc_roll, &dc_yaw, &dc_clamp, &dc_extra};

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

  for (size_t i = 0; i < NUM_STEPPER_MOTORS; i++) {
    StepperCtrl_Init(stepper_motors[i], stepper_motors[i]->config);
  }

  for (size_t i = 0; i < NUM_DC_MOTORS; i++) {
    MotorCtrl_Init(dc_motors[i], dc_motors[i]->drv,
      dc_motors[i]->enc, dc_motors[i]->pid_gains, CONTROL_TIME_STEP_S);
  }

  HAL_TIM_PWM_Start(led_strip.tim, led_strip.tim_channel);
  HAL_TIM_PWM_Start(fan.tim, fan.tim_channel);
  HAL_TIM_PWM_Start(pump.tim, pump.tim_channel);
}
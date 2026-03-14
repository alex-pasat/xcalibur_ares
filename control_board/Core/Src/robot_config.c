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

// -- PV Definitions ----------------------------------------------------------

volatile uint16_t adc_dma_buf[ADC_NUM_CHANNELS];

uint8_t spi_rx_store[SPI_BUF_SIZE];
tiny_ring_buffer_t spi_rx_buf;

uint8_t spi_tx_store[SPI_BUF_SIZE];
tiny_ring_buffer_t spi_tx_buf;

uint8_t usb_rx_buf[64];
volatile uint8_t usb_rx_flag = 0;
volatile uint32_t usb_rx_len = 0;

// -- Stepper Configurations --------------------------------------------------

extern TIM_HandleTypeDef htim7;

stepper_ctrl_t stepper_underpass = {
    .config =
        &(drv88xx_config_t){
            .step_port = UNDERPASS_STEP_GPIO_Port,
            .step_pin = UNDERPASS_STEP_Pin,
            .dir_port = UNDERPASS_DIR_GPIO_Port,
            .dir_pin = UNDERPASS_DIR_Pin,
            .dir_inverted = false, // TODO: check wiring and set this correctly
            .en_port = NULL,
            .en_pin = 0xFF,       // not used
            .en_inverted = false, // not used
            .nfault_port = NULL,
            .nfault_pin = 0xFF, // not used
            
            .tim = &htim7,

            .MICROSTEPS = 16,
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
    .dir_inverted = false, // TODO: check wiring and set this correctly
    .MIN_RPM = 0, // TODO: set these
    .MAX_RPM = 1500,
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
    .MIN_RPM = 0, // TODO: set these
    .MAX_RPM = 1500,
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
    .MIN_RPM = 0, // TODO: set these
    .MAX_RPM = 1500,
};

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

void RobotConfig_Init(void) {
  // Initialize ring buffers
  memset(&spi_rx_buf, 0, sizeof(spi_rx_buf));
  memset(&spi_tx_buf, 0, sizeof(spi_tx_buf));
  memset(&usb_rx_buf, 0, sizeof(usb_rx_buf));
  
  tiny_ring_buffer_init(&spi_rx_buf, spi_rx_store, SPI_BUF_SIZE, sizeof(uint8_t));
  tiny_ring_buffer_init(&spi_tx_buf, spi_tx_store, SPI_BUF_SIZE, sizeof(uint8_t));  

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
}
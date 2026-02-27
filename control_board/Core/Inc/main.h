/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FAN2_GPIO_Pin GPIO_PIN_2
#define FAN2_GPIO_GPIO_Port GPIOE
#define FAN1_GPIO_Pin GPIO_PIN_3
#define FAN1_GPIO_GPIO_Port GPIOE
#define PUMP_M_IN_Pin GPIO_PIN_4
#define PUMP_M_IN_GPIO_Port GPIOE
#define RAISE1_STEP_Pin GPIO_PIN_5
#define RAISE1_STEP_GPIO_Port GPIOE
#define RAISE1_DIR_Pin GPIO_PIN_6
#define RAISE1_DIR_GPIO_Port GPIOE
#define RAISE1_NFAULT_Pin GPIO_PIN_13
#define RAISE1_NFAULT_GPIO_Port GPIOC
#define KNIFECLAMP_M_IN_B_Pin GPIO_PIN_9
#define KNIFECLAMP_M_IN_B_GPIO_Port GPIOF
#define KNIFECLAMP_M_IN_A_Pin GPIO_PIN_10
#define KNIFECLAMP_M_IN_A_GPIO_Port GPIOF
#define ADC_TENSION_Pin GPIO_PIN_0
#define ADC_TENSION_GPIO_Port GPIOC
#define ADC_PITCH_Pin GPIO_PIN_1
#define ADC_PITCH_GPIO_Port GPIOC
#define YAW_M_IN_B_Pin GPIO_PIN_2
#define YAW_M_IN_B_GPIO_Port GPIOC
#define YAW_M_IN_A_Pin GPIO_PIN_3
#define YAW_M_IN_A_GPIO_Port GPIOC
#define ADC_KNIFECLAMP_Pin GPIO_PIN_0
#define ADC_KNIFECLAMP_GPIO_Port GPIOA
#define ADC_PUMP_Pin GPIO_PIN_1
#define ADC_PUMP_GPIO_Port GPIOA
#define LASER_SIG_Pin GPIO_PIN_0
#define LASER_SIG_GPIO_Port GPIOB
#define GPIO_1_Pin GPIO_PIN_1
#define GPIO_1_GPIO_Port GPIOB
#define PWM_1_Pin GPIO_PIN_2
#define PWM_1_GPIO_Port GPIOB
#define LOAD_CELL_OUT_Pin GPIO_PIN_7
#define LOAD_CELL_OUT_GPIO_Port GPIOE
#define LOAD_CELL_SCLK_Pin GPIO_PIN_8
#define LOAD_CELL_SCLK_GPIO_Port GPIOE
#define UNDERPASS_LIMIT_Pin GPIO_PIN_9
#define UNDERPASS_LIMIT_GPIO_Port GPIOE
#define KNIFECLAMP_LIMIT_Pin GPIO_PIN_10
#define KNIFECLAMP_LIMIT_GPIO_Port GPIOE
#define BEVEL_LIMIT_Pin GPIO_PIN_11
#define BEVEL_LIMIT_GPIO_Port GPIOE
#define EXTRA_LIMIT_Pin GPIO_PIN_12
#define EXTRA_LIMIT_GPIO_Port GPIOE
#define PITCH_HALL_Pin GPIO_PIN_13
#define PITCH_HALL_GPIO_Port GPIOE
#define ROLL_HALL_Pin GPIO_PIN_14
#define ROLL_HALL_GPIO_Port GPIOE
#define YAW_HALL_Pin GPIO_PIN_15
#define YAW_HALL_GPIO_Port GPIOE
#define EXTRA_HALL_Pin GPIO_PIN_10
#define EXTRA_HALL_GPIO_Port GPIOB
#define ROLL_ENC_B_Pin GPIO_PIN_12
#define ROLL_ENC_B_GPIO_Port GPIOB
#define ROLL_ENC_A_Pin GPIO_PIN_13
#define ROLL_ENC_A_GPIO_Port GPIOB
#define TENSION_ENC_B_Pin GPIO_PIN_14
#define TENSION_ENC_B_GPIO_Port GPIOB
#define TENSION_ENC_A_Pin GPIO_PIN_15
#define TENSION_ENC_A_GPIO_Port GPIOB
#define KNIFECLAMP_ENC_B_Pin GPIO_PIN_10
#define KNIFECLAMP_ENC_B_GPIO_Port GPIOD
#define KNIFECLAMP_ENC_A_Pin GPIO_PIN_11
#define KNIFECLAMP_ENC_A_GPIO_Port GPIOD
#define YAW_ENC_B_Pin GPIO_PIN_12
#define YAW_ENC_B_GPIO_Port GPIOD
#define YAW_ENC_A_Pin GPIO_PIN_13
#define YAW_ENC_A_GPIO_Port GPIOD
#define SCLAMP2_ENC_B_Pin GPIO_PIN_14
#define SCLAMP2_ENC_B_GPIO_Port GPIOD
#define SCLAMP2_ENC_A_Pin GPIO_PIN_15
#define SCLAMP2_ENC_A_GPIO_Port GPIOD
#define PITCH_ENC_B_Pin GPIO_PIN_6
#define PITCH_ENC_B_GPIO_Port GPIOC
#define PITCH_ENC_A_Pin GPIO_PIN_7
#define PITCH_ENC_A_GPIO_Port GPIOC
#define SCLAMP1_ENC_B_Pin GPIO_PIN_8
#define SCLAMP1_ENC_B_GPIO_Port GPIOC
#define SCLAMP1_ENC_A_Pin GPIO_PIN_9
#define SCLAMP1_ENC_A_GPIO_Port GPIOC
#define RAISE2_STEP_Pin GPIO_PIN_15
#define RAISE2_STEP_GPIO_Port GPIOA
#define RAISE2_DIR_Pin GPIO_PIN_10
#define RAISE2_DIR_GPIO_Port GPIOC
#define RAISE2_NFAULT_Pin GPIO_PIN_11
#define RAISE2_NFAULT_GPIO_Port GPIOC
#define BEVEL_DIR_Pin GPIO_PIN_12
#define BEVEL_DIR_GPIO_Port GPIOC
#define BEVEL_STEP_Pin GPIO_PIN_0
#define BEVEL_STEP_GPIO_Port GPIOD
#define UNDERPASS_DIR_Pin GPIO_PIN_1
#define UNDERPASS_DIR_GPIO_Port GPIOD
#define UNDERPASS_STEP_Pin GPIO_PIN_2
#define UNDERPASS_STEP_GPIO_Port GPIOD
#define PITCH_M_IN_B_Pin GPIO_PIN_3
#define PITCH_M_IN_B_GPIO_Port GPIOD
#define PITCH_M_IN_A_Pin GPIO_PIN_4
#define PITCH_M_IN_A_GPIO_Port GPIOD
#define SCLAMP2_M_IN_B_Pin GPIO_PIN_6
#define SCLAMP2_M_IN_B_GPIO_Port GPIOD
#define SCLAMP2_M_IN_A_Pin GPIO_PIN_7
#define SCLAMP2_M_IN_A_GPIO_Port GPIOD
#define SCLAMP1_M_IN_B_Pin GPIO_PIN_4
#define SCLAMP1_M_IN_B_GPIO_Port GPIOB
#define SCLAMP1_M_IN_A_Pin GPIO_PIN_5
#define SCLAMP1_M_IN_A_GPIO_Port GPIOB
#define TENSION_M_IN_B_Pin GPIO_PIN_6
#define TENSION_M_IN_B_GPIO_Port GPIOB
#define TENSION_M_IN_A_Pin GPIO_PIN_7
#define TENSION_M_IN_A_GPIO_Port GPIOB
#define ROLL_M_IN_B_Pin GPIO_PIN_8
#define ROLL_M_IN_B_GPIO_Port GPIOB
#define ROLL_M_IN_A_Pin GPIO_PIN_9
#define ROLL_M_IN_A_GPIO_Port GPIOB
#define SPOOL_STEP_Pin GPIO_PIN_0
#define SPOOL_STEP_GPIO_Port GPIOE
#define SPOOL_DIR_Pin GPIO_PIN_1
#define SPOOL_DIR_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

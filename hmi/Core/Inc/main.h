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
#include "stm32h7xx_hal.h"

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
#define RED_LED_Pin GPIO_PIN_2
#define RED_LED_GPIO_Port GPIOE
#define GREEN_LED_Pin GPIO_PIN_3
#define GREEN_LED_GPIO_Port GPIOE
#define SELECT_BTTN_Pin GPIO_PIN_12
#define SELECT_BTTN_GPIO_Port GPIOB
#define RIGHT_BTTN_Pin GPIO_PIN_13
#define RIGHT_BTTN_GPIO_Port GPIOB
#define LEFT_BTTN_Pin GPIO_PIN_14
#define LEFT_BTTN_GPIO_Port GPIOB
#define DOWN_BTTN_Pin GPIO_PIN_15
#define DOWN_BTTN_GPIO_Port GPIOB
#define UP_BTTN_Pin GPIO_PIN_8
#define UP_BTTN_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOD
#define DISP_Pin GPIO_PIN_8
#define DISP_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define O8A_Pin GPIO_PIN_13
#define O8A_GPIO_Port GPIOC
#define O8B_Pin GPIO_PIN_14
#define O8B_GPIO_Port GPIOC
#define O1B_Pin GPIO_PIN_11
#define O1B_GPIO_Port GPIOB
#define O1A_Pin GPIO_PIN_12
#define O1A_GPIO_Port GPIOB
#define O2B_Pin GPIO_PIN_13
#define O2B_GPIO_Port GPIOB
#define O2A_Pin GPIO_PIN_14
#define O2A_GPIO_Port GPIOB
#define O3B_Pin GPIO_PIN_15
#define O3B_GPIO_Port GPIOB
#define O3A_Pin GPIO_PIN_8
#define O3A_GPIO_Port GPIOA
#define O4B_Pin GPIO_PIN_15
#define O4B_GPIO_Port GPIOA
#define O4A_Pin GPIO_PIN_3
#define O4A_GPIO_Port GPIOB
#define O5B_Pin GPIO_PIN_4
#define O5B_GPIO_Port GPIOB
#define O5A_Pin GPIO_PIN_5
#define O5A_GPIO_Port GPIOB
#define O6B_Pin GPIO_PIN_6
#define O6B_GPIO_Port GPIOB
#define O6A_Pin GPIO_PIN_7
#define O6A_GPIO_Port GPIOB
#define O7B_Pin GPIO_PIN_8
#define O7B_GPIO_Port GPIOB
#define O7A_Pin GPIO_PIN_9
#define O7A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

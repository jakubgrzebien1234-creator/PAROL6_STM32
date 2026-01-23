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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define J2_Pin GPIO_PIN_6
#define J2_GPIO_Port GPIOA
#define J2_EXTI_IRQn EXTI9_5_IRQn
#define J3_Pin GPIO_PIN_7
#define J3_GPIO_Port GPIOA
#define J3_EXTI_IRQn EXTI9_5_IRQn
#define J5_Pin GPIO_PIN_4
#define J5_GPIO_Port GPIOC
#define J5_EXTI_IRQn EXTI4_IRQn
#define J6_Pin GPIO_PIN_5
#define J6_GPIO_Port GPIOC
#define J6_EXTI_IRQn EXTI9_5_IRQn
#define J4_Pin GPIO_PIN_0
#define J4_GPIO_Port GPIOB
#define J4_EXTI_IRQn EXTI0_IRQn
#define J1_Pin GPIO_PIN_1
#define J1_GPIO_Port GPIOB
#define J1_EXTI_IRQn EXTI1_IRQn
#define PUMP_Pin GPIO_PIN_12
#define PUMP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

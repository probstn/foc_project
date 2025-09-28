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
#include "stm32f1xx_hal.h"

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
#define CURR_A_Pin GPIO_PIN_0
#define CURR_A_GPIO_Port GPIOA
#define CURR_B_Pin GPIO_PIN_2
#define CURR_B_GPIO_Port GPIOA
#define CURR_C_Pin GPIO_PIN_4
#define CURR_C_GPIO_Port GPIOA
#define VBAT_Pin GPIO_PIN_6
#define VBAT_GPIO_Port GPIOA
#define HALL_A_Pin GPIO_PIN_0
#define HALL_A_GPIO_Port GPIOB
#define HALL_A_EXTI_IRQn EXTI0_IRQn
#define HALL_B_Pin GPIO_PIN_1
#define HALL_B_GPIO_Port GPIOB
#define HALL_B_EXTI_IRQn EXTI1_IRQn
#define HALL_C_Pin GPIO_PIN_2
#define HALL_C_GPIO_Port GPIOB
#define HALL_C_EXTI_IRQn EXTI2_IRQn
#define DEBUG_Pin GPIO_PIN_6
#define DEBUG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

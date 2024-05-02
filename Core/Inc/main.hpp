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
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
}
#endif
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <functional>

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
void Error_Handler(uint8_t errorCode = 0);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define CAN1_CS_Pin GPIO_PIN_0
#define CAN1_CS_GPIO_Port GPIOA
#define CAN2_CS_Pin GPIO_PIN_1
#define CAN2_CS_GPIO_Port GPIOA
#define CAN3_CS_Pin GPIO_PIN_2
#define CAN3_CS_GPIO_Port GPIOA
#define CAN4_CS_Pin GPIO_PIN_3
#define CAN4_CS_GPIO_Port GPIOA
#define CAN1_INT_Pin GPIO_PIN_0
#define CAN1_INT_GPIO_Port GPIOB
#define CAN1_INT_EXTI_IRQn EXTI0_IRQn
#define CAN2_INT_Pin GPIO_PIN_1
#define CAN2_INT_GPIO_Port GPIOB
#define CAN2_INT_EXTI_IRQn EXTI1_IRQn
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOA
#define CAN3_INT_Pin GPIO_PIN_3
#define CAN3_INT_GPIO_Port GPIOB
#define CAN3_INT_EXTI_IRQn EXTI3_IRQn
#define CAN4_INT_Pin GPIO_PIN_4
#define CAN4_INT_GPIO_Port GPIOB
#define CAN4_INT_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#endif /* __MAIN_H */

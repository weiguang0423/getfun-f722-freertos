/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define MOTOR8_Pin GPIO_PIN_10
#define MOTOR8_GPIO_Port GPIOB
#define MOTOR7_Pin GPIO_PIN_11
#define MOTOR7_GPIO_Port GPIOB
#define MOTOR6_Pin GPIO_PIN_8
#define MOTOR6_GPIO_Port GPIOC
#define MOTOR5_Pin GPIO_PIN_9
#define MOTOR5_GPIO_Port GPIOC
#define MOTOR4_Pin GPIO_PIN_8
#define MOTOR4_GPIO_Port GPIOA
#define MOTOR3_Pin GPIO_PIN_9
#define MOTOR3_GPIO_Port GPIOA
#define MOTOR2_Pin GPIO_PIN_10
#define MOTOR2_GPIO_Port GPIOA
#define MOTOR1_Pin GPIO_PIN_15
#define MOTOR1_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

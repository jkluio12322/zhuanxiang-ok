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
#define pwm1_1_Pin GPIO_PIN_1
#define pwm1_1_GPIO_Port GPIOF
#define pwm1_0_Pin GPIO_PIN_2
#define pwm1_0_GPIO_Port GPIOF
#define pwm2_1_Pin GPIO_PIN_3
#define pwm2_1_GPIO_Port GPIOF
#define pwm2_0_Pin GPIO_PIN_4
#define pwm2_0_GPIO_Port GPIOF
#define IMU963RA_INT_Pin GPIO_PIN_15
#define IMU963RA_INT_GPIO_Port GPIOE
#define K3_Pin GPIO_PIN_14
#define K3_GPIO_Port GPIOD
#define K4_Pin GPIO_PIN_15
#define K4_GPIO_Port GPIOD
#define K1_Pin GPIO_PIN_2
#define K1_GPIO_Port GPIOG
#define K2_Pin GPIO_PIN_3
#define K2_GPIO_Port GPIOG
#define buzzer_Pin GPIO_PIN_6
#define buzzer_GPIO_Port GPIOG
#define IR2_Pin GPIO_PIN_3
#define IR2_GPIO_Port GPIOD
#define IR3_Pin GPIO_PIN_4
#define IR3_GPIO_Port GPIOD
#define IR4_Pin GPIO_PIN_5
#define IR4_GPIO_Port GPIOD
#define IR5_Pin GPIO_PIN_6
#define IR5_GPIO_Port GPIOD
#define IR6_Pin GPIO_PIN_7
#define IR6_GPIO_Port GPIOD
#define IR7_Pin GPIO_PIN_9
#define IR7_GPIO_Port GPIOG
#define IR8_Pin GPIO_PIN_10
#define IR8_GPIO_Port GPIOG
#define IR1_Pin GPIO_PIN_11
#define IR1_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

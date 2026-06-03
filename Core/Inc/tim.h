/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** TIM1: PWM 电机驱动 (PE9=CH1, PE11=CH2), 244Hz */
extern TIM_HandleTypeDef htim1;

/** TIM2: 编码器模式, 左轮 (PA0=CH1, PA1=CH2) */
extern TIM_HandleTypeDef htim2;

/** TIM3: 编码器模式, 右轮 (PA6=CH1, PA7=CH2) */
extern TIM_HandleTypeDef htim3;

/** TIM5: millis 计数器, 1kHz (TIM5_IRQn) */
extern TIM_HandleTypeDef htim5;

/** TIM6: 周期标志发生器, 1kHz (TIM6_IRQn) */
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM5_Init(void);
void MX_TIM6_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */


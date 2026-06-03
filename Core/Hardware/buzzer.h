/**
 * @file    buzzer.h
 * @brief   有源蜂鸣器控制模块头文件
 * @note    使用 PG6, 低电平开启
 */

#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H
#include "main.h"

/** 蜂鸣器引脚定义 */
#define BUZZER_PORT  GPIOG
#define BUZZER_PIN   GPIO_PIN_6

void Buzzer_Init(void);
void Buzzer_ON(void);
void Buzzer_OFF(void);
void Buzzer_Turn(void);

#endif

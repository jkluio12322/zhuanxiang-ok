/**
 * @file    buzzer.h
 * @brief   有源蜂鸣器控制模块头文件
 * @note    使用 PG6, 低电平开启
 */

#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H  /* 头文件保护宏，防止蜂鸣器接口重复声明。 */
#include "main.h"

/* 蜂鸣器控制 GPIO 端口。 */
#define BUZZER_PORT  GPIOG

/* 蜂鸣器控制 GPIO 引脚，当前使用 PG6。 */
#define BUZZER_PIN   GPIO_PIN_6

/**
 * @brief  初始化蜂鸣器并保持关闭状态。
 */
void Buzzer_Init(void);

/**
 * @brief  打开蜂鸣器。
 */
void Buzzer_ON(void);

/**
 * @brief  关闭蜂鸣器。
 */
void Buzzer_OFF(void);

/**
 * @brief  翻转蜂鸣器当前开关状态。
 */
void Buzzer_Turn(void);

#endif

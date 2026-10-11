/* 有源蜂鸣器：PG6，低电平开启 */

#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H  /* 头文件保护 */
#include "main.h"

#define BUZZER_PORT  GPIOG       /* GPIO 端口 */
#define BUZZER_PIN   GPIO_PIN_6  /* PG6 */

void Buzzer_Init(void);   /* 初始化并关闭 */
void Buzzer_ON(void);     /* 开 */
void Buzzer_OFF(void);    /* 关 */
void Buzzer_Turn(void);   /* 翻转 */

#endif

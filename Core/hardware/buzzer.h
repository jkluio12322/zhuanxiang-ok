/* 有源蜂鸣器：PG6，低电平开启 */

#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H  /* 头文件保护 */
#include "main.h"

#define BUZZER_PORT  GPIOG       /* GPIO 端口 */
#define BUZZER_PIN   GPIO_PIN_6  /* PG6 */

void buzzer_init(void);   /* 初始化并关闭 */
void buzzer_on(void);     /* 开 */
void buzzer_off(void);    /* 关 */
void buzzer_turn(void);   /* 翻转 */

#endif

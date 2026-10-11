/* TB6612 双路直流电机驱动头文件 */

#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H  /* 头文件保护 */
#include "main.h"

void Motor_Init(void);               /* 启动 TIM1_CH1/CH2 PWM */
void Motor_SetSpeedA(int16_t Speed); /* A 轮：正转/负反转/0 停 */
void Motor_SetSpeedB(int16_t Speed); /* B 轮：正转/负反转/0 停 */

#endif

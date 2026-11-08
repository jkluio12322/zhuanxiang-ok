/* TB6612 双路直流电机驱动头文件 */

#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H  /* 头文件保护 */
#include "main.h"

void motor_init(void);               /* 启动 TIM1_CH1/CH2 PWM */
void motor_a_set(float speed_percent); /* A 轮：小数百分比，正转/负反转/0 停 */
void motor_b_set(float speed_percent); /* B 轮：小数百分比，正转/负反转/0 停 */

#endif

/**
 * @file    motor.h
 * @brief   TB6612 双路直流电机驱动模块头文件
 */

#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H  /* 头文件保护宏，防止电机接口重复声明。 */
#include "main.h"

/**
 * @brief  初始化两路电机 PWM 输出。
 * @note   启动 TIM1_CH1 和 TIM1_CH2，方向引脚由 GPIO 初始化阶段配置。
 */
void Motor_Init(void);

/**
 * @brief  设置 A 轮速度和方向。
 * @param  Speed PWM 百分比，正数正转，负数反转，0 停止。
 */
void Motor_SetSpeedA(int16_t Speed);

/**
 * @brief  设置 B 轮速度和方向。
 * @param  Speed PWM 百分比，正数正转，负数反转，0 停止。
 */
void Motor_SetSpeedB(int16_t Speed);

#endif

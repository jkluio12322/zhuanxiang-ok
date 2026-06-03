/**
 * @file    motor.h
 * @brief   TB6612 双路直流电机驱动模块头文件
 */

#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H
#include "main.h"

void Motor_Init(void);
void Motor_SetSpeedA(int16_t Speed);
void Motor_SetSpeedB(int16_t Speed);

#endif

/**
 * @file    timer.h
 * @brief   系统定时器模块头文件
 * @note    TIM5 驱动毫秒计数，TIM6 调度应用层周期任务。
 */

#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H  /* 头文件保护宏，防止接口重复声明。 */
#include "main.h"

/** 毫秒时间戳 (TIM5 驱动, 1ms 精度, 持续递增) */
extern volatile uint64_t milliseconds;

/**
 * @brief  初始化并启动 TIM6 周期任务调度。
 * @note   TIM6 负责产生 10 ms 传感器任务、20 ms 按键任务和 50 ms 控制任务。
 */
void Timer_Init(void);

/**
 * @brief  初始化并启动 TIM5 毫秒计数。
 * @note   TIM5 每 1 ms 递增 milliseconds，用于超时和显示周期判断。
 */
void TIM5_Init(void);

#endif

/**
 * @file    timer.h
 * @brief   系统定时器模块头文件
 * @note    TIM5 驱动 millis 计数器 (1ms)
 *          TIM6 驱动 flag_10ms / flag_50ms 周期性标志
 */

#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H
#include "main.h"

/** 毫秒时间戳 (TIM5 驱动, 1ms 精度, 持续递增) */
extern volatile uint64_t millis;

/** 10ms 周期标志 (TIM6 驱动, 主循环检测后手动清零) */
extern volatile uint8_t flag_10ms;

/** 50ms 周期标志 (TIM6 驱动, 主循环检测后手动清零) */
extern volatile uint8_t flag_50ms;

void Timer_Init(void);
void TIM5_Init(void);

#endif

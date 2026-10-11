/* 系统定时器：TIM5 驱动毫秒计数，TIM6 调度应用层周期任务 */

#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H  /* 头文件保护 */
#include "main.h"

extern volatile uint64_t milliseconds;   /* 毫秒时间戳 (TIM5 驱动, 1ms 精度, 持续递增) */

void timer_sched_init(void);   /* 启动 TIM6 中断，供应用层分频调度 */
void timer_ms_init(void);    /* 启动 TIM5 毫秒计数 */

#endif

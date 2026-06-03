/**
 * @file    timer.c
 * @brief   系统定时器模块 (毫秒计数器 + 周期性标志)
 * @note    使用 TIM6 产生周期性标志, TIM5 产生毫秒时间戳
 *          - TIM5: 72MHz / 72 / 1000 = 1kHz → millis 计数器 (每 1ms 递增)
 *          - TIM6: 72MHz / 72 / 1000 = 1kHz → flag_10ms / flag_50ms 周期标志
 */

#include "timer.h"
#include "key.h"

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim5;

/** 毫秒计数器 (TIM5 驱动, 每 1ms +1) */
volatile uint64_t millis = 0;

/** 周期性标志位: 主循环每 10ms 轮询 */
volatile uint8_t flag_10ms = 0;

/** 周期性标志位: 主循环每 50ms 轮询 */
volatile uint8_t flag_50ms = 0;

/**
 * @brief  启动 TIM6 (1kHz 时基, 产生 flag_10ms/flag_50ms)
 */
void Timer_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief  启动 TIM5 (1kHz 时基, 驱动 millis 计数器)
 */
void TIM5_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief  定时器周期中断回调 (HAL 框架调用)
 * @param  htim 触发中断的定时器句柄
 * @note   TIM5: millis 自增
 *         TIM6: 产生 10ms 和 50ms 周期标志供主循环使用
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5) {
        millis++;  /* 每 1ms 递增 */
    }
    if (htim->Instance == TIM6) {
        Key_Tick();  /* 按键状态机, 每 1ms 推进 */

        /* 分频器: 每 1ms 触发一次, 累积产生 10ms/50ms 标志 */
        static uint8_t tick_10ms = 0;
        static uint8_t tick_50ms = 0;
        if (++tick_10ms >= 10) {
            tick_10ms = 0;
            flag_10ms = 1;  /* 主循环检测后需手动清零 */
        }
        if (++tick_50ms >= 50) {
            tick_50ms = 0;
            flag_50ms = 1;  /* 主循环检测后需手动清零 */
        }
    }
}

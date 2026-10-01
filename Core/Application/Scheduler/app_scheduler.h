/**
 * @file    app_scheduler.h
 * @brief   应用层任务调度：TIM6 中断分发 + 延后任务表。
 * @note    硬实时任务（控制环、循迹、按键、姿态）在 TIM6 中断里直接执行；
 *          耗时任务（遥测的 snprintf、OLED 的 I2C）只置一个待办标志，
 *          由主循环里的 AppDeferred_Run() 执行。
 */
#ifndef __APP_SCHEDULER_H
#define __APP_SCHEDULER_H

#include <stdint.h>

/* 延后任务编号。 */
typedef enum {
    DEFERRED_TELEMETRY = 0,   /* 遥测格式化与发送 */
    DEFERRED_DISPLAY,         /* OLED 状态页刷新 */
    DEFERRED_COUNT
} AppDeferredId_t;

/**
 * @brief  「连续 N 次成立」的确认计数器。
 * @note   取代原来散落的裸 tick 计数器：调用方不再自己 ++ / -- / 比较阈值。
 */
typedef struct {
    uint16_t need;   /* 需要的连续次数。 */
    uint16_t count;  /* 当前已连续次数。 */
} ConfirmCounter;

/**
 * @brief  复位确认计数器并设定需要的连续次数。
 */
void Confirm_Reset(ConfirmCounter *c, uint16_t need);

/**
 * @brief  推进一次确认。
 * @param  c               计数器。
 * @param  condition_true  本次条件是否成立。
 * @return 1 表示已经连续成立 need 次。
 * @note   条件不成立时计数清零；到达 need 后保持在 need。
 */
uint8_t Confirm_Update(ConfirmCounter *c, uint8_t condition_true);

/**
 * @brief  初始化调度器并放行中断分发。
 * @note   必须在所有模块的 *_Init() 之后、Timer_Init() 之前调用。
 */
void AppScheduler_Init(void);

/**
 * @brief  TIM6 1 ms 中断入口，分频调度各周期任务。
 * @note   在中断上下文中执行，只允许调用耗时很短的函数。
 */
void AppScheduler_OnTick(void);

/**
 * @brief  执行所有已置位的延后任务。
 * @note   主循环唯一入口。
 */
void AppDeferred_Run(void);

/**
 * @brief  请求执行一个延后任务。
 * @param  id 延后任务编号。
 * @note   只置标志，不排队；同一个任务重复请求只执行一次。
 */
void AppScheduler_RequestDeferred(AppDeferredId_t id);

/**
 * @brief  取走一个延后任务的待办标志。
 * @param  id 延后任务编号。
 * @return 1 表示该任务待办且标志已清除，0 表示无待办。
 */
uint8_t AppScheduler_TakeDeferred(AppDeferredId_t id);

#endif /* __APP_SCHEDULER_H */

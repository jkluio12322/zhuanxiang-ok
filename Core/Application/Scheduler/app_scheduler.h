/* 应用层任务调度：TIM6 中断分发 + 延后任务表。
   硬实时任务（控制环、循迹、按键、姿态）在 TIM6 中断里直接执行；
   耗时任务（遥测的 snprintf、OLED 的 I2C）只置一个待办标志，由主循环的 AppDeferred_Run() 执行。 */
#ifndef __APP_SCHEDULER_H
#define __APP_SCHEDULER_H

#include <stdint.h>

/* 延后任务编号 */
typedef enum {
    DEFERRED_TELEMETRY = 0,   /* 遥测格式化与发送 */
    DEFERRED_DISPLAY,         /* OLED 状态页刷新 */
    DEFERRED_COUNT
} AppDeferredId_t;

/* 「连续 N 次成立」的确认计数器，取代原来散落的裸 tick 计数器。 */
typedef struct {
    uint16_t need;   /* 需要的连续次数 */
    uint16_t count;  /* 当前已连续次数 */
} ConfirmCounter;

void Confirm_Reset(ConfirmCounter *c, uint16_t need);   /* 复位计数器并设定需要的连续次数 */

/* 推进一次确认；1 = 已连续成立 need 次。条件不成立时计数清零，到达 need 后保持。 */
uint8_t Confirm_Update(ConfirmCounter *c, uint8_t condition_true);

/* 初始化调度器并放行中断分发。必须在所有 *_Init() 之后、Timer_Init() 之前调用。 */
void AppScheduler_Init(void);

/* TIM6 的 1 ms 中断入口，分频调度各周期任务。中断上下文，只能调耗时很短的函数。 */
void AppScheduler_OnTick(void);

void AppDeferred_Run(void);   /* 执行所有已置位的延后任务，主循环唯一入口 */

/* 请求执行一个延后任务。只置标志不排队；同一任务重复请求也只执行一次。 */
void AppScheduler_RequestDeferred(AppDeferredId_t id);

/* 取走延后任务的待办标志；1 = 待办且标志已清除，0 = 无待办。 */
uint8_t AppScheduler_TakeDeferred(AppDeferredId_t id);

#endif

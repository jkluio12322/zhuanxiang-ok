/* 毫秒计时与任务调度中断入口：TIM5 出 1 ms 时间戳，TIM6 每秒 1000 次调用 AppScheduler_OnTick()。
   本文件不直接调用任何应用任务。 */

#include "timer.h"
#include "app_scheduler.h"

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim5;

volatile uint64_t milliseconds = 0;   /* 溢出周期约 49.7 天 */

void Timer_Init(void)          /* 启动 TIM6 中断，供应用层分频调度 */
{
    HAL_TIM_Base_Start_IT(&htim6);
}

void TIM5_Init(void)           /* 启动 TIM5 中断，供 1 ms 毫秒计时 */
{
    HAL_TIM_Base_Start_IT(&htim5);
}

/* HAL 定时器周期中断统一回调。只做两件事：累加毫秒计数、把调度交给 app_scheduler。
   遥测格式化、OLED 刷新等耗时操作都由主循环的延后任务执行。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5) {
        milliseconds++;
    } else if (htim->Instance == TIM6) {
        AppScheduler_OnTick();
    }
}

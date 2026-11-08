/* 双时基入口：TIM4 出 2 ms 采样拍，TIM7 出 5 ms 控制拍，TIM5 只管毫秒计时。
   本文件不直接调用任何应用任务，全部交给 sched 的两个 ISR 入口。
   两个时基同源于 72 MHz 并经各自 72 分频，所以它们之间没有累积漂移，
   只有启动时刻相差几微秒的固定相位差 —— 这个相位差不参与任何计算。 */

#include "timer.h"
#include "sched.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;

volatile uint64_t milliseconds = 0;   /* 64 位毫秒，实际使用中不会溢出 */

/* 启动两个调度时基。必须先清 CNT 再开中断：否则第一拍的 CNT 探针读到的是随机值。
   TIM4 先启动、TIM7 后启动，保证「控制拍首次进入时采样拍已经跑过至少一次」。 */
void timer_sched_init(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    __HAL_TIM_SET_COUNTER(&htim7, 0U);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim7);
}

void timer_ms_init(void)           /* 启动 TIM5 中断，供 1 ms 毫秒计时 */
{
    HAL_TIM_Base_Start_IT(&htim5);
}

/* HAL 定时器周期中断统一回调。按 handle 分发到各自的 ISR 入口，
   这里不做任何业务判断，业务全部在 sched.c 里。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        sched_sample_isr();
    } else if (htim->Instance == TIM7) {
        sched_control_isr();
    } else if (htim->Instance == TIM5) {
        milliseconds++;
    }
}

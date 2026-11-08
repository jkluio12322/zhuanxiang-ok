/* BlueCar 任务编排：TIM4 2 ms 采样拍 + TIM7 5 ms 控制拍，两个硬件时基各自独立中断。
 * 本模块是全工程唯一的任务编排点（有意越层调用 bsp/app 各层）。 */
#ifndef __SCHED_H
#define __SCHED_H

#include <stdint.h>

#define SCHED_FLAG_TELEMETRY  0x01U   /* 主循环待发 VOFA 帧 */
#define SCHED_FLAG_DISPLAY    0x02U   /* 主循环待刷 OLED */
#define SCHED_FLAG_IMU        0x04U   /* 主循环采一次 IMU（SPI 忙等不能在中断里） */

void sched_init(void);          /* 必须在 timer_sched_init() 之前调用 */
void sched_sample_isr(void);    /* TIM4 周期中断入口：只读编码器、推进测速窗 */
void sched_control_isr(void);   /* TIM7 周期中断入口：红外 → 循迹 → 速度环 → 电机 */
uint8_t sched_take_flags(void); /* 读并清零延后任务标志（临界区保护） */

/* DWT->CYCCNT 换算的微秒时间戳（72 MHz，13.9 ns 分辨率，59.6 s 回绕）。
   所有需要 dt 的模块都用它，不要用「假设的周期常量」。 */
uint32_t sched_now_us(void);

/* IMU 标志被置位那一刻的时间戳。主循环拿它算「置位 → 真正采样」的排队延迟：
   这个值一大，说明主循环里别的事情（OLED/串口/命令解析）在拖后腿。 */
uint32_t sched_imu_mark_us(void);

#endif /* __SCHED_H */

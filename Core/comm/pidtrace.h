/* 转向环数据采集：把转向环的偏差/微分/陀螺/输出打成定长二进制帧，50 Hz 从 USART1 送出。
   只做观测，不参与控制，也不改任何增益；帧格式见 pidtrace.c 顶部说明。 */
#ifndef __PIDTRACE_H
#define __PIDTRACE_H

#include <stdint.h>

void pidtrace_init(void);

/* 主循环调用：内部按 state()->tick_count（5 ms 控制拍）分频到 TASK_TRACE_MS，
   同一拍不会重复发，主循环慢下来导致漏发时帧序号会出现缺口，主机据此统计丢帧。 */
void pidtrace_task(void);

uint8_t pidtrace_on(void);
void pidtrace_set_on(uint8_t on);
uint8_t pidtrace_inject(void);
void pidtrace_set_inject(uint8_t on);

uint32_t pidtrace_frames(void);    /* 已发出帧数 */
uint32_t pidtrace_skipped(void);   /* 发送队列装不下整帧而丢弃的次数 */

#endif /* __PIDTRACE_H */
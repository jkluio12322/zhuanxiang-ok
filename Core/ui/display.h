/* OLED 状态页：红外 8 路、IMU 三角度、圈数、PID 目标与实际、调度耗时。 */
#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <stdint.h>

void display_task(void);     /* 主循环调用，100 ms 一次 */
void display_boot(void);

#endif /* __DISPLAY_H */

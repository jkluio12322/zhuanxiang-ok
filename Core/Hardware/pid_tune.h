/**
 * @file    pid_tune.h
 * @brief   PID 串口在线调参 (USART3 RX, 115200bps)
 * @note    命令格式:
 *            skp 2.0  → 速度环 Kp
 *            ski 0.5  → 速度环 Ki
 *            skd 0.1  → 速度环 Kd
 *            tkp 1.0  → 转向环 Kp
 *            tki 0.3  → 转向环 Ki
 *            tkd 0.05 → 转向环 Kd
 *            show     → 显示当前参数
 */
#ifndef __PID_TUNE_H
#define __PID_TUNE_H
#include "main.h"

void PIDTune_Init(void);
void PIDTune_RxCallback(uint8_t byte);

#endif

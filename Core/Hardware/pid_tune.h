/**
 * @file    pid_tune.h
 * @brief   PID 串口在线调参 — 串级 PID (USART1 RX, 115200bps)
 * @note    PIDTune_RxCallback 仅在 ISR 中缓冲字节,
 *          PIDTune_Process 在主循环中解析命令 (避免 ISR 竞争)
 */

#ifndef __PID_TUNE_H
#define __PID_TUNE_H
#include "main.h"

extern volatile uint8_t pid_tune_rx_byte;
extern volatile uint8_t tune_rsp_pending;
extern volatile uint8_t tune_line_ready;
extern char           tune_rsp_buf[128];
extern char           tune_line[32];

void PIDTune_Init(void);
void PIDTune_RxCallback(uint8_t byte);   /* ISR: 缓冲字节 */
void PIDTune_Process(void);              /* 主循环: 解析命令 */

#endif

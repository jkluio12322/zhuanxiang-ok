/**
 * @file    pid_tune.h
 * @brief   PID 串口在线调参 + 电机控制 — 串级 PID (USART3 RX, 115200bps)
 *
 *          内环 SpeedPID (50ms) + 外环 YawPID (100ms) + 手动差速
 *
 *          限制参数 (show 命令可查当前值, 修改需直接改 main.c):
 *            SpeedPID: OutOffset=15 IntegralLimit=30 Deadband=3 RateLimit=20
 *            YawPID:   OutOffset=0  IntegralLimit=40 Deadband=1 RateLimit=10
 *
 *          可调参数:
 *            skp/ski/skd N  → 内环 SpeedPID Kp/Ki/Kd
 *            ykp/yki/ykd N  → 外环 YawPID   Kp/Ki/Kd
 *            soff/sdb/srl N → 内环 OutOffset/Deadband/OutputRateLimit
 *            t N            → 速度目标 RPM
 *            yaw N          → 航向目标 °
 *            turn N         → 手动差速 RPM (直接叠加)
 *            start/stop     → 启停
 *            duty N         → 开环 PWM (-100~100)
 *            show           → 查看所有参数
 */
#ifndef __PID_TUNE_H
#define __PID_TUNE_H
#include "main.h"

extern volatile uint8_t pid_tune_rx_byte;
extern volatile uint8_t tune_rsp_pending;
extern char           tune_rsp_buf[128];

void PIDTune_Init(void);
void PIDTune_RxCallback(uint8_t byte);

#endif

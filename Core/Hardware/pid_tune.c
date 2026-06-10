/**
 * @file    pid_tune.c
 * @brief   PID 串口在线调参 + 电机控制 (串级 PID)
 * @note    通过 USB-TTL 接 USART3 (PD8=TX, PD9=RX), 115200bps
 *
 *          控制架构:
 *            内环 SpeedPID (50ms): 平均转速 → base PWM
 *              限制: OutMax±100 OutOffset=15 IntegralLimit=30 Deadband=3 RateLimit=20
 *
 *            外环 YawPID (100ms):  航向角 → diff_yaw
 *              限制: OutMax±20 OutOffset=0 IntegralLimit=40 Deadband=1 RateLimit=10
 *
 *            手动差速: TurnTarget 直接叠加到电机 (不走PID, 响应即时)
 *
 *          可用命令:
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

#include "pid_tune.h"
#include "pid.h"
#include "motor.h"
#include "serial.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern PID_t      SpeedPID;
extern PID_t      YawPID;
extern UART_HandleTypeDef huart3;
extern uint8_t    RunFlag;
extern int16_t    PWMDuty;
extern int        Mode;
extern float      TurnTarget;
extern float      YawTarget;
extern uint32_t   bt_last_rx;
extern volatile uint64_t millis;

volatile uint8_t pid_tune_rx_byte;

static char    rx_line[32];
static uint8_t rx_idx = 0;

char    tune_rsp_buf[128];
volatile uint8_t tune_rsp_pending = 0;

static void tune_rsp(const char *fmt, ...)
{
    if (tune_rsp_pending) return;
    va_list arg;
    va_start(arg, fmt);
    vsnprintf(tune_rsp_buf, sizeof(tune_rsp_buf), fmt, arg);
    va_end(arg);
    tune_rsp_pending = 1;
}

static void PIDTune_Parse(char *cmd)
{
    char  tok[16];
    float val = 0.0f;
    int   n   = 0;

    while (*cmd == ' ') cmd++;
    n = sscanf(cmd, "%15s %f", tok, &val);

    /* ---- 内环 SpeedPID ---- */
    if (strcmp(tok, "skp") == 0 && n >= 2) {
        SpeedPID.Kp = val;
        tune_rsp("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                 SpeedPID.Kp, SpeedPID.Ki, SpeedPID.Kd);
    }
    else if (strcmp(tok, "ski") == 0 && n >= 2) {
        SpeedPID.Ki = val;
        tune_rsp("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                 SpeedPID.Kp, SpeedPID.Ki, SpeedPID.Kd);
    }
    else if (strcmp(tok, "skd") == 0 && n >= 2) {
        SpeedPID.Kd = val;
        tune_rsp("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                 SpeedPID.Kp, SpeedPID.Ki, SpeedPID.Kd);
    }

    /* ---- 外环 YawPID ---- */
    else if (strcmp(tok, "ykp") == 0 && n >= 2) {
        YawPID.Kp = val;
        tune_rsp("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                 YawPID.Kp, YawPID.Ki, YawPID.Kd);
    }
    else if (strcmp(tok, "yki") == 0 && n >= 2) {
        YawPID.Ki = val;
        tune_rsp("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                 YawPID.Kp, YawPID.Ki, YawPID.Kd);
    }
    else if (strcmp(tok, "ykd") == 0 && n >= 2) {
        YawPID.Kd = val;
        tune_rsp("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                 YawPID.Kp, YawPID.Ki, YawPID.Kd);
    }

    /* ---- 电机控制 ---- */
    else if (strcmp(tok, "t") == 0 && n >= 2) {
        PID_SetTarget(&SpeedPID, val);
        RunFlag = 1;
        Mode = 0;
        bt_last_rx = millis;
        tune_rsp("[Motor] SpeedTarget=%.1f RPM, started\r\n", val);
    }
    else if (strcmp(tok, "yaw") == 0 && n >= 2) {
        YawTarget = val;
        PID_SetTarget(&YawPID, val);
        bt_last_rx = millis;
        tune_rsp("[Motor] YawTarget=%.1f deg\r\n", val);
    }
    else if (strcmp(tok, "turn") == 0 && n >= 2) {
        TurnTarget = val;
        bt_last_rx = millis;
        tune_rsp("[Motor] TurnDiff=%.1f RPM (manual)\r\n", val);
    }
    else if (strcmp(tok, "start") == 0) {
        RunFlag = 1;
        Mode = 0;
        bt_last_rx = millis;
        tune_rsp("[Motor] Started\r\n");
    }
    else if (strcmp(tok, "stop") == 0) {
        RunFlag = 0;
        Motor_SetSpeedA(0);
        Motor_SetSpeedB(0);
        PID_SetTarget(&SpeedPID, 0);
        PID_SetTarget(&YawPID, 0);
        TurnTarget = 0;
        YawTarget = 0;
        tune_rsp("[Motor] Stopped\r\n");
    }
    else if (strcmp(tok, "duty") == 0 && n >= 2) {
        PWMDuty = (int16_t)val;
        if (PWMDuty >  100) PWMDuty =  100;
        if (PWMDuty < -100) PWMDuty = -100;
        RunFlag = (PWMDuty != 0);
        Mode = 1;
        bt_last_rx = millis;
        tune_rsp("[Motor] Duty=%d%%\r\n", PWMDuty);
    }

    /* ---- SpeedPID 参数 ---- */
    else if (strcmp(tok, "soff") == 0 && n >= 2) {
        SpeedPID.OutOffset = val;
        tune_rsp("[SpeedPID] OutOffset=%.1f\r\n", val);
    }
    else if (strcmp(tok, "sdb") == 0 && n >= 2) {
        SpeedPID.Deadband = val;
        tune_rsp("[SpeedPID] Deadband=%.1f\r\n", val);
    }
    else if (strcmp(tok, "srl") == 0 && n >= 2) {
        SpeedPID.OutputRateLimit = val;
        tune_rsp("[SpeedPID] RateLimit=%.1f\r\n", val);
    }

    /* ---- 查询 ---- */
    else if (strcmp(tok, "show") == 0) {
        tune_rsp("=== Cascade PID ===\r\n"
                 "[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f  Tgt=%.1f Out=%.1f\r\n"
                 "           Off=%.1f DB=%.1f RL=%.1f Intgl=%.1f\r\n"
                 "[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f  Tgt=%.1f Out=%.1f\r\n"
                 "TurnDiff=%.1f  YawTarget=%.1f  Run=%d Mode=%d\r\n",
                 SpeedPID.Kp, SpeedPID.Ki, SpeedPID.Kd, SpeedPID.Kf,
                 SpeedPID.Target, SpeedPID.Out,
                 SpeedPID.OutOffset, SpeedPID.Deadband, SpeedPID.OutputRateLimit,
                 SpeedPID.IOut,
                 YawPID.Kp, YawPID.Ki, YawPID.Kd,
                 YawPID.Target, YawPID.Out,
                 TurnTarget, YawTarget, RunFlag, Mode);
    }
    else if (strcmp(tok, "help") == 0 || strcmp(tok, "?") == 0) {
        tune_rsp("=== Cascade PID ===\r\n"
                 " Inner:  skp/ski/skd N     (SpeedPID)\r\n"
                 "         soff/sdb/srl N    (Offset/Deadband/RateLimit)\r\n"
                 " Outer:  ykp/yki/ykd N     (YawPID)\r\n"
                 " Motor:  t N | yaw N | turn N\r\n"
                 "         start | stop | duty N\r\n"
                 " Info:   show\r\n");
    }
    else if (strlen(tok) > 0) {
        tune_rsp("? '%s'  (type 'help')\r\n", tok);
    }
}

void PIDTune_RxCallback(uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (rx_idx > 0) {
            rx_line[rx_idx] = '\0';
            PIDTune_Parse(rx_line);
            rx_idx = 0;
        }
    }
    else if (rx_idx < sizeof(rx_line) - 1) {
        rx_line[rx_idx++] = byte;
    }
}

void PIDTune_Init(void)
{
    rx_idx = 0;
    tune_rsp_pending = 0;
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&pid_tune_rx_byte, 1);
}

/**
 * @file    pid_tune.c
 * @brief   PID 串口在线调参 + 电机控制 (命令通过 CmdQueue 解耦)
 * @note    PIDTune_RxCallback: ISR 中仅缓冲字节到 tune_line
 *          PIDTune_Process:    主循环中解析 → CmdQueue_Put / 直接改 PID 参数
 *
 *          命令:
 *            skp/ski/skd/skf N → SpeedPID Kp/Ki/Kd/Kf (直接生效)
 *            ykp/yki/ykd/ykf N → YawPID   Kp/Ki/Kd/Kf (直接生效)
 *            t N / turn N / start / stop / duty N → CmdQueue → Control
 *            quiet / log       → CmdQueue → Control
 *            show / help       → tune_rsp
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
extern UART_HandleTypeDef huart1;
extern uint8_t    RunFlag;
extern int16_t    PWMDuty;
extern Mode_t     Mode;
extern float      TurnTarget;
extern float      YawTarget;
extern uint32_t   bt_last_rx;
extern uint8_t    VOFA_Enable;
extern volatile uint64_t millis;

volatile uint8_t pid_tune_rx_byte;
volatile uint8_t tune_rsp_pending = 0;
volatile uint8_t tune_line_ready  = 0;
char    tune_rsp_buf[128];
char    tune_line[32];
static uint8_t rx_idx = 0;

static void tune_rsp(const char *fmt, ...)
{
    if (tune_rsp_pending) return;
    va_list arg;
    va_start(arg, fmt);
    vsnprintf(tune_rsp_buf, sizeof(tune_rsp_buf), fmt, arg);
    va_end(arg);
    tune_rsp_pending = 1;
}

/**
 * @brief  串口命令解析 (主循环调用, 非 ISR 上下文)
 * @note   PID 参数直接修改 (即时生效)
 *         电机控制命令通过 CmdQueue_Put 解耦 (下一 50ms 周期生效)
 */
void PIDTune_Process(void)
{
    if (!tune_line_ready) return;

    char  tok[16];
    float val = 0.0f;
    int   n   = 0;

    char *cmd = tune_line;
    while (*cmd == ' ') cmd++;
    n = sscanf(cmd, "%15s %f", tok, &val);

    CmdMsg msg;

    /* ---- 内环 SpeedPID (直接修改, 即时生效) ---- */
    if (strcmp(tok, "skp") == 0 && n >= 2) {
        SpeedPID.Kp = val;
        tune_rsp("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                 SpeedPID.Kp, SpeedPID.Ki, SpeedPID.Kd, SpeedPID.Kf);
    }
    else if (strcmp(tok, "ski") == 0 && n >= 2) {
        SpeedPID.Ki = val;
        tune_rsp("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                 SpeedPID.Kp, SpeedPID.Ki, SpeedPID.Kd, SpeedPID.Kf);
    }
    else if (strcmp(tok, "skd") == 0 && n >= 2) {
        SpeedPID.Kd = val;
        tune_rsp("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                 SpeedPID.Kp, SpeedPID.Ki, SpeedPID.Kd, SpeedPID.Kf);
    }
    else if (strcmp(tok, "skf") == 0 && n >= 2) {
        SpeedPID.Kf = val;
        tune_rsp("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                 SpeedPID.Kp, SpeedPID.Ki, SpeedPID.Kd, SpeedPID.Kf);
    }

    /* ---- 外环 YawPID (直接修改, 即时生效) ---- */
    else if (strcmp(tok, "ykp") == 0 && n >= 2) {
        YawPID.Kp = val;
        tune_rsp("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                 YawPID.Kp, YawPID.Ki, YawPID.Kd, YawPID.Kf);
    }
    else if (strcmp(tok, "yki") == 0 && n >= 2) {
        YawPID.Ki = val;
        tune_rsp("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                 YawPID.Kp, YawPID.Ki, YawPID.Kd, YawPID.Kf);
    }
    else if (strcmp(tok, "ykd") == 0 && n >= 2) {
        YawPID.Kd = val;
        tune_rsp("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                 YawPID.Kp, YawPID.Ki, YawPID.Kd, YawPID.Kf);
    }
    else if (strcmp(tok, "ykf") == 0 && n >= 2) {
        YawPID.Kf = val;
        tune_rsp("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                 YawPID.Kp, YawPID.Ki, YawPID.Kd, YawPID.Kf);
    }

    /* ---- 电机控制 → cmd_queue (下一 50ms 周期生效) ---- */
    else if (strcmp(tok, "t") == 0 && n >= 2) {
        msg.type = CMD_SPEED; msg.val = val; CmdQueue_Put(msg);
        msg.type = CMD_START; msg.val = 0;   CmdQueue_Put(msg);
        bt_last_rx = millis;
        tune_rsp("[Motor] SpeedTarget=%.1f RPM\r\n", val);
    }
    else if (strcmp(tok, "yaw") == 0 && n >= 2) {
        msg.type = CMD_YAW_TARGET; msg.val = val; CmdQueue_Put(msg);
        bt_last_rx = millis;
        tune_rsp("[Motor] YawTarget=%.1f deg\r\n", val);
    }
    else if (strcmp(tok, "turn") == 0 && n >= 2) {
        msg.type = CMD_TURN; msg.val = val; CmdQueue_Put(msg);
        bt_last_rx = millis;
        tune_rsp("[Motor] TurnDiff=%.1f RPM\r\n", val);
    }
    else if (strcmp(tok, "start") == 0) {
        msg.type = CMD_START; msg.val = 0; CmdQueue_Put(msg);
        bt_last_rx = millis;
        tune_rsp("[Motor] Started\r\n");
    }
    else if (strcmp(tok, "stop") == 0) {
        msg.type = CMD_STOP; msg.val = 0; CmdQueue_Put(msg);
        tune_rsp("[Motor] Stopped\r\n");
    }
    else if (strcmp(tok, "quiet") == 0) {
        msg.type = CMD_QUIET; msg.val = 0; CmdQueue_Put(msg);
        tune_rsp("[VOFA] Silent mode, 'log' to resume\r\n");
    }
    else if (strcmp(tok, "log") == 0) {
        msg.type = CMD_LOG; msg.val = 0; CmdQueue_Put(msg);
        tune_rsp("[VOFA] Log resumed\r\n");
    }
    else if (strcmp(tok, "duty") == 0 && n >= 2) {
        PWMDuty = (int16_t)val;
        if (PWMDuty >  100) PWMDuty =  100;
        if (PWMDuty < -100) PWMDuty = -100;
        msg.type = PWMDuty ? CMD_START : CMD_STOP; msg.val = 0;
        CmdQueue_Put(msg);
        Mode = MODE_PWM_SET;
        bt_last_rx = millis;
        tune_rsp("[Motor] Duty=%d%%\r\n", PWMDuty);
    }

    /* ---- 查询 ---- */
    else if (strcmp(tok, "show") == 0) {
        tune_rsp("=== Cascade PID ===\r\n"
                 "[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f  Tgt=%.1f Out=%.1f\r\n"
                 "[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f  Tgt=%.1f Out=%.1f\r\n"
                 "TurnDiff=%.1f  Run=%d Mode=%d\r\n",
                 SpeedPID.Kp, SpeedPID.Ki, SpeedPID.Kd, SpeedPID.Kf,
                 SpeedPID.Target, SpeedPID.Out,
                 YawPID.Kp, YawPID.Ki, YawPID.Kd, YawPID.Kf,
                 YawPID.Target, YawPID.Out,
                 TurnTarget, RunFlag, Mode);
    }
    else if (strcmp(tok, "help") == 0 || strcmp(tok, "?") == 0) {
        tune_rsp("=== Cascade PID ===\r\n"
                 " Speed:  skp/ski/skd/skf N\r\n"
                 " Yaw:    ykp/yki/ykd/ykf N\r\n"
                 " Motor:  t N | yaw N | turn N\r\n"
                 "         start | stop | duty N\r\n"
                 " VOFA:   quiet | log\r\n"
                 " Info:   show\r\n");
    }
    else if (strlen(tok) > 0) {
        tune_rsp("? '%s'  (type 'help')\r\n", tok);
    }

    tune_line_ready = 0;
}

/**
 * @brief  UART RX 回调 (ISR 上下文) — 仅缓冲字节, 不做解析
 */
void PIDTune_RxCallback(uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (rx_idx > 0) {
            tune_line[rx_idx] = '\0';
            tune_line_ready = 1;    /* 主循环稍后消费 */
            rx_idx = 0;
        }
    }
    else if (rx_idx < sizeof(tune_line) - 1) {
        tune_line[rx_idx++] = byte;
    }
}

void PIDTune_Init(void)
{
    rx_idx = 0;
    tune_rsp_pending = 0;
    tune_line_ready = 0;
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&pid_tune_rx_byte, 1);
}

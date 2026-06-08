/**
 * @file    pid_tune.c
 * @brief   PID 串口在线调参实现
 * @note    通过 USB-TTL 接 USART3 (PD8=TX, PD9=RX), 115200bps
 *          串口终端发送命令, 实时调整 PID 参数, 效果立即可见
 */

#include "pid_tune.h"
#include "pid.h"
#include "serial.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern PID SpeedPID;
extern PID TurnPID;
extern UART_HandleTypeDef huart3;

static char  rx_line[32];       /* 命令行缓冲区 */
static uint8_t rx_idx = 0;     /* 缓冲区写入位置 */

/**
 * @brief  解析并执行 PID 调参命令
 * @param  cmd 命令行字符串
 */
static void PIDTune_Parse(char *cmd)
{
    char  tok[16];
    float val = 0.0f;
    int   n   = 0;

    /* 跳过前导空格 */
    while (*cmd == ' ') cmd++;

    /* 解析: "命令 数值" */
    n = sscanf(cmd, "%15s %f", tok, &val);

    if (strcmp(tok, "skp") == 0 && n >= 2) {
        setPIDParam(&SpeedPID, val, SpeedPID.ki, SpeedPID.kd);
        Serial_Printf("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                      SpeedPID.kp, SpeedPID.ki, SpeedPID.kd);
    }
    else if (strcmp(tok, "ski") == 0 && n >= 2) {
        setPIDParam(&SpeedPID, SpeedPID.kp, val, SpeedPID.kd);
        Serial_Printf("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                      SpeedPID.kp, SpeedPID.ki, SpeedPID.kd);
    }
    else if (strcmp(tok, "skd") == 0 && n >= 2) {
        setPIDParam(&SpeedPID, SpeedPID.kp, SpeedPID.ki, val);
        Serial_Printf("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                      SpeedPID.kp, SpeedPID.ki, SpeedPID.kd);
    }
    else if (strcmp(tok, "tkp") == 0 && n >= 2) {
        setPIDParam(&TurnPID, val, TurnPID.ki, TurnPID.kd);
        Serial_Printf("[TurnPID]  Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                      TurnPID.kp, TurnPID.ki, TurnPID.kd);
    }
    else if (strcmp(tok, "tki") == 0 && n >= 2) {
        setPIDParam(&TurnPID, TurnPID.kp, val, TurnPID.kd);
        Serial_Printf("[TurnPID]  Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                      TurnPID.kp, TurnPID.ki, TurnPID.kd);
    }
    else if (strcmp(tok, "tkd") == 0 && n >= 2) {
        setPIDParam(&TurnPID, TurnPID.kp, TurnPID.ki, val);
        Serial_Printf("[TurnPID]  Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                      TurnPID.kp, TurnPID.ki, TurnPID.kd);
    }
    else if (strcmp(tok, "show") == 0) {
        Serial_Printf("[SpeedPID] Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                      SpeedPID.kp, SpeedPID.ki, SpeedPID.kd);
        Serial_Printf("[TurnPID]  Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                      TurnPID.kp, TurnPID.ki, TurnPID.kd);
    }
    else if (strcmp(tok, "help") == 0 || strcmp(tok, "?") == 0) {
        Serial_Printf("PID Tune Commands:\r\n");
        Serial_Printf("  skp/ski/skd N  - SpeedPID Kp/Ki/Kd\r\n");
        Serial_Printf("  tkp/tki/tkd N  - TurnPID  Kp/Ki/Kd\r\n");
        Serial_Printf("  show           - print params\r\n");
    }
    else if (strlen(tok) > 0) {
        Serial_Printf("? '%s'\r\n", tok);
    }
}

/**
 * @brief  处理接收到的字节 (由中断回调函数调用)
 * @param  byte 收到的字节
 */
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

/**
 * @brief  启动 PID 调参串口接收
 */
void PIDTune_Init(void)
{
    rx_idx = 0;
    HAL_UART_Receive_IT(&huart3, (uint8_t *)rx_line, 1);
}

/**
 * @file    pid_tune.c
 * @brief   USART1 在线调参和电机测试命令解析。
 * @note    UART 中断只负责缓存字节，命令在主循环中解析。
 */

#include "pid_tune.h"
#include "yingYongMingLing.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "motor.h"
#include "pid.h"
#include "serial.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 调参响应缓冲区大小，足够容纳帮助信息和 PID 参数。 */
#define PID_TUNE_RESPONSE_BUFFER_SIZE 256U

/* 单行串口命令缓冲区大小，超过长度的内容会被忽略。 */
#define PID_TUNE_LINE_BUFFER_SIZE     32U

/* 命令名称解析缓冲区大小。 */
#define PID_TUNE_TOKEN_BUFFER_SIZE    16U

extern UART_HandleTypeDef huart1;
extern volatile uint64_t milliseconds;

volatile uint8_t pidTuneRxByte;

static volatile uint8_t tuneResponsePending = 0;
static volatile uint8_t tuneLineReady = 0;
static char tuneResponseBuffer[PID_TUNE_RESPONSE_BUFFER_SIZE];
static char tuneLine[PID_TUNE_LINE_BUFFER_SIZE];
static uint8_t tuneRxIndex = 0;

/**
 * @brief  格式化一条调参响应并标记为待发送。
 * @param  format printf 风格格式字符串。
 * @param  ... 可变参数列表。
 * @note   如果上一帧响应尚未发送，则丢弃新响应，避免覆盖待发送内容。
 */
static void pidTuneRespond(const char *format, ...)
{
    va_list arguments;

    /* 上一帧响应未发送时丢弃新响应，避免覆盖待发送内容。 */
    if (tuneResponsePending) return;

    va_start(arguments, format);
    vsnprintf(tuneResponseBuffer, sizeof(tuneResponseBuffer),
              format, arguments);
    va_end(arguments);

    tuneResponsePending = 1;
}

/**
 * @brief  解析一行 USART1 调参命令并执行对应动作。
 * @note   支持左右独立增量式速度环和航向环 PID 修改、电机测试、启停、
 *         VOFA 开关、航向角清零以及 show/help 查询。
 */
void pidTuneProcess(void)
{
    char token[PID_TUNE_TOKEN_BUFFER_SIZE];
    char *command;
    float value = 0.0f;
    int argumentCount;
    commandMessage_t message;

    /* 只有收到完整换行后才处理一行命令。 */
    if (!tuneLineReady) return;

    token[0] = '\0';
    command = tuneLine;
    while (*command == ' ') command++;

    argumentCount = sscanf(command, "%15s %f", token, &value);
    if (argumentCount < 1) {
        tuneLineReady = 0;
        return;
    }

    /* skp/ski/skd 作为兼容命令，同时修改左右两个增量速度环。 */
    if (strcmp(token, "skp") == 0 && argumentCount >= 2) {
        speedPidLeft.Kp = value;
        speedPidRight.Kp = value;
        pidTuneRespond("[SpeedPID] Both Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                       speedPidLeft.Kp, speedPidLeft.Ki, speedPidLeft.Kd);
    }
    else if (strcmp(token, "ski") == 0 && argumentCount >= 2) {
        speedPidLeft.Ki = value;
        speedPidRight.Ki = value;
        pidTuneRespond("[SpeedPID] Both Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                       speedPidLeft.Kp, speedPidLeft.Ki, speedPidLeft.Kd);
    }
    else if (strcmp(token, "skd") == 0 && argumentCount >= 2) {
        speedPidLeft.Kd = value;
        speedPidRight.Kd = value;
        pidTuneRespond("[SpeedPID] Both Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                       speedPidLeft.Kp, speedPidLeft.Ki, speedPidLeft.Kd);
    }
    /* lkp/lki/lkd 只修改左轮增量速度环。 */
    else if (strcmp(token, "lkp") == 0 && argumentCount >= 2) {
        speedPidLeft.Kp = value;
        pidTuneRespond("[SpeedPID] L Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                       speedPidLeft.Kp, speedPidLeft.Ki, speedPidLeft.Kd);
    }
    else if (strcmp(token, "lki") == 0 && argumentCount >= 2) {
        speedPidLeft.Ki = value;
        pidTuneRespond("[SpeedPID] L Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                       speedPidLeft.Kp, speedPidLeft.Ki, speedPidLeft.Kd);
    }
    else if (strcmp(token, "lkd") == 0 && argumentCount >= 2) {
        speedPidLeft.Kd = value;
        pidTuneRespond("[SpeedPID] L Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                       speedPidLeft.Kp, speedPidLeft.Ki, speedPidLeft.Kd);
    }
    /* rkp/rki/rkd 只修改右轮增量速度环。 */
    else if (strcmp(token, "rkp") == 0 && argumentCount >= 2) {
        speedPidRight.Kp = value;
        pidTuneRespond("[SpeedPID] R Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                       speedPidRight.Kp, speedPidRight.Ki, speedPidRight.Kd);
    }
    else if (strcmp(token, "rki") == 0 && argumentCount >= 2) {
        speedPidRight.Ki = value;
        pidTuneRespond("[SpeedPID] R Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                       speedPidRight.Kp, speedPidRight.Ki, speedPidRight.Kd);
    }
    else if (strcmp(token, "rkd") == 0 && argumentCount >= 2) {
        speedPidRight.Kd = value;
        pidTuneRespond("[SpeedPID] R Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                       speedPidRight.Kp, speedPidRight.Ki, speedPidRight.Kd);
    }
    /* skf 同时设置两轮前馈，skfl/skfr 分别设置左轮和右轮前馈。 */
    else if (strcmp(token, "skf") == 0 && argumentCount >= 2) {
        speedPidLeft.Kf = value;
        speedPidRight.Kf = value;
        pidTuneRespond("[SpeedPID] Kf L=%.2f R=%.2f\r\n",
                       speedPidLeft.Kf, speedPidRight.Kf);
    }
    else if (strcmp(token, "skfl") == 0 && argumentCount >= 2) {
        speedPidLeft.Kf = value;
        pidTuneRespond("[SpeedPID] Kf L=%.2f R=%.2f\r\n",
                       speedPidLeft.Kf, speedPidRight.Kf);
    }
    else if (strcmp(token, "skfr") == 0 && argumentCount >= 2) {
        speedPidRight.Kf = value;
        pidTuneRespond("[SpeedPID] Kf L=%.2f R=%.2f\r\n",
                       speedPidLeft.Kf, speedPidRight.Kf);
    }
    /* 航向环 PID 参数只作用于 yawPid。 */
    else if (strcmp(token, "ykp") == 0 && argumentCount >= 2) {
        yawPid.Kp = value;
        pidTuneRespond("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                       yawPid.Kp, yawPid.Ki, yawPid.Kd, yawPid.Kf);
    }
    else if (strcmp(token, "yki") == 0 && argumentCount >= 2) {
        yawPid.Ki = value;
        pidTuneRespond("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                       yawPid.Kp, yawPid.Ki, yawPid.Kd, yawPid.Kf);
    }
    else if (strcmp(token, "ykd") == 0 && argumentCount >= 2) {
        yawPid.Kd = value;
        pidTuneRespond("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                       yawPid.Kp, yawPid.Ki, yawPid.Kd, yawPid.Kf);
    }
    else if (strcmp(token, "ykf") == 0 && argumentCount >= 2) {
        yawPid.Kf = value;
        pidTuneRespond("[YawPID]   Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\r\n",
                       yawPid.Kp, yawPid.Ki, yawPid.Kd, yawPid.Kf);
    }
    /* 电机相关命令写入命令队列，在下一个 50 ms 控制周期生效。 */
    else if (strcmp(token, "t") == 0 && argumentCount >= 2) {
        message.type = commandSpeed;
        message.value = value;
        appCommandQueuePush(message);
        message.type = commandStart;
        message.value = 0.0f;
        appCommandQueuePush(message);
        bluetoothLastRx = milliseconds;
        pidTuneRespond("[Motor] SpeedTarget=%.1f RPM\r\n", value);
    }
    else if (strcmp(token, "tl") == 0 && argumentCount >= 2) {
        message.type = commandSpeedLeft;
        message.value = value;
        appCommandQueuePush(message);
        message.type = commandStart;
        message.value = 0.0f;
        appCommandQueuePush(message);
        bluetoothLastRx = milliseconds;
        pidTuneRespond("[Motor] LeftTarget=%.1f RPM\r\n", value);
    }
    else if (strcmp(token, "tr") == 0 && argumentCount >= 2) {
        message.type = commandSpeedRight;
        message.value = value;
        appCommandQueuePush(message);
        message.type = commandStart;
        message.value = 0.0f;
        appCommandQueuePush(message);
        bluetoothLastRx = milliseconds;
        pidTuneRespond("[Motor] RightTarget=%.1f RPM\r\n", value);
    }
    else if (strcmp(token, "yaw") == 0 && argumentCount >= 2) {
        message.type = commandYawTarget;
        message.value = value;
        appCommandQueuePush(message);
        bluetoothLastRx = milliseconds;
        pidTuneRespond("[Motor] YawTarget=%.1f deg\r\n", value);
    }
    else if (strcmp(token, "turn") == 0 && argumentCount >= 2) {
        message.type = commandTurn;
        message.value = value;
        appCommandQueuePush(message);
        bluetoothLastRx = milliseconds;
        pidTuneRespond("[Motor] TurnDiff=%.1f RPM\r\n", value);
    }
    else if (strcmp(token, "pivot") == 0 && argumentCount >= 2) {
        message.type = commandPivot;
        message.value = value;
        appCommandQueuePush(message);
        message.type = commandStart;
        message.value = 0.0f;
        appCommandQueuePush(message);
        bluetoothLastRx = milliseconds;
        pidTuneRespond("[Motor] Pivot=%.1f RPM\r\n", value);
    }
    else if (strcmp(token, "start") == 0) {
        message.type = commandStart;
        message.value = 0.0f;
        appCommandQueuePush(message);
        bluetoothLastRx = milliseconds;
        pidTuneRespond("[Motor] Started\r\n");
    }
    else if (strcmp(token, "stop") == 0) {
        message.type = commandStop;
        message.value = 0.0f;
        appCommandQueuePush(message);
        pidTuneRespond("[Motor] Stopped\r\n");
    }
    else if (strcmp(token, "quiet") == 0) {
        message.type = commandQuiet;
        message.value = 0.0f;
        appCommandQueuePush(message);
        pidTuneRespond("[VOFA] Silent mode, 'log' to resume\r\n");
    }
    else if (strcmp(token, "log") == 0) {
        message.type = commandLog;
        message.value = 0.0f;
        appCommandQueuePush(message);
        pidTuneRespond("[VOFA] Log resumed\r\n");
    }
    else if (strcmp(token, "duty") == 0 && argumentCount >= 2) {
        pwmDuty = (int16_t)value;
        if (pwmDuty > PWM_TEST_MAX_PERCENT) {
            pwmDuty = PWM_TEST_MAX_PERCENT;
        }
        if (pwmDuty < PWM_TEST_MIN_PERCENT) {
            pwmDuty = PWM_TEST_MIN_PERCENT;
        }

        message.type = pwmDuty ? commandStart : commandStop;
        message.value = 0.0f;
        appCommandQueuePush(message);
        workMode = modePwmSet;
        bluetoothLastRx = milliseconds;
        pidTuneRespond("[Motor] Open-loop test updated\r\n");
    }
    else if (strcmp(token, "yaw0") == 0) {
        message.type = commandImuCalib;
        message.value = 0.0f;
        appCommandQueuePush(message);
        pidTuneRespond("[IMU] Yaw reset to 0\r\n");
    }
    /* 查询当前 PID 和运行状态。 */
    else if (strcmp(token, "show") == 0) {
        pidTuneRespond(
            "=== Incremental Speed PID ===\r\n"
            "[L] Kp=%.2f Ki=%.2f Kd=%.2f Tgt=%.1f Act=%.1f Out=%.1f Inc=%.1f\r\n"
            "[R] Kp=%.2f Ki=%.2f Kd=%.2f Tgt=%.1f Act=%.1f Out=%.1f Inc=%.1f\r\n"
            "Turn=%.1f Run=%d Mode=%d\r\n",
            speedPidLeft.Kp, speedPidLeft.Ki, speedPidLeft.Kd,
            speedPidLeft.Target, speedPidLeft.Actual, speedPidLeft.Out,
            speedPidLeft.LastIncrement,
            speedPidRight.Kp, speedPidRight.Ki, speedPidRight.Kd,
            speedPidRight.Target, speedPidRight.Actual, speedPidRight.Out,
            speedPidRight.LastIncrement,
            turnTarget, runFlag, workMode);
    }
    else if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0) {
        pidTuneRespond(
            "=== Incremental Speed PID ===\r\n"
            " Both:   skp/ski/skd N\r\n"
            " Left:   lkp/lki/lkd N\r\n"
            " Right:  rkp/rki/rkd N\r\n"
            " Yaw:    ykp/yki/ykd/ykf N\r\n"
            " Motor:  t N | tl N | tr N | yaw N | turn N | pivot N\r\n"
            "         start | stop | duty N\r\n"
            " VOFA:   quiet | log\r\n"
            " Info:   show\r\n");
    }
    else if (strlen(token) > 0) {
        pidTuneRespond("? '%s'  (type 'help')\r\n", token);
    }

    tuneLineReady = 0;
}

/**
 * @brief  在 UART 中断中缓存接收字节。
 * @param  byte 当前接收字节。
 * @note   收到回车或换行且缓冲区非空时，将整行标记为就绪并复位索引。
 */
void pidTuneRxCallback(uint8_t byte)
{
    /* 中断上下文只缓存字节，收到换行时标记一行命令就绪。 */
    if (byte == '\r' || byte == '\n') {
        if (tuneRxIndex > 0) {
            tuneLine[tuneRxIndex] = '\0';
            tuneLineReady = 1;
            tuneRxIndex = 0;
        }
    }
    else if (tuneRxIndex < sizeof(tuneLine) - 1U) {
        tuneLine[tuneRxIndex++] = byte;
    }
}

/**
 * @brief  发送待处理的调参响应。
 * @note   只有 tuneResponsePending 为 1 时才处理；Serial_SendString 将内容
 *         复制到发送环形缓冲区后立即返回，因此入队完成后清除待发送标志。
 */
void pidTuneProcessResponse(void)
{
    if (!tuneResponsePending) return;

    /* 发送响应期间短暂关闭中断，保证字符串不会被并发修改。 */
    __disable_irq();
    Serial_SendString(tuneResponseBuffer);
    __enable_irq();
    tuneResponsePending = 0;
}

/**
 * @brief  初始化在线调参模块。
 * @note   复位行缓冲索引和状态标志，并启动 USART1 单字节接收中断。
 */
void pidTuneInit(void)
{
    tuneRxIndex = 0;
    tuneResponsePending = 0;
    tuneLineReady = 0;
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&pidTuneRxByte, 1);
}

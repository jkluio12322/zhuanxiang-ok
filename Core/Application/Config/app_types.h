/**
 * @file    app_types.h
 * @brief   应用层共享的枚举与结构体定义。
 * @note    只依赖 <stdint.h>，不包含任何硬件头。
 */
#ifndef __APP_TYPES_H
#define __APP_TYPES_H

#include <stdint.h>

/**
 * @brief  命令队列里的命令类型。
 * @note   两个来源都往同一个队列里放：按键任务（app_cmd.c）和串口命令行
 *         （app_console.c）。执行方是 50 ms 控制任务（app_speedloop.c）。
 * @note   删命令时**不能对号入座地删枚举**——有些枚举值命令行已经不用了，
 *         但按键路径还在发。当前每个值的来源见注释。
 */
typedef enum {
    commandNone = 0,
    commandStart,            /* 按键：KEY2（循迹/PWM循迹）；命令行已不提供 */
    commandStop,             /* 按键：KEY3 / 命令行 stop */
    commandLapCycle,         /* 按键：循迹模式下 KEY4（仅停车时） */
    commandModeTrack,        /* 按键：KEY1 轮换 */
    commandModePwmSpeed,     /* 按键：KEY1 轮换 */
    commandModePwmTrack,     /* 按键：KEY1 轮换 */
    commandPwmSpeedNextGear, /* 按键：PWM 速度模式下 KEY2 */
    commandPwmTrackNext,     /* 按键：PWM 循迹模式下 KEY2 */
    commandPwmTrackBaseStep, /* 按键：PWM 循迹模式下 KEY4 短按/长按 */
    commandSetLeftRpm,       /* 命令行：l <rpm>，只改左轮目标 */
    commandSetRightRpm,      /* 命令行：r <rpm>，只改右轮目标 */
    commandSetBothRpm,       /* 命令行：t <rpm>，两轮同时改 */
    commandQuiet,            /* 命令行：quiet，暂停 VOFA */
    commandLog               /* 命令行：log，恢复 VOFA */
} commandType_t;

/* 一条待执行的命令。 */
typedef struct {
    commandType_t type;
    float value;
} commandMessage_t;

/* 当前工作模式。 */
typedef enum {
    modeTrack = 0,
    modePwmSpeed,
    modePwmTrack
} workMode_t;

/* PWM 测试阶段。 */
typedef enum {
    PWM_TEST_PHASE_NONE = 0,
    PWM_TEST_PHASE_FORWARD = 1,
    PWM_TEST_PHASE_REVERSE = 2,
    PWM_TEST_PHASE_LEFT = 3,
    PWM_TEST_PHASE_RIGHT = 4
} PwmTestPhase_t;

/* 停车原因。 */
typedef enum {
    stopReasonNone = 0,
    stopReasonUser = 1,
    stopReasonTargetLaps = 2,
    stopReasonAllWhite = 3,
    stopReasonCornerTimeout = 4,
    stopReasonModeSwitch = 5
} appStopReason_t;

#endif /* __APP_TYPES_H */

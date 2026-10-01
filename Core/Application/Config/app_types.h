/**
 * @file    app_types.h
 * @brief   应用层共享的枚举与结构体定义。
 * @note    只依赖 <stdint.h>，不包含任何硬件头。
 */
#ifndef __APP_TYPES_H
#define __APP_TYPES_H

#include <stdint.h>

/* 命令队列里的命令类型。 */
typedef enum {
    commandNone = 0,
    commandStart,
    commandStop,
    commandLapCycle,
    commandModeTrack,
    commandModePwmSpeed,
    commandModePwmTrack,
    commandPwmSpeedNextGear,
    commandSetSpeedGear,
    commandPwmTrackNext,
    commandPwmTrackSetAngle,
    commandPwmTrackBaseStep,
    commandPwmTrackSim,
    commandQuiet,
    commandLog
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

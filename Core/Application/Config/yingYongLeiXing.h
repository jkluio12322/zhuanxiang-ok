#ifndef __YING_YONG_LEI_XING_H
#define __YING_YONG_LEI_XING_H

#include <stdint.h>

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

typedef struct {
    commandType_t type;
    float value;
} commandMessage_t;

typedef enum {
    modeTrack = 0,
    modePwmSpeed,
    modePwmTrack
} workMode_t;

typedef enum {
    PWM_TEST_PHASE_NONE = 0,
    PWM_TEST_PHASE_FORWARD = 1,
    PWM_TEST_PHASE_REVERSE = 2,
    PWM_TEST_PHASE_LEFT = 3,
    PWM_TEST_PHASE_RIGHT = 4
} PwmTestPhase_t;

typedef enum {
    stopReasonNone = 0,
    stopReasonUser = 1,
    stopReasonTargetLaps = 2,
    stopReasonAllWhite = 3,
    stopReasonCornerTimeout = 4,
    stopReasonModeSwitch = 5
} appStopReason_t;

#endif

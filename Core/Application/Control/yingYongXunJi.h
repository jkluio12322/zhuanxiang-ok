#ifndef __YING_YONG_XUN_JI_H
#define __YING_YONG_XUN_JI_H

#include <stdint.h>
#include "yingYongLeiXing.h"

typedef enum {
    TRACK_CONTROL_FOLLOW = 0,
    TRACK_CONTROL_TURN_LEFT,
    TRACK_CONTROL_TURN_RIGHT,
    TRACK_CONTROL_FINISH,
    TRACK_CONTROL_LOST
} TrackControlState_t;

typedef enum {
    TRACK_PHASE_FOLLOW = 0,
    TRACK_PHASE_TURN,
    TRACK_PHASE_RECOVER,
    TRACK_PHASE_RECENTER
} TrackTurnPhase_t;

void appTrackingResetState(void);
void appTrackingStart(void);
void appTrackingSetPidGains(float kp, float ki, float kd);
void appTrackingGetPidGains(float *kp, float *ki, float *kd);
void appTrackingSetSteerRpmToPwmGain(float gain);
void appTrackingTask20ms(void);
TrackControlState_t appTrackingGetState(void);
float appTrackingGetSteer(void);
float appTrackingGetAverageTargetRpm(void);
float appTrackingGetSteerRpmToPwmGain(void);
float appTrackingGetWeightedError(void);
TrackTurnPhase_t appTrackingGetTurnPhase(void);
float appTrackingComputeWeightedError(void);
float appTrackingUpdatePidForError(float error, float diffLimit);
void appTrackingSetPwmSteer(float steerRpm);

#endif

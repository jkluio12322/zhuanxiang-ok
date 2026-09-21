/**
 * @file    yingYongXunJi.h
 * @brief   Digital eight-channel line tracking control interface.
 */
#ifndef __YING_YONG_XUN_JI_H
#define __YING_YONG_XUN_JI_H

#include <stdint.h>

typedef enum {
    TRACK_CONTROL_FOLLOW = 0,
    TRACK_CONTROL_TURN_LEFT,
    TRACK_CONTROL_TURN_RIGHT,
    TRACK_CONTROL_REACQUIRE,
    TRACK_CONTROL_FINISH,
    TRACK_CONTROL_LOST
} TrackControlState_t;

typedef struct {
    TrackControlState_t state;
    uint8_t pattern;
    uint8_t blackCount;
    int8_t candidateDirection;
    uint8_t candidateTicks;
    int8_t pendingCornerDirection;
    uint8_t pendingCornerTicks;
    uint8_t invalidLineTicks;
    uint16_t turnTicks;
    uint16_t reacquireTicks;
} TrackDiagnostics_t;

void appTrackingResetState(void);
void appTrackingStart(void);
void appTrackingTask20ms(void);

TrackControlState_t appTrackingGetState(void);
float appTrackingGetPosition(void);
float appTrackingGetSteer(void);
void appTrackingGetDiagnostics(TrackDiagnostics_t *output);

#endif

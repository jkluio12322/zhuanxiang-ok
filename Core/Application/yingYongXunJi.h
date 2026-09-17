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

void appTrackingResetState(void);
void appTrackingStart(void);
void appTrackingTask20ms(void);

TrackControlState_t appTrackingGetState(void);
float appTrackingGetPosition(void);
float appTrackingGetSteer(void);

#endif

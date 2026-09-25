/**
 * @file    yingYongXunJi.h
 * @brief   Line-tracking control interface.
 */
#ifndef __YING_YONG_XUN_JI_H
#define __YING_YONG_XUN_JI_H  /* 头文件保护宏，防止循迹接口重复声明。 */

#include <stdint.h>

/**
 * @brief  Line-tracking state machine states.
 */
typedef enum {
    TRACK_CONTROL_FOLLOW = 0,
    TRACK_CONTROL_TURN_LEFT,
    TRACK_CONTROL_TURN_RIGHT,
    TRACK_CONTROL_FINISH,
    TRACK_CONTROL_LOST
} TrackControlState_t;

/**
 * @brief  Line-tracking PID output phases.
 * @note   FOLLOW and RECENTER use the same PID; RECENTER is only a
 *         telemetry label for patterns without IR4/IR5.
 */
typedef enum {
    TRACK_PHASE_FOLLOW = 0,
    TRACK_PHASE_TURN,
    TRACK_PHASE_RECOVER,
    TRACK_PHASE_RECENTER
} TrackTurnPhase_t;

/**
 * @brief  Reset the tracking state machine and infrared data.
 */
void appTrackingResetState(void);

/**
 * @brief  Start a new tracking run and clear lap progress.
 */
void appTrackingStart(void);

/**
 * @brief  Set the RPM differential to PWM conversion gain.
 */
void appTrackingSetSteerRpmToPwmGain(float gain);

/**
 * @brief  Run one 20 ms tracking task.
 */
void appTrackingTask20ms(void);

/**
 * @brief  Get the current tracking control state.
 */
TrackControlState_t appTrackingGetState(void);

/**
 * @brief  Get the latest steering differential in RPM.
 */
float appTrackingGetSteer(void);

/**
 * @brief  Get the latest average wheel-speed target in RPM.
 */
float appTrackingGetAverageTargetRpm(void);

/**
 * @brief  Get the RPM differential to PWM conversion gain.
 */
float appTrackingGetSteerRpmToPwmGain(void);

/**
 * @brief  Get the negative PWM limit allowed during a turn.
 */
float appTrackingGetTurnReverseLimitPwm(void);

/**
 * @brief  Get the latest weighted error used by the tracking PID.
 */
float appTrackingGetWeightedError(void);

/**
 * @brief  Get the current tracking PID output phase.
 */
TrackTurnPhase_t appTrackingGetTurnPhase(void);

#endif

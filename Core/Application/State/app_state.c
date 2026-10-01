/**
 * @file    app_state.c
 * @brief   应用层唯一可变状态的实现。
 */

#include "app_state.h"
#include "app_config.h"

/* 状态本体是 static，外部只能通过 AppState() 读、通过 AppState_SetXxx() 写。 */
static AppStateView_t state;

static const float speedTable[SPEED_LEVEL_COUNT] = {
    SPEED_TABLE_G1_RPM,
    SPEED_TABLE_G2_RPM,
    SPEED_TABLE_G3_RPM,
    SPEED_TABLE_G4_RPM,
    SPEED_TABLE_G5_RPM,
    SPEED_TABLE_G6_RPM
};

static appStopReason_t lastStopReason = stopReasonNone;
static volatile uint8_t stopEventPending = 0U;
static volatile uint8_t stopRequested = 0U;

const AppStateView_t *AppState(void)
{
    return &state;
}

void AppState_Init(void)
{
    state.mode = modeTrack;
    state.running = 0U;
    state.vofa = 1U;
    state.gear = SPEED_LEVEL_DEFAULT;
    state.gear_rpm = speedTable[state.gear - 1U];
    state.lap_target = TRACK_LAP_TARGET_DEFAULT;
    state.laps_done = 0U;
    state.target_left_rpm = 0.0f;
    state.target_right_rpm = 0.0f;

    lastStopReason = stopReasonNone;
    stopEventPending = 0U;
}

void AppState_SetMode(workMode_t mode)
{
    state.mode = mode;
}

void AppState_SetRunning(uint8_t running)
{
    state.running = (running != 0U) ? 1U : 0U;
}

void AppState_SetVofa(uint8_t on)
{
    state.vofa = (on != 0U) ? 1U : 0U;
}

uint8_t AppState_SetGear(uint8_t gear)
{
    if (gear < 1U) {
        gear = SPEED_LEVEL_COUNT;
    }
    if (gear > SPEED_LEVEL_COUNT) {
        gear = 1U;
    }

    if (gear == state.gear) {
        return 0U;
    }

    state.gear = gear;
    state.gear_rpm = speedTable[gear - 1U];
    return 1U;
}

void AppState_StepGear(int8_t direction)
{
    int16_t next = (int16_t)state.gear + (int16_t)direction;

    if (next > (int16_t)SPEED_LEVEL_COUNT) {
        next = 1;
    }
    if (next < 1) {
        next = (int16_t)SPEED_LEVEL_COUNT;
    }

    (void)AppState_SetGear((uint8_t)next);
}

void AppState_CycleLapTarget(void)
{
    uint8_t next = (uint8_t)(state.lap_target + 1U);

    if (next > TRACK_LAP_TARGET_MAX) {
        next = 1U;
    }
    state.lap_target = next;
}

void AppState_IncrementLap(void)
{
    if (state.laps_done < state.lap_target) {
        state.laps_done++;
    }
}

void AppState_ResetLapProgress(void)
{
    state.laps_done = 0U;
}

void AppState_SetWheelTargets(float left_rpm, float right_rpm)
{
    state.target_left_rpm = left_rpm;
    state.target_right_rpm = right_rpm;
}

void AppState_ReportStop(appStopReason_t reason)
{
    if (reason == stopReasonNone) {
        return;
    }

    lastStopReason = reason;
    stopEventPending = 1U;
    stopRequested = 1U;
}

uint8_t AppState_TakeStopRequest(void)
{
    if (stopRequested == 0U) {
        return 0U;
    }

    stopRequested = 0U;
    return 1U;
}

uint8_t AppState_TakeStopEvent(appStopReason_t *reason)
{
    if ((stopEventPending == 0U) || (reason == 0)) {
        return 0U;
    }

    *reason = lastStopReason;
    return 1U;
}

void AppState_ClearStopEvent(void)
{
    lastStopReason = stopReasonNone;
    stopEventPending = 0U;
    stopRequested = 0U;
}

/**
 * @file    yingYongXunJi.c
 * @brief   Direct lookup eight-channel line tracking controller.
 */

#include "yingYongXunJi.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "irtrack.h"

static TrackControlState_t trackControlState = TRACK_CONTROL_FOLLOW;
static uint8_t trackFinishConfirmTicks = 0U;
static uint8_t trackFinishClearTicks = 0U;
static uint8_t trackFinishArmed = 0U;
static uint8_t trackStartupTicks = 0U;
static uint8_t trackLineAcquired = 0U;
static uint8_t trackLostSweepTicks = 0U;
static uint8_t trackLostSweepPhase = 0U;
static int8_t trackLostSweepStartDirection = -1;
static float trackSteerOutput = 0.0f;

static float appTrackingClampValue(float value, float minValue, float maxValue)
{
    if (value > maxValue) return maxValue;
    if (value < minValue) return minValue;
    return value;
}

static void appTrackingSetTargets(float nominalLeft, float nominalRight)
{
    float targetLeft = nominalLeft;
    float targetRight = nominalRight;
    float swap;

    if (TRACK_STEERING_SIGN < 0.0f) {
        swap = targetLeft;
        targetLeft = targetRight;
        targetRight = swap;
    }

    targetLeft = appTrackingClampValue(targetLeft,
                                       TRACK_MIN_FORWARD_RPM,
                                       SPEED_TARGET_LIMIT_RPM);
    targetRight = appTrackingClampValue(targetRight,
                                        TRACK_MIN_FORWARD_RPM,
                                        SPEED_TARGET_LIMIT_RPM);

    targetLeftRpm = targetLeft;
    targetRightRpm = targetRight;
    turnTarget = (targetLeft - targetRight) * 0.5f;
    trackSteerOutput = turnTarget;
}

static void appTrackingApplyLutTargets(float baseSpeed)
{
    float nominalLeft = baseSpeed
                        * (float)ir_data.left_percent / 100.0f;
    float nominalRight = baseSpeed
                         * (float)ir_data.right_percent / 100.0f;

    appTrackingSetTargets(nominalLeft, nominalRight);
}

static void appTrackingSetSearchTargets(int8_t nominalSide)
{
    float nominalLeft;
    float nominalRight;

    if (nominalSide > 0) {
        nominalLeft = TRACK_SWEEP_OUTER_RPM;
        nominalRight = TRACK_SWEEP_INNER_RPM;
    } else {
        nominalLeft = TRACK_SWEEP_INNER_RPM;
        nominalRight = TRACK_SWEEP_OUTER_RPM;
    }

    appTrackingSetTargets(nominalLeft, nominalRight);
}

static void appTrackingStop(void)
{
    pwmDuty = 0;
    appStateResetMotion();
    appTrackingResetState();
}

static uint8_t appTrackingHasValidLine(void)
{
    return (uint8_t)((ir_data.black_count > 0U)
                     && (ir_data.black_count <= TRACK_LINE_VALID_MAX_BLACK)
                     && (ir_data.pattern != TRACK_FINISH_PATTERN)
                     && (ir_data.pattern != TRACK_LOST_PATTERN));
}

static void appTrackingBeginLineSearch(void)
{
    trackControlState = TRACK_CONTROL_LOST;
    trackLostSweepTicks = 0U;
    trackLostSweepPhase = 0U;
    trackLostSweepStartDirection = (ir_data.last_side > 0) ? 1 : -1;
}

static uint8_t appTrackingUpdateStartupForward(float baseSpeed)
{
    if (trackLineAcquired) {
        return 0U;
    }

    if (appTrackingHasValidLine()) {
        trackLineAcquired = 1U;
        trackStartupTicks = 0U;
        return 0U;
    }

    if (trackStartupTicks < TRACK_START_FORWARD_TICKS) {
        trackStartupTicks++;
        trackControlState = TRACK_CONTROL_FOLLOW;
        appTrackingSetTargets(baseSpeed, baseSpeed);
        return 1U;
    }

    return 0U;
}

static uint8_t appTrackingUpdateFinish(void)
{
    uint8_t finishSeen = (uint8_t)((ir_data.pattern == TRACK_FINISH_PATTERN)
                         && (ir_data.black_count == IRTRACK_CHANNELS));
    uint8_t validLineSeen = appTrackingHasValidLine();

    if (!trackFinishArmed) {
        if (finishSeen) {
            trackFinishClearTicks = 0U;
            trackControlState = TRACK_CONTROL_FINISH;
            return 1U;
        }

        if (trackLineAcquired && validLineSeen) {
            if (trackFinishClearTicks < TRACK_FINISH_CLEAR_TICKS) {
                trackFinishClearTicks++;
            }
        } else {
            trackFinishClearTicks = 0U;
        }

        if (trackFinishClearTicks >= TRACK_FINISH_CLEAR_TICKS) {
            trackFinishClearTicks = 0U;
            trackFinishConfirmTicks = 0U;
            trackFinishArmed = 1U;
            trackControlState = TRACK_CONTROL_FOLLOW;
        }

        return 0U;
    }

    if (!finishSeen) {
        trackFinishConfirmTicks = 0U;
        if ((trackControlState == TRACK_CONTROL_FINISH) && validLineSeen) {
            trackControlState = TRACK_CONTROL_FOLLOW;
        }
        return 0U;
    }

    trackControlState = TRACK_CONTROL_FINISH;

    if (trackFinishConfirmTicks < TRACK_FINISH_CONFIRM_TICKS) {
        trackFinishConfirmTicks++;
    }

    if (trackFinishConfirmTicks >= TRACK_FINISH_CONFIRM_TICKS) {
        trackFinishConfirmTicks = 0U;
        trackFinishClearTicks = 0U;
        trackFinishArmed = 0U;
        appStateIncrementCompletedLaps();

        if (appStateGetCompletedLaps() >= appStateGetLapTarget()) {
            appTrackingStop();
            return 1U;
        }

        return 1U;
    }

    return 1U;
}

static void appTrackingUpdateLineSearch(float baseSpeed)
{
    int8_t direction;

    if (appTrackingHasValidLine()) {
        trackControlState = TRACK_CONTROL_FOLLOW;
        trackLostSweepTicks = 0U;
        trackLostSweepPhase = 0U;
        appTrackingApplyLutTargets(baseSpeed);
        return;
    }

    if (trackLostSweepPhase >= (TRACK_SWEEP_ROUNDS * 2U)) {
        appTrackingStop();
        return;
    }

    if (trackLostSweepTicks >= TRACK_SWEEP_HALF_TICKS) {
        trackLostSweepTicks = 0U;
        trackLostSweepPhase++;

        if (trackLostSweepPhase >= (TRACK_SWEEP_ROUNDS * 2U)) {
            appTrackingStop();
            return;
        }
    }

    direction = trackLostSweepStartDirection;
    if ((trackLostSweepPhase & 1U) != 0U) {
        direction = (int8_t)-direction;
    }

    appTrackingSetSearchTargets(direction);
    trackLostSweepTicks++;
}

void appTrackingResetState(void)
{
    trackControlState = TRACK_CONTROL_FOLLOW;
    trackFinishConfirmTicks = 0U;
    trackFinishClearTicks = 0U;
    trackFinishArmed = 0U;
    trackStartupTicks = 0U;
    trackLineAcquired = 0U;
    trackLostSweepTicks = 0U;
    trackLostSweepPhase = 0U;
    trackLostSweepStartDirection = -1;
    trackSteerOutput = 0.0f;

    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] = 0U;
    }
    ir_data.weight_sum = 0;
    ir_data.position = 0.0f;
    ir_data.left_percent = 100U;
    ir_data.right_percent = 100U;
    ir_data.black_count = 0U;
    ir_data.state = TRACK_NORMAL;
    ir_data.pattern = 0U;
    ir_data.last_side = 0;
}

void appTrackingStart(void)
{
    appStateResetLapProgress();
    appTrackingResetState();
    pwmDuty = 0;
}

void appTrackingTask20ms(void)
{
    float baseSpeed;

    if ((workMode != modeTrack) || !runFlag) {
        return;
    }

    IRTrack_Read();
    IRTrack_Compute();
    (void)IRTrack_DetectCrossroad();

    if (appStateGetSpeedLevel() > TRACK_MAX_SPEED_LEVEL) {
        appStateSetSpeedLevel(TRACK_MAX_SPEED_LEVEL);
    }

    baseSpeed = appStateGetSpeedRpm() * TRACK_DRIVE_SPEED_SCALE;

    if (appTrackingUpdateStartupForward(baseSpeed)) {
        return;
    }

    if (appTrackingUpdateFinish()) {
        if (!runFlag) {
            return;
        }
        appTrackingApplyLutTargets(baseSpeed);
        return;
    }

    if (!appTrackingHasValidLine()) {
        if (trackControlState != TRACK_CONTROL_LOST) {
            appTrackingBeginLineSearch();
        }
        appTrackingUpdateLineSearch(baseSpeed);
        return;
    }

    trackControlState = TRACK_CONTROL_FOLLOW;
    appTrackingApplyLutTargets(baseSpeed);
}

TrackControlState_t appTrackingGetState(void)
{
    return trackControlState;
}

float appTrackingGetPosition(void)
{
    return ir_data.position;
}

float appTrackingGetSteer(void)
{
    return trackSteerOutput;
}

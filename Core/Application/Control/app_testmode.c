/**
 * @file    app_testmode.c
 * @brief   两种开环测试模式的实现。
 */

#include "app_testmode.h"
#include "app_config.h"
#include "app_state.h"
#include "irtrack.h"
#include "app_tracking.h"

/* 允许的模拟角度表，app_console 的 simlevel 校验也用这几个值。 */
static const uint8_t angleTable[] = {5U, 10U, 45U, 90U};
#define ANGLE_TABLE_SIZE (sizeof(angleTable) / sizeof(angleTable[0]))

static PwmTestPhase_t speedPhase = PWM_TEST_PHASE_FORWARD;
static uint32_t       speedElapsedMs = 0U;

static PwmTestPhase_t trackPhase = PWM_TEST_PHASE_LEFT;
static uint32_t       trackElapsedMs = 0U;
static uint8_t        trackSimEnabled = 1U;
static uint8_t        trackAngleDeg = 5U;
static int16_t        trackBasePwm = PWM_TRACK_BASE_DEFAULT_PERCENT;

static int16_t clampBasePwm(int16_t value)
{
    if (value > PWM_TEST_MAX_PERCENT) return PWM_TEST_MAX_PERCENT;
    if (value < 0) return 0;
    return value;
}

void AppTestMode_Init(void)
{
    AppTestMode_ResetPwmSpeed();
    AppTestMode_ResetPwmTrack();
    trackSimEnabled = 1U;
    trackAngleDeg = 5U;
    trackBasePwm = PWM_TRACK_BASE_DEFAULT_PERCENT;
}

/* ---------------- PWM 速度测试 ---------------- */

void AppTestMode_ResetPwmSpeed(void)
{
    speedPhase = PWM_TEST_PHASE_FORWARD;
    speedElapsedMs = 0U;
}

uint8_t AppTestMode_TickPwmSpeed(void)
{
    speedElapsedMs += APP_CONTROL_PERIOD_MS;
    if (speedElapsedMs < PWM_SPEED_TEST_PHASE_MS) {
        return 0U;
    }

    speedElapsedMs = 0U;
    speedPhase = (speedPhase == PWM_TEST_PHASE_FORWARD)
                 ? PWM_TEST_PHASE_REVERSE
                 : PWM_TEST_PHASE_FORWARD;
    return 1U;
}

int8_t AppTestMode_GetPwmSpeedSign(void)
{
    return (speedPhase == PWM_TEST_PHASE_REVERSE) ? -1 : 1;
}

PwmTestPhase_t AppTestMode_GetSpeedPhase(void)
{
    return speedPhase;
}

/* ---------------- PWM 循迹测试 ---------------- */

void AppTestMode_ResetPwmTrack(void)
{
    trackPhase = PWM_TEST_PHASE_LEFT;
    trackElapsedMs = 0U;
}

/* 模拟角度线性映射到循迹加权误差，左偏为负。 */
static float simError(void)
{
    float magnitude = (float)trackAngleDeg
                      * PWM_TRACK_ERROR_FULL_SCALE
                      / PWM_TRACK_ANGLE_FULL_SCALE;

    return (trackPhase == PWM_TEST_PHASE_LEFT) ? -magnitude : magnitude;
}

/* 每 PWM_TRACK_TEST_PHASE_MS 换一次偏转方向。 */
static void advanceTrackPhase(void)
{
    trackElapsedMs += APP_TRACK_PERIOD_MS;
    if (trackElapsedMs < PWM_TRACK_TEST_PHASE_MS) {
        return;
    }

    trackElapsedMs = 0U;
    trackPhase = (trackPhase == PWM_TEST_PHASE_LEFT)
                 ? PWM_TEST_PHASE_RIGHT
                 : PWM_TEST_PHASE_LEFT;
    AppTracking_ResetState();
}

void AppTestMode_Task20ms(void)
{
    float error;
    float turn;

    if (!AppState()->running) {
        return;
    }

    if (trackSimEnabled != 0U) {
        advanceTrackPhase();
        error = simError();
    } else {
        IRTrack_Read();
        IRTrack_Compute();
        error = AppTracking_ComputeWeightedError();
    }

    turn = AppTracking_UpdatePidForError(error, TRACK_FOLLOW_DIFF_LIMIT_RPM);
    AppTracking_SetPwmSteer(turn);
}

void AppTestMode_SetAngle(uint8_t angleDeg)
{
    for (uint8_t i = 0U; i < ANGLE_TABLE_SIZE; i++) {
        if (angleTable[i] == angleDeg) {
            trackAngleDeg = angleDeg;
            break;
        }
    }

    AppTracking_ResetState();
    AppTestMode_ResetPwmTrack();
}

void AppTestMode_NextAngle(void)
{
    uint8_t index = 0U;

    for (uint8_t i = 0U; i < ANGLE_TABLE_SIZE; i++) {
        if (angleTable[i] == trackAngleDeg) {
            index = (uint8_t)((i + 1U) % ANGLE_TABLE_SIZE);
            break;
        }
    }

    trackAngleDeg = angleTable[index];

    AppTracking_ResetState();
    AppTestMode_ResetPwmTrack();
}

uint8_t AppTestMode_GetAngle(void)
{
    return trackAngleDeg;
}

void AppTestMode_StepBasePwm(int16_t step)
{
    trackBasePwm = clampBasePwm((int16_t)(trackBasePwm + step));
}

int16_t AppTestMode_GetBasePwm(void)
{
    return trackBasePwm;
}

void AppTestMode_SetSimEnabled(uint8_t enabled)
{
    trackSimEnabled = (enabled != 0U) ? 1U : 0U;

    AppTracking_ResetState();
    if (trackSimEnabled != 0U) {
        AppTestMode_ResetPwmTrack();
    }
}

uint8_t AppTestMode_GetSimEnabled(void)
{
    return trackSimEnabled;
}

PwmTestPhase_t AppTestMode_GetTrackPhase(void)
{
    return trackPhase;
}

const char *AppPhaseName(PwmTestPhase_t phase)
{
    switch (phase) {
    case PWM_TEST_PHASE_FORWARD: return "FORWARD";
    case PWM_TEST_PHASE_REVERSE: return "REVERSE";
    case PWM_TEST_PHASE_LEFT:    return "LEFT";
    case PWM_TEST_PHASE_RIGHT:   return "RIGHT";
    default:                     return "--";
    }
}

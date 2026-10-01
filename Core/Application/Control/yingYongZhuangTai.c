#include "yingYongZhuangTai.h"
#include "yingYongPeiZhi.h"
#include "motor.h"

#define SPEED_LEVEL_DEFAULT 2U
#define SPEED_TABLE_G1_RPM 30.0f
#define SPEED_TABLE_G2_RPM 55.0f
#define SPEED_TABLE_G3_RPM 80.0f
#define SPEED_TABLE_G4_RPM 105.0f
#define SPEED_TABLE_G5_RPM 130.0f
#define SPEED_TABLE_G6_RPM 170.0f
#define SPEED_PID_KP 0.25f
#define SPEED_PID_KI 0.020f
#define SPEED_PID_KD 0.0f
#define SPEED_PID_OUT_MAX 100.0f
#define SPEED_PID_OUT_MIN (-100.0f)

workMode_t workMode = modeTrack;
uint8_t runFlag = 0;
uint8_t vofaEnabled = 1;
PID_t speedPidLeft;
PID_t speedPidRight;
PID_t speedPidAverage;
float targetLeftRpm = 0.0f;
float targetRightRpm = 0.0f;
Encoder encoderLeft;
Encoder encoderRight;
MPU6050 imu;

static uint8_t speedLevel = SPEED_LEVEL_DEFAULT;
static uint8_t lapTarget = TRACK_LAP_TARGET_DEFAULT;
static uint8_t completedLaps = 0U;
static appStopReason_t lastStopReason = stopReasonNone;
static volatile uint8_t stopEventPending = 0U;

static const float speedTable[SPEED_LEVEL_COUNT] = {
    SPEED_TABLE_G1_RPM,
    SPEED_TABLE_G2_RPM,
    SPEED_TABLE_G3_RPM,
    SPEED_TABLE_G4_RPM,
    SPEED_TABLE_G5_RPM,
    SPEED_TABLE_G6_RPM
};

static const Parameter wheelParameter = {
    .multiple = ENCODER_MULTIPLE,
    .reduction_ratio = ENCODER_REDUCTION_RATIO,
    .ppr = ENCODER_PPR,
    .r = ENCODER_WHEEL_RADIUS_M
};

static void appStateInitSpeedPid(PID_t *pid)
{
    PID_Init(pid);
    PID_SetParam(pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);
    pid->OutMax = SPEED_PID_OUT_MAX;
    pid->OutMin = SPEED_PID_OUT_MIN;
}

static void appStateResetSpeedPid(PID_t *pid)
{
    PID_SetTarget(pid, 0.0f);
    PID_ResetRuntime(pid);
}

void appStateInit(void)
{
    initEncoder(&encoderLeft, wheelParameter);
    initEncoder(&encoderRight, wheelParameter);
    appStateInitSpeedPid(&speedPidLeft);
    appStateInitSpeedPid(&speedPidRight);
    appStateInitSpeedPid(&speedPidAverage);
    targetLeftRpm = 0.0f;
    targetRightRpm = 0.0f;
}

void appStateResetMotion(void)
{
    runFlag = 0;
    targetLeftRpm = 0.0f;
    targetRightRpm = 0.0f;
    appStateResetSpeedPid(&speedPidLeft);
    appStateResetSpeedPid(&speedPidRight);
    appStateResetSpeedPid(&speedPidAverage);
    Motor_SetSpeedA(0);
    Motor_SetSpeedB(0);
}

void appStateResetSpeedPidRuntime(void)
{
    appStateResetSpeedPid(&speedPidLeft);
    appStateResetSpeedPid(&speedPidRight);
    appStateResetSpeedPid(&speedPidAverage);
}

void appStateReportStop(appStopReason_t reason)
{
    if (reason == stopReasonNone) return;
    lastStopReason = reason;
    stopEventPending = 1U;
}

uint8_t appStateGetStopEvent(appStopReason_t *reason)
{
    if ((stopEventPending == 0U) || (reason == 0)) return 0U;
    *reason = lastStopReason;
    return 1U;
}

void appStateClearStopEvent(void)
{
    lastStopReason = stopReasonNone;
    stopEventPending = 0U;
}

uint8_t appStateGetSpeedLevel(void) { return speedLevel; }

void appStateSetSpeedLevel(uint8_t level)
{
    if (level < 1U) level = SPEED_LEVEL_COUNT;
    if (level > SPEED_LEVEL_COUNT) level = 1U;
    speedLevel = level;
}

void appStateStepSpeedLevel(int8_t direction)
{
    int16_t next = (int16_t)speedLevel + direction;
    if (next > (int16_t)SPEED_LEVEL_COUNT) next = 1;
    if (next < 1) next = (int16_t)SPEED_LEVEL_COUNT;
    speedLevel = (uint8_t)next;
}

float appStateGetSpeedRpm(void) { return speedTable[speedLevel - 1U]; }

uint8_t appStateGetLapTarget(void) { return lapTarget; }

void appStateSetLapTarget(uint8_t target)
{
    if (target < 1U) target = 1U;
    if (target > TRACK_LAP_TARGET_MAX) target = TRACK_LAP_TARGET_MAX;
    lapTarget = target;
}

void appStateCycleLapTarget(void)
{
    uint8_t next = (uint8_t)(lapTarget + 1U);
    if (next > TRACK_LAP_TARGET_MAX) next = 1U;
    appStateSetLapTarget(next);
}

uint8_t appStateGetCompletedLaps(void) { return completedLaps; }

void appStateIncrementCompletedLaps(void)
{
    if (completedLaps < lapTarget) completedLaps++;
}

void appStateResetLapProgress(void) { completedLaps = 0U; }

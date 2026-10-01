#include "yingYongKongZhi.h"
#include "yingYongMingLing.h"
#include "app_config.h"
#include "app_imu.h"
#include "yingYongZhuangTai.h"
#include "yingYongYaoCe.h"
#include "yingYongXunJi.h"
#include "motor.h"
#include "irtrack.h"

typedef struct {
    PwmTestPhase_t speedPhase;
    uint32_t speedElapsedMs;
    PwmTestPhase_t trackPhase;
    uint32_t trackElapsedMs;
    uint8_t trackSimEnabled;
    uint8_t trackAngleDeg;
    int16_t trackBasePwm;
    int16_t finalLeftPwm;
    int16_t finalRightPwm;
} PwmTestRuntime_t;

static float filteredSpeedLeft = 0.0f;
static float filteredSpeedRight = 0.0f;
static uint8_t speedFilterInitialized = 0U;
static float speedSampleLeft[3] = {0.0f, 0.0f, 0.0f};
static float speedSampleRight[3] = {0.0f, 0.0f, 0.0f};
static uint8_t speedSampleIndex = 0U;
static const uint8_t pwmTrackAngleTable[] = {5U, 10U, 45U, 90U};
static PwmTestRuntime_t pwmTest = {
    PWM_TEST_PHASE_FORWARD,
    0U,
    PWM_TEST_PHASE_LEFT,
    0U,
    1U,
    5U,
    PWM_TRACK_BASE_DEFAULT_PERCENT,
    0,
    0
};

static float appControlMedian3(float a, float b, float c)
{
    if (((a >= b) && (a <= c)) || ((a >= c) && (a <= b))) return a;
    if (((b >= a) && (b <= c)) || ((b >= c) && (b <= a))) return b;
    return c;
}

static float appControlClampSpeedTarget(float target)
{
    if (target > SPEED_TARGET_LIMIT_RPM) target = SPEED_TARGET_LIMIT_RPM;
    if (target < -SPEED_TARGET_LIMIT_RPM) target = -SPEED_TARGET_LIMIT_RPM;
    return target;
}

static int16_t appControlClampMotorOutput(int16_t output)
{
    if (output > MOTOR_OUTPUT_MAX_PERCENT) output = MOTOR_OUTPUT_MAX_PERCENT;
    if (output < MOTOR_OUTPUT_MIN_PERCENT) output = MOTOR_OUTPUT_MIN_PERCENT;
    return output;
}

static void appControlResetSpeedFilter(void)
{
    filteredSpeedLeft = 0.0f;
    filteredSpeedRight = 0.0f;
    speedSampleLeft[0] = 0.0f;
    speedSampleLeft[1] = 0.0f;
    speedSampleLeft[2] = 0.0f;
    speedSampleRight[0] = 0.0f;
    speedSampleRight[1] = 0.0f;
    speedSampleRight[2] = 0.0f;
    speedSampleIndex = 0U;
    speedFilterInitialized = 0U;
}

static void appControlResetPwmSpeedTest(void)
{
    pwmTest.speedPhase = PWM_TEST_PHASE_FORWARD;
    pwmTest.speedElapsedMs = 0U;
    appStateResetSpeedPidRuntime();
    targetLeftRpm = 0.0f;
    targetRightRpm = 0.0f;
}

static void appControlStartPwmSpeedTest(void)
{
    float target = appStateGetSpeedRpm();
    appControlResetPwmSpeedTest();
    targetLeftRpm = target;
    targetRightRpm = target;
}

static void appControlUpdatePwmSpeedPhase(void)
{
    float target;

    if (!runFlag) return;
    pwmTest.speedElapsedMs += APP_CONTROL_PERIOD_MS;
    if (pwmTest.speedElapsedMs < PWM_SPEED_TEST_PHASE_MS) return;

    pwmTest.speedElapsedMs = 0U;
    pwmTest.speedPhase = (pwmTest.speedPhase == PWM_TEST_PHASE_FORWARD)
                         ? PWM_TEST_PHASE_REVERSE
                         : PWM_TEST_PHASE_FORWARD;
    target = appStateGetSpeedRpm();
    if (pwmTest.speedPhase == PWM_TEST_PHASE_REVERSE) target = -target;
    appStateResetSpeedPidRuntime();
    targetLeftRpm = target;
    targetRightRpm = target;
}

static void appControlStepPwmTrackBase(int16_t step)
{
    pwmTest.trackBasePwm += step;
    if (pwmTest.trackBasePwm > 100) pwmTest.trackBasePwm = 100;
    if (pwmTest.trackBasePwm < 0) pwmTest.trackBasePwm = 0;
}

static void appControlResetPwmTrackTest(void)
{
    pwmTest.trackPhase = PWM_TEST_PHASE_LEFT;
    pwmTest.trackElapsedMs = 0U;
}

static void appControlPwmTrackAdvance(void)
{
    pwmTest.trackElapsedMs += APP_TRACK_PERIOD_MS;
    if (pwmTest.trackElapsedMs < PWM_TRACK_TEST_PHASE_MS) return;
    pwmTest.trackElapsedMs = 0U;
    pwmTest.trackPhase = (pwmTest.trackPhase == PWM_TEST_PHASE_LEFT)
                         ? PWM_TEST_PHASE_RIGHT
                         : PWM_TEST_PHASE_LEFT;
    appTrackingResetState();
}

static float appControlPwmTrackSimError(void)
{
    float magnitude = (float)pwmTest.trackAngleDeg
                      * PWM_TRACK_ERROR_FULL_SCALE
                      / PWM_TRACK_ANGLE_FULL_SCALE;
    return (pwmTest.trackPhase == PWM_TEST_PHASE_LEFT) ? -magnitude : magnitude;
}

void appControlPwmTrackTask20ms(void)
{
    float error;
    float turn;

    if (!runFlag) return;
    if (pwmTest.trackSimEnabled) {
        appControlPwmTrackAdvance();
        error = appControlPwmTrackSimError();
    } else {
        IRTrack_Read();
        IRTrack_Compute();
        error = appTrackingComputeWeightedError();
    }
    turn = appTrackingUpdatePidForError(error, TRACK_FOLLOW_DIFF_LIMIT_RPM);
    appTrackingSetPwmSteer(turn);
}

void appControlPwmTrackStart(void)
{
    appTrackingResetState();
    appControlResetPwmTrackTest();
}

void appControlPwmTrackSetSimEnabled(uint8_t enabled)
{
    pwmTest.trackSimEnabled = (enabled != 0U) ? 1U : 0U;
    appTrackingResetState();
    if (pwmTest.trackSimEnabled) appControlResetPwmTrackTest();
}

uint8_t appControlPwmTrackGetSimEnabled(void)
{
    return pwmTest.trackSimEnabled;
}

void appControlPwmTrackSetAngle(uint8_t angleDeg)
{
    uint8_t i;
    for (i = 0U; i < sizeof(pwmTrackAngleTable); i++) {
        if (pwmTrackAngleTable[i] == angleDeg) {
            pwmTest.trackAngleDeg = angleDeg;
            break;
        }
    }
    appTrackingResetState();
    appControlResetPwmTrackTest();
}

void appControlPwmTrackNextAngle(void)
{
    uint8_t i;
    uint8_t index = 0U;
    for (i = 0U; i < sizeof(pwmTrackAngleTable); i++) {
        if (pwmTrackAngleTable[i] == pwmTest.trackAngleDeg) {
            index = (uint8_t)((i + 1U) % sizeof(pwmTrackAngleTable));
            break;
        }
    }
    pwmTest.trackAngleDeg = pwmTrackAngleTable[index];
    appTrackingResetState();
    appControlResetPwmTrackTest();
}

uint8_t appControlPwmTrackGetAngle(void)
{
    return pwmTest.trackAngleDeg;
}

PwmTestPhase_t appControlPwmTrackGetPhase(void)
{
    return pwmTest.trackPhase;
}

static void appControlResetControlState(void)
{
    appStateResetMotion();
    appControlResetSpeedFilter();
    appTrackingResetState();
    appControlResetPwmSpeedTest();
    appControlResetPwmTrackTest();
    pwmTest.finalLeftPwm = 0;
    pwmTest.finalRightPwm = 0;
}

static void appControlHandleCommand(commandMessage_t message)
{
    switch (message.type) {
    case commandStart:
        appStateClearStopEvent();
        if (!runFlag) {
            if (workMode == modeTrack) appTrackingStart();
            else if (workMode == modePwmSpeed) appControlStartPwmSpeedTest();
            else if (workMode == modePwmTrack) appControlPwmTrackStart();
        }
        runFlag = 1;
        break;
    case commandStop:
        if (runFlag) appStateReportStop(stopReasonUser);
        appControlResetControlState();
        break;
    case commandLapCycle:
        if ((workMode == modeTrack) && !runFlag) appStateCycleLapTarget();
        break;
    case commandModeTrack:
        if (runFlag) appStateReportStop(stopReasonModeSwitch);
        appControlResetControlState();
        workMode = modeTrack;
        appTrackingStart();
        break;
    case commandModePwmSpeed:
        if (runFlag) appStateReportStop(stopReasonModeSwitch);
        appControlResetControlState();
        workMode = modePwmSpeed;
        appControlStartPwmSpeedTest();
        break;
    case commandModePwmTrack:
        if (runFlag) appStateReportStop(stopReasonModeSwitch);
        appControlResetControlState();
        workMode = modePwmTrack;
        appControlPwmTrackStart();
        break;
    case commandPwmSpeedNextGear:
        appStateStepSpeedLevel(1);
        if (workMode == modePwmSpeed) appControlStartPwmSpeedTest();
        break;
    case commandSetSpeedGear:
        appStateSetSpeedLevel((uint8_t)message.value);
        if (workMode == modePwmSpeed) appControlStartPwmSpeedTest();
        break;
    case commandPwmTrackNext:
        if (workMode == modePwmTrack) appControlPwmTrackNextAngle();
        break;
    case commandPwmTrackSetAngle:
        appControlPwmTrackSetAngle((uint8_t)message.value);
        break;
    case commandPwmTrackBaseStep:
        appControlStepPwmTrackBase((int16_t)message.value);
        break;
    case commandPwmTrackSim:
        appControlPwmTrackSetSimEnabled(message.value != 0.0f ? 1U : 0U);
        break;
    case commandQuiet:
        vofaEnabled = 0;
        break;
    case commandLog:
        vofaEnabled = 1;
        break;
    default:
        break;
    }
}

PwmTestPhase_t appControlGetPwmSpeedPhase(void)
{
    return pwmTest.speedPhase;
}

uint8_t appControlGetPwmTrackBasePwm(void)
{
    return (uint8_t)pwmTest.trackBasePwm;
}

void appControlResetModeState(void)
{
    appControlResetControlState();
}

static void appControlFillTelemetry(float targetLeft, float targetRight,
                                    int16_t leftPwm, int16_t rightPwm,
                                    float steerRpm, float trackError,
                                    int16_t turnPhase, float rpmDifference)
{
    telemetryData.targetLeftRpm = targetLeft;
    telemetryData.targetRightRpm = targetRight;
    telemetryData.actualLeftRpm = encoderLeft.velocity.angular;
    telemetryData.actualRightRpm = encoderRight.velocity.angular;
    telemetryData.filteredLeftRpm = filteredSpeedLeft;
    telemetryData.filteredRightRpm = filteredSpeedRight;
    telemetryData.steerRpm = steerRpm;
    telemetryData.actualDiffRpm = rpmDifference;
    telemetryData.yawAngleDeg = AppImu_Get()->yaw;
    telemetryData.trackError = trackError;
    telemetryData.mode = (int16_t)workMode;
    telemetryData.runFlag = runFlag;
    telemetryData.gear = (int16_t)appStateGetSpeedLevel();
    telemetryData.turnPhase = turnPhase;
    telemetryData.commandVoltageLeft =
        (float)leftPwm * MOTOR_SUPPLY_VOLTAGE / 100.0f;
    telemetryData.commandVoltageRight =
        (float)rightPwm * MOTOR_SUPPLY_VOLTAGE / 100.0f;
    telemetryData.fresh = 1;
    pwmTest.finalLeftPwm = leftPwm;
    pwmTest.finalRightPwm = rightPwm;
}

void appControlTask50ms(void)
{
    float rpmDifference;
    float targetLeft;
    float targetRight;
    int16_t leftPwm;
    int16_t rightPwm;

    updateEncoderLoopSimpleVersion(&encoderRight, APP_CONTROL_PERIOD_MS, TIM3);
    updateEncoderLoopSimpleVersion(&encoderLeft, APP_CONTROL_PERIOD_MS, TIM2);

    while (appCommandQueueAvailable()) {
        commandMessage_t message;
        if (appCommandQueuePop(&message) < 0) break;
        appControlHandleCommand(message);
    }

    if (workMode == modePwmSpeed) appControlUpdatePwmSpeedPhase();

    rpmDifference = encoderLeft.velocity.angular - encoderRight.velocity.angular;

    if (!runFlag) {
        appControlFillTelemetry(0.0f, 0.0f, 0, 0, 0.0f, 0.0f,
                                (int16_t)TRACK_PHASE_FOLLOW, rpmDifference);
        appControlResetSpeedFilter();
        Motor_SetSpeedA(0);
        Motor_SetSpeedB(0);
        return;
    }

    targetLeft = appControlClampSpeedTarget(targetLeftRpm);
    targetRight = appControlClampSpeedTarget(targetRightRpm);

    if (workMode == modePwmTrack) {
        float steerRpm = appTrackingGetSteer();
        float steerPwm = steerRpm * appTrackingGetSteerRpmToPwmGain();
        leftPwm = (int16_t)((float)pwmTest.trackBasePwm + steerPwm);
        rightPwm = (int16_t)((float)pwmTest.trackBasePwm - steerPwm);
        if (leftPwm > 100) leftPwm = 100;
        if (leftPwm < 0) leftPwm = 0;
        if (rightPwm > 100) rightPwm = 100;
        if (rightPwm < 0) rightPwm = 0;

        Motor_SetSpeedA(rightPwm);
        Motor_SetSpeedB(leftPwm);
        appControlFillTelemetry(0.0f, 0.0f, leftPwm, rightPwm,
                                steerRpm, appTrackingGetWeightedError(),
                                (int16_t)appControlPwmTrackGetPhase(),
                                rpmDifference);
        return;
    }

    if (!speedFilterInitialized) {
        speedSampleLeft[0] = encoderLeft.velocity.angular;
        speedSampleLeft[1] = speedSampleLeft[0];
        speedSampleLeft[2] = speedSampleLeft[0];
        speedSampleRight[0] = encoderRight.velocity.angular;
        speedSampleRight[1] = speedSampleRight[0];
        speedSampleRight[2] = speedSampleRight[0];
        filteredSpeedLeft = speedSampleLeft[0];
        filteredSpeedRight = speedSampleRight[0];
        speedFilterInitialized = 1U;
    } else {
        speedSampleLeft[speedSampleIndex] = encoderLeft.velocity.angular;
        speedSampleRight[speedSampleIndex] = encoderRight.velocity.angular;
        filteredSpeedLeft = filteredSpeedLeft * ENCODER_FILTER_PREVIOUS
                            + appControlMedian3(speedSampleLeft[0],
                                                speedSampleLeft[1],
                                                speedSampleLeft[2])
                              * ENCODER_FILTER_CURRENT;
        filteredSpeedRight = filteredSpeedRight * ENCODER_FILTER_PREVIOUS
                             + appControlMedian3(speedSampleRight[0],
                                                 speedSampleRight[1],
                                                 speedSampleRight[2])
                               * ENCODER_FILTER_CURRENT;
        speedSampleIndex++;
        if (speedSampleIndex >= 3U) speedSampleIndex = 0U;
    }

    if (workMode == modeTrack) {
        float averageTarget = appTrackingGetAverageTargetRpm();
        float averageActual = (filteredSpeedLeft + filteredSpeedRight) * 0.5f;
        float steerRpm = appTrackingGetSteer();
        float steerPwm = steerRpm * appTrackingGetSteerRpmToPwmGain();
        float basePwm;

        basePwm = PID_UpdateSpeed(&speedPidAverage, averageTarget, averageActual);
        leftPwm = (int16_t)(basePwm + steerPwm);
        rightPwm = (int16_t)(basePwm - steerPwm);
        if (leftPwm < 0) leftPwm = 0;
        if (rightPwm < 0) rightPwm = 0;
    } else {
        leftPwm = (int16_t)PID_UpdateSpeed(&speedPidLeft, targetLeft, filteredSpeedLeft);
        rightPwm = (int16_t)PID_UpdateSpeed(&speedPidRight, targetRight, filteredSpeedRight);
    }

    leftPwm = appControlClampMotorOutput(leftPwm);
    rightPwm = appControlClampMotorOutput(rightPwm);
    Motor_SetSpeedA(rightPwm);
    Motor_SetSpeedB(leftPwm);

    if (workMode == modeTrack) {
        appControlFillTelemetry(
            appTrackingGetAverageTargetRpm() + appTrackingGetSteer(),
            appTrackingGetAverageTargetRpm() - appTrackingGetSteer(),
            leftPwm, rightPwm, appTrackingGetSteer(),
            appTrackingGetWeightedError(),
            (int16_t)appTrackingGetTurnPhase(), rpmDifference);
    } else {
        appControlFillTelemetry(targetLeft, targetRight, leftPwm, rightPwm,
                                0.0f, 0.0f, (int16_t)pwmTest.speedPhase,
                                rpmDifference);
    }
}

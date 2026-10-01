/**
 * @file    app_speedloop.c
 * @brief   50 ms 控制任务的实现。
 */

#include "app_speedloop.h"
#include "app_config.h"
#include "app_imu.h"
#include "app_motion.h"
#include "app_state.h"
#include "app_testmode.h"
#include "app_cmd.h"
#include "app_tracking.h"
#include "app_telemetry.h"

/* 左右各一个增量式速度环，参数来自配置，可以在线整定。 */
static Pid_t pidLeft;
static Pid_t pidRight;

static const PidConfig_t speedPidConfig = {
    .kp = SPEED_PID_KP,
    .ki = SPEED_PID_KI,
    .kd = SPEED_PID_KD,
    .dt = 0.0f,
    .out_min = SPEED_PID_OUT_MIN,
    .out_max = SPEED_PID_OUT_MAX,
    .zero_target_hold = SPEED_PID_ZERO_HOLD_RPM,
    .out_offset = SPEED_PID_OUT_OFFSET_PERCENT,
    .mode = PID_MODE_INCREMENTAL,
    .reset_on_sign_change = 1U,
    .derivative_on_measurement = 1U,
};

static float clampSpeedTarget(float target)
{
    if (target > SPEED_TARGET_LIMIT_RPM) return SPEED_TARGET_LIMIT_RPM;
    if (target < -SPEED_TARGET_LIMIT_RPM) return -SPEED_TARGET_LIMIT_RPM;
    return target;
}

void AppSpeedLoop_Init(void)
{
    Pid_Init(&pidLeft, &speedPidConfig);
    Pid_Init(&pidRight, &speedPidConfig);
}

void AppSpeedLoop_ResetRuntime(void)
{
    Pid_Reset(&pidLeft);
    Pid_Reset(&pidRight);
}

void AppSpeedLoop_ResetMotion(void)
{
    AppState_SetRunning(0U);
    AppState_SetWheelTargets(0.0f, 0.0f);
    AppSpeedLoop_ResetRuntime();
    AppMotion_Stop();
}

void AppSpeedLoop_ResetModeState(void)
{
    AppSpeedLoop_ResetMotion();
    AppTracking_ResetState();
    AppTestMode_ResetPwmSpeed();
    AppTestMode_ResetPwmTrack();
}

void AppSpeedLoop_SetGain(SpeedChannel_t channel, PidField_t field, float value)
{
    Pid_t *pid = (channel == SPEED_CH_LEFT) ? &pidLeft : &pidRight;
    Pid_SetGain(pid, field, value);
}

void AppSpeedLoop_GetInfo(SpeedChannel_t channel, SpeedPidInfo_t *out)
{
    const Pid_t *pid = (channel == SPEED_CH_LEFT) ? &pidLeft : &pidRight;

    if (out == 0) {
        return;
    }

    out->kp = pid->cfg.kp;
    out->ki = pid->cfg.ki;
    out->kd = pid->cfg.kd;
    out->target = pid->target;
    out->actual = pid->actual;
    out->out = pid->out;
}

/* PWM 速度测试：左右轮同向同速，按档位速度跑，定时换向。 */
static void startPwmSpeedTest(void)
{
    float target = AppState()->gear_rpm;

    AppTestMode_ResetPwmSpeed();
    AppSpeedLoop_ResetRuntime();
    AppState_SetWheelTargets(target, target);
}

/* 模式切换前的统一复位。 */
static void resetBeforeModeChange(void)
{
    if (AppState()->running) {
        AppState_ReportStop(stopReasonModeSwitch);
    }
    AppSpeedLoop_ResetModeState();
}

static void handleCommand(commandMessage_t message)
{
    switch (message.type) {
    case commandStart:
        AppState_ClearStopEvent();
        if (!AppState()->running) {
            if (AppState()->mode == modeTrack) {
                AppTracking_Start();
            } else if (AppState()->mode == modePwmSpeed) {
                startPwmSpeedTest();
            } else {
                AppTracking_ResetState();
                AppTestMode_ResetPwmTrack();
            }
        }
        AppState_SetRunning(1U);
        break;

    case commandStop:
        if (AppState()->running) {
            AppState_ReportStop(stopReasonUser);
        }
        AppSpeedLoop_ResetModeState();
        break;

    case commandLapCycle:
        if ((AppState()->mode == modeTrack) && !AppState()->running) {
            AppState_CycleLapTarget();
        }
        break;

    case commandModeTrack:
        resetBeforeModeChange();
        AppState_SetMode(modeTrack);
        AppTracking_Start();
        break;

    case commandModePwmSpeed:
        resetBeforeModeChange();
        AppState_SetMode(modePwmSpeed);
        startPwmSpeedTest();
        break;

    case commandModePwmTrack:
        resetBeforeModeChange();
        AppState_SetMode(modePwmTrack);
        AppTracking_ResetState();
        AppTestMode_ResetPwmTrack();
        break;

    case commandPwmSpeedNextGear:
        AppState_StepGear(1);
        if (AppState()->mode == modePwmSpeed) {
            startPwmSpeedTest();
        }
        break;

    case commandSetSpeedGear:
        (void)AppState_SetGear((uint8_t)message.value);
        if (AppState()->mode == modePwmSpeed) {
            startPwmSpeedTest();
        }
        break;

    case commandPwmTrackNext:
        if (AppState()->mode == modePwmTrack) {
            AppTestMode_NextAngle();
        }
        break;

    case commandPwmTrackSetAngle:
        AppTestMode_SetAngle((uint8_t)message.value);
        break;

    case commandPwmTrackBaseStep:
        AppTestMode_StepBasePwm((int16_t)message.value);
        break;

    case commandPwmTrackSim:
        AppTestMode_SetSimEnabled((message.value != 0.0f) ? 1U : 0U);
        break;

    case commandQuiet:
        AppState_SetVofa(0U);
        break;

    case commandLog:
        AppState_SetVofa(1U);
        break;

    default:
        break;
    }
}

static void fillTelemetry(float targetLeft, float targetRight,
                          int16_t leftPwm, int16_t rightPwm,
                          float steerRpm, float trackError,
                          int16_t turnPhase, float rpmDifference)
{
    telemetryData.targetLeftRpm = targetLeft;
    telemetryData.targetRightRpm = targetRight;
    telemetryData.actualLeftRpm = AppMotion()->left_rpm_raw;
    telemetryData.actualRightRpm = AppMotion()->right_rpm_raw;
    telemetryData.filteredLeftRpm = AppMotion()->left_rpm;
    telemetryData.filteredRightRpm = AppMotion()->right_rpm;
    telemetryData.steerRpm = steerRpm;
    telemetryData.actualDiffRpm = rpmDifference;
    telemetryData.yawAngleDeg = AppImu_Get()->yaw;
    telemetryData.trackError = trackError;
    telemetryData.mode = (int16_t)AppState()->mode;
    telemetryData.runFlag = AppState()->running;
    telemetryData.gear = (int16_t)AppState()->gear;
    telemetryData.turnPhase = turnPhase;
    telemetryData.commandVoltageLeft =
        (float)leftPwm * MOTOR_SUPPLY_VOLTAGE / 100.0f;
    telemetryData.commandVoltageRight =
        (float)rightPwm * MOTOR_SUPPLY_VOLTAGE / 100.0f;
    telemetryData.fresh = 1;
}

void AppSpeedLoop_Task50ms(void)
{
    int16_t leftPwm = 0;
    int16_t rightPwm = 0;
    float   targetLeft = 0.0f;
    float   targetRight = 0.0f;
    float   steerRpm = 0.0f;
    float   trackError = 0.0f;
    int16_t turnPhase = (int16_t)TRK_FOLLOW;
    float   rpmDifference;

    AppMotion_Update();

    while (AppCmd_Available()) {
        commandMessage_t message;
        if (AppCmd_Pop(&message) < 0) {
            break;
        }
        handleCommand(message);
    }

    rpmDifference = AppMotion()->left_rpm_raw - AppMotion()->right_rpm_raw;

    /* 循迹模块请求停车时在这里落地，避免它直接依赖速度环。 */
    if (AppState_TakeStopRequest() != 0U) {
        AppSpeedLoop_ResetMotion();
    }

    if (!AppState()->running) {
        AppSpeedLoop_ResetRuntime();   /* 停止时清积分，避免下次启动窜车 */
        AppMotion_Stop();
        AppState_SetWheelTargets(0.0f, 0.0f);
        fillTelemetry(0.0f, 0.0f, 0, 0, 0.0f, 0.0f,
                      (int16_t)TRK_FOLLOW, rpmDifference);
        return;
    }

    if (AppState()->mode == modePwmTrack) {
        /* 开环测试：只把差速 PWM 叠到基础占空比上，不走速度环。 */
        float steerPwm = AppTracking_GetSteer()
                         * AppTracking_GetSteerRpmToPwmGain();
        int16_t base = AppTestMode_GetBasePwm();

        leftPwm = (int16_t)((float)base + steerPwm);
        rightPwm = (int16_t)((float)base - steerPwm);
        if (leftPwm > MOTOR_OUTPUT_MAX_PERCENT) leftPwm = MOTOR_OUTPUT_MAX_PERCENT;
        if (leftPwm < 0) leftPwm = 0;
        if (rightPwm > MOTOR_OUTPUT_MAX_PERCENT) rightPwm = MOTOR_OUTPUT_MAX_PERCENT;
        if (rightPwm < 0) rightPwm = 0;

        steerRpm = AppTracking_GetSteer();
        trackError = AppTracking_Output()->e_y;
        turnPhase = (int16_t)AppTracking_Output()->state;

    } else if (AppState()->mode == modeTrack) {
        /* 级联：循迹环出目标转速，左右速度环各自闭环到 PWM。
           差速是闭环的，某个轮子跟不上目标时内环会自己补。 */
        float base = AppTracking_Output()->base_rpm;
        float steer = AppTracking_GetSteer();

        targetLeft = clampSpeedTarget(base + steer);
        targetRight = clampSpeedTarget(base - steer);

        /* 循迹时不允许轮子反转：把输出下限收到 0，由 PID 自己限幅，
           不在外面把负数改成 0——那会让内部输出停在 -100，出弯爬不回来。 */
        Pid_SetLimits(&pidLeft, 0.0f, SPEED_PID_OUT_MAX);
        Pid_SetLimits(&pidRight, 0.0f, SPEED_PID_OUT_MAX);

        leftPwm = (int16_t)Pid_Update(&pidLeft, targetLeft, AppMotion()->left_rpm);
        rightPwm = (int16_t)Pid_Update(&pidRight, targetRight, AppMotion()->right_rpm);

        AppState_SetWheelTargets(targetLeft, targetRight);

        steerRpm = steer;
        trackError = AppTracking_Output()->e_y;
        turnPhase = (int16_t)AppTracking_Output()->state;

    } else {
        /* 手动模式：左右各自目标，允许反转（原地转需要一正一反）。
           PWM 速度测试也走这条分支，只是目标由测试相位决定。 */
        if (AppState()->mode == modePwmSpeed) {
            if (AppTestMode_TickPwmSpeed() != 0U) {
                float target = AppState()->gear_rpm
                               * (float)AppTestMode_GetPwmSpeedSign();
                AppSpeedLoop_ResetRuntime();
                AppState_SetWheelTargets(target, target);
            }
        }

        targetLeft = clampSpeedTarget(AppState()->target_left_rpm);
        targetRight = clampSpeedTarget(AppState()->target_right_rpm);

        Pid_SetLimits(&pidLeft, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);
        Pid_SetLimits(&pidRight, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);

        leftPwm = (int16_t)Pid_Update(&pidLeft, targetLeft, AppMotion()->left_rpm);
        rightPwm = (int16_t)Pid_Update(&pidRight, targetRight, AppMotion()->right_rpm);
    }

    AppMotion_Apply(leftPwm, rightPwm);
    fillTelemetry(targetLeft, targetRight, leftPwm, rightPwm,
                  steerRpm, trackError, turnPhase, rpmDifference);
}

/**
 * @file    yingYongKongZhi.c
 * @brief   应用层运动控制任务。
 * @note    本文件负责消费统一命令队列、执行左右轮独立增量式速度环、
 *          低速补偿、直行航向修正、堵转保护和遥测快照更新。
 */

#include "yingYongKongZhi.h"
#include "yingYongMingLing.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "yingYongYaoCe.h"
#include "yingYongXunJi.h"
#include "motor.h"
#include "mpu6050.h"

#include <math.h>

/* ------------------------------------------------------------------------- */
/* 速度滤波、低速补偿和电机输出常调参数                                      */
/* ------------------------------------------------------------------------- */

/* 编码器一阶低通滤波权重。 */
#define ENCODER_FILTER_PREVIOUS 0.75f
/* 编码器当前采样在一阶低通滤波中的权重。 */
#define ENCODER_FILTER_CURRENT 0.25f

/* 电机 PWM 输出上下限，单位百分比。 */
#define MOTOR_OUTPUT_MAX_PERCENT 100
/* 电机 PWM 输出下限，允许反向驱动到 -100%。 */
#define MOTOR_OUTPUT_MIN_PERCENT (-100)

/* 低速目标下的最小 PWM 补偿。 */
#define MOTOR_LOW_SPEED_THRESHOLD_RPM 20.0f
/* 目标转速低于低速阈值时施加的最小 PWM 补偿值，单位百分比。 */
#define MOTOR_LOW_SPEED_MIN_PWM 12

/* Speeds below this value are treated as a zero-speed target. */
#define SPEED_PID_STOP_HOLD_RPM 0.5f

/* 左右轮一阶低通滤波后的转速，作为两个独立增量式速度环的反馈。 */
static float filteredSpeedLeft = 0.0f;
static float filteredSpeedRight = 0.0f;
static uint8_t speedFilterInitialized = 0;

/* 编码器三点中值滤波缓存，用于抑制单周期异常计数尖峰。 */
static float speedSampleLeft[3] = {0.0f, 0.0f, 0.0f};
static float speedSampleRight[3] = {0.0f, 0.0f, 0.0f};
static uint8_t speedSampleIndex = 0;

/* 电机堵转条件连续成立的周期数，达到阈值后立即停车保护。 */
static uint8_t motorStallTicks = 0;

static uint8_t straightYawControlActive = 0U;
static uint8_t yawUpdateTick = 0U;

/**
 * @brief  对三个浮点数取中值。
 * @param  valueA 第一个输入值。
 * @param  valueB 第二个输入值。
 * @param  valueC 第三个输入值。
 * @return 三个输入值的中值。
 * @note   编码器连续采样三次后取中值，可滤除单周期出现的异常大数或异常小数，
 *         同时不会像平均值那样把真实尖峰扩散到多个控制周期。
 */
static float appControlMedian3(float valueA, float valueB, float valueC)
{
    if (((valueA >= valueB) && (valueA <= valueC))
        || ((valueA >= valueC) && (valueA <= valueB))) {
        return valueA;
    }

    if (((valueB >= valueA) && (valueB <= valueC))
        || ((valueB >= valueC) && (valueB <= valueA))) {
        return valueB;
    }

    return valueC;
}

/**
 * @brief  将速度目标限制在允许的正反转范围内。
 * @param  target 待限制的目标转速，单位 RPM。
 * @return 限制后的目标转速。
 * @note   仅等价收敛原有左右目标限幅逻辑，阈值与执行顺序不变。
 */
static float appControlClampSpeedTarget(float target)
{
    if (target > SPEED_TARGET_LIMIT_RPM) {
        target = SPEED_TARGET_LIMIT_RPM;
    }
    if (target < -SPEED_TARGET_LIMIT_RPM) {
        target = -SPEED_TARGET_LIMIT_RPM;
    }
    return target;
}

/**
 * @brief  将最终 PWM 输出限制在统一范围内。
 * @param  outputPercent 待限制的 PWM 百分比。
 * @return 限制后的 PWM 百分比。
 * @note   仅等价收敛原有 PWM 限幅分支，最大和最小百分比不变。
 */
static int16_t appControlClampMotorOutput(int16_t outputPercent)
{
    if (outputPercent > MOTOR_OUTPUT_MAX_PERCENT) {
        outputPercent = MOTOR_OUTPUT_MAX_PERCENT;
    }
    if (outputPercent < MOTOR_OUTPUT_MIN_PERCENT) {
        outputPercent = MOTOR_OUTPUT_MIN_PERCENT;
    }
    return outputPercent;
}

/**
 * @brief  对低速非零目标补偿最小 PWM。
 * @param  targetRpm 当前速度环目标，单位 RPM。
 * @param  outputPercent 当前 PWM 输出，单位百分比。
 * @return 补偿后的 PWM 输出。
 * @note   仅在目标未到低速阈值、输出方向正确且小于最小补偿值时抬高，
 *         零输出和已经超过补偿值的输出保持不变；补偿阈值和顺序与原逻辑相同。
 */
static float appControlApplyLowSpeedCompensation(float targetRpm,
                                                  float outputPercent)
{
    if (targetRpm > 0.0f
        && targetRpm < MOTOR_LOW_SPEED_THRESHOLD_RPM
        && outputPercent > 0.0f
        && outputPercent < (float)MOTOR_LOW_SPEED_MIN_PWM) {
        return (float)MOTOR_LOW_SPEED_MIN_PWM;
    }

    if (targetRpm < 0.0f
        && targetRpm > -MOTOR_LOW_SPEED_THRESHOLD_RPM
        && outputPercent < 0.0f
        && outputPercent > -(float)MOTOR_LOW_SPEED_MIN_PWM) {
        return -(float)MOTOR_LOW_SPEED_MIN_PWM;
    }

    return outputPercent;
}

/**
 * @brief  判断一侧电机是否满足堵转保护条件。
 * @param  targetRpm 该侧目标转速，单位 RPM。
 * @param  actualRpm 该侧编码器实际转速，单位 RPM。
 * @param  outputPercent 该侧当前 PWM 输出，单位百分比。
 * @return 1 表示满足堵转条件，0 表示不满足。
 * @note   只有“目标速度已经较高、实际速度接近零、PWM 持续增大”三个条件同时
 *         成立时才判定堵转，避免起步瞬间或低速行驶时误触发。
 */
static uint8_t appControlMotorStalled(float targetRpm,
                                      float actualRpm,
                                      float outputPercent)
{
#if MOTOR_STALL_PROTECT_ENABLE
    return (fabsf(targetRpm) > MOTOR_STALL_TARGET_MIN_RPM)
           && (fabsf(actualRpm) < MOTOR_STALL_ACTUAL_MAX_RPM)
           && (fabsf(outputPercent) > MOTOR_STALL_OUTPUT_MIN_PERCENT);
#else
    (void)targetRpm;
    (void)actualRpm;
    (void)outputPercent;
    return 0U;
#endif
}

/**
 * @brief  复位左右轮速度滤波器和滤波输出。
 * @note   停车、切换模式或重新启动时调用，防止旧转速进入下一轮速度环。
 */
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
    motorStallTicks = 0U;
    speedFilterInitialized = 0;
}

/**
 * @brief  设置增量式速度环目标，并在目标方向变化时清除旧方向输出。
 * @param  pid 待设置目标的增量式速度 PID。
 * @param  target 新目标转速，单位 RPM，正负号表示旋转方向。
 * @note   增量式 PID 的 Out 会跨周期累加；目标从正转切换到反转或 0 时只清积分，
 *         旧 PWM 会让内侧轮继续正转。方向变化时将输出和误差历史归零，使停车
 *         或反转周期立即按新目标重新建立 PWM。
 */
static void appControlSetSpeedPidTarget(PID_t *pid, float target)
{
    float targetDelta = target - pid->Target;
    float feedforwardDelta = pid->Kf * targetDelta;
    int8_t oldDirection = (pid->Target > 0.0f)
                          ? 1
                          : ((pid->Target < 0.0f) ? -1 : 0);
    int8_t newDirection = (target > 0.0f)
                          ? 1
                          : ((target < 0.0f) ? -1 : 0);

    if (oldDirection != newDirection) {
        PID_ClearHistory(pid);
        pid->Out = 0.0f;
        pid->LastOutput = 0.0f;

        /*
         * 方向切换时从目标对应的前馈基线重新起调，避免增量 PID 从零开始
         * 缓慢爬过电机死区；正反转原地转时也能立即建立正确方向的转矩。
         */
        pid->Out = pid->Kf * target;
        pid->LastOutput = pid->Out;
    } else {
        /*
         * 目标同方向变化时，只补入目标差对应的前馈增量。速度环仍保持原来的
         * 增量式 PI 结构，目标不变时不会重复叠加前馈。
         */
        pid->Out += feedforwardDelta;
    }

    PID_ClampOutput(pid);
    pid->LastOutput = pid->Out;

    PID_SetTarget(pid, target);
}

/**
 * @brief  Force zero PWM when the speed target is zero.
 * @param  pid Incremental speed PID whose output will be constrained.
 * @param  target Current target speed in RPM.
 * @note   Nonzero targets retain the normal PID output range so that the
 *         controller can brake smoothly through zero.
 */
static void appControlConstrainSpeedPidOutput(PID_t *pid, float target)
{
    if (fabsf(target) <= SPEED_PID_STOP_HOLD_RPM) {
        pid->Out = 0.0f;
    }

    PID_ClampOutput(pid);
    pid->LastOutput = pid->Out;
}

/**
 * @brief  复位运动、速度和循迹控制状态。
 */
static void appControlResetControlState(void)
{
    appStateResetMotion();
    appControlResetSpeedFilter();
    appTrackingResetState();
}

/**
 * @brief  获取当前左右目标转速的平均值。
 * @return 左右目标转速平均值，单位 RPM。
 * @note   该平均值用于档位切换和手动转向命令生成；实际速度控制由左右轮
 *         两个独立增量式 PID 分别完成。
 */
static float appControlGetAverageTarget(void)
{
    return (targetLeftRpm + targetRightRpm) * 0.5f;
}

/**
 * @brief  以当前平均目标为中心生成左右差速目标。
 * @param  averageTargetRpm 左右目标转速的平均值，单位 RPM。
 * @param  turnRpm 左轮相对于平均目标的正向差速量，单位 RPM。
 * @note   该函数只生成目标量；当前阶段不叠加开环 PWM 差速前馈。
 */
static void appControlSetTurnTarget(float averageTargetRpm, float turnRpm)
{
    targetLeftRpm = averageTargetRpm + turnRpm;
    targetRightRpm = averageTargetRpm - turnRpm;
    turnTarget = turnRpm;
}

/**
 * @brief  档位变化后刷新手动模式的目标速度。
 */
static void appControlRefreshRunTargetAfterGearChange(void)
{
    if (workMode != modeRun || !runFlag) {
        return;
    }

    if (targetLeftRpm * targetRightRpm < 0.0f) {
        float sign = (targetLeftRpm < 0.0f) ? -1.0f : 1.0f;
        appControlSetTurnTarget(0.0f, appStateGetSpeedRpm() * sign);
    } else {
        float sign = (appControlGetAverageTarget() < 0.0f) ? -1.0f : 1.0f;
        appControlSetTurnTarget(appStateGetSpeedRpm() * sign, turnTarget);
    }
}

/**
 * @brief  执行一条应用层控制命令。
 * @param  message 待执行的命令消息。
 * @note   所有进入统一队列的控制命令都在此集中修改应用状态；USART1
 *         在线调参的 PID 增益和 duty 命令由 pid_tune 模块直接处理。
 */
static void appControlHandleCommand(commandMessage_t message)
{
    /* 所有控制命令在这里集中执行，避免按键和蓝牙分别修改电机状态。 */
    switch (message.type) {
    case commandSpeed:
        if (workMode == modeRun) {
            appControlSetTurnTarget(message.value, 0.0f);
        }
        break;
    case commandSpeedLeft:
        if (workMode == modeRun) {
            targetLeftRpm = message.value;
            turnTarget = 0.0f;
        }
        break;
    case commandSpeedRight:
        if (workMode == modeRun) {
            targetRightRpm = message.value;
            turnTarget = 0.0f;
        }
        break;
    case commandTurn:
        if (workMode == modeRun) {
            appControlSetTurnTarget(appControlGetAverageTarget(), message.value);
        }
        break;
    case commandPivot:
        /* 蓝牙满舵或串口 pivot 命令使用一正一反目标。 */
        if (workMode == modeRun) {
            appControlSetTurnTarget(0.0f, message.value);
        }
        break;
    case commandStart:
        appStateClearStopEvent();
        if ((workMode == modeTrack) && !runFlag) {
            appTrackingStart();
        }
        runFlag = 1;
        break;
    case commandStop:
        if (runFlag) {
            appStateReportStop(stopReasonUser);
        }
        pwmDuty = 0;
        appControlResetControlState();
        break;
    case commandLapCycle:
        if ((workMode == modeTrack) && !runFlag) {
            appStateCycleLapTarget();
        }
        break;
    case commandGearUp:
        appStateStepSpeedLevel(1);
        appControlRefreshRunTargetAfterGearChange();
        break;
    case commandGearDown:
        appStateStepSpeedLevel(-1);
        appControlRefreshRunTargetAfterGearChange();
        break;
    case commandModeTrack:
        /* 切换循迹前清除运动状态、红外图案和丢线计数。 */
        if (runFlag) {
            appStateReportStop(stopReasonModeSwitch);
        }
        appStateResetMotion();
        appControlResetSpeedFilter();
        pwmDuty = 0;
        appTrackingStart();
        workMode = modeTrack;
        if (appStateGetSpeedLevel() > TRACK_MAX_SPEED_LEVEL) {
            appStateSetSpeedLevel(TRACK_MAX_SPEED_LEVEL);
        }
        break;
    case commandModeRun:
        if (runFlag) {
            appStateReportStop(stopReasonModeSwitch);
        }
        appControlResetControlState();
        pwmDuty = 0;
        workMode = modeRun;
        break;
    case commandModePwm:
        if (runFlag) {
            appStateReportStop(stopReasonModeSwitch);
        }
        appControlResetControlState();
        workMode = modePwmSet;
        break;
    case commandSetSpeedKp:
        PID_SetField(&speedPidLeft, PID_FIELD_KP, message.value);
        PID_SetField(&speedPidRight, PID_FIELD_KP, message.value);
        break;
    case commandSetSpeedKi:
        PID_SetField(&speedPidLeft, PID_FIELD_KI, message.value);
        PID_SetField(&speedPidRight, PID_FIELD_KI, message.value);
        break;
    case commandSetSpeedKd:
        PID_SetField(&speedPidLeft, PID_FIELD_KD, message.value);
        PID_SetField(&speedPidRight, PID_FIELD_KD, message.value);
        break;
    case commandSetSpeedKf:
        PID_SetField(&speedPidLeft, PID_FIELD_KF, message.value);
        PID_SetField(&speedPidRight, PID_FIELD_KF, message.value);
        break;
    case commandSetYawKp:
        PID_SetField(&yawPid, PID_FIELD_KP, message.value);
        break;
    case commandSetYawKi:
        PID_SetField(&yawPid, PID_FIELD_KI, message.value);
        break;
    case commandSetYawKd:
        PID_SetField(&yawPid, PID_FIELD_KD, message.value);
        break;
    case commandSetYawKf:
        PID_SetField(&yawPid, PID_FIELD_KF, message.value);
        break;
    case commandYawTarget:
        yawTarget = message.value;
        PID_SetTarget(&yawPid, message.value);
        break;
    case commandQuiet:
        vofaEnabled = 0;
        break;
    case commandLog:
        vofaEnabled = 1;
        break;
    case commandImuCalib:
    case commandImuZero:
        MPU6050_Set_Angle0(&imu);
        break;
    default:
        break;
    }
}

/**
 * @brief  判断左右基础目标是否构成直行条件。
 * @param  targetLeft 左轮目标转速，单位 RPM。
 * @param  targetRight 右轮目标转速，单位 RPM。
 * @return 1 表示左右目标近似相等且平均速度达到航向环启用阈值，0 表示不启用。
 */
static uint8_t appControlIsStraightTarget(float targetLeft, float targetRight)
{
    float averageTarget = (targetLeft + targetRight) * 0.5f;

    return (fabsf(targetLeft - targetRight)
            <= YAW_STRAIGHT_EPSILON_RPM)
           && (fabsf(averageTarget) >= YAW_MIN_TARGET_RPM);
}

/**
 * @brief  将左右速度环输出同步为两者平均值。
 * @note   进入直行航向闭环前调用，避免左右速度环历史输出不一致导致航向环
 *         一开始就产生大差速修正。
 */
static void appControlSynchronizeSpeedPidOutputs(void)
{
    float averagePwm = (speedPidLeft.Out + speedPidRight.Out) * 0.5f;

    PID_ClearHistory(&speedPidLeft);
    PID_ClearHistory(&speedPidRight);
    speedPidLeft.Out = averagePwm;
    speedPidRight.Out = averagePwm;
    speedPidLeft.LastOutput = averagePwm;
    speedPidRight.LastOutput = averagePwm;
}

/**
 * @brief  进入直行航向闭环并锁存当前航向目标。
 */
static void appControlEnterStraightYawControl(void)
{
    appControlSynchronizeSpeedPidOutputs();
    appStateResetYawPid();
    yawTarget = imu.yaw;
    PID_SetTarget(&yawPid, yawTarget);
    yawUpdateTick = 0U;
    straightYawControlActive = 1U;
}

/**
 * @brief  退出直行航向闭环并清除航向环状态。
 */
static void appControlExitStraightYawControl(void)
{
    if (straightYawControlActive) {
        appStateResetYawPid();
    }

    yawUpdateTick = 0U;
    straightYawControlActive = 0U;
}

/**
 * @brief  按 100 ms 分频更新一次直行航向 PI/PID 环。
 * @note   该函数只更新航向环输出，最终左右目标仍由速度环闭环跟踪。
 */
static void appControlUpdateStraightYawControl(void)
{
    if (yawUpdateTick == 0U) {
        yawPid.Target = yawTarget;
        yawPid.Actual = imu.yaw;
        PID_Update(&yawPid);
    }

    yawUpdateTick++;
    if (yawUpdateTick >= YAW_PID_PERIOD_DIVIDER) {
        yawUpdateTick = 0U;
    }
}

/**
 * @brief  复位运动状态、速度滤波器和循迹状态机。
 * @note   模式切换和紧急停车后调用，防止旧目标或旧编码器状态进入下一轮控制。
 */
void appControlResetModeState(void)
{
    appControlResetControlState();
}

/**
 * @brief  执行 50 ms 周期控制任务。
 * @note   先更新编码器速度和命令队列；循迹模式跟踪 20 ms 任务生成的目标，
 *         PWM 测试模式绕过速度环。随后运行左右独立增量式速度环、低速补偿、
 *         输出限幅和遥测快照更新。
 */
void appControlTask50ms(void)
{
    /* 先更新编码器测速，再消费命令队列。
     * 实测本车 TIM3 接物理右轮编码器、TIM2 接物理左轮编码器，与早期命名相反；
     * 按实际接线配对，保证 encoderLeft 对应物理左轮、encoderRight 对应物理右轮。 */
    updateEncoderLoopSimpleVersion(&encoderRight, APP_CONTROL_PERIOD_MS, TIM3);
    updateEncoderLoopSimpleVersion(&encoderLeft, APP_CONTROL_PERIOD_MS, TIM2);

    while (appCommandQueueAvailable()) {
        commandMessage_t message;
        if (appCommandQueuePop(&message) < 0) break;
        appControlHandleCommand(message);
    }

    /* 循迹目标由 20 ms 状态机更新，PWM 测试模式直接输出占空比。 */
    float rpmDifference = encoderLeft.velocity.angular
                          - encoderRight.velocity.angular;

    /* 停车状态下刷新遥测数据并强制关闭电机输出。 */
    if (!runFlag) {
        appControlExitStraightYawControl();
        telemetryData.targetLeftRpm = 0.0f;
        telemetryData.targetRightRpm = 0.0f;
        telemetryData.actualLeftRpm = encoderLeft.velocity.angular;
        telemetryData.actualRightRpm = encoderRight.velocity.angular;
        telemetryData.filteredLeftRpm = encoderLeft.velocity.angular;
        telemetryData.filteredRightRpm = encoderRight.velocity.angular;
        telemetryData.steerRpm = 0.0f;
        telemetryData.actualDiffRpm = rpmDifference;
        telemetryData.yawAngleDeg = imu.yaw;
        telemetryData.trackError = 0.0f;
        telemetryData.mode = (int16_t)workMode;
        telemetryData.runFlag = 0;
        telemetryData.gear = (int16_t)appStateGetSpeedLevel();
        telemetryData.turnPhase = (int16_t)TRACK_PHASE_FOLLOW;
        telemetryData.fresh = 1;

        appControlResetSpeedFilter();
        Motor_SetSpeedA(0);
        Motor_SetSpeedB(0);
        return;
    }

    /* 读取统一目标存储，并限制到允许的正反向转速范围。 */
    float targetLeft = targetLeftRpm;
    float targetRight = targetRightRpm;

    targetLeft = appControlClampSpeedTarget(targetLeft);
    targetRight = appControlClampSpeedTarget(targetRight);

    /* PWM 测试模式绕过速度环，按键值就是最终发送给两轮的 PWM 百分比。 */
    if (workMode == modePwmSet) {
        int16_t pwmOpenLoop = pwmDuty;

        appControlResetSpeedFilter();

        pwmOpenLoop = appControlClampMotorOutput(pwmOpenLoop);

        Motor_SetSpeedA(pwmOpenLoop);
        Motor_SetSpeedB(pwmOpenLoop);

        telemetryData.targetLeftRpm = 0.0f;
        telemetryData.targetRightRpm = 0.0f;
        telemetryData.actualLeftRpm = encoderLeft.velocity.angular;
        telemetryData.actualRightRpm = encoderRight.velocity.angular;
        telemetryData.filteredLeftRpm = encoderLeft.velocity.angular;
        telemetryData.filteredRightRpm = encoderRight.velocity.angular;
        telemetryData.steerRpm = 0.0f;
        telemetryData.actualDiffRpm = rpmDifference;
        telemetryData.yawAngleDeg = imu.yaw;
        telemetryData.trackError = 0.0f;
        telemetryData.mode = (int16_t)workMode;
        telemetryData.runFlag = runFlag;
        telemetryData.gear = (int16_t)appStateGetSpeedLevel();
        telemetryData.turnPhase = (int16_t)TRACK_PHASE_FOLLOW;
        telemetryData.fresh = 1;
        return;
    }

    /* 对左右编码器速度先做三点中值滤波，再做一阶低通滤波。 */
    if (!speedFilterInitialized) {
        float rawSpeedLeft = encoderLeft.velocity.angular;
        float rawSpeedRight = encoderRight.velocity.angular;

        speedSampleLeft[0] = rawSpeedLeft;
        speedSampleLeft[1] = rawSpeedLeft;
        speedSampleLeft[2] = rawSpeedLeft;
        speedSampleRight[0] = rawSpeedRight;
        speedSampleRight[1] = rawSpeedRight;
        speedSampleRight[2] = rawSpeedRight;

        filteredSpeedLeft = rawSpeedLeft;
        filteredSpeedRight = rawSpeedRight;
        speedFilterInitialized = 1;
    } else {
        float medianSpeedLeft;
        float medianSpeedRight;

        speedSampleLeft[speedSampleIndex] = encoderLeft.velocity.angular;
        speedSampleRight[speedSampleIndex] = encoderRight.velocity.angular;

        medianSpeedLeft = appControlMedian3(speedSampleLeft[0],
                                            speedSampleLeft[1],
                                            speedSampleLeft[2]);
        medianSpeedRight = appControlMedian3(speedSampleRight[0],
                                             speedSampleRight[1],
                                             speedSampleRight[2]);

        filteredSpeedLeft = filteredSpeedLeft * ENCODER_FILTER_PREVIOUS
                            + medianSpeedLeft * ENCODER_FILTER_CURRENT;
        filteredSpeedRight = filteredSpeedRight * ENCODER_FILTER_PREVIOUS
                             + medianSpeedRight * ENCODER_FILTER_CURRENT;

        speedSampleIndex++;
        if (speedSampleIndex >= 3U) {
            speedSampleIndex = 0U;
        }
    }

    /* 直行时锁存航向，由 MPU6050 航向角 PID 决定左右目标差。 */
    if ((workMode != modeTrack)
        && appControlIsStraightTarget(targetLeft, targetRight)) {
        if (!straightYawControlActive) {
            appControlEnterStraightYawControl();
        }

        appControlUpdateStraightYawControl();

        float halfYawCorrection = yawPid.Out * 0.5f;
        targetLeft -= halfYawCorrection;
        targetRight += halfYawCorrection;

        targetLeft = appControlClampSpeedTarget(targetLeft);
        targetRight = appControlClampSpeedTarget(targetRight);
    } else {
        appControlExitStraightYawControl();
    }

    /* 左右轮分别执行独立增量 PID，各自跟踪自己的目标转速。 */
    int16_t leftPwm;
    int16_t rightPwm;
    float telemetryTargetLeft = targetLeft;
    float telemetryTargetRight = targetRight;

    if (workMode == modeTrack) {
        float averageTarget = appTrackingGetAverageTargetRpm();
        float averageActual = (filteredSpeedLeft + filteredSpeedRight) * 0.5f;
        float steerRpm = appTrackingGetSteer();
        float steerPwm = steerRpm * appTrackingGetSteerRpmToPwmGain();
        float reverseLimitPwm = appTrackingGetTurnReverseLimitPwm();
        float basePwm;
        TrackTurnPhase_t turnPhase = appTrackingGetTurnPhase();

        telemetryTargetLeft = averageTarget + steerRpm;
        telemetryTargetRight = averageTarget - steerRpm;

        speedPidAverage.Actual = averageActual;
        appControlSetSpeedPidTarget(&speedPidAverage, averageTarget);
        PID_IncrementalUpdate(&speedPidAverage);
        appControlConstrainSpeedPidOutput(&speedPidAverage, averageTarget);
        basePwm = speedPidAverage.Out;

        basePwm = appControlApplyLowSpeedCompensation(averageTarget, basePwm);

        leftPwm = (int16_t)(basePwm + steerPwm);
        rightPwm = (int16_t)(basePwm - steerPwm);

        if (turnPhase == TRACK_PHASE_TURN) {
            if (leftPwm < -(int16_t)reverseLimitPwm) {
                leftPwm = -(int16_t)reverseLimitPwm;
            }
            if (rightPwm < -(int16_t)reverseLimitPwm) {
                rightPwm = -(int16_t)reverseLimitPwm;
            }
        } else {
            if (leftPwm < 0) leftPwm = 0;
            if (rightPwm < 0) rightPwm = 0;
        }

        leftPwm = appControlClampMotorOutput(leftPwm);
        rightPwm = appControlClampMotorOutput(rightPwm);
    } else {
        speedPidLeft.Actual = filteredSpeedLeft;
        appControlSetSpeedPidTarget(&speedPidLeft, targetLeft);
        PID_IncrementalUpdate(&speedPidLeft);
        appControlConstrainSpeedPidOutput(&speedPidLeft, targetLeft);

        speedPidRight.Actual = filteredSpeedRight;
        appControlSetSpeedPidTarget(&speedPidRight, targetRight);
        PID_IncrementalUpdate(&speedPidRight);
        appControlConstrainSpeedPidOutput(&speedPidRight, targetRight);

        leftPwm = (int16_t)speedPidLeft.Out;
        rightPwm = (int16_t)speedPidRight.Out;
    }

    /* 将最终 PWM 限制到允许范围，并按左右目标分别补偿低速死区。 */

    if (workMode != modeTrack) {
        leftPwm = (int16_t)appControlApplyLowSpeedCompensation(
            targetLeft, (float)leftPwm);
        rightPwm = (int16_t)appControlApplyLowSpeedCompensation(
            targetRight, (float)rightPwm);

        leftPwm = appControlClampMotorOutput(leftPwm);
        rightPwm = appControlClampMotorOutput(rightPwm);
    }

    /*
     * 堵转保护：
     * 编码器短时间没有计数时，增量 PID 会继续增大 PWM。若此时轮子被卡住，
     * 一旦突然恢复转动就会以接近满占空比冲出去。这里连续检测约 1.25 秒，
     * 确认一侧目标较高、实际速度接近零且 PWM 已明显增大后立即停车。
     */
    if (appControlMotorStalled(telemetryTargetLeft,
                               encoderLeft.velocity.angular,
                               (float)leftPwm)
        || appControlMotorStalled(telemetryTargetRight,
                                  encoderRight.velocity.angular,
                                  (float)rightPwm)) {
        if (motorStallTicks < 255U) {
            motorStallTicks++;
        }

        if (motorStallTicks >= MOTOR_STALL_CONFIRM_TICKS) {
            appStateReportStop(stopReasonMotorStall);
            appControlExitStraightYawControl();
            appControlResetControlState();
            return;
        }
    } else {
        motorStallTicks = 0U;
    }

    /* 更新电机输出并准备本周期遥测快照。
     * 实测 Motor A 通道接的是物理右轮，这里按物理左右下发。 */
    Motor_SetSpeedA(rightPwm);  /* 物理右轮。 */
    Motor_SetSpeedB(leftPwm);   /* 物理左轮。 */

    telemetryData.targetLeftRpm = telemetryTargetLeft;
    telemetryData.targetRightRpm = telemetryTargetRight;
    telemetryData.actualLeftRpm = encoderLeft.velocity.angular;
    telemetryData.actualRightRpm = encoderRight.velocity.angular;
    telemetryData.filteredLeftRpm = filteredSpeedLeft;
    telemetryData.filteredRightRpm = filteredSpeedRight;
    telemetryData.steerRpm = (workMode == modeTrack)
                             ? appTrackingGetSteer()
                             : (straightYawControlActive ? yawPid.Out : 0.0f);
    telemetryData.actualDiffRpm = rpmDifference;
    telemetryData.yawAngleDeg = imu.yaw;
    telemetryData.trackError = (workMode == modeTrack)
                               ? appTrackingGetWeightedError()
                               : 0.0f;
    telemetryData.mode = (int16_t)workMode;
    telemetryData.runFlag = runFlag;
    telemetryData.gear = (int16_t)appStateGetSpeedLevel();
    telemetryData.turnPhase = (workMode == modeTrack)
                              ? (int16_t)appTrackingGetTurnPhase()
                              : (int16_t)TRACK_PHASE_FOLLOW;
    telemetryData.fresh = 1;
}

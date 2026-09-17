#include "yingYongKongZhi.h"
#include "yingYongMingLing.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "yingYongYaoCe.h"
#include "yingYongXunJi.h"
#include "motor.h"
#include "mpu6050.h"

#include <math.h>

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
 * @brief  清除循迹转向和丢线计数状态。
 * @note   停车、切换模式或直角转弯确认完成后调用，避免下一次循迹沿用旧状态。
 */


/**
 * @brief  设置一正一反原地转目标。
 * @param  pivotSpeed 带符号转向速度，正值表示右转，负值表示左转。
 * @note   正值时左轮前进、右轮反转；负值时左轮反转、右轮前进。
 */
void appControlResetModeState(void)
{
    appStateResetMotion();
    appControlResetSpeedFilter();
    appTrackingResetState();
}

static void appControlSetPivotTarget(float pivotSpeed)
{
    targetLeftRpm = pivotSpeed;
    targetRightRpm = -pivotSpeed;
    turnTarget = pivotSpeed;
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
 * @brief  设置左右轮相同的直行目标速度。
 * @param  targetRpm 左右轮共同目标转速，单位 RPM。
 * @note   直行命令会清除上一次手动转向量，避免旧转向量在后续命令中残留。
 */
static void appControlSetStraightTarget(float targetRpm)
{
    targetLeftRpm = targetRpm;
    targetRightRpm = targetRpm;
    turnTarget = 0.0f;
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
 * @brief  执行一条应用层控制命令。
 * @param  message 待执行的命令消息。
 * @note   所有按键、蓝牙和串口控制命令都集中在此处修改状态，
 *         避免多个输入源直接操作 PID 或电机。
 */
static void appControlApplyGearStep(int8_t direction)
{
    uint8_t level = appStateGetSpeedLevel();

    if (workMode == modeTrack) {
        if (level > TRACK_MAX_SPEED_LEVEL) level = TRACK_MAX_SPEED_LEVEL;
        if (direction > 0 && level < TRACK_MAX_SPEED_LEVEL) level++;
        if (direction < 0 && level > 1U) level--;
        appStateSetSpeedLevel(level);
        return;
    }

    int16_t next = (int16_t)level + direction;
    if (next > (int16_t)SPEED_LEVEL_COUNT) next = 1;
    if (next < 1) next = (int16_t)SPEED_LEVEL_COUNT;
    appStateSetSpeedLevel((uint8_t)next);
}

static void appControlHandleCommand(commandMessage_t message)
{
    /* 所有控制命令在这里集中执行，避免按键和蓝牙分别修改电机状态。 */
    switch (message.type) {
    case commandSpeed:
        if (workMode == modeRun) {
            appControlSetStraightTarget(message.value);
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
            appControlSetPivotTarget(message.value);
        }
        break;
    case commandStart:
        if ((workMode == modeTrack) && !runFlag) {
            appTrackingStart();
        }
        runFlag = 1;
        break;
    case commandStop:
        pwmDuty = 0;
        appStateResetMotion();
        appControlResetSpeedFilter();
        appTrackingResetState();
        break;
    case commandLapCycle:
        if ((workMode == modeTrack) && !runFlag) {
            appStateCycleLapTarget();
        }
        break;
    case commandGearUp:
        {
            appControlApplyGearStep(1);
            if (workMode == modeRun && runFlag) {
                if (targetLeftRpm * targetRightRpm < 0.0f) {
                    float sign = (targetLeftRpm < 0.0f) ? -1.0f : 1.0f;
                    appControlSetPivotTarget(appStateGetSpeedRpm() * sign);
                } else {
                    float sign = (appControlGetAverageTarget() < 0.0f) ? -1.0f : 1.0f;
                    float target = appStateGetSpeedRpm() * sign;
                    appControlSetTurnTarget(target, turnTarget);
                }
            }
        }
        break;
    case commandGearDown:
        {
            appControlApplyGearStep(-1);
            if (workMode == modeRun && runFlag) {
                if (targetLeftRpm * targetRightRpm < 0.0f) {
                    float sign = (targetLeftRpm < 0.0f) ? -1.0f : 1.0f;
                    appControlSetPivotTarget(appStateGetSpeedRpm() * sign);
                } else {
                    float sign = (appControlGetAverageTarget() < 0.0f) ? -1.0f : 1.0f;
                    float target = appStateGetSpeedRpm() * sign;
                    appControlSetTurnTarget(target, turnTarget);
                }
            }
        }
        break;
    case commandModeTrack:
        /* 切换循迹前清除运动状态、红外图案和丢线计数。 */
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
        appStateResetMotion();
        appControlResetSpeedFilter();
        appTrackingResetState();
        pwmDuty = 0;
        workMode = modeRun;
        break;
    case commandModePwm:
        appStateResetMotion();
        appControlResetSpeedFilter();
        appTrackingResetState();
        workMode = modePwmSet;
        break;
    case commandSetSpeedKp:
        speedPidLeft.Kp = message.value;
        speedPidRight.Kp = message.value;
        break;
    case commandSetSpeedKi:
        speedPidLeft.Ki = message.value;
        speedPidRight.Ki = message.value;
        break;
    case commandSetSpeedKd:
        speedPidLeft.Kd = message.value;
        speedPidRight.Kd = message.value;
        break;
    case commandSetSpeedKf:
        /* 增量式速度环不使用前馈，保留命令类型以兼容旧协议。 */
        break;
    case commandSetYawKp:
        yawPid.Kp = message.value;
        break;
    case commandSetYawKi:
        yawPid.Ki = message.value;
        break;
    case commandSetYawKd:
        yawPid.Kd = message.value;
        break;
    case commandSetYawKf:
        yawPid.Kf = message.value;
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

static uint8_t appControlIsStraightTarget(float targetLeft, float targetRight)
{
    float averageTarget = (targetLeft + targetRight) * 0.5f;

    return (fabsf(targetLeft - targetRight)
            <= YAW_STRAIGHT_EPSILON_RPM)
           && (fabsf(averageTarget) >= YAW_MIN_TARGET_RPM);
}

static void appControlSynchronizeSpeedPidOutputs(void)
{
    float averagePwm = (speedPidLeft.Out + speedPidRight.Out) * 0.5f;

    speedPidLeft.Out = averagePwm;
    speedPidRight.Out = averagePwm;
    speedPidLeft.LastOutput = averagePwm;
    speedPidRight.LastOutput = averagePwm;

    speedPidLeft.Error0 = 0.0f;
    speedPidLeft.Error1 = 0.0f;
    speedPidLeft.Error2 = 0.0f;
    speedPidRight.Error0 = 0.0f;
    speedPidRight.Error1 = 0.0f;
    speedPidRight.Error2 = 0.0f;
    speedPidLeft.LastIncrement = 0.0f;
    speedPidRight.LastIncrement = 0.0f;
}

static void appControlEnterStraightYawControl(void)
{
    appControlSynchronizeSpeedPidOutputs();
    appStateResetYawPid();
    yawTarget = imu.yaw;
    PID_SetTarget(&yawPid, yawTarget);
    yawUpdateTick = 0U;
    straightYawControlActive = 1U;
}

static void appControlExitStraightYawControl(void)
{
    if (straightYawControlActive) {
        appStateResetYawPid();
    }

    yawUpdateTick = 0U;
    straightYawControlActive = 0U;
}

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
 * @brief  执行 50 ms 周期控制任务。
 * @note   先更新编码器速度和命令队列，再根据模式执行循迹或 PWM 测试，
 *         随后运行左右独立增量式速度环、低速补偿、输出限幅和遥测快照更新。
 */
void appControlTask50ms(void)
{
    /* 先更新编码器测速，再消费命令队列。 */
    updateEncoderLoopSimpleVersion(&encoderLeft, APP_CONTROL_PERIOD_MS, TIM3);
    updateEncoderLoopSimpleVersion(&encoderRight, APP_CONTROL_PERIOD_MS, TIM2);

    while (appCommandQueueAvailable()) {
        commandMessage_t message;
        if (appCommandQueuePop(&message) < 0) break;
        appControlHandleCommand(message);
    }

    /* 循迹模式使用位置环生成目标速度，PWM 测试模式直接输出占空比。 */
    float rpmDifference = encoderLeft.velocity.angular
                          - encoderRight.velocity.angular;

    /* 停车状态下刷新遥测数据并强制关闭电机输出。 */
    if (!runFlag) {
        appControlExitStraightYawControl();
        telemetryData.targetLeft = 0.0f;
        telemetryData.targetRight = 0.0f;
        telemetryData.rpmRight = encoderRight.velocity.angular;
        telemetryData.rpmLeft = encoderLeft.velocity.angular;
        telemetryData.pwmLeft = 0.0f;
        telemetryData.pwmRight = 0.0f;
        telemetryData.yawOutput = 0.0f;
        telemetryData.rpmDiff = 0.0f;
        telemetryData.yawAngle = imu.yaw;
        telemetryData.bluetoothLv = bluetoothLv;
        telemetryData.bluetoothRh = bluetoothRh;
        telemetryData.runFlag = 0;
        telemetryData.gear = (int16_t)appStateGetSpeedLevel();
        telemetryData.rpmFilteredRight = 0.0f;
        telemetryData.rpmFilteredLeft = 0.0f;
        telemetryData.fresh = 1;

        appControlResetSpeedFilter();
        appTrackingResetState();
        Motor_SetSpeedA(0);
        Motor_SetSpeedB(0);
        return;
    }

    /* 读取统一目标存储，并限制到允许的正反向转速范围。 */
    float targetLeft = targetLeftRpm;
    float targetRight = targetRightRpm;

    if (targetLeft > SPEED_TARGET_LIMIT_RPM) targetLeft = SPEED_TARGET_LIMIT_RPM;
    if (targetLeft < -SPEED_TARGET_LIMIT_RPM) targetLeft = -SPEED_TARGET_LIMIT_RPM;
    if (targetRight > SPEED_TARGET_LIMIT_RPM) targetRight = SPEED_TARGET_LIMIT_RPM;
    if (targetRight < -SPEED_TARGET_LIMIT_RPM) targetRight = -SPEED_TARGET_LIMIT_RPM;

    /* PWM 测试模式绕过速度环，按键值就是最终发送给两轮的 PWM 百分比。 */
    if (workMode == modePwmSet) {
        int16_t pwmOpenLoop = pwmDuty;

        appControlResetSpeedFilter();

        if (pwmOpenLoop > MOTOR_OUTPUT_MAX_PERCENT) {
            pwmOpenLoop = MOTOR_OUTPUT_MAX_PERCENT;
        }
        if (pwmOpenLoop < MOTOR_OUTPUT_MIN_PERCENT) {
            pwmOpenLoop = MOTOR_OUTPUT_MIN_PERCENT;
        }

        Motor_SetSpeedA(pwmOpenLoop);
        Motor_SetSpeedB(pwmOpenLoop);

        telemetryData.targetLeft = (float)pwmOpenLoop;
        telemetryData.targetRight = (float)pwmOpenLoop;
        telemetryData.rpmRight = encoderRight.velocity.angular;
        telemetryData.rpmLeft = encoderLeft.velocity.angular;
        telemetryData.pwmLeft = (float)pwmOpenLoop;
        telemetryData.pwmRight = (float)pwmOpenLoop;
        telemetryData.yawOutput = 0.0f;
        telemetryData.rpmDiff = rpmDifference;
        telemetryData.yawAngle = imu.yaw;
        telemetryData.bluetoothLv = bluetoothLv;
        telemetryData.bluetoothRh = bluetoothRh;
        telemetryData.runFlag = runFlag;
        telemetryData.gear = (int16_t)appStateGetSpeedLevel();
        telemetryData.rpmFilteredRight = 0.0f;
        telemetryData.rpmFilteredLeft = 0.0f;
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

        if (targetLeft > SPEED_TARGET_LIMIT_RPM) {
            targetLeft = SPEED_TARGET_LIMIT_RPM;
        }
        if (targetLeft < -SPEED_TARGET_LIMIT_RPM) {
            targetLeft = -SPEED_TARGET_LIMIT_RPM;
        }
        if (targetRight > SPEED_TARGET_LIMIT_RPM) {
            targetRight = SPEED_TARGET_LIMIT_RPM;
        }
        if (targetRight < -SPEED_TARGET_LIMIT_RPM) {
            targetRight = -SPEED_TARGET_LIMIT_RPM;
        }
    } else {
        appControlExitStraightYawControl();
    }

    /* 左右轮分别执行独立增量 PID，各自跟踪自己的目标转速。 */
    speedPidLeft.Actual = filteredSpeedLeft;
    PID_SetTarget(&speedPidLeft, targetLeft);
    PID_IncrementalUpdate(&speedPidLeft);

    speedPidRight.Actual = filteredSpeedRight;
    PID_SetTarget(&speedPidRight, targetRight);
    PID_IncrementalUpdate(&speedPidRight);

    int16_t leftPwm = (int16_t)speedPidLeft.Out;
    int16_t rightPwm = (int16_t)speedPidRight.Out;

    /* 将最终 PWM 限制到允许范围，并按左右目标分别补偿低速死区。 */

    if (targetLeft > 0.0f && targetLeft < MOTOR_LOW_SPEED_THRESHOLD_RPM) {
        if (leftPwm > 0 && leftPwm < MOTOR_LOW_SPEED_MIN_PWM) {
            leftPwm = MOTOR_LOW_SPEED_MIN_PWM;
        }
    } else if (targetLeft < 0.0f
               && targetLeft > -MOTOR_LOW_SPEED_THRESHOLD_RPM) {
        if (leftPwm < 0 && leftPwm > -MOTOR_LOW_SPEED_MIN_PWM) {
            leftPwm = -MOTOR_LOW_SPEED_MIN_PWM;
        }
    }

    if (targetRight > 0.0f && targetRight < MOTOR_LOW_SPEED_THRESHOLD_RPM) {
        if (rightPwm > 0 && rightPwm < MOTOR_LOW_SPEED_MIN_PWM) {
            rightPwm = MOTOR_LOW_SPEED_MIN_PWM;
        }
    } else if (targetRight < 0.0f
               && targetRight > -MOTOR_LOW_SPEED_THRESHOLD_RPM) {
        if (rightPwm < 0 && rightPwm > -MOTOR_LOW_SPEED_MIN_PWM) {
            rightPwm = -MOTOR_LOW_SPEED_MIN_PWM;
        }
    }

    if (leftPwm > MOTOR_OUTPUT_MAX_PERCENT) leftPwm = MOTOR_OUTPUT_MAX_PERCENT;
    if (leftPwm < MOTOR_OUTPUT_MIN_PERCENT) leftPwm = MOTOR_OUTPUT_MIN_PERCENT;
    if (rightPwm > MOTOR_OUTPUT_MAX_PERCENT) rightPwm = MOTOR_OUTPUT_MAX_PERCENT;
    if (rightPwm < MOTOR_OUTPUT_MIN_PERCENT) rightPwm = MOTOR_OUTPUT_MIN_PERCENT;

    /*
     * 堵转保护：
     * 编码器短时间没有计数时，增量 PID 会继续增大 PWM。若此时轮子被卡住，
     * 一旦突然恢复转动就会以接近满占空比冲出去。这里连续检测约 1.25 秒，
     * 确认一侧目标较高、实际速度接近零且 PWM 已明显增大后立即停车。
     */
    if (appControlMotorStalled(targetLeft,
                               encoderLeft.velocity.angular,
                               (float)leftPwm)
        || appControlMotorStalled(targetRight,
                                  encoderRight.velocity.angular,
                                  (float)rightPwm)) {
        if (motorStallTicks < 255U) {
            motorStallTicks++;
        }

        if (motorStallTicks >= MOTOR_STALL_CONFIRM_TICKS) {
            appControlExitStraightYawControl();
            appStateResetMotion();
            appControlResetSpeedFilter();
            appTrackingResetState();
            return;
        }
    } else {
        motorStallTicks = 0U;
    }

    /* 更新电机输出并准备本周期遥测快照。 */
    Motor_SetSpeedA(leftPwm);
    Motor_SetSpeedB(rightPwm);

    telemetryData.targetLeft = targetLeft;
    telemetryData.targetRight = targetRight;
    telemetryData.rpmRight = encoderRight.velocity.angular;
    telemetryData.rpmLeft = encoderLeft.velocity.angular;
    telemetryData.pwmLeft = (float)leftPwm;
    telemetryData.pwmRight = (float)rightPwm;
    telemetryData.yawOutput = (workMode == modeTrack)
                              ? appTrackingGetSteer()
                              : (straightYawControlActive ? yawPid.Out : 0.0f);
    telemetryData.rpmDiff = rpmDifference;
    telemetryData.yawAngle = imu.yaw;
    telemetryData.bluetoothLv = bluetoothLv;
    telemetryData.bluetoothRh = bluetoothRh;
    telemetryData.runFlag = runFlag;
    telemetryData.gear = (int16_t)appStateGetSpeedLevel();
    telemetryData.rpmFilteredRight = filteredSpeedRight;
    telemetryData.rpmFilteredLeft = filteredSpeedLeft;
    telemetryData.fresh = 1;
}

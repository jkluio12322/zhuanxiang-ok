/**
 * @file    yingYongXunJi.c
 * @brief   八路数字循迹、急弯确认状态机。
 */

#include "yingYongXunJi.h"
#include "app_config.h"
#include "yingYongZhuangTai.h"
#include "pid.h"
#include "irtrack.h"

#include <math.h>

/* 八路红外 raw 的物理语义：0 表示压到黑线，1 表示丢失黑线。
   这是本文件内部的判定约定，不放到 app_config.h。 */
#define ZHAN_SHU_BAO_HEI_XIAN_RAW_MIE 0U
#define ZHAN_SHU_BAO_DIU_XIAN_RAW_BAI 1U

static TrackControlState_t trackControlState = TRACK_CONTROL_FOLLOW;
static uint8_t trackFinishConfirmTicks = 0U;
static uint8_t trackFinishArmed = 0U;
static uint8_t trackPostLapLineGraceTicks = 0U;
static uint8_t trackStartupTicks = 0U;
static uint8_t trackLineAcquired = 0U;
static uint8_t trackLapCornerCount = 0U;
static int8_t trackLapTurnDirection = 0;
static int8_t trackCornerCandidateDirection = 0;
static uint8_t trackCornerCandidateTicks = 0U;
static int8_t trackPendingCornerDirection = 0;
static uint8_t trackPendingCornerTicks = 0U;
static uint8_t trackInvalidLineTicks = 0U;
static uint8_t trackAllWhiteTicks = 0U;
static uint8_t trackCornerLostLineTicks = 0U;
static uint8_t trackCornerArmed = 0U;
static uint8_t trackCornerCompleteTicks = 0U;
static uint8_t trackCornerCenterSinceTurn = 0U;
static uint8_t trackCornerCenterCleared = 0U;
static uint8_t trackCornerCenterClearTicks = 0U;
static int8_t trackTurnDirection = 0;
static uint16_t trackTurnTicks = 0U;
static float trackSteerOutput = 0.0f;
static float trackAverageTargetRpm = 0.0f;
static float trackScheduledBaseRpm = 0.0f;
static float trackSteerRpmToPwmGain = TRACK_STEER_RPM_TO_PWM_DEFAULT;
static float trackWeightedError = 0.0f;
static TrackTurnPhase_t trackTurnPhase = TRACK_PHASE_FOLLOW;
static TrackPid_t trackPid = {TRACK_PID_KP, TRACK_PID_KI, TRACK_PID_KD, 0.0f, 0.0f};
/**
 * @brief  将浮点数限制在指定范围内。
 * @param  value 待限制的值。
 * @param  minValue 允许的最小值。
 * @param  maxValue 允许的最大值。
 * @return 限制后的值。
 */
static float appTrackingClampValue(float value, float minValue, float maxValue)
{
    if (value > maxValue) return maxValue;
    if (value < minValue) return minValue;
    return value;
}

/**
 * @brief  判断八路红外 raw 是否全部等于指定状态。
 * @param  rawState 待比较的 raw 状态。
 * @return 1 表示八路全部匹配，0 表示至少一路不同。
 * @note   调用前需要先执行 IRTrack_Read() 更新 raw 数组。
 */
static uint8_t appTrackingAllChannelsEqual(uint8_t rawState)
{
    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        if (ir_data.raw[i] != rawState) {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief  判断八路红外是否全部压到黑线。
 * @return 1 表示八路 raw 全部为 0；0 表示尚未全部压线。
 */
static uint8_t zhanShuBaoYuDaoHeiXian(void)
{
    return appTrackingAllChannelsEqual(ZHAN_SHU_BAO_HEI_XIAN_RAW_MIE);
}

/**
 * @brief  判断八路红外是否全部丢失黑线。
 * @return 1 表示八路 raw 全部为 1；0 表示仍有通道检测到黑线。
 */
static uint8_t zhanShuBaoDiuXian(void)
{
    return appTrackingAllChannelsEqual(ZHAN_SHU_BAO_DIU_XIAN_RAW_BAI);
}

/**
 * @brief  判断当前压黑图案是否只有一个连续黑线段。
 * @return 1 表示最低压黑位到最高压黑位之间全部为黑，0 表示没有压线或存在白洞。
 * @note   终点横线可包含多个通道，但仍由终点分支优先处理。
 */
static uint8_t appTrackingPatternIsContiguous(void)
{
    uint8_t firstBlack = 0U;
    uint8_t lastBlack = 0U;
    uint8_t foundBlack = 0U;

    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        if ((ir_data.pattern & (uint8_t)(1U << i)) != 0U) {
            if (foundBlack == 0U) {
                firstBlack = i;
                foundBlack = 1U;
            }
            lastBlack = i;
        }
    }

    if (foundBlack == 0U) {
        return 0U;
    }

    for (uint8_t i = firstBlack; i <= lastBlack; i++) {
        if ((ir_data.pattern & (uint8_t)(1U << i)) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief  应用已经完成方向转换的左右轮目标。
 * @param  averageTargetRpm 左右轮平均目标转速，单位 RPM。
 * @param  signedSteerRpm 已应用 TRACK_STEERING_SIGN 的差速量，单位 RPM。
 * @note   丢线保持方向时直接复用该入口，避免再次乘方向符号。
 */
static void appTrackingApplySignedTargets(float averageTargetRpm,
                                          float signedSteerRpm)
{
    trackAverageTargetRpm = appTrackingClampValue(averageTargetRpm,
                                                  0.0f,
                                                  SPEED_TARGET_LIMIT_RPM);
    trackSteerOutput = appTrackingClampValue(signedSteerRpm,
                                             -SPEED_TARGET_LIMIT_RPM,
                                             SPEED_TARGET_LIMIT_RPM);
}

/**
 * @brief  应用左右轮目标并限制到指定范围内。
 * @param  averageTargetRpm 左右轮平均目标转速，单位 RPM。
 * @param  steerRpm 循迹 PID 生成的差速量，单位 RPM。
 * @note   仅保留最终目标和差速限幅，PID 输出不再经过独立速率限制。
 */
static void appTrackingApplyTargets(float averageTargetRpm,
                                    float steerRpm)
{
    appTrackingApplySignedTargets(averageTargetRpm,
                                  steerRpm * TRACK_STEERING_SIGN);
}

/**
 * @brief  判断当前是否处于已经锁存方向的急弯标记阶段。
 * @return 1 表示允许急弯丢线方向兜底，0 表示不处于有效急弯标记。
 */
static uint8_t appTrackingIsCornerTurnActive(void)
{
    return (uint8_t)(
        ((trackControlState == TRACK_CONTROL_TURN_LEFT)
         || (trackControlState == TRACK_CONTROL_TURN_RIGHT))
        && (trackTurnDirection != 0));
}

/**
 * @brief  生成急弯丢线期间使用的最低方向差速。
 * @return 已应用 TRACK_STEERING_SIGN 的差速量，单位 RPM。
 * @note   历史同向差速足够大时继续使用，否则回落到锁存方向的最低差速。
 */
static float appTrackingGetCornerLostLineSteer(void)
{
    float lockedDirection =
        (float)trackTurnDirection * TRACK_STEERING_SIGN;
    float minimumSteer =
        lockedDirection * TRACK_CORNER_LOST_LINE_MIN_DIFF_RPM;

    if ((trackTurnDirection != 0)
        && ((trackSteerOutput * lockedDirection) > 0.0f)
        && (fabsf(trackSteerOutput) > fabsf(minimumSteer))) {
        return trackSteerOutput;
    }

    return minimumSteer;
}

/**
 * @brief  根据完整八路权重误差调度弯道基础速度。
 * @param  selectedBaseRpm 当前档位选择的基础速度，单位 RPM。
 * @return 本周期允许使用的基础速度，单位 RPM。
 * @note   误差增大时立即降速；线路居中后的回升量受每周期步长限制。
 */
static float appTrackingScheduleCurveBaseSpeed(float selectedBaseRpm)
{
    float absoluteError = fabsf((float)ir_data.weight_sum);
    float speedScale;
    float minimumBaseRpm;
    float curveBaseRpm;
    float nextBaseRpm;

    if (absoluteError <= TRACK_CURVE_SPEED_ERROR_START) {
        speedScale = 1.0f;
    } else if (absoluteError >= TRACK_CURVE_SPEED_ERROR_FULL) {
        speedScale = TRACK_CURVE_SPEED_MIN_SCALE;
    } else {
        speedScale = 1.0f
                     - ((1.0f - TRACK_CURVE_SPEED_MIN_SCALE)
                        * ((absoluteError - TRACK_CURVE_SPEED_ERROR_START)
                           / (TRACK_CURVE_SPEED_ERROR_FULL
                              - TRACK_CURVE_SPEED_ERROR_START)));
    }

    minimumBaseRpm = selectedBaseRpm;
    if (minimumBaseRpm > TRACK_CURVE_SPEED_MIN_RPM) {
        minimumBaseRpm = TRACK_CURVE_SPEED_MIN_RPM;
    }

    curveBaseRpm = selectedBaseRpm * speedScale;
    if (curveBaseRpm < minimumBaseRpm) {
        curveBaseRpm = minimumBaseRpm;
    }

    if (trackScheduledBaseRpm <= 0.0f) {
        nextBaseRpm = curveBaseRpm;
    } else if (curveBaseRpm < trackScheduledBaseRpm) {
        nextBaseRpm = curveBaseRpm;
    } else {
        nextBaseRpm = trackScheduledBaseRpm + TRACK_SPEED_RECOVERY_STEP_RPM;
        if (nextBaseRpm > curveBaseRpm) {
            nextBaseRpm = curveBaseRpm;
        }
    }

    trackScheduledBaseRpm = nextBaseRpm;
    return nextBaseRpm;
}

/**
 * @brief  在确认线路丢失后按指定上限限制基础速度。
 * @param  selectedBaseRpm 当前档位选择的基础速度，单位 RPM。
 * @param  maxBaseRpm 丢线阶段允许的最高基础速度，单位 RPM。
 * @return 丢线保持阶段允许使用的基础速度，单位 RPM。
 * @note   只允许立即降速，不允许在丢线期间反向提速。
 */
static float appTrackingScheduleLostLineBaseSpeedLimited(float selectedBaseRpm,
                                                         float maxBaseRpm)
{
    float limitedBaseRpm = selectedBaseRpm;
    float nextBaseRpm;

    if (limitedBaseRpm > maxBaseRpm) {
        limitedBaseRpm = maxBaseRpm;
    }

    if (trackScheduledBaseRpm <= 0.0f) {
        nextBaseRpm = limitedBaseRpm;
    } else if (limitedBaseRpm < trackScheduledBaseRpm) {
        nextBaseRpm = limitedBaseRpm;
    } else {
        nextBaseRpm = trackScheduledBaseRpm;
    }

    trackScheduledBaseRpm = nextBaseRpm;
    return nextBaseRpm;
}

/**
 * @brief  限制普通丢线保持阶段的基础速度。
 * @param  selectedBaseRpm 当前档位选择的基础速度，单位 RPM。
 * @return 普通丢线保持阶段允许的基础速度，单位 RPM。
 */
static float appTrackingScheduleLostLineBaseSpeed(float selectedBaseRpm)
{
    return appTrackingScheduleLostLineBaseSpeedLimited(
        selectedBaseRpm,
        TRACK_LOST_LINE_MAX_BASE_RPM);
}

/**
 * @brief  限制急弯丢线兜底阶段的基础速度。
 * @param  selectedBaseRpm 当前档位选择的基础速度，单位 RPM。
 * @return 急弯丢线兜底阶段允许的基础速度，单位 RPM。
 */
static float appTrackingScheduleCornerLostLineBaseSpeed(float selectedBaseRpm)
{
    return appTrackingScheduleLostLineBaseSpeedLimited(
        selectedBaseRpm,
        TRACK_CORNER_LOST_LINE_MAX_BASE_RPM);
}

/**
 * @brief  停止循迹运动并清除状态机状态。
 * @param  reason 本次停车原因。
 */
static void appTrackingStop(appStopReason_t reason)
{
    appStateReportStop(reason);
    appStateResetMotion();
    appTrackingResetState();
}

/**
 * @brief  判断当前红外图案是否包含有效黑线。
 * @return 1 表示存在 1~7 个连续黑线通道且不是全黑或全白，0 表示无效。
 */
static uint8_t appTrackingHasValidLine(void)
{
    return (uint8_t)((zhanShuBaoDiuXian() == 0U)
                     && (zhanShuBaoYuDaoHeiXian() == 0U)
                     && (ir_data.black_count > 0U)
                     && (ir_data.black_count <= TRACK_LINE_VALID_MAX_BLACK)
                     && (appTrackingPatternIsContiguous() != 0U));
}

/**
 * @brief  清除循迹 PID 的误差和积分历史。
 */
static void appTrackingResetLinePid(void)
{
    trackWeightedError = 0.0f;
    trackSteerOutput = 0.0f;
    TrackPid_Reset(&trackPid);
}

/**
 * @brief  根据八路权重计算用于 PID 的连续误差。
 * @return 负值偏左、正值偏右，范围限制在正负误差上限内。
 */
static float appTrackingComputeWeightedErrorInternal(void)
{
    float weightSum = (float)ir_data.weight_sum;

    if (!appTrackingHasValidLine()) {
        return trackWeightedError;
    }

    return appTrackingClampValue(weightSum,
                                 -TRACK_WEIGHTED_ERROR_LIMIT,
                                 TRACK_WEIGHTED_ERROR_LIMIT);
}

/**
 * @brief  使用当前八路加权误差更新循迹 PID 输出。
 * @param  error 当前加权误差。
 * @param  diffLimit 本次允许的最大差速，单位 RPM。
 * @return 限幅后的 PID 差速输出，单位 RPM。
 */
static float appTrackingUpdateLinePidInternal(float error, float diffLimit)
{
    trackWeightedError = error;
    return TrackPid_Update(&trackPid, error, TRACK_PID_PERIOD_SECONDS, diffLimit);
}

/**
 * @brief  使用统一八路加权 PID 应用普通循迹目标。
 * @param  baseSpeed 当前档位对应的循迹基准速度，单位 RPM。
 * @param  immediate 非零时用当前误差初始化微分历史。
 */
static void appTrackingApplyFollowTargetsInternal(float baseSpeed,
                                                  uint8_t immediate)
{
    uint8_t centerBits;
    float error = 0.0f;
    float turn = 0.0f;
    float averageTarget;

    if (!appTrackingHasValidLine()) {
        if (appTrackingIsCornerTurnActive() != 0U) {
            trackTurnPhase = TRACK_PHASE_TURN;
            averageTarget =
                appTrackingScheduleCornerLostLineBaseSpeed(baseSpeed);
            appTrackingApplySignedTargets(
                averageTarget,
                appTrackingGetCornerLostLineSteer());
            return;
        }

        trackTurnPhase = TRACK_PHASE_FOLLOW;
        if (trackInvalidLineTicks < TRACK_LINE_INVALID_CONFIRM_TICKS) {
            trackInvalidLineTicks++;
        }

        if (trackInvalidLineTicks < TRACK_LINE_INVALID_CONFIRM_TICKS) {
            return;
        }

        averageTarget = appTrackingScheduleLostLineBaseSpeed(baseSpeed);
        appTrackingApplySignedTargets(averageTarget, trackSteerOutput);
        return;
    } else {
        trackInvalidLineTicks = 0U;
        averageTarget = appTrackingScheduleCurveBaseSpeed(baseSpeed);
        centerBits = ir_data.pattern & TRACK_CENTER_SENSOR_MASK;
        trackTurnPhase = (centerBits == 0U)
                         ? TRACK_PHASE_RECENTER
                         : TRACK_PHASE_FOLLOW;
        error = appTrackingComputeWeightedErrorInternal();

        if (immediate != 0U) {
            trackPid.ErrorPrevious = error;
        }

        turn = appTrackingUpdateLinePidInternal(error,
                                        TRACK_FOLLOW_DIFF_LIMIT_RPM);
    }

    appTrackingApplyTargets(averageTarget, turn);
}

static void appTrackingApplyFollowTargets(float baseSpeed)
{
    appTrackingApplyFollowTargetsInternal(baseSpeed, 0U);
}

/**
 * @brief  复位 PWM-TRACK 相位计时到左偏阶段。
 */
static void appTrackingClearTurnLatch(void)
{
    trackTurnDirection = 0;
    trackTurnTicks = 0U;
    trackCornerCompleteTicks = 0U;
    trackCornerCenterSinceTurn = 0U;
    trackCornerCenterCleared = 0U;
    trackCornerCenterClearTicks = 0U;
    trackCornerLostLineTicks = 0U;
    trackAllWhiteTicks = 0U;
    trackTurnPhase = TRACK_PHASE_FOLLOW;
}

/**
 * @brief  清除急弯边缘候选状态。
 * @note   正常循迹重新稳定、进入转弯、停车或重新启动时调用。
 */
static void appTrackingClearCornerCandidate(void)
{
    trackCornerCandidateDirection = 0;
    trackCornerCandidateTicks = 0U;
}

/**
 * @brief  清除待执行的下一弯方向。
 */
static void appTrackingClearPendingCorner(void)
{
    trackPendingCornerDirection = 0;
    trackPendingCornerTicks = 0U;
}

/**
 * @brief  更新急弯边缘候选。
 * @note   当线路靠近最外侧时记录方向并保持约 100 ms。若随后全部传感器
 *         离线，状态机仍能按照最后看到的方向开始转弯。
 */
static void appTrackingUpdateCornerCandidate(void)
{
    int8_t direction = 0;
    uint8_t leftCount = 0U;
    uint8_t rightCount = 0U;

    if ((ir_data.black_count > TRACK_LINE_VALID_MAX_BLACK)
        || (appTrackingPatternIsContiguous() == 0U)) {
        if (ir_data.black_count < IRTRACK_CHANNELS) {
            if (trackCornerCandidateTicks > 0U) {
                trackCornerCandidateTicks--;
                if (trackCornerCandidateTicks == 0U) {
                    trackCornerCandidateDirection = 0;
                }
            }
        } else {
            appTrackingClearCornerCandidate();
        }
        return;
    }

    if (ir_data.black_count >= TRACK_CORNER_EDGE_MIN_BLACK) {
        for (uint8_t i = 0U; i < 4U; i++) {
            if ((ir_data.pattern & (uint8_t)(1U << i)) != 0U) {
                leftCount++;
            }
        }

        for (uint8_t i = 4U; i < IRTRACK_CHANNELS; i++) {
            if ((ir_data.pattern & (uint8_t)(1U << i)) != 0U) {
                rightCount++;
            }
        }

        if ((leftCount >= 2U)
            && (leftCount >= (uint8_t)(rightCount + 2U))) {
            direction = -1;
        } else if ((rightCount >= 2U)
                   && (rightCount >= (uint8_t)(leftCount + 2U))) {
            direction = 1;
        } else if (fabsf(ir_data.position)
                   >= TRACK_CORNER_EDGE_POSITION_MIN) {
            direction = (ir_data.position > 0.0f) ? 1 : -1;
        }
    }

    if (direction != 0) {
        trackCornerCandidateDirection = direction;
        trackCornerCandidateTicks = TRACK_CORNER_CANDIDATE_HOLD_TICKS;
        return;
    }

    if (trackCornerCandidateTicks > 0U) {
        trackCornerCandidateTicks--;
        if (trackCornerCandidateTicks == 0U) {
            trackCornerCandidateDirection = 0;
        }
    }
}

/**
 * @brief  获取可直接触发急弯的单侧强压线方向。
 * @return -1 表示左急弯，+1 表示右急弯，0 表示未满足直接触发条件。
 * @note   只有连续图案的一侧至少 3 路压线，且该侧比另一侧至少多 2 路时才触发。
 */
static int8_t appTrackingGetDirectCornerDirection(void)
{
    uint8_t leftCount = 0U;
    uint8_t rightCount = 0U;

    if ((appTrackingPatternIsContiguous() == 0U)
        || (ir_data.black_count < TRACK_CORNER_DIRECT_MIN_BLACK)
        || (ir_data.black_count > TRACK_LINE_VALID_MAX_BLACK)) {
        return 0;
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        if ((ir_data.pattern & (uint8_t)(1U << i)) != 0U) {
            leftCount++;
        }
    }

    for (uint8_t i = 4U; i < IRTRACK_CHANNELS; i++) {
        if ((ir_data.pattern & (uint8_t)(1U << i)) != 0U) {
            rightCount++;
        }
    }

    if ((leftCount >= TRACK_CORNER_DIRECT_MIN_BLACK)
        && (leftCount >= (uint8_t)(rightCount + 2U))) {
        return -1;
    }
    if ((rightCount >= TRACK_CORNER_DIRECT_MIN_BLACK)
        && (rightCount >= (uint8_t)(leftCount + 2U))) {
        return 1;
    }
    return 0;
}

/**
 * @brief  获取当前弯之后可能出现的下一弯方向。
 * @return -1 表示下一弯向左，+1 表示下一弯向右，0 表示尚未形成有效方向。
 * @note   转弯完成后的重新捕获阶段也可能马上遇到下一弯。先使用强压线方向，
 *         若线路突然全白，则使用此前保持的边缘候选方向。
 */
static int8_t appTrackingGetNextCornerDirection(void)
{
    int8_t direction = appTrackingGetDirectCornerDirection();

    if ((direction == 0) && zhanShuBaoDiuXian()
        && (trackCornerCandidateTicks > 0U)) {
        direction = trackCornerCandidateDirection;
    }

    if ((direction == trackTurnDirection)
        && (trackCornerCenterSinceTurn == 0U)) {
        return 0;
    }

    return direction;
}

/**
 * @brief  连续弯方向确认与保持。
 * @return 1 表示下一弯方向已经确认，可以立即启动；0 表示仍需等待。
 */
static uint8_t appTrackingUpdatePendingCorner(void)
{
    int8_t direction = appTrackingGetNextCornerDirection();

    if (direction != 0) {
        if (direction == trackPendingCornerDirection) {
            if (trackPendingCornerTicks < TRACK_CORNER_PENDING_CONFIRM_TICKS) {
                trackPendingCornerTicks++;
            }
        } else {
            trackPendingCornerDirection = direction;
            trackPendingCornerTicks = 1U;
        }

        return (uint8_t)(trackPendingCornerTicks
                         >= TRACK_CORNER_PENDING_CONFIRM_TICKS);
    }

    if (trackPendingCornerTicks > 0U) {
        trackPendingCornerTicks--;
        if (trackPendingCornerTicks == 0U) {
            trackPendingCornerDirection = 0;
        }
    }

    return 0U;
}

/**
 * @brief  以指定方向启动一次急弯转向。
 * @param  direction -1 表示左转，+1 表示右转。
 * @return 1 表示已启动，0 表示方向无效。
 */
static uint8_t appTrackingStartCornerTurnDirection(int8_t direction)
{
    if ((direction == 0) || (trackFinishArmed != 0U)) {
        return 0U;
    }

    if ((trackLapTurnDirection != 0)
        && (direction != trackLapTurnDirection)) {
        return 0U;
    }

    trackTurnDirection = direction;
    trackTurnTicks = 0U;
    trackCornerCompleteTicks = 0U;
    trackCornerCenterSinceTurn = 0U;
    trackCornerCenterCleared = 0U;
    trackCornerCenterClearTicks = 0U;
    trackTurnPhase = TRACK_PHASE_TURN;
    trackInvalidLineTicks = 0U;
    trackCornerLostLineTicks = 0U;
    trackCornerArmed = 0U;
    appTrackingClearPendingCorner();
    trackControlState = (direction > 0)
                        ? TRACK_CONTROL_TURN_RIGHT
                        : TRACK_CONTROL_TURN_LEFT;
    appTrackingClearCornerCandidate();
    return 1U;
}

/**
 * @brief  根据强压线图案或边缘候选启动一次急弯。
 * @return 1 表示本周期已经启动转弯，0 表示未启动。
 */
static uint8_t appTrackingStartCornerTurn(void)
{
    int8_t direction = 0;

    if ((trackControlState != TRACK_CONTROL_FOLLOW)
        || (trackCornerArmed == 0U)
        || (trackFinishArmed != 0U)
        || (ir_data.black_count >= TRACK_FINISH_BLACK_COUNT_MIN)) {
        return 0U;
    }

    direction = appTrackingGetDirectCornerDirection();
    if ((direction == 0) && zhanShuBaoDiuXian()
        && (trackCornerCandidateTicks > 0U)) {
        direction = trackCornerCandidateDirection;
    }

    if (direction == 0) {
        return 0U;
    }

    return appTrackingStartCornerTurnDirection(direction);
}

/**
 * @brief  判断完整八路权重位置是否已经重新回到中心。
 * @return 1 表示线路有效且归一化位置接近零，0 表示尚未居中。
 * @note   不依赖 IR4/IR5 掩码，中间传感器熄灭时仍可由其他通道判断位置。
 */
static uint8_t appTrackingCornerRecenterSeen(void)
{
    return (uint8_t)((appTrackingHasValidLine() != 0U)
                     && (fabsf(ir_data.position)
                         <= TRACK_CORNER_REACQUIRE_POSITION_MAX));
}

/**
 * @brief  结束一次急弯动作并恢复普通 PID 循迹。
 * @param  baseSpeed 当前普通循迹基准速度，单位 RPM。
 */
static void appTrackingFinishCornerTurn(float baseSpeed)
{
    int8_t completedDirection = trackTurnDirection;
    int8_t pendingDirection = trackPendingCornerDirection;
    uint8_t pendingAllowed = 0U;

    if ((trackLapTurnDirection == 0) && (completedDirection != 0)) {
        trackLapTurnDirection = completedDirection;
    }

    if ((completedDirection != 0)
        && (completedDirection == trackLapTurnDirection)
        && (trackLapCornerCount < TRACK_CORNERS_PER_LAP)) {
        trackLapCornerCount++;
    }

    appTrackingClearTurnLatch();
    trackCornerArmed = 1U;

    if (trackLapCornerCount >= TRACK_CORNERS_PER_LAP) {
        appTrackingClearPendingCorner();
        trackFinishArmed = 1U;
        trackTurnPhase = TRACK_PHASE_FOLLOW;
        trackControlState = TRACK_CONTROL_FOLLOW;
        appTrackingApplyFollowTargetsInternal(baseSpeed, 1U);
        return;
    }

    pendingAllowed = (uint8_t)(
        (pendingDirection != 0)
        && (trackPendingCornerTicks >= TRACK_CORNER_PENDING_CONFIRM_TICKS)
        && ((trackLapTurnDirection == 0)
            || (pendingDirection == trackLapTurnDirection)));

    /*
     * 如果重新捕获期间已经锁存了下一弯，第一弯结束后不要回到普通循迹，
     * 直接以锁存方向启动下一次急弯转向。
     */
    if (pendingAllowed != 0U) {
        (void)appTrackingStartCornerTurnDirection(pendingDirection);
        return;
    }

    appTrackingClearPendingCorner();
    trackTurnPhase = TRACK_PHASE_FOLLOW;
    trackControlState = TRACK_CONTROL_FOLLOW;
    appTrackingApplyFollowTargetsInternal(baseSpeed, 1U);
}

/**
 * @brief  使用普通八路 PID 循迹并等待本次弯道标记完成。
 * @param  baseSpeed 当前档位对应的循迹基准速度，单位 RPM。
 * @note   急弯标记只负责方向锁存和计圈；轮速目标与普通循迹完全相同。
 */
static void appTrackingUpdateCornerTurn(float baseSpeed)
{
    uint8_t recenterSeen;

    trackTurnTicks++;
    appTrackingApplyFollowTargetsInternal(baseSpeed, 0U);
    trackTurnPhase = TRACK_PHASE_TURN;

    recenterSeen = appTrackingCornerRecenterSeen();
    if (recenterSeen != 0U) {
        trackCornerCenterSinceTurn = 1U;
    }

    if (trackCornerCenterCleared == 0U) {
        if ((appTrackingHasValidLine() != 0U)
            && (fabsf(ir_data.position)
                > TRACK_CORNER_CENTER_CLEAR_POSITION_MIN)) {
            if (trackCornerCenterClearTicks
                < TRACK_CORNER_CENTER_CLEAR_TICKS) {
                trackCornerCenterClearTicks++;
            }

            if (trackCornerCenterClearTicks
                >= TRACK_CORNER_CENTER_CLEAR_TICKS) {
                trackCornerCenterCleared = 1U;
            }
        } else if (recenterSeen != 0U) {
            trackCornerCenterClearTicks = 0U;
        }
    }

    (void)appTrackingUpdatePendingCorner();

    if ((trackTurnTicks >= TRACK_CORNER_MIN_TICKS)
        && (trackCornerCenterCleared != 0U)
        && (recenterSeen != 0U)) {
        if (trackCornerCompleteTicks
            < TRACK_CORNER_REACQUIRE_CENTER_TICKS) {
            trackCornerCompleteTicks++;
        }

        if (trackCornerCompleteTicks
            >= TRACK_CORNER_REACQUIRE_CENTER_TICKS) {
            appTrackingFinishCornerTurn(baseSpeed);
            return;
        }
    } else {
        trackCornerCompleteTicks = 0U;
    }
}

/**
 * @brief  执行启动后的直线前进阶段，直到检测到有效线路。
 * @param  baseSpeed 当前档位对应的循迹基准速度，单位 RPM。
 * @return 1 表示本周期仍处于启动前进阶段，0 表示允许进入普通循迹。
 */
static uint8_t appTrackingUpdateStartupForward(float baseSpeed)
{
    if (trackLineAcquired) {
        return 0U;
    }

    if (appTrackingCornerRecenterSeen() != 0U) {
        trackLineAcquired = 1U;
        trackStartupTicks = 0U;
        return 0U;
    }

    if (trackStartupTicks < TRACK_START_FORWARD_TICKS) {
        trackStartupTicks++;
        trackControlState = TRACK_CONTROL_FOLLOW;
        trackTurnPhase = TRACK_PHASE_FOLLOW;
        appTrackingApplyTargets(baseSpeed, 0.0f);
        return 1U;
    }

    return 0U;
}

/**
 * @brief  在完成四弯后处理终点横线并计圈。
 * @return 1 表示本周期终点状态已接管控制，0 表示继续普通循迹。
 * @note   起点横线不再用于自动布防；只有每圈四个同向急弯完成后才允许计圈。
 */
static uint8_t appTrackingUpdateFinish(void)
{
    uint8_t finishSeen =
        (ir_data.black_count >= TRACK_FINISH_BLACK_COUNT_MIN);
    uint8_t validLineSeen = appTrackingHasValidLine();

    if (trackFinishArmed == 0U) {
        trackFinishConfirmTicks = 0U;
        return 0U;
    }

    if (finishSeen == 0U) {
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
        trackFinishArmed = 0U;
        trackCornerArmed = 0U;
        trackLapCornerCount = 0U;
        trackLapTurnDirection = 0;
        appStateIncrementCompletedLaps();

        if (appStateGetCompletedLaps() >= appStateGetLapTarget()) {
            appTrackingStop(stopReasonTargetLaps);
            return 1U;
        }

        trackPostLapLineGraceTicks = TRACK_POST_LAP_LINE_GRACE_TICKS;
        trackTurnPhase = TRACK_PHASE_FOLLOW;
        trackControlState = TRACK_CONTROL_FOLLOW;
        return 1U;
    }

    return 1U;
}

/**
 * @brief  复位循迹状态机、急弯状态、滤波器和红外数据。
 * @note   停车、重新启动或切换模式时调用；不会修改 PID 参数。
 */
void appTrackingResetState(void)
{
    trackControlState = TRACK_CONTROL_FOLLOW;
    trackFinishConfirmTicks = 0U;
    trackFinishArmed = 0U;
    trackPostLapLineGraceTicks = 0U;
    trackStartupTicks = 0U;
    trackLineAcquired = 0U;
    trackLapCornerCount = 0U;
    trackLapTurnDirection = 0;
    trackInvalidLineTicks = 0U;
    trackAllWhiteTicks = 0U;
    appTrackingClearTurnLatch();
    trackCornerArmed = 0U;
    appTrackingResetLinePid();
    appTrackingClearCornerCandidate();
    appTrackingClearPendingCorner();
    trackSteerOutput = 0.0f;
    trackAverageTargetRpm = 0.0f;
    trackScheduledBaseRpm = 0.0f;

    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] = 1U;
    }
    ir_data.weight_sum = 0;
    ir_data.position = 0.0f;
    ir_data.black_count = 0U;
    ir_data.pattern = 0U;
}

/**
 * @brief  启动一次新的循迹任务。
 * @note   清零已完成圈数并复位状态机，但不直接设置运行标志。
 */
void appTrackingStart(void)
{
    appStateResetLapProgress();
    appTrackingResetState();
}

void appTrackingSetPidGains(float kp, float ki, float kd)
{
    TrackPid_SetGains(&trackPid, kp, ki, kd);
}

void appTrackingGetPidGains(float *kp, float *ki, float *kd)
{
    TrackPid_GetGains(&trackPid, kp, ki, kd);
}

void appTrackingSetSteerRpmToPwmGain(float gain)
{
    trackSteerRpmToPwmGain = appTrackingClampValue(gain, 0.0f, 5.0f);
}

/**
 * @brief  执行 20 ms 数字循迹任务。
 * @note   先读取并计算八路红外，再按停止、启动、终点、急弯、丢线和普通循迹
 *         的优先级更新目标速度。本函数只生成目标，电机输出由 50 ms 控制任务执行。
 */
void appTrackingTask20ms(void)
{
    float baseSpeed;
    uint8_t cornerTurnActive;

    if (workMode != modeTrack) {
        return;
    }

    IRTrack_Read();
    IRTrack_Compute();

    if (!runFlag) {
        return;
    }

    cornerTurnActive = appTrackingIsCornerTurnActive();

    if (cornerTurnActive != 0U) {
        if (appTrackingHasValidLine() == 0U) {
            if (trackCornerLostLineTicks
                < TRACK_CORNER_LOST_LINE_STOP_TICKS) {
                trackCornerLostLineTicks++;
            }

            if (trackCornerLostLineTicks
                >= TRACK_CORNER_LOST_LINE_STOP_TICKS) {
                appTrackingStop(stopReasonCornerTimeout);
                return;
            }
        } else {
            trackCornerLostLineTicks = 0U;
            trackAllWhiteTicks = 0U;
        }
    } else if (zhanShuBaoDiuXian() != 0U) {
        if (trackAllWhiteTicks < TRACK_ALL_WHITE_STOP_TICKS) {
            trackAllWhiteTicks++;
        }

        if (trackAllWhiteTicks >= TRACK_ALL_WHITE_STOP_TICKS) {
            appTrackingStop(stopReasonAllWhite);
            return;
        }
    } else {
        trackAllWhiteTicks = 0U;
    }

    appTrackingUpdateCornerCandidate();

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
        appTrackingApplyFollowTargets(baseSpeed);
        return;
    }

    if ((trackControlState == TRACK_CONTROL_TURN_LEFT)
        || (trackControlState == TRACK_CONTROL_TURN_RIGHT)) {
        appTrackingUpdateCornerTurn(baseSpeed);
        return;
    }

    if (appTrackingStartCornerTurn()) {
        return;
    }

    if (!appTrackingHasValidLine()) {
        appTrackingApplyFollowTargetsInternal(baseSpeed, 0U);

        if ((trackInvalidLineTicks >= TRACK_LINE_INVALID_CONFIRM_TICKS)
            && (trackPostLapLineGraceTicks > 0U)) {
            trackPostLapLineGraceTicks--;
        }

        return;
    }

    trackInvalidLineTicks = 0U;
    appTrackingClearTurnLatch();
    appTrackingClearPendingCorner();
    if (trackControlState == TRACK_CONTROL_LOST) {
        appTrackingClearCornerCandidate();
    }
    if (appTrackingCornerRecenterSeen() != 0U) {
        trackCornerArmed = 1U;
    }
    trackControlState = TRACK_CONTROL_FOLLOW;
    appTrackingApplyFollowTargets(baseSpeed);
}

/**
 * @brief  获取当前循迹控制状态。
 * @return 当前状态机枚举值。
 */
TrackControlState_t appTrackingGetState(void)
{
    return trackControlState;
}

/**
 * @brief  获取最近一次循迹差速输出。
 * @return 左右目标差的一半，单位 RPM。
 */
float appTrackingGetSteer(void)
{
    return trackSteerOutput;
}

float appTrackingGetAverageTargetRpm(void)
{
    return trackAverageTargetRpm;
}

float appTrackingGetSteerRpmToPwmGain(void)
{
    return trackSteerRpmToPwmGain;
}

/**
 * @brief  获取最近一次用于 PID 的加权误差。
 * @return 负值偏左，正值偏右。
 */
float appTrackingGetWeightedError(void)
{
    return trackWeightedError;
}

/**
 * @brief  获取当前循迹 PID 输出阶段。
 * @return 普通跟随、转弯或外侧误差标记阶段；RECOVER 为兼容保留值。
 */
TrackTurnPhase_t appTrackingGetTurnPhase(void)
{
    return trackTurnPhase;
}

float appTrackingComputeWeightedError(void)
{
    return appTrackingComputeWeightedErrorInternal();
}

float appTrackingUpdatePidForError(float error, float diffLimit)
{
    return appTrackingUpdateLinePidInternal(error, diffLimit);
}

void appTrackingSetPwmSteer(float steerRpm)
{
    appTrackingApplyTargets(0.0f, steerRpm);
}

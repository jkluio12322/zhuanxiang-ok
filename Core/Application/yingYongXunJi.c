/**
 * @file    yingYongXunJi.c
 * @brief   八路数字循迹、急弯确认和丢线搜索状态机。
 */

#include "yingYongXunJi.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "irtrack.h"
#include "zhanShuBao.h"

#include <math.h>

/* ------------------------------------------------------------------------- */
/* 急弯检测与转弯确认常调参数                                                */
/* ------------------------------------------------------------------------- */

/* 急弯边缘位置阈值，绝对值越大表示线路越靠外。 */
#define TRACK_CORNER_EDGE_POSITION_MIN 4.5f

/* 判断边缘候选所需的最少压线通道数。 */
#define TRACK_CORNER_EDGE_MIN_BLACK 1U

/* 无需等待离线即可直接触发急弯所需的单侧最少压线通道数。 */
#define TRACK_CORNER_DIRECT_MIN_BLACK 3U

/* 边缘候选保持时间，5 个 20 ms 周期约为 100 ms。 */
#define TRACK_CORNER_CANDIDATE_HOLD_TICKS 5U

/* 下一个弯需要连续确认的次数，避免单帧噪声触发连续转向。 */
#define TRACK_CORNER_PENDING_CONFIRM_TICKS 2U

/* 转弯后等待最中间两路确认时使用的前进速度，单位 RPM。 */
#define TRACK_CORNER_REACQUIRE_BASE_RPM 15.0f

/* 查表循迹允许的最低前进目标，单位 RPM。 */
#define TRACK_MIN_FORWARD_RPM 15.0f

/* 中心单路和中路双路图案，使用零差速目标并参与目标低通滤波。 */
#define TRACK_CENTER_LEFT_PATTERN   0x08U
#define TRACK_CENTER_RIGHT_PATTERN  0x10U
#define TRACK_CENTER_PAIR_PATTERN   0x18U

/* 丢线搜索只使用正向差速，不允许车轮反转。 */
#define TRACK_SWEEP_OUTER_RPM 35.0f
#define TRACK_SWEEP_INNER_RPM 10.0f

/* 最中间两路图案掩码，只检查 bit3~bit4。 */
#define TRACK_CORNER_COMPLETE_MASK 0x18U

/* 转弯后低速找线的最长时间，50 个 20 ms 周期约为 1 s。 */
#define TRACK_CORNER_REACQUIRE_TIMEOUT_TICKS 50U

/* 急弯最长锁存时间，100 个 20 ms 周期约为 2 s。 */
#define TRACK_CORNER_TIMEOUT_TICKS 100U

static TrackControlState_t trackControlState = TRACK_CONTROL_FOLLOW;
static uint8_t trackFinishConfirmTicks = 0U;
static uint8_t trackFinishClearTicks = 0U;
static uint8_t trackFinishArmed = 0U;
static uint8_t trackStartupTicks = 0U;
static uint8_t trackLineAcquired = 0U;
#if 0
static uint8_t trackLostSweepTicks = 0U;
static uint8_t trackLostSweepPhase = 0U;
static int8_t trackLostSweepStartDirection = -1;
#endif
static int8_t trackCornerCandidateDirection = 0;
static uint8_t trackCornerCandidateTicks = 0U;
static int8_t trackPendingCornerDirection = 0;
static uint8_t trackPendingCornerTicks = 0U;
static uint8_t trackInvalidLineTicks = 0U;
static uint8_t trackCornerArmed = 1U;
static uint8_t trackCornerCompleteTicks = 0U;
static uint8_t trackCornerCenterSinceTurn = 0U;
static uint16_t trackCornerReacquireTicks = 0U;
static int8_t trackTurnDirection = 0;
static uint16_t trackTurnTicks = 0U;
static float trackSteerOutput = 0.0f;
static float trackFilteredLeftPercent = 100.0f;
static float trackFilteredRightPercent = 100.0f;

#if 0
static void appTrackingUpdateLineSearch(float baseSpeed);
#endif

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
 * @brief  应用左右轮目标并限制到指定范围内。
 * @param  nominalLeft 未限幅的左轮目标转速，单位 RPM。
 * @param  nominalRight 未限幅的右轮目标转速，单位 RPM。
 * @param  minRpm 左右轮允许的最小目标转速，单位 RPM。
 * @note   普通循迹使用正转下限，避免内轮反转；需要反向内轮的急弯动作
 *         由战术包模块直接设置目标。
 */
static void appTrackingApplyTargets(float nominalLeft,
                                    float nominalRight,
                                    float minRpm)
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
                                       minRpm,
                                       SPEED_TARGET_LIMIT_RPM);
    targetRight = appTrackingClampValue(targetRight,
                                        minRpm,
                                        SPEED_TARGET_LIMIT_RPM);

    targetLeftRpm = targetLeft;
    targetRightRpm = targetRight;
    turnTarget = (targetLeft - targetRight) * 0.5f;
    trackSteerOutput = turnTarget;
}

/**
 * @brief  应用普通循迹的左右轮目标。
 * @param  nominalLeft 未限幅的左轮目标转速，单位 RPM。
 * @param  nominalRight 未限幅的右轮目标转速，单位 RPM。
 * @note   普通循迹不允许内轮反转，因此使用正转最低目标限幅。
 */
static void appTrackingSetTargets(float nominalLeft, float nominalRight)
{
    appTrackingApplyTargets(nominalLeft,
                            nominalRight,
                            TRACK_MIN_FORWARD_RPM);
}

/**
 * @brief  复位左右轮百分比低通滤波器。
 * @note   进入普通循迹、完成急弯或重新启动时调用，避免旧线路位置继续影响输出。
 */
static void appTrackingResetTargetFilter(void)
{
    trackFilteredLeftPercent = 100.0f;
    trackFilteredRightPercent = 100.0f;
}

/**
 * @brief  根据当前红外查表结果生成左右轮目标转速。
 * @param  baseSpeed 当前档位对应的循迹基准速度，单位 RPM。
 * @note   窄线图案对百分比输出进行低通滤波，宽线图案直接采用查表结果；
 *         中心单路和中心双路图案使用零差速以抑制直线抖动。
 */
static void appTrackingApplyLutTargets(float baseSpeed)
{
    float leftPercent = (float)ir_data.left_percent;
    float rightPercent = (float)ir_data.right_percent;
    float nominalLeft;
    float nominalRight;

    if ((ir_data.pattern == TRACK_CENTER_LEFT_PATTERN)
        || (ir_data.pattern == TRACK_CENTER_RIGHT_PATTERN)
        || (ir_data.pattern == TRACK_CENTER_PAIR_PATTERN)) {
        leftPercent = 100.0f;
        rightPercent = 100.0f;
    }

    if (ir_data.black_count <= 2U) {
        trackFilteredLeftPercent =
            (TRACK_POSITION_FILTER_PREVIOUS * trackFilteredLeftPercent)
            + (TRACK_POSITION_FILTER_CURRENT * leftPercent);
        trackFilteredRightPercent =
            (TRACK_POSITION_FILTER_PREVIOUS * trackFilteredRightPercent)
            + (TRACK_POSITION_FILTER_CURRENT * rightPercent);
    } else {
        trackFilteredLeftPercent = leftPercent;
        trackFilteredRightPercent = rightPercent;
    }

    nominalLeft = baseSpeed * trackFilteredLeftPercent / 100.0f;
    nominalRight = baseSpeed * trackFilteredRightPercent / 100.0f;

    appTrackingSetTargets(nominalLeft, nominalRight);
}

/**
 * @brief  生成丢线搜索时的左右轮正转目标。
 * @param  nominalSide 搜索偏向，正值向右，负值向左。
 * @note   外轮使用 35 RPM、内轮使用 10 RPM，只允许正向运动。
 */
#if 0
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

    appTrackingApplyTargets(nominalLeft, nominalRight, 0.0f);
}
#endif

/**
 * @brief  停止循迹运动并清除状态机状态。
 * @note   急弯时内侧轮的具体速度由 zhanShuBao.c 配置；当前内侧轮使用
 *         -50 RPM 反转。该入口用于停车和重新启动。
 */
static void appTrackingStop(void)
{
    pwmDuty = 0;
    appStateResetMotion();
    appTrackingResetState();
}

/**
 * @brief  判断当前红外图案是否包含有效黑线。
 * @return 1 表示存在 1~6 个黑线通道且不是全黑或全白，0 表示无效。
 */
static uint8_t appTrackingHasValidLine(void)
{
    return (uint8_t)((zhanShuBaoDiuXian() == 0U)
                     && (zhanShuBaoYuDaoHeiXian() == 0U)
                     && (ir_data.black_count > 0U)
                     && (ir_data.black_count <= TRACK_LINE_VALID_MAX_BLACK));
}

/**
 * @brief  清除一次急弯锁存及其完成确认计数。
 */
static void appTrackingClearTurnLatch(void)
{
    trackTurnDirection = 0;
    trackTurnTicks = 0U;
    trackCornerCompleteTicks = 0U;
    trackCornerCenterSinceTurn = 0U;
    trackCornerReacquireTicks = 0U;
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
 * @brief  进入丢线搜索状态。
 * @param  preferredSide 首选搜索方向，正值向右，负值向左，0 表示使用最后检测侧。
 */
#if 0
static void appTrackingBeginLineSearch(int8_t preferredSide)
{
    trackControlState = TRACK_CONTROL_LOST;
    trackLostSweepTicks = 0U;
    trackLostSweepPhase = 0U;
    trackLostSweepStartDirection =
        (preferredSide != 0) ? preferredSide : ((ir_data.last_side > 0) ? 1 : -1);
    appTrackingClearCornerCandidate();
    appTrackingClearPendingCorner();
}
#endif

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

    if (ir_data.black_count > TRACK_LINE_VALID_MAX_BLACK) {
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
 * @note   只有一侧至少 3 路压线，且该侧比另一侧至少多 2 路时才触发。
 */
static int8_t appTrackingGetDirectCornerDirection(void)
{
    uint8_t leftCount = 0U;
    uint8_t rightCount = 0U;

    if ((ir_data.black_count < TRACK_CORNER_DIRECT_MIN_BLACK)
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
    if (direction == 0) {
        return 0U;
    }

    trackTurnDirection = direction;
    trackTurnTicks = 0U;
    trackCornerCompleteTicks = 0U;
    trackCornerCenterSinceTurn = 0U;
    trackCornerReacquireTicks = 0U;
    trackInvalidLineTicks = 0U;
    trackCornerArmed = 0U;
    appTrackingClearPendingCorner();
    trackControlState = (direction > 0)
                        ? TRACK_CONTROL_TURN_RIGHT
                        : TRACK_CONTROL_TURN_LEFT;
    (void)zhanShuBaoZhuan90Du(trackTurnDirection);
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
        || (trackCornerArmed == 0U)) {
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
 * @brief  判断最中间两路中是否至少有一路压线。
 * @return 1 表示有效线路中存在正中间压线，0 表示尚未达到居中条件。
 * @note   调用方仍要求连续多个周期确认，避免单帧噪声结束转弯。
 */
static uint8_t appTrackingCornerCenterSeen(void)
{
    return (uint8_t)((appTrackingHasValidLine() != 0U)
                     && ((ir_data.pattern & TRACK_CORNER_COMPLETE_MASK) != 0U)
                     && (fabsf(ir_data.position)
                         <= TRACK_CORNER_REACQUIRE_POSITION_MAX));
}

/**
 * @brief  结束一次急弯动作并恢复普通查表循迹。
 * @param  baseSpeed 当前普通循迹基准速度，单位 RPM。
 * @note   转弯过程中连续确认居中可以调用该函数；编码器达到硬阈值时先进入
 *         低速重捕阶段，重捕成功后再由该函数恢复普通循迹。
 */
static void appTrackingFinishCornerTurn(float baseSpeed)
{
    int8_t pendingDirection = trackPendingCornerDirection;

    zhanShuBaoChongZhiZhuan90Du();
    appTrackingClearTurnLatch();
    trackCornerArmed = 1U;

    /*
     * 如果重新捕获期间已经锁存了下一弯，第一弯结束后不要回到普通循迹，
     * 直接以锁存方向启动下一次急弯转向。
     */
    if ((pendingDirection != 0)
        && (trackPendingCornerTicks >= TRACK_CORNER_PENDING_CONFIRM_TICKS)) {
        (void)appTrackingStartCornerTurnDirection(pendingDirection);
        return;
    }

    appTrackingClearPendingCorner();
    trackControlState = TRACK_CONTROL_FOLLOW;
    appTrackingResetTargetFilter();
    appTrackingApplyLutTargets(baseSpeed);
}

/**
 * @brief  执行急弯动作或低速重捕，并等待新线路重新居中。
 * @param  baseSpeed 当前普通循迹基准速度，单位 RPM。
 * @note   最中间两路在编码器上限前连续确认即可提前完成；达到编码器上限后
 *         先进入 REACQUIRE 低速前进，连续居中后回到 FOLLOW。重捕期间确认到
 *         下一弯时直接启动下一次转向，1 秒内仍未居中才进入丢线搜索。
 */
static void appTrackingUpdateCornerTurn(float baseSpeed)
{
    if ((trackControlState == TRACK_CONTROL_TURN_LEFT)
        || (trackControlState == TRACK_CONTROL_TURN_RIGHT)) {
        trackTurnTicks++;

        if (zhanShuBaoZhuan90Du(trackTurnDirection)) {
            /*
             * 编码器已经达到标定阈值。此时线路可能已经转过弯口但尚未稳定
             * 居中，先以低速前进继续找线，不立即判定丢线。
             */
            trackControlState = TRACK_CONTROL_REACQUIRE;
            trackCornerCompleteTicks = 0U;
            trackCornerCenterSinceTurn = 0U;
            trackCornerReacquireTicks = 0U;
            appTrackingSetTargets(TRACK_CORNER_REACQUIRE_BASE_RPM,
                                  TRACK_CORNER_REACQUIRE_BASE_RPM);
            return;
        }

        /*
         * 编码器还没到目标值时，如果循迹模块已经连续三次检测到
         * 最中间任意一路压线，就按循迹模块的结果提前结束急弯转向。
         */
        if ((trackTurnTicks >= TRACK_CORNER_MIN_TICKS)
            && appTrackingCornerCenterSeen()) {
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

        if (trackTurnTicks >= TRACK_CORNER_TIMEOUT_TICKS) {
            zhanShuBaoChongZhiZhuan90Du();
            /*
             * 暂时禁用丢线搜索。
             * appTrackingBeginLineSearch(trackTurnDirection);
             * appTrackingUpdateLineSearch(baseSpeed);
             */
            appTrackingStop();
            return;
        }

        return;
    }

    trackCornerReacquireTicks++;

    if (appTrackingCornerCenterSeen()) {
        trackCornerCenterSinceTurn = 1U;
    }

    if (appTrackingUpdatePendingCorner()) {
        int8_t nextDirection = trackPendingCornerDirection;

        (void)appTrackingStartCornerTurnDirection(nextDirection);
        return;
    }

    if (appTrackingCornerCenterSeen()) {
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

    if (trackCornerReacquireTicks
        >= TRACK_CORNER_REACQUIRE_TIMEOUT_TICKS) {
        /*
         * 暂时禁用丢线搜索。
         * appTrackingBeginLineSearch(trackTurnDirection);
         * appTrackingUpdateLineSearch(baseSpeed);
         */
        appTrackingStop();
        return;
    }

    appTrackingSetTargets(TRACK_CORNER_REACQUIRE_BASE_RPM,
                          TRACK_CORNER_REACQUIRE_BASE_RPM);
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

/**
 * @brief  处理八路全黑终点图案和计圈状态。
 * @return 1 表示本周期终点状态已接管控制，0 表示继续普通循迹。
 * @note   转弯和转弯后找线期间忽略全黑图案，防止扫线过程被误判为终点。
 */
static uint8_t appTrackingUpdateFinish(void)
{
    uint8_t finishSeen = zhanShuBaoYuDaoHeiXian();
    uint8_t validLineSeen = appTrackingHasValidLine();

    /*
     * 转弯和转弯后找线期间，八路同时压黑线属于转弯扫过黑线的正常现象，
     * 不能把它当成终点，否则第一次转弯后可能直接结束循迹。
     */
    if ((trackControlState == TRACK_CONTROL_TURN_LEFT)
        || (trackControlState == TRACK_CONTROL_TURN_RIGHT)
        || (trackControlState == TRACK_CONTROL_REACQUIRE)) {
        trackFinishConfirmTicks = 0U;
        return 0U;
    }

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

/**
 * @brief  更新丢线搜索状态并生成摆头找线目标。
 * @param  baseSpeed 恢复线路后使用的循迹基准速度，单位 RPM。
 * @note   搜索超过规定往返次数仍找不到线路时立即停车。
 */
#if 0
static void appTrackingUpdateLineSearch(float baseSpeed)
{
    int8_t direction;

    if (appTrackingHasValidLine()) {
        trackControlState = TRACK_CONTROL_FOLLOW;
        trackLostSweepTicks = 0U;
        trackLostSweepPhase = 0U;

        /*
         * 第一次转弯可能由编码器先完成，但最中间两路没有连续确认三次，
         * 随后会进入丢线搜索。搜索重新找到线时必须重新布防急弯检测，
         * 否则 trackCornerArmed 会一直为 0，后续所有弯都不会再触发。
         */
        appTrackingClearTurnLatch();
        appTrackingClearCornerCandidate();
        trackCornerArmed = 1U;

        appTrackingResetTargetFilter();
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
#endif

/**
 * @brief  复位循迹状态机、急弯状态、滤波器和红外数据。
 * @note   停车、重新启动或切换模式时调用；不会修改 PID 参数。
 */
void appTrackingResetState(void)
{
    trackControlState = TRACK_CONTROL_FOLLOW;
    trackFinishConfirmTicks = 0U;
    trackFinishClearTicks = 0U;
    trackFinishArmed = 0U;
    trackStartupTicks = 0U;
    trackLineAcquired = 0U;
#if 0
    trackLostSweepTicks = 0U;
    trackLostSweepPhase = 0U;
    trackLostSweepStartDirection = -1;
#endif
    trackInvalidLineTicks = 0U;
    appTrackingClearTurnLatch();
    trackCornerArmed = 1U;
    zhanShuBaoChongZhiZhuan90Du();
    appTrackingClearCornerCandidate();
    appTrackingClearPendingCorner();
    trackSteerOutput = 0.0f;
    appTrackingResetTargetFilter();

    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] = 1U;
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

/**
 * @brief  启动一次新的循迹任务。
 * @note   清零已完成圈数并复位状态机，但不直接设置运行标志。
 */
void appTrackingStart(void)
{
    appStateResetLapProgress();
    appTrackingResetState();
    pwmDuty = 0;
}

/**
 * @brief  执行 20 ms 数字循迹任务。
 * @note   先读取并计算八路红外，再按停止、启动、终点、急弯、丢线和普通循迹
 *         的优先级更新目标速度。本函数只生成目标，电机输出由 50 ms 控制任务执行。
 */
void appTrackingTask20ms(void)
{
    float baseSpeed;

    if (workMode != modeTrack) {
        return;
    }

    IRTrack_Read();
    IRTrack_Compute();
    (void)IRTrack_DetectCrossroad();

    if (!runFlag) {
        return;
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
        appTrackingApplyLutTargets(baseSpeed);
        return;
    }

    if ((trackControlState == TRACK_CONTROL_TURN_LEFT)
        || (trackControlState == TRACK_CONTROL_TURN_RIGHT)
        || (trackControlState == TRACK_CONTROL_REACQUIRE)) {
        appTrackingUpdateCornerTurn(baseSpeed);
        return;
    }

    if (appTrackingStartCornerTurn()) {
        return;
    }

    if (!appTrackingHasValidLine()) {
        if (trackInvalidLineTicks < TRACK_LINE_INVALID_CONFIRM_TICKS) {
            trackInvalidLineTicks++;
        }

        if (trackInvalidLineTicks < TRACK_LINE_INVALID_CONFIRM_TICKS) {
            return;
        }

        if (trackControlState != TRACK_CONTROL_LOST) {
            /*
             * 暂时禁用丢线搜索。
             * appTrackingBeginLineSearch(ir_data.last_side);
             * appTrackingUpdateLineSearch(baseSpeed);
             */
            appTrackingStop();
            return;
        }
        return;
    }

    trackInvalidLineTicks = 0U;
    appTrackingClearTurnLatch();
    appTrackingClearPendingCorner();
    if (trackControlState == TRACK_CONTROL_LOST) {
        appTrackingClearCornerCandidate();
    }
    trackCornerArmed = 1U;
    trackControlState = TRACK_CONTROL_FOLLOW;
    appTrackingApplyLutTargets(baseSpeed);
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
 * @brief  获取归一化线路位置。
 * @return 位置值，负值偏左，正值偏右，范围约为 -7...+7。
 */
float appTrackingGetPosition(void)
{
    return ir_data.position;
}

/**
 * @brief  获取最近一次循迹差速输出。
 * @return 左右目标差的一半，单位 RPM。
 */
float appTrackingGetSteer(void)
{
    return trackSteerOutput;
}

/**
 * @brief  复制循迹状态机的诊断字段。
 * @param  output 输出结构体指针；为 NULL 时函数直接返回。
 */
void appTrackingGetDiagnostics(TrackDiagnostics_t *output)
{
    if (output == 0) {
        return;
    }

    output->state = trackControlState;
    output->pattern = ir_data.pattern;
    output->blackCount = ir_data.black_count;
    output->candidateDirection = trackCornerCandidateDirection;
    output->candidateTicks = trackCornerCandidateTicks;
    output->pendingCornerDirection = trackPendingCornerDirection;
    output->pendingCornerTicks = trackPendingCornerTicks;
    output->invalidLineTicks = trackInvalidLineTicks;
    output->turnTicks = trackTurnTicks;
    output->reacquireTicks = trackCornerReacquireTicks;
}

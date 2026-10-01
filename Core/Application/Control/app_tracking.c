/**
 * @file    app_tracking.c
 * @brief   八路循迹的实现。
 * @note    整个循迹就是「一个位置式 PID + 一个计圈计数器」，装在一个函数里，
     *          没有 track_xxx_step() 分支函数，也没有单独的发布函数。
 */

#include "app_tracking.h"
#include "app_config.h"
#include "app_scheduler.h"
#include "app_state.h"
#include "irtrack.h"

/* 本文件内部的线路判定结果。 */
typedef struct {
    uint8_t pattern;      /* bit0..7 对应 IR1..8，压黑置 1。 */
    uint8_t black_count;
    uint8_t contiguous;   /* 是否为单一连续黑线段。 */
    uint8_t valid;        /* 压黑 1~7 路且连续。 */
    uint8_t all_white;    /* 八路全白。 */
    float   e_y;          /* 归一化横向偏差，直接取 ir_data.position。 */
} LineSample_t;

/* Control/app_tracking.c 的全部静态状态 —— 11 个，没有别的。 */
static LineSample_t   sample;
static Pid_t          pid;
static TrackState_t   trackState = TRK_STOPPED;
static TrackOutput_t  output;
static float          lastSteer;
static ConfirmCounter invalidCounter;
static ConfirmCounter allWhiteCounter;
static ConfirmCounter lostStopCounter;
static ConfirmCounter cornerCounter;
static uint16_t       cornerGap;
static uint8_t        lapCorners;
static uint8_t        startupTicks;
static float          steerRpmToPwmGain = TRACK_STEER_RPM_TO_PWM_DEFAULT;

static const PidConfig_t trackPidConfig = {
    .kp = TRACK_PID_KP,
    .ki = TRACK_PID_KI,
    .kd = TRACK_PID_KD,
    .dt = TRACK_PID_PERIOD_SECONDS,
    .out_min = -TRACK_FOLLOW_DIFF_LIMIT_RPM,
    .out_max = TRACK_FOLLOW_DIFF_LIMIT_RPM,
    .zero_target_hold = 0.0f,
    .out_offset = 0.0f,
    .mode = PID_MODE_POSITIONAL,
    .reset_on_sign_change = 0U,        /* 误差正负交替是正常的，不能复位。 */
    .derivative_on_measurement = 0U,   /* 位置式的微分作用于误差。 */
};

/* 数字量传感器容易出现「IR1 和 IR8 都黑、中间全白」这种断续图案，
   必须判无效，否则质心会算出个「居中」的假象。 */
static uint8_t line_is_contiguous(uint8_t pattern)
{
    uint8_t first = 0U;
    uint8_t last = 0U;
    uint8_t found = 0U;

    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        if ((pattern & (uint8_t)(1U << i)) != 0U) {
            if (found == 0U) {
                first = i;
                found = 1U;
            }
            last = i;
        }
    }

    if (found == 0U) {
        return 0U;
    }

    for (uint8_t i = first; i <= last; i++) {
        if ((pattern & (uint8_t)(1U << i)) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

/* 读八路并做判定。e_y 直接用驱动层算好的归一化质心，不重算。 */
static void sample_line(void)
{
    IRTrack_Read();
    IRTrack_Compute();

    sample.pattern = ir_data.pattern;
    sample.black_count = ir_data.black_count;
    sample.e_y = ir_data.position;
    sample.contiguous = line_is_contiguous(ir_data.pattern);

    sample.valid = (uint8_t)((ir_data.black_count >= 1U)
                             && (ir_data.black_count <= TRACK_LINE_VALID_MAX_BLACK)
                             && (sample.contiguous != 0U));
    sample.all_white = (ir_data.black_count == 0U) ? 1U : 0U;
}

const TrackOutput_t *AppTracking_Output(void)
{
    return &output;
}

void AppTracking_Init(void)
{
    Pid_Init(&pid, &trackPidConfig);
    AppTracking_ResetState();
}

void AppTracking_ResetState(void)
{
    Pid_Reset(&pid);
    Pid_SetLimits(&pid, -TRACK_FOLLOW_DIFF_LIMIT_RPM, TRACK_FOLLOW_DIFF_LIMIT_RPM);

    Confirm_Reset(&invalidCounter, TRACK_LINE_INVALID_CONFIRM_TICKS);
    Confirm_Reset(&allWhiteCounter, TRACK_ALL_WHITE_STOP_TICKS);
    Confirm_Reset(&lostStopCounter, TRACK_LINE_LOST_STOP_TICKS);
    Confirm_Reset(&cornerCounter, TRACK_CORNER_CONFIRM_TICKS);

    cornerGap = 0U;
    lapCorners = 0U;
    startupTicks = 0U;
    lastSteer = 0.0f;
    trackState = TRK_STOPPED;

    output.base_rpm = 0.0f;
    output.steer_rpm = 0.0f;
    output.e_y = 0.0f;
    output.state = trackState;
}

void AppTracking_Start(void)
{
    AppTracking_ResetState();
    trackState = TRK_ACQUIRE;
    output.state = trackState;
}

void AppTracking_SetGain(PidField_t field, float value)
{
    Pid_SetGain(&pid, field, value);
}

float AppTracking_GetGain(PidField_t field)
{
    return Pid_GetGain(&pid, field);
}

void AppTracking_SetSteerRpmToPwmGain(float gain)
{
    steerRpmToPwmGain = gain;
}

float AppTracking_GetSteerRpmToPwmGain(void)
{
    return steerRpmToPwmGain;
}

float AppTracking_ComputeWeightedError(void)
{
    IRTrack_Read();
    IRTrack_Compute();
    return ir_data.position;
}

float AppTracking_UpdatePidForError(float error, float diffLimit)
{
    Pid_SetLimits(&pid, -diffLimit, diffLimit);
    return Pid_UpdatePosition(&pid, error);
}

void AppTracking_SetPwmSteer(float steerRpm)
{
    lastSteer = steerRpm;
    output.steer_rpm = steerRpm;
}

float AppTracking_GetSteer(void)
{
    return output.steer_rpm;
}

void AppTracking_Task20ms(void)
{
    if (AppState()->mode != modeTrack) {
        return;
    }

    sample_line();

    if (!AppState()->running) {
        Pid_Reset(&pid);
        trackState = TRK_STOPPED;
        output.state = trackState;
        return;
    }

    /* --- 线路质量：无效确认 / 全白确认 --- */
    if (sample.valid != 0U) {
        Confirm_Reset(&invalidCounter, TRACK_LINE_INVALID_CONFIRM_TICKS);
        Confirm_Reset(&lostStopCounter, TRACK_LINE_LOST_STOP_TICKS);
        if (trackState == TRK_LOST) {
            Pid_Reset(&pid);
            trackState = TRK_FOLLOW;
        }
    } else if (Confirm_Update(&invalidCounter, 1U) != 0U) {
        trackState = TRK_LOST;
    }

    if (Confirm_Update(&allWhiteCounter, sample.all_white) != 0U) {
        AppState_ReportStop(stopReasonAllWhite);
        return;
    }

    /* 丢线持续超时后停车：只有在 LOST 状态里才计时。 */
    if (trackState == TRK_LOST) {
        if (Confirm_Update(&lostStopCounter, 1U) != 0U) {
            AppState_ReportStop(stopReasonCornerTimeout);
            return;
        }
    }

    /* --- 状态动作：只有 FOLLOW 分支在做实事 --- */
    if (trackState == TRK_ACQUIRE) {
        if (++startupTicks >= TRACK_START_FORWARD_TICKS) {
            trackState = TRK_FOLLOW;
        }
    } else if ((trackState == TRK_FOLLOW) && (sample.valid != 0U)) {

        /* ★ 唯一的转向计算。 */
        lastSteer = Pid_UpdatePosition(&pid, sample.e_y);

        /* ★ 唯一的计圈逻辑：e_y 偏到一侧即在过弯；
           起点横线正对时八路全压、e_y≈0，天然不计。 */
        if (cornerGap > 0U) {
            cornerGap--;
        } else if (Confirm_Update(&cornerCounter,
                                  (uint8_t)((sample.e_y >= TRACK_CORNER_E_Y_MIN)
                                            || (sample.e_y <= -TRACK_CORNER_E_Y_MIN)))
                   != 0U) {
            Confirm_Reset(&cornerCounter, TRACK_CORNER_CONFIRM_TICKS);
            cornerGap = TRACK_CORNER_GAP_TICKS;

            if (++lapCorners >= TRACK_CORNERS_PER_LAP) {
                lapCorners = 0U;
                AppState_IncrementLap();
            }
        }
    }
    /* TRK_LOST：什么都不做，lastSteer 保持丢线前的值。 */

    /* --- 发布快照给速度环 / 遥测 / 显示 --- */
    output.base_rpm = AppState()->gear_rpm;
    output.steer_rpm = lastSteer;
    output.e_y = sample.e_y;
    output.state = trackState;
}

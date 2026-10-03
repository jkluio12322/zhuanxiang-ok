/**
 * @file    app_tracking.c
 * @brief   八路数字循迹 — 归一化质心 + 单个位置式 PD + 计时计圈。
 *
 * @note    整个循迹就是「一个 PID + 一个计圈计数器」，装在 AppTracking_Task20ms()
 *          一个函数里，没有 track_xxx_step() 分支函数，也没有单独的发布函数。
 *
 * @note    误差信号取的是**归一化质心** `e_y = ir_data.position`，不是权重和：
 *          - `weight_sum` 压 1 路最大 ±25、压 5 路能到 ±65，同一个增益要面对
 *            2.6 倍的量纲漂移；
 *          - `position` 已经除以压黑路数，量纲恒定 ±7。
 *          这是取代旧实现的关键一步，详见 README §4.3.1。
 *
 * @note    本模块**不直接驱电机**：只发布 base_rpm 与 steer_rpm，
 *          由 app_speedloop.c 的左右速度环各自闭环成 PWM。
 *
 * @note    执行上下文：AppTracking_Task20ms() 在 TIM6 中断里，20 ms 一次。
 *
 * @see     README.md §4.3   完整算法说明
 * @see     app_speedloop.c  速度环与最终输出
 */

#include "app_tracking.h"
#include "app_config.h"
#include "app_scheduler.h"
#include "app_state.h"
#include "irtrack.h"

/**
 * @brief  一次采样的线路判定结果。
 * @note   只在本文件内使用，不导出。
 */
typedef struct {
    uint8_t pattern;      /* bit0..7 对应 IR1..8，压黑置 1。 */
    uint8_t black_count;  /* 压黑通道数，0~8。 */
    uint8_t contiguous;   /* 是否为单一连续黑线段。 */
    uint8_t valid;        /* 压黑 1~TRACK_LINE_VALID_MAX_BLACK 路且连续。 */
    uint8_t all_white;    /* 八路全白。 */
    float   e_y;          /* 归一化横向偏差，直接取 ir_data.position，范围约 ±7。 */
} LineSample_t;

/* ── 全部静态状态：11 个，没有别的 ───────────────────────────────────── */

static LineSample_t   sample;                 /* 本周期采样结果。 */
static Pid_t          pid;                    /* 循迹位置环。 */
static TrackState_t   trackState = TRK_STOPPED; /* 状态机当前状态。 */
static TrackOutput_t  output;                 /* 对外发布的只读快照。 */
static float          lastSteer;              /* 最近一次有效舵量，丢线时保持不变。 */
static ConfirmCounter invalidCounter;         /* 线路无效确认，连续 N 次转入 LOST。 */
static ConfirmCounter allWhiteCounter;        /* 八路全白确认，连续 N 次停车。 */
static ConfirmCounter lostStopCounter;        /* 丢线持续确认，连续 N 次超时停车。 */
static ConfirmCounter cornerCounter;          /* 「偏到一侧」确认，用于计圈。 */
static uint16_t       cornerGap;              /* 计弯后的间隔，防止一个弯计成多个。 */
static uint8_t        lapCorners;             /* 本圈已累计的弯数，到 4 计一圈。 */
static uint8_t        startupTicks;           /* 起步直行计时。 */
static float          steerRpmToPwmGain = TRACK_STEER_RPM_TO_PWM_DEFAULT;

/* 循迹环配置：位置式 PD。
   输出限幅取 TRACK_FOLLOW_DIFF_LIMIT_RPM（±60 RPM），这是差速量的上限。
   zero_target_hold 与 out_offset 都必须是 0——它们是给速度环用的，
   位置环的「目标」就是误差本身，套上死区会把小偏差直接吃掉。 */
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

/**
 * @brief  判断压黑图案是否连成一段。
 * @param  pattern 黑线掩码，bit0..7 对应 IR1..8。
 * @return 1 表示最低压黑位到最高压黑位之间全为黑；0 表示没有压线或存在白洞。
 * @note   数字量传感器容易出现「IR1 和 IR8 都黑、中间全白」这种断续图案
 *         （斜穿横线、岔路口），必须判无效，否则质心会算出个「居中」的假象。
 */
static uint8_t line_is_contiguous(uint8_t pattern)
{
    uint8_t first = 0U;
    uint8_t last = 0U;
    uint8_t found = 0U;

    /* 先找出最低和最高的压黑通道。 */
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

    /* 再检查两端之间有没有空洞。 */
    for (uint8_t i = first; i <= last; i++) {
        if ((pattern & (uint8_t)(1U << i)) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief  读八路传感器并完成线路判定。
 * @note   调用驱动层 IRTrack_Read()/IRTrack_Compute() 刷新 ir_data，
 *         再把需要的结果拷进本文件的 sample。e_y 直接用驱动层算好的
 *         归一化质心，不在这里重算。
 */
static void sample_line(void)
{
    IRTrack_Read();
    IRTrack_Compute();

    sample.pattern = ir_data.pattern;
    sample.black_count = ir_data.black_count;
    sample.e_y = ir_data.position;
    sample.contiguous = line_is_contiguous(ir_data.pattern);

    /* 有效 = 压黑 1~7 路**且**连续。压满 8 路视为横线，不作为循迹线路。 */
    sample.valid = (uint8_t)((ir_data.black_count >= 1U)
                             && (ir_data.black_count <= TRACK_LINE_VALID_MAX_BLACK)
                             && (sample.contiguous != 0U));
    sample.all_white = (ir_data.black_count == 0U) ? 1U : 0U;
}

/**
 * @brief  获取只读循迹快照。
 * @return 指向内部快照的指针，调用方只读。
 */
const TrackOutput_t *AppTracking_Output(void)
{
    return &output;
}

/**
 * @brief  初始化循迹环并复位状态。
 */
void AppTracking_Init(void)
{
    Pid_Init(&pid, &trackPidConfig);
    AppTracking_ResetState();
}

/**
 * @brief  复位状态机、PID、各确认计数器和计时器。
 * @note   也把 PID 的输出限幅恢复成循迹用的 ±TRACK_FOLLOW_DIFF_LIMIT_RPM——
 *         PWM 循迹测试会把它改成别的值（见 AppTracking_UpdatePidForError），
 *         切回循迹模式时必须复位。
 * @note   复位后状态是 TRK_STOPPED；要开始跑必须再调 AppTracking_Start()。
 */
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

/**
 * @brief  启动循迹，进入起步直行阶段。
 * @note   先复位再置 ACQUIRE：ACQUIRE 阶段固定直行
 *         TRACK_START_FORWARD_TICKS 个周期（约 1 秒），
 *         让车在开始转向之前先在线路上摆正。
 */
void AppTracking_Start(void)
{
    AppTracking_ResetState();
    trackState = TRK_ACQUIRE;
    output.state = trackState;
}

/**
 * @brief  在线设置循迹环的一个增益。
 * @param  field 要改的增益，见 PidField_t。
 * @param  value 新的增益值。
 * @note   默认 Ki = 0（位置环用 PD）；改 Ki 前先想清楚出弯超调的问题。
 */
void AppTracking_SetGain(PidField_t field, float value)
{
    Pid_SetGain(&pid, field, value);
}

/**
 * @brief  读取循迹环的一个增益。
 * @param  field 要读的增益。
 * @return 当前增益值。
 */
float AppTracking_GetGain(PidField_t field)
{
    return Pid_GetGain(&pid, field);
}

/**
 * @brief  设置「差速 RPM 换算为 PWM 百分比」的增益。
 * @param  gain 换算增益，单位 %/RPM。
 * @note   只影响 PWM 循迹测试（那条路径不经过速度环，直接出 PWM）。
 *         正常循迹模式下差速在目标转速空间生效，用不到这个增益。
 */
void AppTracking_SetSteerRpmToPwmGain(float gain)
{
    steerRpmToPwmGain = gain;
}

/**
 * @brief  读取差速 RPM 转 PWM 的换算增益。
 * @return 当前换算增益，单位 %/RPM。
 */
float AppTracking_GetSteerRpmToPwmGain(void)
{
    return steerRpmToPwmGain;
}

/**
 * @brief  读一次八路传感器并返回归一化横向偏差。
 * @return e_y，单位无量纲，范围约 ±7（正值表示线在车右侧）。
 * @note   供 PWM 循迹测试的「真实红外」路径使用；正常循迹走 sample_line()。
 */
float AppTracking_ComputeWeightedError(void)
{
    IRTrack_Read();
    IRTrack_Compute();
    return ir_data.position;
}

/**
 * @brief  用循迹环对指定误差算一次差速输出。
 * @param  error     误差值，通常来自模拟角度或真实红外。
 * @param  diffLimit 本次的输出限幅（会写进 PID 配置）。
 * @return 差速输出，单位 RPM。
 * @note   供 PWM 循迹测试复用同一个 PID，保证测试与实战转向特性一致。
 * @warning 会把 PID 的输出限幅改成 diffLimit，测试结束后必须调
 *          AppTracking_ResetState() 恢复，否则正常循迹的差速上限被改掉。
 */
float AppTracking_UpdatePidForError(float error, float diffLimit)
{
    Pid_SetLimits(&pid, -diffLimit, diffLimit);
    return Pid_UpdatePosition(&pid, error);
}

/**
 * @brief  直接写入差速输出，旁路循迹环。
 * @param  steerRpm 差速量，单位 RPM。
 * @note   供 PWM 循迹测试使用；同时更新 lastSteer，使丢线保持逻辑拿到的是
 *         最新的测试输出而不是上一次循迹的残留。
 */
void AppTracking_SetPwmSteer(float steerRpm)
{
    lastSteer = steerRpm;
    output.steer_rpm = steerRpm;
}

/**
 * @brief  读取当前差速输出。
 * @return 差速量，单位 RPM。
 */
float AppTracking_GetSteer(void)
{
    return output.steer_rpm;
}

/**
 * @brief  20 ms 循迹任务 —— 采样、判线路、跑状态机、发快照。
 * @note   在 TIM6 中断里执行。非循迹模式直接返回，不占用时间。
 *
 * @note   处理顺序（顺序会影响行为，不要随意调整）：
 * @code
 *   1. 采一次样
 *   2. 未运行 -> 清 PID、回到 STOPPED 并返回
 *   3. 线路质量：无效确认（-> LOST）、全白确认（-> 请求停车）
 *   4. 丢线超时（只在 LOST 里计时）-> 请求停车
 *   5. 状态动作：ACQUIRE 计时 / FOLLOW 算转向 + 计圈
 *   6. 发布快照
 * @endcode
 *
 * @note   各分支提前 return 时会跳过第 6 步，output 保留上一周期的值。
 *         此时要么已经请求停车、要么在未运行状态，快照内容不影响控制。
 */
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
        /* 线路恢复有效：把两个确认计时都清零，避免旧计数残留。 */
        Confirm_Reset(&invalidCounter, TRACK_LINE_INVALID_CONFIRM_TICKS);
        Confirm_Reset(&lostStopCounter, TRACK_LINE_LOST_STOP_TICKS);
        if (trackState == TRK_LOST) {
            Pid_Reset(&pid);
            trackState = TRK_FOLLOW;
        }
    } else if (Confirm_Update(&invalidCounter, 1U) != 0U) {
        /* 连续无效达到阈值才真正判定丢线，滤掉单帧噪声。 */
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
           起点横线正对时八路全压、e_y≈0，天然不计。
           判据用**持续时间**而不是幅度——斜穿横线时单侧压黑也能让 |e_y|
           超过阈值，但横线一闪而过（约 30 ms，1.5 个周期），
           而转弯持续几十到几百毫秒，所以 3 个周期的确认窗口能把两者分开。 */
        if (cornerGap > 0U) {
            /* 刚计过一个弯，本次不参与判定，防止一个弯被计成多个。 */
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

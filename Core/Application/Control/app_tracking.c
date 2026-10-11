/**
 * @file    app_tracking.c
 * @brief   八路数字循迹 — 归一化质心 + 单个位置式 PD + 计时计圈。
 *
 * @note    文件内排序：核心任务放最上方，辅助函数与对外接口放下面。
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

/* ── 全部静态状态：12 个，没有别的 ───────────────────────────────── */

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

/* ── 辅助函数前置声明（定义在文件下方） ───────────────────────────── */
static uint8_t line_is_contiguous(uint8_t pattern);
static void    evaluate_line(void);
static void    sample_line(void);
static void    inject_pattern(uint8_t pattern);
static uint8_t run_line_quality(void);

/* ══════════════ 核心任务 ══════════════ */

/**
 * @brief  20 ms 循迹任务 —— 采样、判线路、跑状态机、发快照。
 * @note   在 TIM6 中断里执行。非循迹模式直接返回，不占用时间。
 *
 * @note   处理顺序（顺序会影响行为，不要随意调整）：
 * @code
 *   1. 采一次样
 *   2. 未运行 -> 清 PID、回到 STOPPED 并返回
 *   3. 线路质量与停车规则：run_line_quality()
 *      （无效确认 -> LOST、全白确认 -> 请求停车、丢线超时 -> 请求停车）
 *   4. 状态动作：ACQUIRE 计时 / FOLLOW 算转向 + 计圈
 *   5. 发布快照
 * @endcode
 *
 * @note   第 1 步与第 3 步被 PWM 循迹测试借用（见 AppTracking_TestStep20ms）：
 *         测试把图案注入 ir_data 再走同一套判定与停车规则，只是不读 GPIO、
 *         不计圈。改这里的行为会同时改到那个测试模式。
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

    /* --- 线路质量与停车规则：无效确认 / 全白确认 / 丢线超时 --- */
    if (run_line_quality() != 0U) {
        return;
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
    output.pattern = sample.pattern;
}

/* ══════════════ 辅助函数 ══════════════ */

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
 * @brief  把 ir_data 里现有的读数判定成一份线路样本。
 * @note   只读 ir_data 的 pattern / black_count / position，不碰 GPIO，
 *         也不管读数是怎么来的——实车路径与 PWM 循迹测试共用这一份判定。
 * @note   e_y 直接用驱动层算好的归一化质心，不在这里重算。
 */
static void evaluate_line(void)
{
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
 * @brief  读八路传感器并完成线路判定 —— 实车路径。
 * @note   调用驱动层 IRTrack_Read()/IRTrack_Compute() 刷新 ir_data，
 *         再交给 evaluate_line() 判定。
 */
static void sample_line(void)
{
    IRTrack_Read();
    IRTrack_Compute();

    evaluate_line();
}

/**
 * @brief  把一段图案当成传感器读数写进驱动层（仅供 PWM 循迹测试）。
 * @param  pattern 要模拟的黑线掩码，bit0..7 对应 IR1..8，压黑置 1。
 * @note   只写 ir_data.raw[] 再调 IRTrack_Compute()，走的是与实车**逐字节相同**
 *         的计算链——掩码、压黑数、归一化质心全由驱动层算，本模块不重算，
 *         所以误差的量纲天然与实车一致。
 * @note   raw[] 是反相表示（0 = 压黑），这里逐位取反填入。
 * @warning 注入之后**不能**再调 IRTrack_Read()，否则注入的读数立刻被 GPIO 覆盖。
 *          实车路径的 sample_line() 里就有那个调用，所以两条路径不能混着走。
 */
static void inject_pattern(uint8_t pattern)
{
    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] = ((pattern & (uint8_t)(1U << i)) != 0U) ? 0U : 1U;
    }

    IRTrack_Compute();
}

/**
 * @brief  线路质量判定与停车规则。
 * @return 1 表示本周期已经请求停车；0 表示继续跑。
 * @note   实车循迹与 PWM 循迹测试共用：无效确认达到阈值转 LOST，恢复有效则
 *         回到 FOLLOW 并复位 PID；全白连续达阈值请求停车（视为到终点线）；
 *         丢线超时只在 LOST 里计时，到阈值请求停车。
 * @note   不含计圈——过弯判定只属于实车路径。
 */
static uint8_t run_line_quality(void)
{
    /* --- 无效确认 --- */
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

    /* --- 全白确认：视为到终点线 --- */
    if (Confirm_Update(&allWhiteCounter, sample.all_white) != 0U) {
        AppState_ReportStop(stopReasonAllWhite);
        return 1U;
    }

    /* --- 丢线持续超时后停车：只有在 LOST 状态里才计时 --- */
    if (trackState == TRK_LOST) {
        if (Confirm_Update(&lostStopCounter, 1U) != 0U) {
            AppState_ReportStop(stopReasonCornerTimeout);
            return 1U;
        }
    }

    return 0U;
}

/* ══════════════ 对外接口 ══════════════ */

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
 * @note   也把 PID 的输出限幅重新写成 ±TRACK_FOLLOW_DIFF_LIMIT_RPM。Pid_Init()
 *         只在初始化时读一次配置，限幅一旦被谁改过就得靠这里恢复——目前已经
 *         没有任何路径在运行时改它（PWM 循迹测试用的是同一个限幅），这一步是
 *         兜底，别删。
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
    output.pattern = 0U;
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
 * @brief  读取循迹环的一个增益。
 * @param  field 要读的增益。
 * @return 当前增益值。
 * @note   只读。循迹环增益不再支持在线修改——在 app_config.h 里改后重新编译。
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
 * @brief  PWM 循迹测试的一步 —— 把图案当传感器读数，走与实车相同的判定与输出。
 * @param  pattern 要模拟的黑线掩码，bit0..7 对应 IR1..8，压黑置 1。
 * @return 1 表示本周期没有算差速（未运行，或已请求停车）；0 表示正常算过。
 *
 * @note   和实车路径共用 sample / trackState / 三个确认计数 / lastSteer，
 *         所以「丢线保持差速」「全白停车」「丢线超时停车」的行为与实车逐字一致。
 *         差别只有两点：读数来自注入的图案而不是 GPIO；不做过弯判定与计圈。
 *
 * @note   差速用的就是循迹 PID 自己的限幅（±TRACK_FOLLOW_DIFF_LIMIT_RPM），
 *         与实车完全相同——**不要**为测试另设限幅，早先那个做法会原地改写
 *         共享 PID 的配置，一旦某次模式切换没走到复位，真实循迹的差速上限
 *         就被永久改掉了。
 *
 * @note   未运行时只把状态机收回 STOPPED 并发布快照，不注入图案、也不推进
 *         确认计数——停车期间不该继续判线路。这一条与实车路径的 !running
 *         分支对齐，否则自动停车后 output.state 会一直停在 TRK_LOST。
 *
 * @note   测试没有 ACQUIRE 起步直行阶段（那是实车起步用的），只要没丢线就在 FOLLOW。
 *
 * @note   在 TIM6 的 20 ms 时隙里调用，与 AppTracking_Task20ms() 互斥
 *         （两者各自只在自己那个模式下被调度）。
 */
uint8_t AppTracking_TestStep20ms(uint8_t pattern)
{
    if (!AppState()->running) {
        Pid_Reset(&pid);
        trackState = TRK_STOPPED;
        output.state = trackState;
        return 1U;
    }

    inject_pattern(pattern);
    evaluate_line();

    if (run_line_quality() != 0U) {
        return 1U;
    }

    /* 测试没有起步直行阶段：没丢线就一直算。 */
    if (trackState != TRK_LOST) {
        trackState = TRK_FOLLOW;
    }

    if ((trackState == TRK_FOLLOW) && (sample.valid != 0U)) {
        AppTracking_SetPwmSteer(Pid_UpdatePosition(&pid, sample.e_y));
    }
    /* TRK_LOST：什么都不做，lastSteer 保持丢线前的值。 */

    /* 发布快照。PWM 循迹测试是开环的，不吃目标转速，所以 base_rpm 恒 0；
       e_y 与 pattern 必须发出去，否则 OLED 的 ERR 和遥测的 trackError 会一直
       显示上一次循迹模式留下的陈旧值。 */
    output.base_rpm = 0.0f;
    output.e_y = sample.e_y;
    output.state = trackState;
    output.pattern = sample.pattern;

    return 0U;
}

/**
 * @brief  写入本周期算出的差速输出。
 * @param  steerRpm 差速量，单位 RPM。
 * @note   同时更新 lastSteer，使丢线保持逻辑拿到的是最新输出而不是上一次循迹的残留。
 *         正常循迹在 AppTracking_Task20ms() 里直接写 lastSteer，只有 PWM 循迹测试
 *         走这条；两条路径写的是同一个 lastSteer。
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

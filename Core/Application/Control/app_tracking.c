/* 八路数字循迹：归一化质心 + 单个位置式 PD + 计时计圈。误差取归一化质心 e_y = ir_data.position
   （已除压黑路数，量纲恒定 ±7），不直接驱电机，只发布 base_rpm 与 steer_rpm。20 ms 任务在 TIM6 中断里。 */

#include "app_tracking.h"
#include "app_config.h"
#include "app_scheduler.h"
#include "app_state.h"
#include "irtrack.h"

/* 一次采样的线路判定结果。只在本文件内使用。 */
typedef struct {
    uint8_t pattern;      /* bit0..7 对应 IR1..8，压黑置 1。 */
    uint8_t black_count;  /* 压黑通道数，0~8。 */
    uint8_t contiguous;   /* 是否为单一连续黑线段。 */
    uint8_t valid;        /* 压黑 1~TRACK_LINE_VALID_MAX_BLACK 路且连续。 */
    uint8_t all_white;    /* 八路全白。 */
    float   e_y;          /* 归一化横向偏差，直接取 ir_data.position，范围约 ±7。 */
} LineSample_t;

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

/* 循迹环配置：位置式 PD，输出限幅 ±TRACK_FOLLOW_DIFF_LIMIT_RPM（差速上限）。
   zero_target_hold 与 out_offset 必须 0——它们是速度环用的，套上会把小偏差吃掉。 */
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

static uint8_t line_is_contiguous(uint8_t pattern);
static void    evaluate_line(void);
static void    sample_line(void);
static void    inject_pattern(uint8_t pattern);
static uint8_t run_line_quality(void);

/* 20 ms 循迹任务：采样→判线路→跑状态机→发快照。TIM6 中断里，非循迹模式直接返回，顺序勿调。
   采样与线路质量两步被 PWM 循迹测试借用，改这里会同时改到测试模式。 */
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

    if (run_line_quality() != 0U) {
        return;
    }

    if (trackState == TRK_ACQUIRE) {
        if (++startupTicks >= TRACK_START_FORWARD_TICKS) {
            trackState = TRK_FOLLOW;
        }
    } else if ((trackState == TRK_FOLLOW) && (sample.valid != 0U)) {

        /* 唯一的转向计算。 */
        lastSteer = Pid_UpdatePosition(&pid, sample.e_y);

        /* 唯一的计圈逻辑：e_y 偏到一侧即过弯（起点横线八路全压 e_y≈0 天然不计）。
           判据用持续时间不用幅度——斜穿横线单侧压黑也能超阈值，但横线一闪而过（约 30ms），
           转弯持续几十到几百 ms，3 周期确认窗口能把两者分开。 */
        if (cornerGap > 0U) {
            /* 刚计过一个弯，本次不判定，防止一个弯被计成多个。 */
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

    output.base_rpm = AppState()->gear_rpm;
    output.steer_rpm = lastSteer;
    output.e_y = sample.e_y;
    output.state = trackState;
    output.pattern = sample.pattern;
}

/* 判断压黑图案是否连成一段。数字量传感器易出「两头黑中间白」断续图案（斜穿横线/岔路口），
   必须判无效，否则质心会算出「居中」的假象。 */
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

/* 把 ir_data 现有读数判定成线路样本。只读 pattern/black_count/position，不碰 GPIO，
   读数怎么来的不管——实车与 PWM 循迹测试共用。e_y 直接用驱动层算好的归一化质心。 */
static void evaluate_line(void)
{
    sample.pattern = ir_data.pattern;
    sample.black_count = ir_data.black_count;
    sample.e_y = ir_data.position;
    sample.contiguous = line_is_contiguous(ir_data.pattern);

    /* 有效 = 压黑 1~7 路且连续。压满 8 路视为横线，不作为循迹线路。 */
    sample.valid = (uint8_t)((ir_data.black_count >= 1U)
                             && (ir_data.black_count <= TRACK_LINE_VALID_MAX_BLACK)
                             && (sample.contiguous != 0U));
    sample.all_white = (ir_data.black_count == 0U) ? 1U : 0U;
}

/* 读八路传感器并完成线路判定（实车路径）。 */
static void sample_line(void)
{
    IRTrack_Read();
    IRTrack_Compute();

    evaluate_line();
}

/* 把图案当传感器读数写进驱动层（仅 PWM 循迹测试）。走与实车相同的计算链，误差量纲一致。
   raw[] 是反相表示（0 = 压黑），这里逐位取反填入。
   注入后不能再调 IRTrack_Read()，否则读数立刻被 GPIO 覆盖——两条路径不能混走。 */
static void inject_pattern(uint8_t pattern)
{
    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] = ((pattern & (uint8_t)(1U << i)) != 0U) ? 0U : 1U;
    }

    IRTrack_Compute();
}

/* 线路质量判定与停车规则。实车与 PWM 循迹测试共用：无效确认转 LOST，恢复有效回 FOLLOW 并复位 PID；
   全白连续达阈值请求停车（视为终点线）；丢线超时只在 LOST 里计时。不含计圈（只属实车）。 */
static uint8_t run_line_quality(void)
{
    if (sample.valid != 0U) {
        /* 线路恢复有效：两个确认计时清零，避免旧计数残留。 */
        Confirm_Reset(&invalidCounter, TRACK_LINE_INVALID_CONFIRM_TICKS);
        Confirm_Reset(&lostStopCounter, TRACK_LINE_LOST_STOP_TICKS);
        if (trackState == TRK_LOST) {
            Pid_Reset(&pid);
            trackState = TRK_FOLLOW;
        }
    } else if (Confirm_Update(&invalidCounter, 1U) != 0U) {
        /* 连续无效达到阈值才判定丢线，滤掉单帧噪声。 */
        trackState = TRK_LOST;
    }

    if (Confirm_Update(&allWhiteCounter, sample.all_white) != 0U) {
        AppState_ReportStop(stopReasonAllWhite);
        return 1U;
    }

    if (trackState == TRK_LOST) {
        if (Confirm_Update(&lostStopCounter, 1U) != 0U) {
            AppState_ReportStop(stopReasonCornerTimeout);
            return 1U;
        }
    }

    return 0U;
}

/* 取只读循迹快照。 */
const TrackOutput_t *AppTracking_Output(void)
{
    return &output;
}

/* 初始化循迹环并复位状态。 */
void AppTracking_Init(void)
{
    Pid_Init(&pid, &trackPidConfig);
    AppTracking_ResetState();
}

/* 复位状态机、PID、确认计数与计时器。也重写 PID 限幅为 ±TRACK_FOLLOW_DIFF_LIMIT_RPM——Pid_Init
   只在初始化读一次配置，限幅被改过就得靠这里恢复（目前无路径在运行时改它，这步是兜底，别删）。
   复位后为 TRK_STOPPED，要跑须再调 AppTracking_Start。 */
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

/* 启动循迹：先复位再进 ACQUIRE 起步直行（约 1 秒），让车在转向之前先摆正。 */
void AppTracking_Start(void)
{
    AppTracking_ResetState();
    trackState = TRK_ACQUIRE;
    output.state = trackState;
}

/* 读循迹环增益。只读，无在线改增益接口。 */
float AppTracking_GetGain(PidField_t field)
{
    return Pid_GetGain(&pid, field);
}

/* 设「差速 RPM 换算 PWM 百分比」增益（%/RPM）。只影响 PWM 循迹测试（旁路速度环直接出 PWM）。 */
void AppTracking_SetSteerRpmToPwmGain(float gain)
{
    steerRpmToPwmGain = gain;
}

/* 读上面的换算增益，单位 %/RPM。 */
float AppTracking_GetSteerRpmToPwmGain(void)
{
    return steerRpmToPwmGain;
}

/* PWM 循迹测试一步：把图案当传感器读数，走与实车相同的判定与输出。TIM6 20 ms 时隙，与 Task20ms 互斥。
   差速用循迹 PID 自己的限幅（±TRACK_FOLLOW_DIFF_LIMIT_RPM）——不要为测试另设限幅，早先做法会原地
   改写共享 PID 配置，某次模式切换没复位就把真实循迹的差速上限永久改掉了。 */
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

    /* 发布快照。测试是开环的，不吃目标转速，base_rpm 恒 0；e_y 与 pattern 必须发出去，
       否则 OLED 的 ERR 和遥测 trackError 会一直显示上一次循迹模式留下的陈旧值。 */
    output.base_rpm = 0.0f;
    output.e_y = sample.e_y;
    output.state = trackState;
    output.pattern = sample.pattern;

    return 0U;
}

/* 写本周期差速输出，同时更新 lastSteer（丢线保持取最新值，而非上一次循迹残留）。 */
void AppTracking_SetPwmSteer(float steerRpm)
{
    lastSteer = steerRpm;
    output.steer_rpm = steerRpm;
}

/* 读当前差速输出，单位 RPM。 */
float AppTracking_GetSteer(void)
{
    return output.steer_rpm;
}

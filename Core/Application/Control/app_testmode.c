/**
 * @file    app_testmode.c
 * @brief   两种开环测试模式的记账 — PWM 速度测试与 PWM 循迹测试。
 *
 * @note    文件内排序：两个核心任务放最上方，辅助函数与对外接口放下面。
 *
 * @note    本模块只维护**状态**：相位、计时、模拟图案、基础占空比。
 *         「谁来设目标、谁来复位 PID、谁来回传遥测」全部由 50 ms 控制任务
 *         （app_speedloop.c）决定——这样 app_testmode 与 app_speedloop
 *         之间保持单向依赖，不会互相调用成环。
 *
 * @note    两种模式的输出路径不同，务必分清：
 * @code
 *   PWM 速度测试：左右轮同向同速，走**速度环**，目标由 AppState_SetWheelTargets()
 *                 下发，按时长翻转方向；也可由串口 l/r/t 命令切成手动。
 *   PWM 循迹测试：固定基础占空比加减差速，**旁路速度环**直接出 PWM，
 *                 误差来自注入的八路图案（见 AppTracking_TestStep20ms()，
 *                 量纲与实车共用同一条计算链）。
 *                 它的参数（图案、基础占空比）**只能用按键改**。
 * @endcode
 *
 * @note    执行上下文：
 *         - AppTestMode_TickPwmSpeed() 在 50 ms 控制任务里（中断上下文）；
 *         - AppTestMode_Task20ms() 在 20 ms 循迹时隙里（中断上下文）；
 *         - 其余设置类接口由主循环的命令行或按键路径调用。
 */

#include "app_testmode.h"
#include "app_config.h"
#include "app_state.h"
#include "app_tracking.h"

/* PWM 循迹测试的模拟图案表。
   按 IR1→IR8（左→右）书写，1 = 压黑；左起第一个字符就是 bit0，所以十六进制值
   与图案一一对应。KEY2 按这个顺序轮换，每 PWM_TRACK_TEST_PHASE_MS 毫秒把当前
   图案镜像一次（左右对调）。表对镜像封闭，镜像后的图案仍在表内。
   注释里的 e_y 是按驱动层质心公式 ((Σ权重 × 28) ÷ 压黑数) ÷ 100 算出的期望值。 */
static const uint8_t patternTable[] = {
    0x03U,  /* 1100 0000  直线偏移·大（左）     e_y -5.88 */
    0x0FU,  /* 1111 0000  左转弯口（4 路压黑）  e_y -3.78 */
    0x06U,  /* 0110 0000  直线偏移·中（左）     e_y -3.64 */
    0x0CU,  /* 0011 0000  直线偏移·小（左）     e_y -1.68 */
    0x18U,  /* 0001 1000  居中（IR4+IR5）       e_y  0.00 */
    0x30U,  /* 0000 1100  直线偏移·小（右）     e_y +1.68 */
    0x60U,  /* 0000 0110  直线偏移·中（右）     e_y +3.64 */
    0xF0U,  /* 0000 1111  右转弯口（4 路压黑）  e_y +3.78 */
    0xC0U,  /* 0000 0011  直线偏移·大（右）     e_y +5.88 */
    0xFFU,  /* 1111 1111  全黑 = 丢线          无效（压黑 8 路 > 上限 7） */
    0x00U,  /* 0000 0000  全白 = 到终点线      无效（all_white） */
    0x81U   /* 1000 0001  岔路口（中间断开）   无效（不连续） */
};
#define PATTERN_TABLE_SIZE (sizeof(patternTable) / sizeof(patternTable[0]))

/* 开机默认图案：居中（e_y = 0），轮子直行，不会有意外动作。
   注意它是自对称的，所以开机状态下 5 秒自动镜像看不出变化——按一次 KEY2
   选到任何非对称图案，左右扫就开始工作。 */
#define PATTERN_DEFAULT 0x18U

/* ── PWM 速度测试状态 ─────────────────────────────────────────────── */
static PwmTestPhase_t speedPhase = PWM_TEST_PHASE_FORWARD;
/* 当前相位已持续的时间，单位 ms；到 PWM_SPEED_TEST_PHASE_MS 就换向。 */
static uint32_t       speedElapsedMs = 0U;
/* 自动换向开关。1 = 按时间自动来回换向；0 = 由串口 l/r/t 命令手动控制。
   stop 与模式切换经 AppTestMode_ResetPwmSpeed() 恢复为 1。 */
static uint8_t        autoPhaseEnabled = 1U;

/* ── PWM 循迹测试状态 ─────────────────────────────────────────────── */
static PwmTestPhase_t trackPhase = PWM_TEST_PHASE_LEFT;
/* 当前偏转方向已持续的时间，单位 ms。 */
static uint32_t       trackElapsedMs = 0U;
/* 当前选中的图案（patternTable 里的一项）。实际注入的是它的相位镜像，
   见 effectivePattern()。 */
static uint8_t        trackPattern = PATTERN_DEFAULT;
/* 基础占空比百分比，0~100，差速在它上面加减。 */
static int16_t        trackBasePwm = PWM_TRACK_BASE_DEFAULT_PERCENT;

/* ── 辅助函数前置声明（定义在文件下方） ───────────────────────────── */
static int16_t clampBasePwm(int16_t value);
static uint8_t mirror8(uint8_t pattern);
static uint8_t effectivePattern(void);
static void    advanceTrackPhase(void);

/* ══════════════ 核心任务 ══════════════ */

/**
 * @brief  推进 PWM 速度测试计时，到点则翻转相位。
 * @return 1 表示本次调用发生了相位翻转；0 表示还没到，或自动换向已关闭。
 * @note   在 50 ms 控制任务里调用。**返回 1 时调用方必须重新建立目标转速
 *         并复位速度环**——否则 PID 会带着上个方向的积分继续跑。
 * @note   累计用 APP_CONTROL_PERIOD_MS 而不是实测时间，所以本函数
 *         必须跟着固定周期的控制任务跑。
 * @note   自动换向关闭时直接返回 0，既不推进计时也不动目标，
 *         手动模式下的目标因此不会被覆盖。
 */
uint8_t AppTestMode_TickPwmSpeed(void)
{
    if (autoPhaseEnabled == 0U) {
        return 0U;
    }

    speedElapsedMs += APP_CONTROL_PERIOD_MS;
    if (speedElapsedMs < PWM_SPEED_TEST_PHASE_MS) {
        return 0U;
    }

    speedElapsedMs = 0U;
    speedPhase = (speedPhase == PWM_TEST_PHASE_FORWARD)
                 ? PWM_TEST_PHASE_REVERSE
                 : PWM_TEST_PHASE_FORWARD;
    return 1U;
}

/**
 * @brief  20 ms PWM 循迹测试任务。
 * @note   在 TIM6 的 20 ms 时隙里执行（modePwmTrack 时替代循迹任务）。
 *         未运行时不推进相位（停车期间相位不该继续走），但仍走一步循迹测试——
 *         那一步负责把状态机收回 STOPPED 并发布快照。
 * @note   模拟输入是**八路图案**：把当前图案注入驱动层当作传感器读数，再走与实车
 *         逐字相同的线路判定、丢线/全白规则和循迹 PID（见 AppTracking_TestStep20ms）。
 *         所以误差的量纲天然与实车一致，不需要任何手工标定的映射系数。
 * @note   算出的差速由 tracking 模块写进快照，50 ms 控制任务读出来叠到基础占空比上
 *         （见 app_speedloop.c）。停车（全白到终点线 / 丢线超时）也由它自己上报，
 *         这里不需要额外动作。
 */
void AppTestMode_Task20ms(void)
{
    /* 停车期间不推进相位——但**仍要**走一步循迹测试：未运行时它负责把状态机
       收回 STOPPED 并发布快照（与实车路径的不变量一致），代价只是两次比较。 */
    if (AppState()->running) {
        advanceTrackPhase();
    }

    (void)AppTracking_TestStep20ms(effectivePattern());
}

/* ══════════════ 辅助函数 ══════════════ */

/**
 * @brief  把基础占空比夹到合法范围。
 * @param  value 待夹取的值。
 * @return 0 ~ PWM_TEST_MAX_PERCENT 之间的值。
 * @note   下限是 0 而不是 -100：PWM 循迹测试用于观察寻线，不需要反转基础速度。
 */
static int16_t clampBasePwm(int16_t value)
{
    if (value > PWM_TEST_MAX_PERCENT) return PWM_TEST_MAX_PERCENT;
    if (value < 0) return 0;
    return value;
}

/**
 * @brief  把八位图案左右镜像。
 * @param  pattern 黑线掩码，bit0..7 对应 IR1..8。
 * @return 镜像后的掩码：原来的 IR1 变 IR8、IR2 变 IR7，以此类推。
 * @note   自动换向就靠它——镜像一下等于把线从左边挪到右边，
 *         不用再维护第二张表，也不会破坏「表对镜像封闭」这个性质。
 */
static uint8_t mirror8(uint8_t pattern)
{
    uint8_t mirrored = 0U;

    for (uint8_t i = 0U; i < PATTERN_BITS; i++) {
        if ((pattern & (uint8_t)(1U << i)) != 0U) {
            mirrored |= (uint8_t)(1U << (uint8_t)(PATTERN_BITS - 1U - i));
        }
    }

    return mirrored;
}

/**
 * @brief  本周期实际要注入的图案。
 * @return 相位为 LEFT 时是选中的图案；为 RIGHT 时是它的镜像。
 * @note   自动换向只镜像、不改 KEY2 的选择，所以选定项不会被 5 秒计时冲掉；
 *         两者叠加的效果就是「选中一个图案，然后左右自动对调」。
 */
static uint8_t effectivePattern(void)
{
    return (trackPhase == PWM_TEST_PHASE_LEFT) ? trackPattern
                                               : mirror8(trackPattern);
}

/**
 * @brief  推进 PWM 循迹测试计时，到点则翻转偏转相位。
 * @note   在 20 ms 任务里调用，所以累计用的是 APP_TRACK_PERIOD_MS。
 * @note   相位只决定「注入的是选中图案还是它的镜像」（见 effectivePattern），
 *         不动 KEY2 选中的那一项。
 * @note   换向时复位循迹状态机：误差符号突变时旧的误差历史会拖着 PID 反着走。
 */
static void advanceTrackPhase(void)
{
    trackElapsedMs += APP_TRACK_PERIOD_MS;
    if (trackElapsedMs < PWM_TRACK_TEST_PHASE_MS) {
        return;
    }

    trackElapsedMs = 0U;
    trackPhase = (trackPhase == PWM_TEST_PHASE_LEFT)
                 ? PWM_TEST_PHASE_RIGHT
                 : PWM_TEST_PHASE_LEFT;
    AppTracking_ResetState();
}

/* ══════════════ 对外接口 ══════════════ */

/**
 * @brief  初始化测试模式状态。
 * @note   PWM 循迹测试固定用模拟图案（见 AppTestMode_Task20ms），
 *         便于架空车轮验证；真实红外的测试走 modeTrack。
 */
void AppTestMode_Init(void)
{
    AppTestMode_ResetPwmSpeed();
    AppTestMode_ResetPwmTrack();
    trackPattern = PATTERN_DEFAULT;
    trackBasePwm = PWM_TRACK_BASE_DEFAULT_PERCENT;
}

/**
 * @brief  复位 PWM 速度测试：相位回到正转，计时清零，**自动换向恢复**。
 * @note   恢复自动换向是刻意放在这里的：stop 和模式切换都会调用本函数，
 *         于是手动转速模式不需要额外的退出处理，自动就恢复了。
 */
void AppTestMode_ResetPwmSpeed(void)
{
    speedPhase = PWM_TEST_PHASE_FORWARD;
    speedElapsedMs = 0U;
    autoPhaseEnabled = 1U;
}

/**
 * @brief  开关 PWM 速度测试的自动换向。
 * @param  enabled 非 0 恢复自动换向，0 交出手动控制权。
 * @note   只需关、不必手动开——AppTestMode_ResetPwmSpeed() 会恢复。
 */
void AppTestMode_SetAutoPhase(uint8_t enabled)
{
    autoPhaseEnabled = (enabled != 0U) ? 1U : 0U;
}

/**
 * @brief  取当前相位对应的方向符号。
 * @return +1 正转，-1 反转。
 * @note   调用方把它乘到档位速度上得到带符号的目标转速。
 */
int8_t AppTestMode_GetPwmSpeedSign(void)
{
    return (speedPhase == PWM_TEST_PHASE_REVERSE) ? -1 : 1;
}

/**
 * @brief  取 PWM 速度测试的当前相位。
 * @return PWM_TEST_PHASE_FORWARD 或 PWM_TEST_PHASE_REVERSE。
 */
PwmTestPhase_t AppTestMode_GetSpeedPhase(void)
{
    return speedPhase;
}

/**
 * @brief  复位 PWM 循迹测试：相位回到左偏，计时清零。
 * @note   不含 appTrackingResetState()——那是调用方的事，
 *         因为循迹状态机的复位时机取决于模式切换的上下文。
 */
void AppTestMode_ResetPwmTrack(void)
{
    trackPhase = PWM_TEST_PHASE_LEFT;
    trackElapsedMs = 0U;
}

/**
 * @brief  设置模拟图案。
 * @param  pattern 目标图案，必须精确等于 patternTable 里的某一项。
 * @note   不在表内的值被**静默忽略**，且**什么都不做**——图案、循迹状态机和
 *         测试相位全都保持原样。被拒绝的输入不该顺手把 PID 和相位清零。
 * @note   命中时复位循迹状态机和测试相位，避免上一个图案残留的误差历史。
 * @note   当前没有调用者——图案由 KEY2 经 AppTestMode_NextPattern() 轮换。
 *         保留它是为了以后要「直接指定某个图案」时有个现成入口。
 */
void AppTestMode_SetPattern(uint8_t pattern)
{
    uint8_t i;

    for (i = 0U; i < PATTERN_TABLE_SIZE; i++) {
        if (patternTable[i] == pattern) {
            trackPattern = pattern;
            AppTracking_ResetState();
            AppTestMode_ResetPwmTrack();
            break;
        }
    }
}

/**
 * @brief  在图案表里循环切到下一项。
 * @note   若当前图案不在表内（异常情况），下标会停在 0，切到表首。
 */
void AppTestMode_NextPattern(void)
{
    uint8_t index = 0U;

    for (uint8_t i = 0U; i < PATTERN_TABLE_SIZE; i++) {
        if (patternTable[i] == trackPattern) {
            index = (uint8_t)((i + 1U) % PATTERN_TABLE_SIZE);
            break;
        }
    }

    trackPattern = patternTable[index];

    AppTracking_ResetState();
    AppTestMode_ResetPwmTrack();
}

/**
 * @brief  取当前实际注入的图案。
 * @return 已按相位镜像过的图案，bit0..7 对应 IR1..8，压黑置 1。
 * @note   取的是**有效图案**而不是 KEY2 选中的那一项，显示才和实际喂给
 *         循迹模块的数据一致。
 */
uint8_t AppTestMode_GetPattern(void)
{
    return effectivePattern();
}

/**
 * @brief  调整 PWM 循迹的基础占空比。
 * @param  step 增量百分比，正加负减；结果会被夹到 0~100。
 */
void AppTestMode_StepBasePwm(int16_t step)
{
    trackBasePwm = clampBasePwm((int16_t)(trackBasePwm + step));
}

/**
 * @brief  取当前基础占空比。
 * @return 0~100 之间的百分比。
 */
int16_t AppTestMode_GetBasePwm(void)
{
    return trackBasePwm;
}

/**
 * @brief  取 PWM 循迹测试的当前相位。
 * @return PWM_TEST_PHASE_LEFT 或 PWM_TEST_PHASE_RIGHT。
 */
PwmTestPhase_t AppTestMode_GetTrackPhase(void)
{
    return trackPhase;
}

/**
 * @brief  取相位名。
 * @param  phase 相位值。
 * @return 静态字符串；未知相位返回 "--"。
 * @note   供 OLED 与串口命令行共用，避免两处各维护一份字符串。
 * @note   同一个枚举里既有速度测试的 FORWARD/REVERSE，也有循迹测试的
 *         LEFT/RIGHT，所以这个函数对两者都适用。
 */
const char *AppPhaseName(PwmTestPhase_t phase)
{
    switch (phase) {
    case PWM_TEST_PHASE_FORWARD: return "FORWARD";
    case PWM_TEST_PHASE_REVERSE: return "REVERSE";
    case PWM_TEST_PHASE_LEFT:    return "LEFT";
    case PWM_TEST_PHASE_RIGHT:   return "RIGHT";
    default:                     return "--";
    }
}

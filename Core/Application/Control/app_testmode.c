/* 两种开环测试模式的记账：PWM 速度测试与 PWM 循迹测试。本模块只管状态（相位/计时/图案/
   基础占空比）；设目标、复位 PID、回传遥测由 50 ms 控制任务决定，保持与 app_speedloop 单向依赖。 */

#include "app_testmode.h"
#include "app_config.h"
#include "app_state.h"
#include "app_tracking.h"

/* PWM 循迹测试的模拟图案表。按 IR1→IR8（左→右）书写，1 = 压黑；bit0 对应最左一路，
   所以十六进制值与图案一一对应。KEY2 按此顺序轮换，每 PWM_TRACK_TEST_PHASE_MS 镜像一次
   （左右对调）。表对镜像封闭。注释里的 e_y 是驱动层质心公式算出的期望值。 */
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

/* 开机默认图案：居中（e_y = 0），自对称——5 秒自动镜像看不出变化；选到非对称图案后才开始左右扫。 */
#define PATTERN_DEFAULT 0x18U

/* PWM 速度测试状态 */
static PwmTestPhase_t speedPhase = PWM_TEST_PHASE_FORWARD;
static uint32_t       speedElapsedMs = 0U; /* 当前相位已持续的时间，单位 ms；到阈值就换向。 */
static uint8_t        autoPhaseEnabled = 1U; /* 1 = 按时长自动换向；0 = 由串口 l/r/t 手动控制。 */

/* PWM 循迹测试状态 */
static PwmTestPhase_t trackPhase = PWM_TEST_PHASE_LEFT;
static uint32_t       trackElapsedMs = 0U; /* 当前偏转方向已持续的时间，单位 ms。 */
static uint8_t        trackPattern = PATTERN_DEFAULT; /* 当前选中图案；实际注入的是它的相位镜像。 */
static int16_t        trackBasePwm = PWM_TRACK_BASE_DEFAULT_PERCENT; /* 基础占空比百分比 0~100，差速在它上面加减。 */

static int16_t clampBasePwm(int16_t value);
static uint8_t mirror8(uint8_t pattern);
static uint8_t effectivePattern(void);
static void    advanceTrackPhase(void);

/* 推进 PWM 速度测试计时，到点翻转相位。返回 1 表示本周期翻转（调用方须重设目标并复位速度环）。
   50 ms 任务里调；累计用固定周期，自动换向关闭时直接返回 0。 */
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

/* 20 ms PWM 循迹测试任务（TIM6 的 20 ms 时隙，替代循迹任务）。模拟输入是八路图案，
   走与实车相同的判定、丢线/全白规则和循迹 PID，误差量纲与实车一致。 */
void AppTestMode_Task20ms(void)
{
    /* 停车期间不推进相位，但仍走一步循迹测试：未运行时它负责收回 STOPPED 并发布快照。 */
    if (AppState()->running) {
        advanceTrackPhase();
    }

    (void)AppTracking_TestStep20ms(effectivePattern());
}

/* 把基础占空比夹到 0~PWM_TEST_MAX_PERCENT。下限是 0 不是 -100：测试无需反转基础速度。 */
static int16_t clampBasePwm(int16_t value)
{
    if (value > PWM_TEST_MAX_PERCENT) return PWM_TEST_MAX_PERCENT;
    if (value < 0) return 0;
    return value;
}

/* 把八位图案左右镜像。bit0..7 对应 IR1..8。镜像 = 把线从左边挪到右边，免维护第二张表。 */
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

/* 本周期实际注入的图案：LEFT 为选中项，RIGHT 为镜像。镜像不改 KEY2 选择，不会被计时冲掉。 */
static uint8_t effectivePattern(void)
{
    return (trackPhase == PWM_TEST_PHASE_LEFT) ? trackPattern
                                               : mirror8(trackPattern);
}

/* 推进 PWM 循迹测试计时，到点翻转相位并复位循迹状态机（误差符号突变会拖着 PID 反走）。 */
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

/* 初始化测试模式状态。 */
void AppTestMode_Init(void)
{
    AppTestMode_ResetPwmSpeed();
    AppTestMode_ResetPwmTrack();
    trackPattern = PATTERN_DEFAULT;
    trackBasePwm = PWM_TRACK_BASE_DEFAULT_PERCENT;
}

/* 复位 PWM 速度测试：相位回正转、计时清零、恢复自动换向（stop 和切模式都走这里）。 */
void AppTestMode_ResetPwmSpeed(void)
{
    speedPhase = PWM_TEST_PHASE_FORWARD;
    speedElapsedMs = 0U;
    autoPhaseEnabled = 1U;
}

/* 开关自动换向。只需关，不必手动开——AppTestMode_ResetPwmSpeed() 会恢复。 */
void AppTestMode_SetAutoPhase(uint8_t enabled)
{
    autoPhaseEnabled = (enabled != 0U) ? 1U : 0U;
}

/* 当前相位方向符号：+1 正转，-1 反转。 */
int8_t AppTestMode_GetPwmSpeedSign(void)
{
    return (speedPhase == PWM_TEST_PHASE_REVERSE) ? -1 : 1;
}

/* 当前速度测试相位。 */
PwmTestPhase_t AppTestMode_GetSpeedPhase(void)
{
    return speedPhase;
}

/* 复位 PWM 循迹测试：相位回左偏、计时清零。不含 AppTracking_ResetState（调用方的事）。 */
void AppTestMode_ResetPwmTrack(void)
{
    trackPhase = PWM_TEST_PHASE_LEFT;
    trackElapsedMs = 0U;
}

/* 设置模拟图案，必须精确等于表内某项；不在表内静默忽略（不清 PID/相位）。命中时复位循迹状态机。
   当前无调用者——图案由 KEY2 经 NextPattern 轮换，保留作「直接指定图案」的入口。 */
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

/* 在图案表里循环切到下一项；当前项不在表内（异常）则下标停在 0，切到表首。 */
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

/* 取当前实际注入的图案（已按相位镜像），bit0..7 对应 IR1..8，压黑置 1。 */
uint8_t AppTestMode_GetPattern(void)
{
    return effectivePattern();
}

/* 调整基础占空比，step 正加负减，结果夹到 0~100。 */
void AppTestMode_StepBasePwm(int16_t step)
{
    trackBasePwm = clampBasePwm((int16_t)(trackBasePwm + step));
}

/* 取当前基础占空比，0~100。 */
int16_t AppTestMode_GetBasePwm(void)
{
    return trackBasePwm;
}

/* 取 PWM 循迹测试当前相位。 */
PwmTestPhase_t AppTestMode_GetTrackPhase(void)
{
    return trackPhase;
}

/* 取相位名，未知返回 "--"。供 OLED 与串口共用。 */
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

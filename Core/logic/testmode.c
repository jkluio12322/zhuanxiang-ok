/* 两种开环测试模式的记账：PWM 速度测试与 PWM 循迹测试。本模块只管状态（相位/计时/图案/
   基础占空比）；设目标、复位 PID、回传遥测由慢速控制任务决定，保持单向依赖。 */

#include "testmode.h"
#include "config.h"
#include "state.h"
#include "track.h"

/* PWM 循迹测试的模拟图案表。按 IR1→IR8（左→右）书写，1 = 压黑；bit0 对应最左一路，
   所以十六进制值与图案一一对应。KEY2 按此顺序轮换，每 PWM_TRACK_TEST_PHASE_MS 镜像一次
   （左右对调）。表对镜像封闭。注释里的值是稳定分母归一化误差。 */
static const uint8_t patternTable[] = {
    0x03U,  /* 1100 0000  直线偏移·大（左）     e -1.00 */
    0x0FU,  /* 1111 0000  左转弯口（4 路压黑）  e -0.72 */
    0x06U,  /* 0110 0000  直线偏移·中（左）     e -0.62 */
    0x0CU,  /* 0011 0000  直线偏移·小（左）     e -0.29 */
    0x18U,  /* 0001 1000  居中（IR4+IR5）       e  0.00 */
    0x30U,  /* 0000 1100  直线偏移·小（右）     e +0.29 */
    0x60U,  /* 0000 0110  直线偏移·中（右）     e +0.62 */
    0xF0U,  /* 0000 1111  右转弯口（4 路压黑）  e +0.72 */
    0xC0U,  /* 0000 0011  直线偏移·大（右）     e +1.00 */
    0xFFU,  /* 1111 1111  全黑 = 丢线          无效（压黑 8 路 > 上限 7） */
    0x00U,  /* 0000 0000  全白 = 到终点线      无效（all_white） */
    0x81U   /* 1000 0001  岔路口（中间断开）   无效（不连续） */
};
#define PATTERN_TABLE_SIZE (sizeof(patternTable) / sizeof(patternTable[0]))

/* 开机默认图案：居中（e = 0），自对称，自动镜像时保持稳定。 */
#define PATTERN_DEFAULT 0x18U

/* PWM 速度测试状态 */
static PwmTestPhase_t speedPhase = PWM_TEST_PHASE_FORWARD;
static uint32_t       speedElapsedMs = 0U; /* 当前相位已持续的时间，单位 ms；到阈值就换向。 */
static uint8_t        autoPhaseEnabled = 1U; /* 1 = 按时长自动换向；0 = 由串口 l/r/t 手动控制。 */

/* PWM-SPEED 手动开环：方向 0=停 / +1=正转 / -1=反转，占空比为百分比。方向为 0 时
   5 ms 任务回落到目标转速 PID 路径（串口 l/r/t 手动转速仍可用）。 */
static int8_t  speedDir  = 0;
static float speedDuty = (float)PWM_SPEED_DUTY_DEFAULT_PERCENT;
static uint8_t speedRawEnabled = 0U;

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
   50 ms 慢任务里调；自动换向关闭时直接返回 0。 */
uint8_t testmode_tick_pwm_speed(void)
{
    if (autoPhaseEnabled == 0U) {
        return 0U;
    }

    speedElapsedMs += CONTROL_SLOW_PERIOD_MS;
    if (speedElapsedMs < PWM_SPEED_TEST_PHASE_MS) {
        return 0U;
    }

    speedElapsedMs = 0U;
    speedPhase = (speedPhase == PWM_TEST_PHASE_FORWARD)
                 ? PWM_TEST_PHASE_REVERSE
                 : PWM_TEST_PHASE_FORWARD;
    return 1U;
}

/* 5 ms PWM 循迹测试任务（TIM6 的 5 ms 时隙，替代循迹任务）。模拟输入是八路图案，
   走与实车相同的判定、丢线/全白规则和循迹 PID，误差量纲与实车一致。 */
void testmode_task(void)
{
    /* 停车期间不推进相位，但仍走一步循迹测试：未运行时它负责收回 STOPPED 并发布快照。 */
    if (state_view()->running) {
        advanceTrackPhase();
    }

    (void)track_test_step(effectivePattern());
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
    trackElapsedMs += TRACK_PERIOD_MS;
    if (trackElapsedMs < PWM_TRACK_TEST_PHASE_MS) {
        return;
    }

    trackElapsedMs = 0U;
    trackPhase = (trackPhase == PWM_TEST_PHASE_LEFT)
                 ? PWM_TEST_PHASE_RIGHT
                 : PWM_TEST_PHASE_LEFT;
    track_reset_state();
}

/* 初始化测试模式状态。 */
void testmode_init(void)
{
    testmode_reset_pwm_speed();
    speedDuty = PWM_SPEED_DUTY_DEFAULT_PERCENT;
    testmode_reset_pwm_track();
    trackPattern = PATTERN_DEFAULT;
    trackBasePwm = PWM_TRACK_BASE_DEFAULT_PERCENT;
}

/* 复位 PWM 速度测试：相位回正转、计时清零、恢复自动换向（stop 和切模式都走这里）。 */
void testmode_reset_pwm_speed(void)
{
    speedPhase = PWM_TEST_PHASE_FORWARD;
    speedDir = 0;
    speedRawEnabled = 0U;
    speedElapsedMs = 0U;
    autoPhaseEnabled = 1U;
}

/* 开关自动换向。只需关，不必手动开——testmode_reset_pwm_speed() 会恢复。 */
void testmode_set_auto_phase(uint8_t enabled)
{
    autoPhaseEnabled = (enabled != 0U) ? 1U : 0U;
}

/* 当前相位方向符号：+1 正转，-1 反转。 */
int8_t testmode_get_pwm_speed_sign(void)
{
    return (speedPhase == PWM_TEST_PHASE_REVERSE) ? -1 : 1;
}

/* 当前速度测试相位。 */
PwmTestPhase_t testmode_get_speed_phase(void)
{
    return speedPhase;
}

/* 复位 PWM 循迹测试：相位回左偏、计时清零。不含 track_reset_state（调用方的事）。 */
void testmode_reset_pwm_track(void)
{
    trackPhase = PWM_TEST_PHASE_LEFT;
    trackElapsedMs = 0U;
}

/* 设置模拟图案，必须精确等于表内某项；不在表内静默忽略（不清 PID/相位）。命中时复位循迹状态机。
   当前无调用者——图案由 KEY2 经 NextPattern 轮换，保留作「直接指定图案」的入口。 */
void testmode_set_pattern(uint8_t pattern)
{
    uint8_t i;

    for (i = 0U; i < PATTERN_TABLE_SIZE; i++) {
        if (patternTable[i] == pattern) {
            trackPattern = pattern;
            track_reset_state();
            testmode_reset_pwm_track();
            break;
        }
    }
}

/* 在图案表里循环切到下一项；当前项不在表内（异常）则下标停在 0，切到表首。 */
void testmode_next_pattern(void)
{
    uint8_t index = 0U;

    for (uint8_t i = 0U; i < PATTERN_TABLE_SIZE; i++) {
        if (patternTable[i] == trackPattern) {
            index = (uint8_t)((i + 1U) % PATTERN_TABLE_SIZE);
            break;
        }
    }

    trackPattern = patternTable[index];

    track_reset_state();
    testmode_reset_pwm_track();
}

/* 取当前实际注入的图案（已按相位镜像），bit0..7 对应 IR1..8，压黑置 1。 */
uint8_t testmode_get_pattern(void)
{
    return effectivePattern();
}

/* 调整基础占空比，step 正加负减，结果夹到 0~100。 */
void testmode_step_base_pwm(int16_t step)
{
    trackBasePwm = clampBasePwm((int16_t)(trackBasePwm + step));
}

/* 取当前基础占空比，0~100。 */
int16_t testmode_get_base_pwm(void)
{
    return trackBasePwm;
}

/* 取 PWM 循迹测试当前相位。 */
PwmTestPhase_t testmode_get_track_phase(void)
{
    return trackPhase;
}

/* 取手动开环方向。 */
int8_t testmode_get_pwm_speed_dir(void)
{
    return speedDir;
}

/* 设置手动开环方向：正数归一化成 +1，负数 -1，0 即停。 */
void testmode_set_pwm_speed_dir(int8_t dir)
{
    if (dir > 0) {
        speedDir = 1;
    } else if (dir < 0) {
        speedDir = -1;
    } else {
        speedDir = 0;
    }
}

/* 取手动开环占空比百分比。 */
float testmode_get_pwm_speed_duty(void)
{
    return speedDuty;
}

/* 调整手动开环占空比，夹到 0~PWM_TEST_MAX_PERCENT（开环测试不需要反转占空比，方向另给）。 */
void testmode_step_pwm_speed_duty(float step)
{
    float value = speedDuty + step;

    if (value > PWM_TEST_MAX_PERCENT) value = PWM_TEST_MAX_PERCENT;
    if (value < 0) value = 0;
    speedDuty = value;
}

uint8_t testmode_get_pwm_speed_raw(void)
{
    return speedRawEnabled;
}

void testmode_set_pwm_speed_raw(uint8_t raw)
{
    speedRawEnabled = (raw != 0U) ? 1U : 0U;
}

/* 取相位名，未知返回 "--"。供 OLED 与串口共用。 */
const char *phase_name(PwmTestPhase_t phase)
{
    switch (phase) {
    case PWM_TEST_PHASE_FORWARD: return "FORWARD";
    case PWM_TEST_PHASE_REVERSE: return "REVERSE";
    case PWM_TEST_PHASE_LEFT:    return "LEFT";
    case PWM_TEST_PHASE_RIGHT:   return "RIGHT";
    default:                     return "--";
    }
}

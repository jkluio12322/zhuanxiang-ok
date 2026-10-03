/**
 * @file    app_testmode.c
 * @brief   两种开环测试模式的记账 — PWM 速度测试与 PWM 循迹测试。
 *
 * @note    本模块只维护**状态**：相位、计时、模拟角度、基础占空比。
 *         「谁来设目标、谁来复位 PID、谁来回传遥测」全部由 50 ms 控制任务
 *         （app_speedloop.c）决定——这样 app_testmode 与 app_speedloop
 *         之间保持单向依赖，不会互相调用成环。
 *
 * @note    两种模式的输出路径不同，务必分清：
 * @code
 *   PWM 速度测试：左右轮同向同速，走**速度环**，目标由 AppState_SetWheelTargets()
 *                 下发，按时长翻转方向。
 *   PWM 循迹测试：固定基础占空比加减差速，**旁路速度环**直接出 PWM，
 *                 差速由 AppTracking_UpdatePidForError() 算出。
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
#include "irtrack.h"
#include "app_tracking.h"

/* 允许的模拟角度表，app_console 的 simlevel 校验也用这几个值。
   改这张表必须同步改 app_console.c 里 TUNE_SIMLEVEL 分支的字面量判断。 */
static const uint8_t angleTable[] = {5U, 10U, 45U, 90U};
#define ANGLE_TABLE_SIZE (sizeof(angleTable) / sizeof(angleTable[0]))

/* ── PWM 速度测试状态 ─────────────────────────────────────────────── */
static PwmTestPhase_t speedPhase = PWM_TEST_PHASE_FORWARD;
/* 当前相位已持续的时间，单位 ms；到 PWM_SPEED_TEST_PHASE_MS 就换向。 */
static uint32_t       speedElapsedMs = 0U;

/* ── PWM 循迹测试状态 ─────────────────────────────────────────────── */
static PwmTestPhase_t trackPhase = PWM_TEST_PHASE_LEFT;
/* 当前偏转方向已持续的时间，单位 ms。 */
static uint32_t       trackElapsedMs = 0U;
/* 1 表示用模拟角度代替真实红外输入。 */
static uint8_t        trackSimEnabled = 1U;
/* 模拟角度，单位度；合法值限于 angleTable。 */
static uint8_t        trackAngleDeg = 5U;
/* 基础占空比百分比，0~100，差速在它上面加减。 */
static int16_t        trackBasePwm = PWM_TRACK_BASE_DEFAULT_PERCENT;

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
 * @brief  初始化测试模式状态。
 * @note   模拟角度默认开启（SIM），便于架空车轮验证；
 *         接上真实赛道前用 sim off 切回红外输入。
 */
void AppTestMode_Init(void)
{
    AppTestMode_ResetPwmSpeed();
    AppTestMode_ResetPwmTrack();
    trackSimEnabled = 1U;
    trackAngleDeg = 5U;
    trackBasePwm = PWM_TRACK_BASE_DEFAULT_PERCENT;
}

/* ---------------- PWM 速度测试 ---------------- */

/**
 * @brief  复位 PWM 速度测试：相位回到正转，计时清零。
 * @note   只复位本模块的记账；速度环的复位由调用方负责。
 */
void AppTestMode_ResetPwmSpeed(void)
{
    speedPhase = PWM_TEST_PHASE_FORWARD;
    speedElapsedMs = 0U;
}

/**
 * @brief  推进 PWM 速度测试计时，到点则翻转相位。
 * @return 1 表示本次调用发生了相位翻转；0 表示还没到。
 * @note   在 50 ms 控制任务里调用。**返回 1 时调用方必须重新建立目标转速
 *         并复位速度环**——否则 PID 会带着上个方向的积分继续跑。
 * @note   累计用 APP_CONTROL_PERIOD_MS 而不是实测时间，所以本函数
 *         必须跟着固定周期的控制任务跑。
 */
uint8_t AppTestMode_TickPwmSpeed(void)
{
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

/* ---------------- PWM 循迹测试 ---------------- */

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
 * @brief  把当前模拟角度线性映射成循迹误差。
 * @return 误差值；左偏为负、右偏为正，量级由两个满量程宏决定。
 * @note   映射是 PWM_TRACK_ANGLE_FULL_SCALE 度 -> PWM_TRACK_ERROR_FULL_SCALE。
 *         误差的量纲要跟真实红外的 e_y 一致，否则同一套 PID 在两种输入下
 *         表现差异过大，也就失去了「复用同一个 PID 验证」的意义。
 */
static float simError(void)
{
    float magnitude = (float)trackAngleDeg
                      * PWM_TRACK_ERROR_FULL_SCALE
                      / PWM_TRACK_ANGLE_FULL_SCALE;

    return (trackPhase == PWM_TEST_PHASE_LEFT) ? -magnitude : magnitude;
}

/**
 * @brief  推进 PWM 循迹测试计时，到点则翻转偏转方向。
 * @note   在 20 ms 任务里调用，所以累计用的是 APP_TRACK_PERIOD_MS。
 * @note   换向时复位循迹状态机：方向突变时旧的误差历史会拖着 PID 反着走。
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

/**
 * @brief  20 ms PWM 循迹测试任务。
 * @note   在 TIM6 的 20 ms 时隙里执行（modePwmTrack 时替代循迹任务）。
 *         未运行时直接返回，不推进相位——停车期间相位不该继续走。
 * @note   误差来源二选一：模拟角度（SIM）或真实红外（IR）。
 *         两条路径最终都交给 AppTracking_UpdatePidForError() 算差速，
 *         所以这里验证的就是循迹环本身。
 * @note   算出的差速通过 AppTracking_SetPwmSteer() 写进快照，
 *         由 50 ms 控制任务读出来叠到基础占空比上（见 app_speedloop.c）。
 */
void AppTestMode_Task20ms(void)
{
    float error;
    float turn;

    if (!AppState()->running) {
        return;
    }

    if (trackSimEnabled != 0U) {
        advanceTrackPhase();
        error = simError();
    } else {
        IRTrack_Read();
        IRTrack_Compute();
        error = AppTracking_ComputeWeightedError();
    }

    turn = AppTracking_UpdatePidForError(error, TRACK_FOLLOW_DIFF_LIMIT_RPM);
    AppTracking_SetPwmSteer(turn);
}

/**
 * @brief  设置模拟角度。
 * @param  angleDeg 目标角度，必须精确等于 angleTable 里的某个值。
 * @note   不在表内的值被**静默忽略**（保留原角度），不是夹到最近的有效值。
 * @note   改角度后复位循迹状态机和测试相位，避免上一次角度残留的误差历史。
 */
void AppTestMode_SetAngle(uint8_t angleDeg)
{
    for (uint8_t i = 0U; i < ANGLE_TABLE_SIZE; i++) {
        if (angleTable[i] == angleDeg) {
            trackAngleDeg = angleDeg;
            break;
        }
    }

    AppTracking_ResetState();
    AppTestMode_ResetPwmTrack();
}

/**
 * @brief  在角度表里循环切到下一个角度。
 * @note   若当前角度不在表内（异常情况），下标会停在 0，切到表首。
 */
void AppTestMode_NextAngle(void)
{
    uint8_t index = 0U;

    for (uint8_t i = 0U; i < ANGLE_TABLE_SIZE; i++) {
        if (angleTable[i] == trackAngleDeg) {
            index = (uint8_t)((i + 1U) % ANGLE_TABLE_SIZE);
            break;
        }
    }

    trackAngleDeg = angleTable[index];

    AppTracking_ResetState();
    AppTestMode_ResetPwmTrack();
}

/**
 * @brief  取当前模拟角度。
 * @return 角度值，单位度。
 */
uint8_t AppTestMode_GetAngle(void)
{
    return trackAngleDeg;
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
 * @brief  开关模拟角度输入。
 * @param  enabled 非 0 用模拟角度，0 用真实红外。
 * @note   切换后复位循迹状态机；切到模拟模式时还把测试相位也复位，
 *         保证从一个确定的起点开始。
 */
void AppTestMode_SetSimEnabled(uint8_t enabled)
{
    trackSimEnabled = (enabled != 0U) ? 1U : 0U;

    AppTracking_ResetState();
    if (trackSimEnabled != 0U) {
        AppTestMode_ResetPwmTrack();
    }
}

/**
 * @brief  查询模拟角度输入是否开启。
 * @return 1 表示 SIM，0 表示用真实红外。
 */
uint8_t AppTestMode_GetSimEnabled(void)
{
    return trackSimEnabled;
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

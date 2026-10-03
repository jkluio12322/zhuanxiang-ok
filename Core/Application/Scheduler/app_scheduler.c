/**
 * @file    app_scheduler.c
 * @brief   应用层任务调度 — TIM6 中断分频 + 延后任务表。
 *
 * @note    为什么任务要分「中断内」和「延后」两类：
 *          遥测的 snprintf 16 个浮点要 100~300 µs，OLED 的 I2C 刷屏可能上 ms，
 *          而 USART1 一个字节只要 86.8 µs。把这几个放进优先级最高的 TIM6 中断
 *          会压住串口接收中断好几个字节，直接丢数据。
 *          反过来，速度环和循迹环按**固定周期常数**（50 ms、dt = 0.02）计算，
 *          挪到可能有抖动的主循环里就会算错，所以必须留在中断。
 *
 * @note    实测代价：延后之后中断体从约 40 行降到 2 行
 *          （见 Hardware/System/timer.c），中断里不再有浮点和 I2C。
 *
 * @note    执行上下文：AppScheduler_OnTick() 在 TIM6 中断里，每 1 ms 一次；
 *          AppDeferred_Run() 在主循环里被反复调用。
 */

#include "app_scheduler.h"
#include "app_cmd.h"
#include "app_config.h"
#include "app_imu.h"
#include "app_speedloop.h"
#include "app_state.h"
#include "app_testmode.h"
#include "key.h"
#include "app_display.h"
#include "app_tracking.h"
#include "app_telemetry.h"

/* TIM6 的 1 ms 计数，用于任务分频。溢出周期约 49.7 天，对本工程足够。 */
static uint32_t tickCount = 0U;

/* 初始化完成前不放行任何任务，防止中断改到还没初始化的对象。 */
static uint8_t schedulerEnabled = 0U;

/* 延后任务的待办标志表。只置标志、不排队：同一个任务重复请求也只执行一次。 */
static volatile uint8_t deferredPending[DEFERRED_COUNT] = {0U, 0U};

/**
 * @brief  初始化调度器并放行中断分发。
 * @note   必须在所有模块的 *_Init() 之后、Timer_Init() 之前调用。
 *         原实现在 Key_Init()/AppState_Init() 之前就开了 TIM6 中断，
 *         中断可能改到尚未初始化的对象；这个门闩就是修那一处。
 */
void AppScheduler_Init(void)
{
    tickCount = 0U;
    deferredPending[DEFERRED_TELEMETRY] = 0U;
    deferredPending[DEFERRED_DISPLAY] = 0U;
    schedulerEnabled = 1U;
}

/**
 * @brief  复位一个「连续 N 次成立」确认计数器。
 * @param  c    计数器。
 * @param  need 需要连续成立的次数。
 * @note   取代原来散落的裸 tick 计数器——调用方不再自己 ++ / -- / 比阈值，
 *         也就不会出现「忘了清零」或「阈值写错」这类错。
 */
void Confirm_Reset(ConfirmCounter *c, uint16_t need)
{
    c->need = need;
    c->count = 0U;
}

/**
 * @brief  推进一次确认。
 * @param  c              计数器。
 * @param  condition_true 本次条件是否成立（非 0 即成立）。
 * @return 1 表示已经连续成立 need 次；0 表示还没到。
 * @note   条件不成立时计数立即清零——这正是「连续」的语义。
 * @note   计数到 need 后不再增长（封顶），所以**不能**拿 count 当累计计时用。
 *         需要「持续 N 次后停车」这类语义时，用独立的计数器。
 */
uint8_t Confirm_Update(ConfirmCounter *c, uint8_t condition_true)
{
    if (condition_true != 0U) {
        if (c->count < c->need) {
            c->count++;
        }
    } else {
        c->count = 0U;
    }

    return (c->count >= c->need) ? 1U : 0U;
}

/**
 * @brief  请求执行一个延后任务。
 * @param  id 延后任务编号。
 * @note   只置标志，不排队也不计数。在中断里调用是安全的（单字节写）。
 * @note   越界的 id 被静默忽略。
 */
void AppScheduler_RequestDeferred(AppDeferredId_t id)
{
    if ((uint32_t)id < (uint32_t)DEFERRED_COUNT) {
        deferredPending[id] = 1U;
    }
}

/**
 * @brief  取走一个延后任务的待办标志。
 * @param  id 延后任务编号。
 * @return 1 表示该任务待办且标志已清除；0 表示无待办或 id 越界。
 * @note   置位在中断、清除在主循环。即便同一周期内被请求多次，
 *         也只会执行一次——延后任务要的是「最新快照」，不是逐次补齐。
 */
uint8_t AppScheduler_TakeDeferred(AppDeferredId_t id)
{
    if ((uint32_t)id >= (uint32_t)DEFERRED_COUNT) {
        return 0U;
    }

    if (deferredPending[id] == 0U) {
        return 0U;
    }

    deferredPending[id] = 0U;
    return 1U;
}

/**
 * @brief  TIM6 的 1 ms 中断入口，分频调度各周期任务。
 * @note   **中断上下文** — 只允许调用耗时很短的函数。本函数里跑的
 *         控制环、循迹环、按键、姿态解算都是「短且必须定周期」的任务；
 *         遥测和 OLED 只置标志，不在这里执行。
 *
 * @note   周期与顺序（顺序会影响行为，改动前先想清楚）：
 * @code
 *   每 10 ms : AppImu_Task10ms()
 *   每 20 ms : Key_Tick() -> AppCmd_Task20ms() -> 姿态采样（先按键后采样）
 *   每 20 ms : AppTracking_Task20ms() 或 AppTestMode_Task20ms()
 *   每 50 ms : AppSpeedLoop_Task50ms() + 请求遥测（延后执行）
 *   每 100 ms: 请求刷新 OLED（延后执行）
 * @endcode
 *
 * @note   20 ms 的循迹任务排在 50 ms 的控制任务**之前**，这是有意的：
 *         控制环要用本周期刚算出的循迹目标，反过来的话会多滞后一个周期。
 *         在 tick 同时是 20 和 50 的倍数时（每 100 ms 一次）这个顺序才体现得出来。
 *
 * @note   按键固定 20 ms 一次，嵌套在 10 ms 分支里判断，
 *         所以 APP_KEY_TICK_PERIOD_MS 必须是 APP_SENSOR_PERIOD_MS 的整数倍。
 */
void AppScheduler_OnTick(void)
{
    if (schedulerEnabled == 0U) {
        return;
    }

    tickCount++;

    /* 每 10 ms 采样姿态；按键固定 20 ms 一次，先跑状态机再产生命令。 */
    if ((tickCount % APP_SENSOR_PERIOD_MS) == 0U) {
        if ((tickCount % APP_KEY_TICK_PERIOD_MS) == 0U) {
            Key_Tick();
            AppCmd_Task20ms();
        }

        AppImu_Task10ms();
    }

    /* 每 20 ms 生成循迹目标，供后续 50 ms 速度控制任务使用。 */
    if ((tickCount % APP_TRACK_PERIOD_MS) == 0U) {
        if (AppState()->mode == modePwmTrack) {
            AppTestMode_Task20ms();
        } else {
            AppTracking_Task20ms();
        }
    }

    /* 每 50 ms 执行速度控制，并把遥测发送交给主循环。 */
    if ((tickCount % APP_CONTROL_PERIOD_MS) == 0U) {
        AppSpeedLoop_Task50ms();
        AppScheduler_RequestDeferred(DEFERRED_TELEMETRY);
    }

    /* 每 100 ms 请求刷新一次 OLED 状态页。 */
    if ((tickCount % APP_DISPLAY_PERIOD_MS) == 0U) {
        AppScheduler_RequestDeferred(DEFERRED_DISPLAY);
    }
}

/**
 * @brief  执行所有已置位的延后任务。
 * @note   主循环的唯一入口。这里可以自由使用浮点、snprintf 和 I2C，
 *         因为不在中断上下文。
 * @note   命令行的解析不在这里——它由 main.c 直接调 AppConsole_Process()，
 *         因为它需要每个主循环都轮询一次，而不是按固定周期。
 * @note   若主循环被拖慢，延后任务只会「丢帧」不会积压：标志是单比特，
 *         同一任务被请求多次也只执行一次最新的一次。
 */
void AppDeferred_Run(void)
{
    if (AppScheduler_TakeDeferred(DEFERRED_TELEMETRY) != 0U) {
        AppTelemetry_Send();
    }

    if (AppScheduler_TakeDeferred(DEFERRED_DISPLAY) != 0U) {
        AppDisplay_Task100ms();
    }
}

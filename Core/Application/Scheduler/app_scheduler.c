/* 应用层任务调度：TIM6 中断分频 + 延后任务表。 */

#include "app_scheduler.h"
#include "app_cmd.h"
#include "app_config.h"
#include "app_display.h"
#include "app_imu.h"
#include "app_speedloop.h"
#include "app_state.h"
#include "app_testmode.h"
#include "app_tracking.h"
#include "app_telemetry.h"
#include "key.h"

static uint32_t tickCount = 0U;                          /* 1 ms 计数，溢出周期约 49.7 天 */
static uint8_t schedulerEnabled = 0U;                    /* 初始化完成前不放行任何任务 */
static volatile uint8_t deferredPending[DEFERRED_COUNT] = {0U, 0U};   /* 延后待办标志，只置位不排队 */

/* TIM6 的 1 ms 中断入口。只跑短且必须定周期的任务；
   遥测 snprintf 要 100~300 µs、OLED 刷屏上 ms，放进中断会压住串口接收而丢字节。 */
void AppScheduler_OnTick(void)
{
    if (schedulerEnabled == 0U) {
        return;
    }

    tickCount++;

    /* 10 ms：先按键（固定 20 ms）再姿态采样。 */
    if ((tickCount % APP_SENSOR_PERIOD_MS) == 0U) {
        if ((tickCount % APP_KEY_TICK_PERIOD_MS) == 0U) {
            Key_Tick();
            AppCmd_Task20ms();
        }

        AppImu_Task10ms();
    }

    /* 20 ms：生成循迹目标。排在下面 50 ms 之前，控制环才能用上本周期刚算出的目标。 */
    if ((tickCount % APP_TRACK_PERIOD_MS) == 0U) {
        if (AppState()->mode == modePwmTrack) {
            AppTestMode_Task20ms();
        } else {
            AppTracking_Task20ms();
        }
    }

    /* 50 ms：速度控制，并把遥测交给主循环。 */
    if ((tickCount % APP_CONTROL_PERIOD_MS) == 0U) {
        AppSpeedLoop_Task50ms();
        AppScheduler_RequestDeferred(DEFERRED_TELEMETRY);
    }

    /* 100 ms：请求刷新 OLED。 */
    if ((tickCount % APP_DISPLAY_PERIOD_MS) == 0U) {
        AppScheduler_RequestDeferred(DEFERRED_DISPLAY);
    }
}

/* 主循环里反复调用，执行已置位的延后任务。命令行解析不在这里，
   由 main.c 每轮直接调 AppConsole_Process()（要每轮轮询，不是定周期）。 */
void AppDeferred_Run(void)
{
    if (AppScheduler_TakeDeferred(DEFERRED_TELEMETRY) != 0U) {
        AppTelemetry_Send();
    }

    if (AppScheduler_TakeDeferred(DEFERRED_DISPLAY) != 0U) {
        AppDisplay_Task100ms();
    }
}

/* 放行中断分发。必须在所有 *_Init() 之后调用，否则中断可能改到未初始化的对象。 */
void AppScheduler_Init(void)
{
    tickCount = 0U;
    deferredPending[DEFERRED_TELEMETRY] = 0U;
    deferredPending[DEFERRED_DISPLAY] = 0U;
    schedulerEnabled = 1U;
}

/* 请求执行一个延后任务。只置标志；越界 id 静默忽略。中断里调用安全（单字节写）。 */
void AppScheduler_RequestDeferred(AppDeferredId_t id)
{
    if ((uint32_t)id < (uint32_t)DEFERRED_COUNT) {
        deferredPending[id] = 1U;
    }
}

/* 取走一个待办标志：1 = 有待办且已清除，0 = 无待办或 id 越界。
   置位在中断、清除在主循环；同一周期被请求多次也只执行一次（要的是最新快照，不是逐次补齐）。 */
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

/* 复位「连续 N 次成立」确认计数器。 */
void Confirm_Reset(ConfirmCounter *c, uint16_t need)
{
    c->need = need;
    c->count = 0U;
}

/* 推进一次确认。条件不成立立即清零（这就是「连续」的语义）；
   计数到 need 后封顶，所以不能拿 count 当累计计时用。目前只有循迹模块用它。 */
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

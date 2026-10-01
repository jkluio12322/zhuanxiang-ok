/**
 * @file    app_scheduler.c
 * @brief   应用层任务调度的实现。
 */

#include "app_scheduler.h"
#include "app_cmd.h"
#include "app_config.h"
#include "app_imu.h"
#include "app_speedloop.h"
#include "app_state.h"
#include "app_testmode.h"
#include "key.h"
#include "yingYongXianShi.h"
#include "app_tracking.h"
#include "app_telemetry.h"

/* TIM6 的 1 ms 计数，用于任务分频。 */
static uint32_t tickCount = 0U;

/* 初始化完成前不放行任何任务，防止中断改到还没初始化的对象。 */
static uint8_t schedulerEnabled = 0U;

/* 延后任务的待办标志表。 */
static volatile uint8_t deferredPending[DEFERRED_COUNT] = {0U, 0U};

void AppScheduler_Init(void)
{
    tickCount = 0U;
    deferredPending[DEFERRED_TELEMETRY] = 0U;
    deferredPending[DEFERRED_DISPLAY] = 0U;
    schedulerEnabled = 1U;
}

void Confirm_Reset(ConfirmCounter *c, uint16_t need)
{
    c->need = need;
    c->count = 0U;
}

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

void AppScheduler_RequestDeferred(AppDeferredId_t id)
{
    if ((uint32_t)id < (uint32_t)DEFERRED_COUNT) {
        deferredPending[id] = 1U;
    }
}

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

void AppDeferred_Run(void)
{
    if (AppScheduler_TakeDeferred(DEFERRED_TELEMETRY) != 0U) {
        AppTelemetry_Send();
    }

    if (AppScheduler_TakeDeferred(DEFERRED_DISPLAY) != 0U) {
        appDisplayUpdate();
    }
}

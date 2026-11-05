/* 应用层任务调度：TIM6 中断分频 + 延后任务表。
   IMU963RA 走 SPI2 一次要连读 12 字节（9 MHz 下约 11 us 忙等），放在 1 ms 中断里
   会拖长中断关闭窗口，因此只置 DEFERRED_IMU，由主循环执行。 */

#include "sched.h"
#include "cmd.h"
#include "config.h"
#include "display.h"
#include "imu.h"
#include "motion.h"
#include "speed_loop.h"
#include "state.h"
#include "testmode.h"
#include "track.h"
#include "telemetry.h"
#include "key.h"
#include "eight.h"
#include "main.h"
#include "trackbin.h"

static uint32_t tickCount = 0U;                          /* 1 ms 计数，溢出周期约 49.7 天 */
static uint8_t schedulerEnabled = 0U;                    /* 初始化完成前不放行任何任务 */
static volatile uint8_t deferredPending[DEFERRED_COUNT] = {0U, 0U, 0U};   /* 延后待办标志，只置位不排队 */
static volatile uint8_t trackPeriodMs = TRACK_PERIOD_MS;
static volatile uint8_t controlPeriodMs = CONTROL_PERIOD_MS;
static uint8_t trackCountdown = TRACK_PERIOD_MS;
static uint8_t controlCountdown = CONTROL_PERIOD_MS;
static uint8_t binCountdown = SQUARE_TRACK_BIN_PERIOD_MS;
static uint8_t dwtEnabled;
static uint8_t trackStartValid;
static uint8_t controlStartValid;
static uint32_t lastTrackStartCycles;
static uint32_t lastControlStartCycles;

static uint16_t cycles_to_10us(uint32_t cycles)
{
    uint32_t clock = SystemCoreClock;
    uint64_t units;

    if (clock == 0U) {
        return 0U;
    }
    units = ((uint64_t)cycles * 100000U + clock / 2U) / clock;
    return (units > 65535U) ? 65535U : (uint16_t)units;
}

static uint16_t cycles_to_20us(uint32_t cycles)
{
    uint32_t clock = SystemCoreClock;
    uint64_t units;

    if (clock == 0U) {
        return 0U;
    }
    units = ((uint64_t)cycles * 50000U + clock / 2U) / clock;
    return (units > 65535U) ? 65535U : (uint16_t)units;
}

static uint16_t interval_10us(uint32_t now, uint32_t previous,
                              uint8_t valid, uint8_t nominal_ms)
{
    uint32_t interval = valid != 0U
        ? (uint32_t)(now - previous)
        : (uint32_t)SystemCoreClock * nominal_ms / 1000U;
    return cycles_to_10us(interval);
}

/* TIM6 的 1 ms 中断入口。只跑短且必须定周期的任务；
   遥测 snprintf 要 100~300 µs、OLED 刷屏上 ms，放进中断会压住串口接收而丢字节。 */
void sched_tick(void)
{
    uint8_t track_due = 0U;
    uint8_t control_due = 0U;
    uint32_t track_interval = 0U;
    uint32_t control_interval = 0U;
    uint32_t track_exec_cycles = 0U;
    uint32_t control_exec_cycles = 0U;

    if (schedulerEnabled == 0U) {
        return;
    }

    tickCount++;

    /* 10 ms：按键（固定 20 ms）留在中断里；姿态采样只请求，由主循环做。 */
    if ((tickCount % SENSOR_PERIOD_MS) == 0U) {
        if ((tickCount % KEY_TICK_PERIOD_MS) == 0U) {
            key_tick();
            cmd_task();
        }

        deferred_set(DEFERRED_IMU);
    }

    if (trackCountdown <= 1U) {
        track_due = 1U;
        trackCountdown = trackPeriodMs;
    } else {
        trackCountdown--;
    }

    if (controlCountdown <= 1U) {
        control_due = 1U;
        controlCountdown = controlPeriodMs;
    } else {
        controlCountdown--;
    }

    if (control_due != 0U) {
        uint32_t started = dwtEnabled != 0U ? DWT->CYCCNT : 0U;
        control_interval = interval_10us(started, lastControlStartCycles,
                                         controlStartValid, controlPeriodMs);
        lastControlStartCycles = started;
        controlStartValid = 1U;

        motion_update(controlPeriodMs);
        control_exec_cycles += dwtEnabled != 0U
            ? (uint32_t)(DWT->CYCCNT - started) : 0U;
        trackbin_capture_motion(motion_view()->left_encoder_delta,
                                motion_view()->right_encoder_delta);
    }

    if (track_due != 0U) {
        const TrackOutput_t *track;
        uint32_t started = dwtEnabled != 0U ? DWT->CYCCNT : 0U;
        track_interval = interval_10us(started, lastTrackStartCycles,
                                       trackStartValid, trackPeriodMs);
        lastTrackStartCycles = started;
        trackStartValid = 1U;

        if (state_view()->mode == modePwmTrack) {
            testmode_task();
        } else if (state_view()->mode == modeEight) {
            eight_task(HAL_GetTick());
        } else {
            track_task();
        }
        track_exec_cycles = dwtEnabled != 0U
            ? (uint32_t)(DWT->CYCCNT - started) : 0U;
        track = track_output();
        trackbin_capture_track(track->pattern, track->line_valid,
                               track->line_error,
                               cycles_to_20us(track_exec_cycles),
                               (uint16_t)track_interval);
    }

    if (control_due != 0U) {
        uint32_t started = dwtEnabled != 0U ? DWT->CYCCNT : 0U;
        speed_loop_task();
        control_exec_cycles += dwtEnabled != 0U
            ? (uint32_t)(DWT->CYCCNT - started) : 0U;
        trackbin_capture_control(cycles_to_20us(control_exec_cycles),
                                 (uint16_t)control_interval);
    }

    /* 50 ms：推进测试阶段并请求慢速遥测。 */
    if ((tickCount % CONTROL_SLOW_PERIOD_MS) == 0U) {
        speed_loop_slow_task();
        deferred_set(DEFERRED_TELEMETRY);
    }

    /* 100 ms：请求刷新 OLED。 */
    if ((tickCount % DISPLAY_PERIOD_MS) == 0U) {
        deferred_set(DEFERRED_DISPLAY);
    }

    if (binCountdown <= 1U) {
        binCountdown = SQUARE_TRACK_BIN_PERIOD_MS;
        trackbin_tick_5ms(tickCount);
    } else {
        binCountdown--;
    }
}

/* 主循环里反复调用，执行已置位的延后任务。命令行解析不在这里，
   由 main.c 每轮直接调 console_process()（要每轮轮询，不是定周期）。 */
void deferred_run(void)
{
    /* 姿态采样排最前：只花 11 us 忙等，却能把采样时刻的抖动压到最小
       （主循环每轮都先到这里；排后面会被遥测格式化/刷屏推迟几百 us）。 */
    if (deferred_take(DEFERRED_IMU) != 0U) {
        imu_task();
    }

    if (deferred_take(DEFERRED_TELEMETRY) != 0U) {
        telemetry_send();
    }

    if (deferred_take(DEFERRED_DISPLAY) != 0U) {
        display_task();
    }
}

/* 放行中断分发。必须在所有 *_Init() 之后调用，否则中断可能改到未初始化的对象。 */
void sched_init(void)
{
    tickCount = 0U;
    trackCountdown = trackPeriodMs;
    controlCountdown = controlPeriodMs;
    binCountdown = SQUARE_TRACK_BIN_PERIOD_MS;
    trackStartValid = 0U;
    controlStartValid = 0U;
    lastTrackStartCycles = 0U;
    lastControlStartCycles = 0U;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwtEnabled = (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
    deferredPending[DEFERRED_TELEMETRY] = 0U;
    deferredPending[DEFERRED_DISPLAY] = 0U;
    deferredPending[DEFERRED_IMU] = 0U;
    schedulerEnabled = 1U;
}

uint8_t sched_set_periods(uint8_t track_ms, uint8_t control_ms,
                          uint8_t speed_window_ms)
{
    uint32_t primask;

    if ((track_ms < TRACK_TASK_PERIOD_MIN_MS)
        || (track_ms > TRACK_TASK_PERIOD_MAX_MS)
        || (control_ms < CONTROL_TASK_PERIOD_MIN_MS)
        || (control_ms > CONTROL_TASK_PERIOD_MAX_MS)
        || (speed_window_ms < SPEED_MEASURE_WINDOW_MIN_MS)
        || (speed_window_ms > SPEED_MEASURE_WINDOW_MAX_CONFIG_MS)
        || (state_view()->running != 0U)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (motion_set_measure_window_ms(speed_window_ms) == 0U) {
        __set_PRIMASK(primask);
        return 0U;
    }
    trackPeriodMs = track_ms;
    controlPeriodMs = control_ms;
    trackCountdown = track_ms;
    controlCountdown = control_ms;
    __set_PRIMASK(primask);
    return 1U;
}

void sched_get_periods(SchedPeriods_t *periods)
{
    uint32_t primask;

    if (periods == 0) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    periods->track_period_ms = trackPeriodMs;
    periods->control_period_ms = controlPeriodMs;
    periods->speed_window_ms = motion_get_measure_window_ms();
    __set_PRIMASK(primask);
}

uint8_t sched_track_period_ms(void)
{
    return trackPeriodMs;
}

uint8_t sched_control_period_ms(void)
{
    return controlPeriodMs;
}

/* 请求执行一个延后任务。只置标志；越界 id 静默忽略。中断里调用安全（单字节写）。 */
void deferred_set(DeferredId id)
{
    if ((uint32_t)id < (uint32_t)DEFERRED_COUNT) {
        deferredPending[id] = 1U;
    }
}

/* 取走一个待办标志：1 = 有待办且已清除，0 = 无待办或 id 越界。
   置位在中断、清除在主循环；同一周期被请求多次也只执行一次（要的是最新快照，不是逐次补齐）。 */
uint8_t deferred_take(DeferredId id)
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
void confirm_reset(ConfirmCounter *c, uint16_t need)
{
    c->need = need;
    c->count = 0U;
}

/* 推进一次确认。条件不成立立即清零（这就是「连续」的语义）；
   计数到 need 后封顶，所以不能拿 count 当累计计时用。目前只有循迹模块用它。 */
uint8_t confirm_update(ConfirmCounter *c, uint8_t condition_true)
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

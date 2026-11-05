/* 应用层唯一可变状态的实现。
   本模块只持有数据，不执行任何动作：读走 state_view() 只读视图，写走 state_set_xxx()。
   启停编排、模式切换等动作留在控制层（app_speedloop.c），否则会互相依赖成环。
   读写从多个上下文发起（中断里的控制/循迹任务，主循环里的显示与命令行）。 */

#include "state.h"
#include "config.h"

/* 状态本体是 static，外部只能通过 state_view() 读、通过 state_set_xxx() 写。 */
static StateView state;

/* 档位转速表，下标 = 档位 - 1。改档位数要同步改表。 */
static const float speedTable[SPEED_LEVEL_COUNT] = {
    SPEED_TABLE_G1_RPM,
    SPEED_TABLE_G2_RPM,
    SPEED_TABLE_G3_RPM,
    SPEED_TABLE_G4_RPM,
    SPEED_TABLE_G5_RPM,
    SPEED_TABLE_G6_RPM
};

static StopReason lastStopReason = stopReasonNone;   /* 最近一次停车原因，配合下面两个标志组成一次性事件 */
static volatile uint8_t stopEventPending = 0U;            /* 停车事件待上报，中断置位/主循环取走 */
static volatile uint8_t stopRequested = 0U;               /* 停车请求待执行，同上，跨上下文 */

/* 取只读状态视图，直接读字段即可（state_view()->gear），不为每个字段另配 getter。 */
const StateView *state_view(void)
{
    return &state;
}

/* 恢复到上电默认值。档位默认值取 SPEED_LEVEL_DEFAULT，同时按表算出 gear_rpm，
   保证 gear 与 gear_rpm 一开始就自洽。三个停车标志全部清零。 */
void state_init(void)
{
    state.mode = modeTrack;
    state.running = 0U;
    state.vofa = 1U;
    state.gear = SPEED_LEVEL_DEFAULT;
    state.gear_rpm = speedTable[state.gear - 1U];
    state.lap_target = TRACK_LAP_TARGET_DEFAULT;
    state.laps_done = 0U;
    state.target_left_rpm = 0.0f;
    state.target_right_rpm = 0.0f;

    lastStopReason = stopReasonNone;
    stopEventPending = 0U;
    stopRequested = 0U;
}

/* 只改字段，不做任何复位。模式切换时的复位由 app_speedloop.c 的 handleCommand()
   负责，因为它要同时动循迹、测试模式和速度环三处的状态。 */
void state_set_mode(Mode mode)
{
    state.mode = mode;
}

/* TRACK -> PWM-SPEED -> PWM-TRACK -> LOOP-TEST -> TRACK。
   返回的是**模式**不是命令：按键路径还要经 app_cmd.c 转成切换命令才能触发复位流程。 */
Mode state_next_mode(void)
{
    if (state.mode == modeTrack) {
        return modePwmSpeed;
    }
    if (state.mode == modePwmSpeed) {
        return modePwmTrack;
    }
    if (state.mode == modePwmTrack) {
        return modeLoopTest;
    }
    if (state.mode == modeLoopTest) {
        return modeEight;
    }
    return modeTrack;
}

/* 返回静态字符串，供 OLED 与串口命令行共用；调用方不需要也不应该释放。 */
const char *mode_name(Mode mode)
{
    if (mode == modePwmSpeed) {
        return "PWM-SPEED";
    }
    if (mode == modePwmTrack) {
        return "PWM-TRACK";
    }
    if (mode == modeLoopTest) {
        return "LOOP-TEST";
    }
    if (mode == modeEight) {
        return "EIGHT";
    }
    return "TRACK";
}

void state_set_running(uint8_t running)   /* 非 0 归一化成 1 */
{
    state.running = (running != 0U) ? 1U : 0U;
}

void state_set_vofa(uint8_t on)           /* 非 0 归一化成 1 */
{
    state.vofa = (on != 0U) ? 1U : 0U;
}

/* 设置档位；越界按**回绕**处理（<1 -> 最高档，>COUNT -> 最低档），而不是夹到边界，
   这样按键连发可以自然循环。返回 1 表示档位确实变了，调用方据此决定是否重初始化。 */
uint8_t state_set_gear(uint8_t gear)
{
    if (gear < 1U) {
        gear = SPEED_LEVEL_COUNT;
    }
    if (gear > SPEED_LEVEL_COUNT) {
        gear = 1U;
    }

    if (gear == state.gear) {
        return 0U;
    }

    state.gear = gear;
    state.gear_rpm = speedTable[gear - 1U];
    return 1U;
}

/* 按方向升/降档。用 int16_t 承接是为了避免 uint8_t 在 1-1 时下溢成 255。 */
void state_step_gear(int8_t direction)
{
    int16_t next = (int16_t)state.gear + (int16_t)direction;

    if (next > (int16_t)SPEED_LEVEL_COUNT) {
        next = 1;
    }
    if (next < 1) {
        next = (int16_t)SPEED_LEVEL_COUNT;
    }

    (void)state_set_gear((uint8_t)next);
}

/* 目标圈数加一，循环 1~TRACK_LAP_TARGET_MAX。
   只允许在停车时调用（由 app_speedloop.c 把关）：运行中改目标圈数会让刚完成的计圈判据突然变化。 */
void state_cycle_lap_target(void)
{
    uint8_t next = (uint8_t)(state.lap_target + 1U);

    if (next > TRACK_LAP_TARGET_MAX) {
        next = 1U;
    }
    state.lap_target = next;
}

uint8_t state_set_lap_target(uint8_t target)
{
    if ((target < 1U) || (target > TRACK_LAP_TARGET_MAX)) {
        return 0U;
    }
    state.lap_target = target;
    state.laps_done = 0U;
    return 1U;
}

/* 完成圈数加一，上限是 lap_target。真正触发停车由停车事件（stopReasonTargetLaps）负责。 */
void state_increment_lap(void)
{
    if (state.laps_done < state.lap_target) {
        state.laps_done++;
    }
}

void state_reset_lap_progress(void)
{
    state.laps_done = 0U;
}

/* 设置左右轮目标转速，单位 RPM。这里不做限幅（限幅在 app_speedloop.c 的 clampSpeedTarget()），
   本字段主要供显示和调试观察实际下发的目标。 */
void state_set_wheel_targets(float left_rpm, float right_rpm)
{
    state.target_left_rpm = left_rpm;
    state.target_right_rpm = right_rpm;
}

/* 上报一次停车事件；stopReasonNone 被忽略。
   同时置两个标志，它们服务于不同的消费者、互不干扰：
stopEventPending 给遥测打印 [STOP] 行，stopRequested 给 5 ms 控制任务真正执行停车。
   分成两个是因为「报告」和「执行」在不同模块，共用一个标志会让先取走的一方让另一方看不到。 */
void state_report_stop(StopReason reason)
{
    if (reason == stopReasonNone) {
        return;
    }

    lastStopReason = reason;
    stopEventPending = 1U;
    stopRequested = 1U;
}

/* 取走并清除「请求停车」标志；由 speed_loop.c 的 5 ms 任务消费后执行停车。 */
uint8_t state_take_stop_request(void)
{
    if (stopRequested == 0U) {
        return 0U;
    }

    stopRequested = 0U;
    return 1U;
}

/* 查看待上报的停车原因；1 = 有事件（*reason 有效）。
   **只读不清除**：通道由 state_clear_stop_event() 在确认整行遥测发送成功后才清除，
   发送失败时下一周期可以重试。这是它与 state_take_stop_request()（取走即清）的区别。 */
uint8_t state_take_stop_event(StopReason *reason)
{
    if ((stopEventPending == 0U) || (reason == 0)) {
        return 0U;
    }

    *reason = lastStopReason;
    return 1U;
}

/* 清除停车事件和停车请求。遥测确认 [STOP] 行写进发送缓冲后调用，收到 start 命令时也会调用。 */
void state_clear_stop_event(void)
{
    lastStopReason = stopReasonNone;
    stopEventPending = 0U;
    stopRequested = 0U;
}

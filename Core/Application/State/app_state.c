/**
 * @file    app_state.c
 * @brief   应用层唯一可变状态的实现。
 *
 * @note    本模块**只持有数据**，不执行任何动作：读走 AppState() 只读视图，
 *          写走 AppState_SetXxx()。启停编排、模式切换等动作留在控制层
 *          （app_speedloop.c），否则 app_state 和 app_testmode 会互相依赖成环。
 *
 * @note    为什么状态要收在一个模块里：旧实现把 11 个全局变量外放在一个头文件里，
 *          被 7 个模块直接改写，出了问题无法判断是谁写的。现在写入口只有这里，
 *          读用 AppState()->字段（不为每个字段单独配 getter）。
 *
 * @note    执行上下文：读操作从多个上下文发起（中断里的控制/循迹任务，主循环里的
 *          显示与命令行）。除下面三个 volatile 标志外，其余字段都是「单写多读」，
 *          且写入方只在定时任务里，读到上一周期的值不影响控制正确性。
 *
 * @see     app_speedloop.c  状态的唯一编排者
 */

#include "app_state.h"
#include "app_config.h"

/* 状态本体是 static，外部只能通过 AppState() 读、通过 AppState_SetXxx() 写。 */
static AppStateView_t state;

/* 档位转速表，下标 = 档位 - 1。与 SPEED_LEVEL_COUNT 一一对应，改档位数要同步改表。 */
static const float speedTable[SPEED_LEVEL_COUNT] = {
    SPEED_TABLE_G1_RPM,
    SPEED_TABLE_G2_RPM,
    SPEED_TABLE_G3_RPM,
    SPEED_TABLE_G4_RPM,
    SPEED_TABLE_G5_RPM,
    SPEED_TABLE_G6_RPM
};

/* 最近一次停车原因，配合 stopEventPending 组成「一次性事件」。 */
static appStopReason_t lastStopReason = stopReasonNone;
/* 停车事件待上报（遥测打印 [STOP] 行）。可能被中断置位、主循环取走，故 volatile。 */
static volatile uint8_t stopEventPending = 0U;
/* 停车请求待执行（由控制任务消费后真正停车）。同上，跨上下文，故 volatile。 */
static volatile uint8_t stopRequested = 0U;

/**
 * @brief  获取只读状态视图。
 * @return 指向内部状态结构体的常量指针。
 * @note   直接读字段即可，例如 AppState()->gear；不要为每个字段另配 getter。
 */
const AppStateView_t *AppState(void)
{
    return &state;
}

/**
 * @brief  把状态恢复到上电默认值。
 * @note   档位默认值取 SPEED_LEVEL_DEFAULT，同时按表算出对应的 gear_rpm，
 *         保证 gear 与 gear_rpm 一开始就自洽。
 * @note   三个停车标志全部清零。原实现漏了 stopRequested，
 *         若在上电流程中再次调用本函数会残留上一次的停车请求。
 */
void AppState_Init(void)
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

/**
 * @brief  设置工作模式。
 * @param  mode 目标模式。
 * @note   只改字段，不做任何复位。模式切换时的复位与重新初始化由
 *         app_speedloop.c 的 handleCommand() 负责，因为它要同时动
 *         循迹、测试模式和速度环三处的状态。
 */
void AppState_SetMode(workMode_t mode)
{
    state.mode = mode;
}

/**
 * @brief  取模式循环里的下一个模式。
 * @return TRACK -> PWM-SPEED -> PWM-TRACK -> TRACK。
 * @note   按键轮换和开机页轮换共用这一处，避免两套循环逻辑各自漂移。
 * @note   注意返回的是**模式**不是命令：按键路径还要经 app_cmd.c 转成
 *         切换命令才能触发复位流程。
 */
workMode_t AppState_NextMode(void)
{
    if (state.mode == modeTrack) {
        return modePwmSpeed;
    }
    if (state.mode == modePwmSpeed) {
        return modePwmTrack;
    }
    return modeTrack;
}

/**
 * @brief  取模式名。
 * @param  mode 工作模式。
 * @return 静态字符串，供 OLED 与串口命令行共用。
 * @note   返回的是常量字面量，调用方不需要、也不应该释放。
 */
const char *AppModeName(workMode_t mode)
{
    if (mode == modePwmSpeed) {
        return "PWM-SPEED";
    }
    if (mode == modePwmTrack) {
        return "PWM-TRACK";
    }
    return "TRACK";
}

/**
 * @brief  设置运行标志。
 * @param  running 0 停车、非 0 运行；内部归一化成 0/1。
 */
void AppState_SetRunning(uint8_t running)
{
    state.running = (running != 0U) ? 1U : 0U;
}

/**
 * @brief  开关 VOFA 遥测。
 * @param  on 0 关闭、非 0 打开；内部归一化成 0/1。
 */
void AppState_SetVofa(uint8_t on)
{
    state.vofa = (on != 0U) ? 1U : 0U;
}

/**
 * @brief  设置档位。
 * @param  gear 目标档位，期望范围 1~SPEED_LEVEL_COUNT。
 * @return 1 表示档位确实变了；0 表示传入值与当前档位相同。
 * @note   越界值按**回绕**处理（<1 -> 最高档，>COUNT -> 最低档），
 *         而不是夹到边界——这样按键连发可以自然循环。
 * @note   返回值让调用方只在档位真变时才做重初始化（如 PWM 速度测试重启）。
 */
uint8_t AppState_SetGear(uint8_t gear)
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

/**
 * @brief  按方向增减档位（循环）。
 * @param  direction 正数升档、负数降档；幅度超过 1 会被夹到边界再回绕。
 * @note   用 int16_t 承接是为了避免 uint8_t 在 1-1 时下溢成 255。
 */
void AppState_StepGear(int8_t direction)
{
    int16_t next = (int16_t)state.gear + (int16_t)direction;

    if (next > (int16_t)SPEED_LEVEL_COUNT) {
        next = 1;
    }
    if (next < 1) {
        next = (int16_t)SPEED_LEVEL_COUNT;
    }

    (void)AppState_SetGear((uint8_t)next);
}

/**
 * @brief  目标圈数加一（循环 1~TRACK_LAP_TARGET_MAX）。
 * @note   只允许在停车时调用（由 app_speedloop.c 把关）：运行中改目标圈数
 *         会让刚完成的计圈判据突然变化。
 */
void AppState_CycleLapTarget(void)
{
    uint8_t next = (uint8_t)(state.lap_target + 1U);

    if (next > TRACK_LAP_TARGET_MAX) {
        next = 1U;
    }
    state.lap_target = next;
}

/**
 * @brief  完成圈数加一。
 * @note   上限是 lap_target，达到后不再增加。真正触发停车由停车事件
 *         （stopReasonTargetLaps）负责，不在这里判断。
 */
void AppState_IncrementLap(void)
{
    if (state.laps_done < state.lap_target) {
        state.laps_done++;
    }
}

/**
 * @brief  把完成圈数清零。
 */
void AppState_ResetLapProgress(void)
{
    state.laps_done = 0U;
}

/**
 * @brief  设置左右轮目标转速。
 * @param  left_rpm  左轮目标，单位 RPM。
 * @param  right_rpm 右轮目标，单位 RPM。
 * @note   这里不做限幅，限幅由 app_speedloop.c 的 clampSpeedTarget() 负责。
 *         本字段主要供显示和调试观察实际下发的目标。
 */
void AppState_SetWheelTargets(float left_rpm, float right_rpm)
{
    state.target_left_rpm = left_rpm;
    state.target_right_rpm = right_rpm;
}

/**
 * @brief  上报一次停车事件。
 * @param  reason 停车原因；传 stopReasonNone 会被忽略。
 * @note   同时置两个标志，它们服务于不同的消费者、互不干扰：
 *         - stopEventPending：给遥测，用来打印一次 [STOP] reason=... 行；
 *         - stopRequested：给 50 ms 控制任务，用来真正执行停车。
 *         分成两个是因为「报告」和「执行」发生在不同模块，
 *         共用一个标志会导致先取走的一方让另一方看不到。
 */
void AppState_ReportStop(appStopReason_t reason)
{
    if (reason == stopReasonNone) {
        return;
    }

    lastStopReason = reason;
    stopEventPending = 1U;
    stopRequested = 1U;
}

/**
 * @brief  取走并清除「请求停车」标志。
 * @return 1 表示有模块请求停车；0 表示没有。
 * @note   由 app_speedloop.c 的 50 ms 任务消费，消费后执行停车。
 */
uint8_t AppState_TakeStopRequest(void)
{
    if (stopRequested == 0U) {
        return 0U;
    }

    stopRequested = 0U;
    return 1U;
}

/**
 * @brief  查看待上报的停车原因。
 * @param  reason 输出参数，有事件时写入原因。
 * @return 1 表示有事件（*reason 有效）；0 表示无事件或 reason 为空指针。
 * @note   **只读不清除**：事件由 AppState_ClearStopEvent() 在确认整行遥测
 *         发送成功后才清除，发送失败时下一周期可以重试。
 * @note   与 AppState_TakeStopRequest() 的区别就在这里——那个是取走即清。
 */
uint8_t AppState_TakeStopEvent(appStopReason_t *reason)
{
    if ((stopEventPending == 0U) || (reason == 0)) {
        return 0U;
    }

    *reason = lastStopReason;
    return 1U;
}

/**
 * @brief  清除停车事件和停车请求。
 * @note   遥测确认 [STOP] 行写进发送缓冲后调用；收到 start 命令时也会调用。
 */
void AppState_ClearStopEvent(void)
{
    lastStopReason = stopReasonNone;
    stopEventPending = 0U;
    stopRequested = 0U;
}

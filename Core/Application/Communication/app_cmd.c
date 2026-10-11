/**
 * @file    app_cmd.c
 * @brief   20 ms 按键任务 + 命令队列。
 *
 * @note    文件内排序：核心任务放最上方，辅助函数与对外接口放下面。
 *
 * @note    数据流（生产者与消费者分处中断和主循环，是本模块最需要小心的地方）：
 * @code
 *   生产者 A：TIM6 中断  AppCmd_Task20ms()       -> AppCmd_PushValue()
 *   生产者 B：主循环     AppConsole_Process()    -> AppCmd_PushValue()
 *   消费者  ：TIM6 中断  AppSpeedLoop_Task50ms() -> AppCmd_Pop()
 * @endcode
 *          队列满时**静默丢弃新命令**，不覆盖尚未执行的控制动作——这是有意的：
 *          丢弃比让"停车"指令被"启动"指令顶掉要安全。
 *
 * @note    两个生产者分处不同执行上下文，所以 AppCmd_Push() 对 commandHead 的
 *          「读-改-写」必须放进临界区——否则中断打断主循环的 Push 时，两边会
 *          读到同一个 commandHead，后写的那条覆盖前一条，表现为**偶发丢命令**。
 *          该临界区已在 AppCmd_Push() 内实现。
 *
 * @see     app_speedloop.c  命令的实际执行者
 * @see     app_console.c    生产者 B 的来源
 */

#include "app_cmd.h"
#include "app_config.h"
#include "app_console.h"
#include "app_state.h"
#include "key.h"

/* 命令环形队列。容量 COMMAND_QUEUE_SIZE（8），实际可用 7 项——留一个空位
   用于区分「空」和「满」，这是环形队列的标准做法。 */
static commandMessage_t commandQueue[COMMAND_QUEUE_SIZE];

/* 写入位置，由生产者推进。ISR 与主循环都会写，故加 volatile。 */
static volatile uint8_t commandHead = 0;

/* 读取位置，只由消费者（TIM6 中断）推进。 */
static volatile uint8_t commandTail = 0;

/* ── 辅助函数前置声明（定义在文件下方） ───────────────────────────── */
static commandType_t appNextMode(void);

/* ══════════════ 核心任务 ══════════════ */

/**
 * @brief  20 ms 按键任务：扫描四路按键并产生对应命令 —— 本文件核心。
 * @note   在 TIM6 中断里执行，由 app_scheduler.c 的 20 ms 时隙调用，
 *         顺序为 Key_Tick() -> AppCmd_Task20ms()（先跑状态机再取事件）。
 *
 * @note   按键在各模式下的含义不同，因此按模式分三段处理：
 * @code
 *   KEY1 短按/长按 : 循环切换模式（所有模式通用）
 *   循迹模式        KEY2 启动/停止  KEY3 停止  KEY4 切换目标圈数（仅停车时）
 *   PWM 循迹模式    KEY2 切换模拟图案  KEY3 启动/停止  KEY4 短按/长按调基础占空比
 *   PWM 速度模式    KEY2 升档  KEY3 启动/停止
 * @endcode
 *
 * @note   在非循迹模式下有一句 (void)Key_CheckSingleOrLong(KEY_4)：KEY4 在这两个
 *         模式里用的是短按和长按事件，需要把「短按+长按」的合成事件吃掉，
 *         否则会残留到下次循环造成误触发。
 */
void AppCmd_Task20ms(void)
{
    if (Key_CheckSingleOrLong(KEY_1)) {
        AppCmd_PushValue(appNextMode(), 0.0f);
    }

    if (AppState()->mode == modeTrack) {
        /* 目标圈数只允许在停车状态下修改，运行中改会让计圈判据突然变化。 */
        if (Key_Check(KEY_4, KEY_SINGLE) && !AppState()->running) {
            AppCmd_PushValue(commandLapCycle, 0.0f);
        }
        (void)Key_CheckSingleOrLong(KEY_4);
        /* KEY2 是启停切换键：当前状态决定发启动还是停止。 */
        if (Key_Check(KEY_2, KEY_SINGLE)) {
            AppCmd_PushValue(AppState()->running ? commandStop : commandStart, 0.0f);
        }
        if (Key_Check(KEY_3, KEY_SINGLE)) {
            AppCmd_PushValue(commandStop, 0.0f);
        }
        return;
    }

    if (AppState()->mode == modePwmTrack) {
        /* KEY4 短按加、长按减，用于在线调整开环基础占空比。 */
        if (Key_Check(KEY_4, KEY_SINGLE)) {
            AppCmd_PushValue(commandPwmTrackBaseStep,
                                (float)PWM_TRACK_BASE_STEP_PERCENT);
        }
        if (Key_Check(KEY_4, KEY_LONG)) {
            AppCmd_PushValue(commandPwmTrackBaseStep,
                                -(float)PWM_TRACK_BASE_STEP_PERCENT);
        }
        (void)Key_CheckSingleOrLong(KEY_4);
        if (Key_Check(KEY_2, KEY_SINGLE)) {
            AppCmd_PushValue(commandPwmTrackNext, 0.0f);
        }
        if (Key_Check(KEY_3, KEY_SINGLE)) {
            AppCmd_PushValue(AppState()->running ? commandStop : commandStart, 0.0f);
        }
        return;
    }

    /* 剩下的是 PWM 速度模式。 */
    (void)Key_CheckSingleOrLong(KEY_4);
    if (Key_Check(KEY_2, KEY_SINGLE)) {
        AppCmd_PushValue(commandPwmSpeedNextGear, 0.0f);
    }
    if (Key_Check(KEY_3, KEY_SINGLE)) {
        AppCmd_PushValue(AppState()->running ? commandStop : commandStart, 0.0f);
    }
}

/* ══════════════ 辅助函数 ══════════════ */

/**
 * @brief  取模式循环里的下一个模式对应的切换命令。
 * @return commandModePwmSpeed / commandModePwmTrack / commandModeTrack。
 * @note   模式顺序为 TRACK -> PWM-SPEED -> PWM-TRACK -> TRACK。
 *         这里返回的是**命令**而非模式本身，因为模式切换必须经过队列、
 *         由 50 ms 控制任务统一执行复位和重新初始化。
 *         开机页的模式轮换走 AppState_NextMode()，两处顺序必须保持一致。
 */
static commandType_t appNextMode(void)
{
    if (AppState()->mode == modeTrack) return commandModePwmSpeed;
    if (AppState()->mode == modePwmSpeed) return commandModePwmTrack;
    return commandModeTrack;
}

/* ══════════════ 对外接口 ══════════════ */

/**
 * @brief  清空命令队列。
 * @note   把两个下标复位到 0 即视为空，不擦除数组内容。
 *         在中断放行之前、以及开机配置流程结束时调用。
 */
void AppCmd_Init(void)
{
    commandHead = 0;
    commandTail = 0;
}

/**
 * @brief  把一条命令写入队列尾部。
 * @param  message 待入队的命令，按值拷贝。
 * @return 1 表示已入队；0 表示队列已满，本条被丢弃。
 * @note   队列满时丢弃新命令——不应在正常运行时发生，若频繁出现说明生产
 *         速度超过了 50 ms 控制任务的消费速度。谁在意结果可以用返回值给出
 *         提示（串口 l/r/t 就靠它回显"队列满"），不在意就忽略。
 * @note   本函数可能被 UART 中断（经由主循环的 AppConsole_Process）与
 *         TIM6 中断（经由 AppCmd_Task20ms）交替调用，因此对 commandHead
 *         的「读-改-写」整体放进临界区。若省掉临界区，两个生产者会
 *         读到同一个 commandHead，后写的那条覆盖前一条，表现为**偶发丢命令**。
 * @note   临界区用保存/恢复 PRIMASK 的写法，所以在中断里调用也安全。注意
 *         不能像早先的注释那样说「中断里 PRIMASK 本来就是 1」——Cortex-M3
 *         上进 ISR 改的是异常优先级掩码，__get_PRIMASK() 在 ISR 里返回 0。
 *         之所以仍然安全，靠的是保存-恢复本身，不是「中断里它已经是 1」。
 */
uint8_t AppCmd_Push(commandMessage_t message)
{
    uint8_t  queued = 0U;
    uint32_t primask = __get_PRIMASK();
    uint8_t  next;

    __disable_irq();

    /* 先算出新的 head，用「预满」判断：留一个空位区分空/满。 */
    next = (uint8_t)((commandHead + 1U) % COMMAND_QUEUE_SIZE);
    if (next != commandTail) {
        commandQueue[commandHead] = message;
        commandHead = next;
        queued = 1U;
    }

    __set_PRIMASK(primask);

    return queued;
}

/**
 * @brief  构造并写入一条命令的便捷入口。
 * @param  type  命令类型，取值见 app_types.h 的 commandType_t。
 * @param  value 命令参数；无参数的命令传 0.0f。
 * @return 1 表示已入队；0 表示队列已满，本条被丢弃。
 * @note   只是把两个字段打包后转交 AppCmd_Push()。
 */
uint8_t AppCmd_PushValue(commandType_t type, float value)
{
    commandMessage_t message;
    message.type = type;
    message.value = value;
    return AppCmd_Push(message);
}

/**
 * @brief  从队列头部取出一条命令。
 * @param  message 输出参数，成功时写入取出的命令。
 * @return 0 表示成功；-1 表示队列为空（此时不修改 *message）。
 * @note   由 AppSpeedLoop_Task50ms() 在中断里循环调用，直到返回 -1。
 */
int AppCmd_Pop(commandMessage_t *message)
{
    if (commandHead == commandTail) return -1;
    *message = commandQueue[commandTail];
    commandTail = (uint8_t)((commandTail + 1U) % COMMAND_QUEUE_SIZE);
    return 0;
}

/**
 * @brief  查询队列中待处理命令的条数。
 * @return 0 ~ COMMAND_QUEUE_SIZE-1 之间的条目数。
 */
uint8_t AppCmd_Available(void)
{
    return (uint8_t)((commandHead + COMMAND_QUEUE_SIZE - commandTail)
                     % COMMAND_QUEUE_SIZE);
}

/* 20 ms 按键任务 + 命令队列。生产者在中断/主循环，消费者在 50 ms 控制任务，
   队列满时静默丢弃新命令（丢弃比让"停车"被"启动"顶掉安全）。 */

#include "app_cmd.h"
#include "app_config.h"
#include "app_console.h"
#include "app_state.h"
#include "key.h"

/* 命令环形队列。容量 COMMAND_QUEUE_SIZE(8)，实际可用 7 项——留一个空位区分空/满。 */
static commandMessage_t commandQueue[COMMAND_QUEUE_SIZE];

/* 写入位置，由生产者推进。ISR 与主循环都会写，故加 volatile。 */
static volatile uint8_t commandHead = 0;

/* 读取位置，只由消费者（TIM6 中断）推进。 */
static volatile uint8_t commandTail = 0;

static commandType_t appNextMode(void);

/* 20 ms 按键任务（TIM6 中断）：按模式扫描按键产生命令。KEY1 通用切模式；循迹 KEY2 启停/
   KEY3 停/KEY4 切圈数(仅停车)；PWM 循迹 KEY2 切图案/KEY3 启停/KEY4 短按加长按减；PWM 速度
   KEY2 升档/KEY3 启停。各处 (void)Key_CheckSingleOrLong(KEY_4) 用于吃掉短按+长按合成事件，
   否则残留到下次循环误触发。 */
void AppCmd_Task20ms(void)
{
    if (Key_CheckSingleOrLong(KEY_1)) {
        AppCmd_PushValue(appNextMode(), 0.0f);
    }

    if (AppState()->mode == modeTrack) {
        /* 目标圈数仅允许停车时改，运行中改会让计圈判据突变。 */
        if (Key_Check(KEY_4, KEY_SINGLE) && !AppState()->running) {
            AppCmd_PushValue(commandLapCycle, 0.0f);
        }
        (void)Key_CheckSingleOrLong(KEY_4);
        /* KEY2 启停切换：当前状态决定发启动还是停止。 */
        if (Key_Check(KEY_2, KEY_SINGLE)) {
            AppCmd_PushValue(AppState()->running ? commandStop : commandStart, 0.0f);
        }
        if (Key_Check(KEY_3, KEY_SINGLE)) {
            AppCmd_PushValue(commandStop, 0.0f);
        }
        return;
    }

    if (AppState()->mode == modePwmTrack) {
        /* KEY4 短按加、长按减，在线调整开环基础占空比。 */
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

/* 取模式循环里的下一个模式对应的切换命令。顺序 TRACK -> PWM-SPEED -> PWM-TRACK -> TRACK。
   返回命令而非模式本身：切换须经队列由 50 ms 控制任务统一复位/初始化；开机页的轮换走
   AppState_NextMode()，两处顺序须一致。 */
static commandType_t appNextMode(void)
{
    if (AppState()->mode == modeTrack) return commandModePwmSpeed;
    if (AppState()->mode == modePwmSpeed) return commandModePwmTrack;
    return commandModeTrack;
}

/* 清空命令队列：两下标复位为 0 即视为空，不擦数组。中断放行前及开机配置结束时调用。 */
void AppCmd_Init(void)
{
    commandHead = 0;
    commandTail = 0;
}

/* 写入一条命令到队尾。满时丢弃并返回 0（不应常发生，常发生说明生产快于 50 ms 消费）。
   本函数会被 UART 中断与 TIM6 中断交替调用，对 commandHead 的读-改-写整体放进
   保存/恢复 PRIMASK 的临界区，否则两生产者读到同一 head 会偶发丢命令。 */
uint8_t AppCmd_Push(commandMessage_t message)
{
    uint8_t  queued = 0U;
    uint32_t primask = __get_PRIMASK();
    uint8_t  next;

    __disable_irq();

    /* 预满判断：留一个空位区分空/满。 */
    next = (uint8_t)((commandHead + 1U) % COMMAND_QUEUE_SIZE);
    if (next != commandTail) {
        commandQueue[commandHead] = message;
        commandHead = next;
        queued = 1U;
    }

    __set_PRIMASK(primask);

    return queued;
}

/* 构造并写入一条命令。type 取值见 commandType_t；无参数命令 value 传 0.0f。 */
uint8_t AppCmd_PushValue(commandType_t type, float value)
{
    commandMessage_t message;
    message.type = type;
    message.value = value;
    return AppCmd_Push(message);
}

/* 从队头取出一条命令。0 成功（message 写入取出命令），-1 空（不修改 *message）。
   由 AppSpeedLoop_Task50ms() 在中断里循环调用到返回 -1。 */
int AppCmd_Pop(commandMessage_t *message)
{
    if (commandHead == commandTail) return -1;
    *message = commandQueue[commandTail];
    commandTail = (uint8_t)((commandTail + 1U) % COMMAND_QUEUE_SIZE);
    return 0;
}

/* 查询待处理命令条数，0 ~ COMMAND_QUEUE_SIZE-1。 */
uint8_t AppCmd_Available(void)
{
    return (uint8_t)((commandHead + COMMAND_QUEUE_SIZE - commandTail)
                     % COMMAND_QUEUE_SIZE);
}

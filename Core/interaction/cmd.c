/* 20 ms 按键任务 + 命令队列。生产者在中断/主循环，消费者在 5 ms 控制任务，
   队列满时静默丢弃新命令（丢弃比让"停车"被"启动"顶掉安全）。

   键位（四个模式共用 K1 切模式）：
     TRACK     K2 速度档（短按升/长按降） K3 启停 K4 圈数（仅停车）
     PWM-SPEED K2 占空比（短按加/长按减） K3 正转 K4 反转（同向再按停）
     PWM-TRK   K2 图案 K3 启停 K4 基础占空比（短按加/长按减）
     LOOP-TEST K2 程序 K3 启停 K4 档位/子测试 */

#include "cmd.h"
#include "config.h"
#include "console.h"
#include "state.h"
#include "testmode.h"
#include "key.h"
#include "loop_test.h"

/* 命令环形队列。容量 COMMAND_QUEUE_SIZE(8)，实际可用 7 项——留一个空位区分空/满。 */
static commandMessage_t commandQueue[COMMAND_QUEUE_SIZE];

/* 写入位置，由生产者推进。ISR 与主循环都会写，故加 volatile。 */
static volatile uint8_t commandHead = 0;

/* 读取位置，只由消费者（TIM6 中断）推进。 */
static volatile uint8_t commandTail = 0;

static commandType_t next_mode(void);

/* 20 ms 按键任务（TIM6 中断）：按模式扫描按键产生命令。KEY1 通用切模式；循迹 KEY2 调速/
   KEY3 启停/KEY4 切圈数(仅停车)；PWM 循迹 KEY2 切图案/KEY3 启停/KEY4 短按加长按减；PWM 速度
   KEY2 占空比/KEY3 正转/KEY4 反转。各处 (void)key_check_single_or_long(KEY_x) 用于吃掉短按+长按
   合成事件，否则残留到下次循环误触发。 */
void cmd_task(void)
{
    if (key_check_single_or_long(KEY_1)) {
        cmd_push_value(next_mode(), 0.0f);
    }

    if (state_view()->mode == modeTrack) {
        /* KEY2 调速：短按升档、长按降档（G1..G6 = 30/55/80/105/130/170 RPM）。
           运行中也生效：循迹 base_rpm 每拍取自档位，改档即时变速。 */
        if (key_check(KEY_2, KEY_SINGLE)) {
            cmd_push_value(commandGearStep, 1.0f);
        }
        if (key_check(KEY_2, KEY_LONG)) {
            cmd_push_value(commandGearStep, -1.0f);
        }
        (void)key_check_single_or_long(KEY_2);

        /* KEY3 启停切换：当前状态决定发启动还是停止。 */
        if (key_check(KEY_3, KEY_SINGLE)) {
            cmd_push_value(state_view()->running ? commandStop : commandStart, 0.0f);
        }

        /* 目标圈数仅允许停车时改，运行中改会让计圈判据突变。 */
        if (key_check(KEY_4, KEY_SINGLE) && !state_view()->running) {
            cmd_push_value(commandLapCycle, 0.0f);
        }
        (void)key_check_single_or_long(KEY_4);
        return;
    }

    if (state_view()->mode == modePwmTrack) {
        /* KEY4 短按加、长按减，在线调整开环基础占空比。 */
        if (key_check(KEY_4, KEY_SINGLE)) {
            cmd_push_value(commandPwmTrackBaseStep,
                                (float)PWM_TRACK_BASE_STEP_PERCENT);
        }
        if (key_check(KEY_4, KEY_LONG)) {
            cmd_push_value(commandPwmTrackBaseStep,
                                -(float)PWM_TRACK_BASE_STEP_PERCENT);
        }
        (void)key_check_single_or_long(KEY_4);
        if (key_check(KEY_2, KEY_SINGLE)) {
            cmd_push_value(commandPwmTrackNext, 0.0f);
        }
        if (key_check(KEY_3, KEY_SINGLE)) {
            cmd_push_value(state_view()->running ? commandStop : commandStart, 0.0f);
        }
        return;
    }

    if (state_view()->mode == modeLoopTest) {
        if (key_check(KEY_2, KEY_SINGLE)) {
            cmd_push_value(commandLoopTestNextProgram, 0.0f);
        }
        if (key_check(KEY_3, KEY_SINGLE)) {
            cmd_push_value(state_view()->running ? commandStop : commandStart, 0.0f);
        }
        if (key_check(KEY_4, KEY_SINGLE)) {
            cmd_push_value(commandLoopTestNextOption, 0.0f);
        }
        return;
    }

    if (state_view()->mode == modeEight) {
        if (key_check(KEY_2, KEY_SINGLE) && !state_view()->running) {
            cmd_push_value(commandEightNextSpeed, 0.0f);
        }
        (void)key_check_single_or_long(KEY_2);
        if (key_check(KEY_3, KEY_SINGLE)) {
            cmd_push_value(state_view()->running ? commandStop : commandStart, 0.0f);
        }
        (void)key_check_single_or_long(KEY_4);
        return;
    }

    /* 剩下的是 PWM 速度模式：手动开环。KEY3 正转、KEY4 反转（同向再按一次停），
   KEY2 短按加占空比、长按减占空比。方向由 5 ms 任务维护，这里只读它决定发哪条命令。 */
    if (key_check(KEY_2, KEY_SINGLE)) {
        cmd_push_value(commandPwmSpeedDutyStep, (float)PWM_SPEED_DUTY_STEP_PERCENT);
    }
    if (key_check(KEY_2, KEY_LONG)) {
        cmd_push_value(commandPwmSpeedDutyStep, -(float)PWM_SPEED_DUTY_STEP_PERCENT);
    }
    (void)key_check_single_or_long(KEY_2);

    if (key_check(KEY_3, KEY_SINGLE)) {
        cmd_push_value((testmode_get_pwm_speed_dir() > 0) ? commandStop
                                                          : commandPwmSpeedForward, 0.0f);
    }
    if (key_check(KEY_4, KEY_SINGLE)) {
        cmd_push_value((testmode_get_pwm_speed_dir() < 0) ? commandStop
                                                          : commandPwmSpeedReverse, 0.0f);
    }
}

/* 取模式循环里的下一个模式对应的切换命令。
返回命令而非模式本身：切换须经队列由 5 ms 控制任务统一复位/初始化；开机页的轮换走
   state_next_mode()，两处顺序须一致。 */
static commandType_t next_mode(void)
{
    if (state_view()->mode == modeTrack) return commandModePwmSpeed;
    if (state_view()->mode == modePwmSpeed) return commandModePwmTrack;
    if (state_view()->mode == modePwmTrack) return commandModeLoopTest;
    if (state_view()->mode == modeLoopTest) return commandModeEight;
    return commandModeTrack;
}

/* 清空命令队列：两下标复位为 0 即视为空，不擦数组。中断放行前及开机配置结束时调用。 */
void cmd_init(void)
{
    commandHead = 0;
    commandTail = 0;
}

/* 写入一条命令到队尾。满时丢弃并返回 0（不应常发生，常发生说明生产快于 5 ms 消费）。
   本函数会被 UART 中断与 TIM6 中断交替调用，对 commandHead 的读-改-写整体放进
   保存/恢复 PRIMASK 的临界区，否则两生产者读到同一 head 会偶发丢命令。 */
uint8_t cmd_push(commandMessage_t message)
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
uint8_t cmd_push_value(commandType_t type, float value)
{
    commandMessage_t message;
    message.type = type;
    message.value = value;
    return cmd_push(message);
}

/* 从队头取出一条命令。0 成功（message 写入取出命令），-1 空（不修改 *message）。
   由 speed_loop_task() 在中断里循环调用到返回 -1。 */
int cmd_pop(commandMessage_t *message)
{
    if (commandHead == commandTail) return -1;
    *message = commandQueue[commandTail];
    commandTail = (uint8_t)((commandTail + 1U) % COMMAND_QUEUE_SIZE);
    return 0;
}

/* 查询待处理命令条数，0 ~ COMMAND_QUEUE_SIZE-1。 */
uint8_t cmd_available(void)
{
    return (uint8_t)((commandHead + COMMAND_QUEUE_SIZE - commandTail)
                     % COMMAND_QUEUE_SIZE);
}

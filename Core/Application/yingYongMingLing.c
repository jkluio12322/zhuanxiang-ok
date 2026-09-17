#include "yingYongMingLing.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "blueserial.h"
#include "key.h"
#include "pid_tune.h"
#include "timer.h"

#include <stdlib.h>
#include <string.h>

/* 环形命令队列：按键、蓝牙和串口调参只负责入队。 */
static commandMessage_t commandQueue[COMMAND_QUEUE_SIZE];
static uint8_t commandHead = 0;
static uint8_t commandTail = 0;

/**
 * @brief  初始化命令队列。
 * @note   将读写索引同时归零，使队列处于空状态。
 */
void appCommandInit(void)
{
    commandHead = 0;
    commandTail = 0;
}

/**
 * @brief  将控制命令写入环形队列。
 * @param  message 待入队的命令消息。
 * @note   队列满时直接丢弃新命令，保留尚未执行的控制动作。
 */
void appCommandQueuePush(commandMessage_t message)
{
    /* 队列满时丢弃新命令，避免覆盖尚未执行的控制命令。 */
    uint8_t next = (uint8_t)((commandHead + 1U) % COMMAND_QUEUE_SIZE);
    if (next == commandTail) return;
    commandQueue[commandHead] = message;
    commandHead = next;
}

/**
 * @brief  从环形队列读取一条命令。
 * @param  message 用于接收命令的指针。
 * @return 0 表示成功，-1 表示队列为空。
 */
int appCommandQueuePop(commandMessage_t *message)
{
    if (commandHead == commandTail) return -1;
    *message = commandQueue[commandTail];
    commandTail = (uint8_t)((commandTail + 1U) % COMMAND_QUEUE_SIZE);
    return 0;
}

/**
 * @brief  计算当前队列中待执行命令数量。
 * @return 队列中的有效命令条数。
 */
uint8_t appCommandQueueAvailable(void)
{
    return (uint8_t)((commandHead + COMMAND_QUEUE_SIZE - commandTail)
                     % COMMAND_QUEUE_SIZE);
}

/**
 * @brief  解析蓝牙数据帧并生成控制命令。
 * @note   joystick 帧转换为速度、差速、原地转和启动命令；key 帧转换为启停或换档命令。
 *         只有 modeRun 模式响应摇杆，其他模式会忽略摇杆控制。
 */
void appCommandProcessBluetooth(void)
{
    if (!BlueSerial_ReceiveFlag()) return;

    BlueSerial_Receive();
    bluetoothLastRx = milliseconds;
    /* 只有蓝牙/手动模式响应摇杆，其他模式保持摇杆值为 0。 */
    if (workMode != modeRun) {
        bluetoothLv = 0;
        bluetoothRh = 0;
        return;
    }

    commandMessage_t message;

    /* 解析 [joystick,左横,左纵,右横,右纵] 数据帧。 */
    if (strcmp(BlueSerial_StringArray[0], "joystick") == 0) {
        int8_t lv = (int8_t)atoi(BlueSerial_StringArray[2]);
        int8_t rh = (int8_t)atoi(BlueSerial_StringArray[3]);
        bluetoothLv = lv;
        bluetoothRh = rh;

        /* 摇杆回中时下发停车命令，否则下发速度、原地转或差速命令。 */
        if (lv != 0 || rh != 0) {
            if (abs((int)rh) >= BLUETOOTH_PIVOT_THRESHOLD_PERCENT) {
                /*
                 * 右摇杆满舵时忽略纵向输入，按当前档位速度直接一正一反原地转。
                 * 正值右转，负值左转。
                 */
                float pivotSpeed = (rh > 0) ? appStateGetSpeedRpm()
                                            : -appStateGetSpeedRpm();
                message.type = commandPivot;
                message.value = pivotSpeed;
                appCommandQueuePush(message);
            } else {
                /* 未满舵时保持原来的前进速度和普通差速转向。 */
                message.type = commandSpeed;
                message.value = (float)lv / BLUETOOTH_STICK_SCALE
                                * appStateGetSpeedRpm();
                appCommandQueuePush(message);

                message.type = commandTurn;
                message.value = (float)rh / BLUETOOTH_STICK_SCALE
                                * appStateGetSpeedRpm() * BLUETOOTH_TURN_SCALE;
                appCommandQueuePush(message);
            }

            message.type = commandStart;
            message.value = 0.0f;
            appCommandQueuePush(message);
        } else {
            message.type = commandStop;
            message.value = 0.0f;
            appCommandQueuePush(message);
        }
    } else if (strcmp(BlueSerial_StringArray[0], "key") == 0) {
        /* 蓝牙按键帧只处理启动停止和加减档，不直接修改运行状态。 */
        if (strcmp(BlueSerial_StringArray[1], "1") == 0
            && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            message.type = runFlag ? commandStop : commandStart;
            message.value = 0.0f;
            appCommandQueuePush(message);
        } else if (strcmp(BlueSerial_StringArray[1], "2") == 0
                   && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            message.type = commandGearDown;
            message.value = 0.0f;
            appCommandQueuePush(message);
        } else if (strcmp(BlueSerial_StringArray[1], "3") == 0
                   && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            message.type = commandGearUp;
            message.value = 0.0f;
            appCommandQueuePush(message);
        }
    }
}

/**
 * @brief  扫描按键状态并生成对应控制命令。
 * @note   KEY1 负责蓝牙/循迹模式切换，KEY4 负责升档；
 *         KEY2/KEY3 在不同模式下分别执行启停、倒车或 PWM 调整。
 */
void appCommandProcessKeys(void)
{
    commandMessage_t message;

    /* KEY1 在蓝牙/手动模式和循迹模式之间切换，短按和长按均只切换一次。 */
    if (Key_CheckSingleOrLong(KEY_1)) {
        message.type = (workMode == modeTrack) ? commandModeRun
                                               : commandModeTrack;
        message.value = 0.0f;
        appCommandQueuePush(message);
    }

    /* KEY4 始终用于升档。 */
    if (workMode == modeTrack) {
        uint8_t lapKey = Key_Check(KEY_4, KEY_SINGLE);

        (void)Key_CheckSingleOrLong(KEY_4);

        if (lapKey && !runFlag) {
            message.type = commandLapCycle;
            message.value = 0.0f;
            appCommandQueuePush(message);
        }
    } else {
        if (Key_Check(KEY_4, KEY_SINGLE)) {
            message.type = commandGearUp;
            message.value = 0.0f;
            appCommandQueuePush(message);
        }

        if (Key_Check(KEY_4, KEY_LONG)) {
            message.type = commandGearDown;
            message.value = 0.0f;
            appCommandQueuePush(message);
        }
    }

    /* 循迹模式下 KEY2 启停，KEY3 立即停车。 */
    if (workMode == modeTrack) {
        if (Key_Check(KEY_2, KEY_SINGLE)) {
            message.type = runFlag ? commandStop : commandStart;
            message.value = 0.0f;
            appCommandQueuePush(message);
        }
        if (Key_Check(KEY_3, KEY_SINGLE)) {
            message.type = commandStop;
            message.value = 0.0f;
            appCommandQueuePush(message);
        }
    } else if (workMode == modePwmSet) {
        /* PWM 测试模式使用 KEY2/KEY3 按步长增减占空比。 */
        int16_t previous = pwmDuty;
        if (Key_Check(KEY_2, KEY_SINGLE)) {
            pwmDuty += PWM_TEST_STEP_PERCENT;
            if (pwmDuty > PWM_TEST_MAX_PERCENT) pwmDuty = PWM_TEST_MAX_PERCENT;
        }
        if (Key_Check(KEY_3, KEY_SINGLE)) {
            pwmDuty -= PWM_TEST_STEP_PERCENT;
            if (pwmDuty < PWM_TEST_MIN_PERCENT) pwmDuty = PWM_TEST_MIN_PERCENT;
        }
        if (pwmDuty != previous) {
            if (pwmDuty == 0) {
                message.type = commandStop;
                message.value = 0.0f;
            } else {
                message.type = commandStart;
                message.value = 0.0f;
                appCommandQueuePush(message);
                message.type = commandSpeed;
                message.value = (float)pwmDuty;
            }
            appCommandQueuePush(message);
        }
    } else {
        /* 蓝牙/手动模式下 KEY2 前进、KEY3 后退。 */
        uint8_t key2 = Key_Check(KEY_2, KEY_SINGLE);
        uint8_t key3 = Key_Check(KEY_3, KEY_SINGLE);

        if (key2 || key3) {
            float speed = appStateGetSpeedRpm();
            if (key2) {
                if (runFlag
                    && (targetLeftRpm + targetRightRpm) > 0.0f) {
                    message.type = commandStop;
                    message.value = 0.0f;
                } else {
                    message.type = commandStart;
                    message.value = 0.0f;
                    appCommandQueuePush(message);
                    message.type = commandSpeed;
                    message.value = speed;
                }
            } else {
                if (runFlag
                    && (targetLeftRpm + targetRightRpm) < 0.0f) {
                    message.type = commandStop;
                    message.value = 0.0f;
                } else {
                    message.type = commandStart;
                    message.value = 0.0f;
                    appCommandQueuePush(message);
                    message.type = commandSpeed;
                    message.value = -speed;
                }
            }
            appCommandQueuePush(message);
            message.type = commandTurn;
            message.value = 0.0f;
            appCommandQueuePush(message);
            bluetoothLastRx = milliseconds;
        }
    }
}

/**
 * @brief  处理 USART1 在线调参输入。
 * @note   解析工作在主循环执行，电机控制类命令仍通过统一命令队列延迟到控制周期执行。
 */
void appCommandProcessTune(void)
{
    /* 串口调参命令在主循环中解析，具体控制命令进入同一命令队列。 */
    pidTuneProcess();
}

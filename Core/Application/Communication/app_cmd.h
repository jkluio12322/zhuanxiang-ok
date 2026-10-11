/**
 * @file    app_cmd.h
 * @brief   命令队列接口 — 按键与命令行产生命令，50 ms 控制任务统一执行。
 *
 * @note    队列本身是单消费者的环形缓冲区，容量 COMMAND_QUEUE_SIZE，
 *          满时丢弃新命令。并发约束见 app_cmd.c 的文件头。
 */

#ifndef __APP_CMD_H
#define __APP_CMD_H

#include "app_types.h"

/**
 * @brief  清空命令队列。
 * @note   在中断放行之前调用；开机配置流程结束时也会再清一次。
 */
void AppCmd_Init(void);

/**
 * @brief  构造并写入一条命令。
 * @param  type  命令类型，见 commandType_t。
 * @param  value 命令参数，无参数的命令传 0.0f。
 * @return 1 表示已入队；0 表示队列已满，本条被丢弃。
 */
uint8_t AppCmd_PushValue(commandType_t type, float value);

/**
 * @brief  写入一条已经构造好的命令。
 * @param  message 待入队的命令。
 * @return 1 表示已入队；0 表示队列已满，本条被丢弃。
 */
uint8_t AppCmd_Push(commandMessage_t message);

/**
 * @brief  取出一条命令。
 * @param  message 输出参数，成功时写入取出的命令。
 * @return 0 成功，-1 队列为空。
 */
int AppCmd_Pop(commandMessage_t *message);

/**
 * @brief  查询待处理命令条数。
 * @return 0 ~ COMMAND_QUEUE_SIZE-1。
 */
uint8_t AppCmd_Available(void);

/**
 * @brief  20 ms 按键任务，扫描按键并产生命令。
 * @note   在 TIM6 中断里执行。
 */
void AppCmd_Task20ms(void);

#endif /* __APP_CMD_H */

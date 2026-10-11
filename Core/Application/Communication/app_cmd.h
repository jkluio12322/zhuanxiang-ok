/* 命令队列接口：按键与命令行产生命令，50 ms 控制任务统一执行。
   单消费者环形缓冲，容量 COMMAND_QUEUE_SIZE，满时丢弃新命令。并发约束见 app_cmd.c。 */
#ifndef __APP_CMD_H
#define __APP_CMD_H

#include "app_types.h"

/* 清空命令队列（中断放行前调用；开机配置流程结束时再清一次）。 */
void AppCmd_Init(void);

/* 构造并写入一条命令。type 见 commandType_t；无参数命令 value 传 0.0f。返回 1 已入队 / 0 队列满丢弃。 */
uint8_t AppCmd_PushValue(commandType_t type, float value);

/* 写入一条已构造好的命令。返回 1 已入队 / 0 队列满丢弃。 */
uint8_t AppCmd_Push(commandMessage_t message);

/* 取出一条命令。0 成功（message 写入取出命令），-1 队列为空。 */
int AppCmd_Pop(commandMessage_t *message);

/* 查询待处理命令条数，0 ~ COMMAND_QUEUE_SIZE-1。 */
uint8_t AppCmd_Available(void);

/* 20 ms 按键任务，扫描按键并产生命令（TIM6 中断里执行）。 */
void AppCmd_Task20ms(void);

#endif /* __APP_CMD_H */

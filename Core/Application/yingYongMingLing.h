/**
 * @file    yingYongMingLing.h
 * @brief   应用层命令队列、蓝牙解析和按键事件处理接口。
 */
#ifndef __YING_YONG_MING_LING_H
#define __YING_YONG_MING_LING_H  /* 头文件保护宏，防止接口重复声明。 */

#include "yingYongLeiXing.h"

/**
 * @brief  初始化命令队列。
 * @note   清空读写索引，使队列进入空状态。
 */
void appCommandInit(void);

/**
 * @brief  将一条命令压入命令队列。
 * @param  message 待入队的命令消息。
 * @note   队列满时丢弃新命令，避免覆盖尚未执行的控制命令。
 */
void appCommandQueuePush(commandMessage_t message);

/**
 * @brief  从命令队列取出一条命令。
 * @param  message 用于接收命令消息的指针。
 * @return 0 表示成功，-1 表示队列为空。
 */
int appCommandQueuePop(commandMessage_t *message);

/**
 * @brief  获取当前队列中的命令数量。
 * @return 尚未执行的有效命令条数。
 */
uint8_t appCommandQueueAvailable(void);

/**
 * @brief  解析并处理蓝牙接收帧。
 * @note   摇杆帧转换为速度、差速、原地转和启动命令；按键帧转换为启动停止或换档命令。
 */
void appCommandProcessBluetooth(void);

/**
 * @brief  扫描四个按键并生成对应控制命令。
 * @note   KEY1 切换模式，KEY2/KEY3 根据模式执行启停、倒车或 PWM 调整，
 *         KEY4 用于升档。
 */
void appCommandProcessKeys(void);

/**
 * @brief  处理 USART1 在线调参命令。
 * @note   实际解析由 pidTuneProcess 完成；运动控制类命令通过统一队列执行，
 *         PID 增益与 duty 调试命令直接修改运行参数。
 */
void appCommandProcessTune(void);

#endif

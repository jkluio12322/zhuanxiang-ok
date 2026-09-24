/**
 * @file    pid_tune.h
 * @brief   USART1 在线 PID 调参和电机测试接口。
 * @note    UART 中断只缓存接收字节，命令行解析在主循环中完成。
 */
#ifndef __PID_TUNE_H
#define __PID_TUNE_H  /* 头文件保护宏，防止接口重复声明。 */

#include "main.h"

/* USART1 单字节接收缓存，由 HAL_UART_Receive_IT 使用。 */
extern volatile uint8_t pidTuneRxByte;

/**
 * @brief  初始化在线调参模块并启动 USART1 单字节中断接收。
 */
void pidTuneInit(void);

/**
 * @brief  缓存 USART1 接收到的单个字节。
 * @param  byte 当前接收字节。
 * @note   仅可在 UART 中断上下文中调用；收到换行时标记一整行命令就绪。
 */
void pidTuneRxCallback(uint8_t byte);

/**
 * @brief  解析并执行一行在线调参命令。
 * @note   该函数在主循环中调用，电机控制类命令会写入统一命令队列。
 */
void pidTuneProcess(void);

/**
 * @brief  发送已经准备好的调参命令响应。
 * @note   响应发送期间短暂关闭中断，防止字符串缓冲区被并发修改。
 */
void pidTuneProcessResponse(void);

#endif

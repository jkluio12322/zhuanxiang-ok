/**
 * @file    app_console.h
 * @brief   USART1 在线 PID 调参和电机测试接口。
 * @note    UART 中断只缓存接收字节，命令行解析在主循环中完成。
 */
#ifndef __APP_CONSOLE_H
#define __APP_CONSOLE_H  /* 头文件保护宏，防止接口重复声明。 */

#include "main.h"

/* 原注释已过时：USART1 的接收缓存 serialRxByte 与中断装填已在阶段 5 移到
   app_serial.c（那里才有 HAL_UART_RxCpltCallback）。本模块只负责把收到的
   字节拼成一行并解析。 */

/**
 * @brief  初始化在线调参模块。
 * @note   不启动 UART 接收——接收中断由 app_serial.c 的 Serial_Init() 启动，
 *         由 HAL_UART_RxCpltCallback 转交给 AppConsole_RxByte()。
 */
void AppConsole_Init(void);

/**
 * @brief  缓存 USART1 接收到的单个字节。
 * @param  byte 当前接收字节。
 * @note   仅可在 UART 中断上下文中调用；收到换行时标记一整行命令就绪。
 */
void AppConsole_RxByte(uint8_t byte);

/**
 * @brief  解析并执行一行在线调参命令。
 * @note   该函数在主循环中调用，电机控制类命令会写入统一命令队列。
 */
void AppConsole_Process(void);

/**
 * @brief  发送已经准备好的调参命令响应。
 * @note   响应发送期间短暂关闭中断，防止字符串缓冲区被并发修改。
 */
void AppConsole_Flush(void);

#endif

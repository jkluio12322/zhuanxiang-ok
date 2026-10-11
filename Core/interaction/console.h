/* USART1 串口命令接口（台架手动转速、状态查询、VOFA 开关）。UART 中断只缓存接收字节，
   命令行解析在主循环完成。 */
#ifndef __CONSOLE_H
#define __CONSOLE_H  /* 头文件保护宏，防止接口重复声明。 */

#include "main.h"

/* 初始化命令行模块。不启动 UART 接收——接收中断由 app_serial.c 的 serial_init() 启动，
   由 HAL_UART_RxCpltCallback 转交给 console_rx_byte()。 */
void console_init(void);

/* 缓存 USART1 接收到的单个字节（仅 UART 中断上下文调用）；收到换行时标记整行就绪。 */
void console_rx_byte(uint8_t byte);

/* 解析并执行一行串口命令（主循环调用）；电机控制类命令写入统一命令队列。 */
void console_process(void);

/* 把已准备好的命令响应交给发送缓冲（主循环调用）。 */
void console_flush(void);

#endif

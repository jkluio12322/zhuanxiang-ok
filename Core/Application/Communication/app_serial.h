/* 非阻塞串口模块：printf/fputc -> Serial_Write -> RingBuf(1024B) -> HAL UART IT 缓冲(256B)
   -> TX Complete ISR 自动接力。多生产者靠短临界区串行化，完整消息不交错。 */
#ifndef __APP_SERIAL_H
#define __APP_SERIAL_H  /* 头文件保护宏，防止串口接口重复声明。 */
#include "main.h"
#include <stdio.h>

/* 初始化非阻塞串口发送模块。huart 为使用的 UART 句柄。 */
void Serial_Init(UART_HandleTypeDef *huart);

/* 批量写入发送环形缓冲。返回实际写入字节数，空间不足时可能小于 len。 */
int  Serial_Write(const uint8_t *data, uint16_t len);

/* 非阻塞发送一个字节。缓冲满时静默丢弃。 */
void Serial_SendByte(uint8_t Byte);

/* 非阻塞发送字节数组。 */
void Serial_SendArray(uint8_t *Array, uint16_t Length);

/* 非阻塞发送字符串（'\0' 结尾）。空间不足时发送已缓存部分。 */
void Serial_SendString(char *String);

/* 发送固定长度十进制整数。 */
void Serial_SendNumber(uint32_t Number, uint8_t Length);

/* 非阻塞格式化输出。 */
void Serial_Printf(char *format, ...);

/* 处理 UART 发送完成事件并启动下一批数据（由 UART 发送完成中断调用）。 */
void Serial_TX_Done(void);

#endif

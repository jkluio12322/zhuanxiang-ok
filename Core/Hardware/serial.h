/**
 * @file    serial.h
 * @brief   非阻塞串口模块 — printf / RingBuf / IT TX (通用可移植)
 *
 *          架构: printf/fputc -> Serial_Write -> RingBuf(1024B)
 *                -> HAL UART IT buffer(256B) -> TX Complete ISR 自动接力
 *          多个生产者通过短临界区串行化，完整消息不会在环形缓冲区中交错。
 *
 *          移植到新项目只需:
 *             1. main.c: Serial_Init(&你的UART句柄)
 *             2. 在你的 HAL_UART_TxCpltCallback 中调用 Serial_TX_Done()
 *             3. 确保 USART_IRQHandler 调用了 HAL_UART_IRQHandler
 */

#ifndef __BSP_SERIAL_H
#define __BSP_SERIAL_H  /* 头文件保护宏，防止串口接口重复声明。 */
#include "main.h"
#include <stdio.h>

/* ── API ──────────────────────────────────── */
/**
 * @brief  初始化非阻塞串口发送模块。
 * @param  huart 使用的 UART 句柄。
 */
void Serial_Init(UART_HandleTypeDef *huart);

/**
 * @brief  非阻塞发送一个字节。
 * @param  Byte 待发送字节。
 * @note   发送缓冲区满时静默丢弃。
 */
void Serial_SendByte(uint8_t Byte);

/**
 * @brief  非阻塞发送字节数组。
 * @param  Array 数据指针。
 * @param  Length 数据长度。
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length);

/**
 * @brief  非阻塞发送字符串。
 * @param  String 以 '\0' 结尾的字符串。
 * @note   空间不足时发送已缓存部分。
 */
void Serial_SendString(char *String);

/**
 * @brief  发送固定长度十进制整数。
 * @param  Number 待发送整数。
 * @param  Length 输出位数。
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length);

/**
 * @brief  非阻塞格式化输出。
 * @param  format printf 风格格式字符串。
 * @param  ... 可变参数列表。
 */
void Serial_Printf(char *format, ...);

/**
 * @brief  将批量数据写入发送环形缓冲区。
 * @param  data 数据指针。
 * @param  len 数据长度。
 * @return 实际写入的字节数，空间不足时可能小于 len。
 */
int  Serial_Write(const uint8_t *data, uint16_t len);

/**
 * @brief  处理 UART 发送完成事件并启动下一批数据。
 * @note   该函数由 UART 发送完成中断调用。
 */
void Serial_TX_Done(void);

#endif

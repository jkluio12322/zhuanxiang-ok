/**
 * @file    bytestream.h
 * @brief   通用字节流输出接口。
 */

#ifndef __BYTE_STREAM_H
#define __BYTE_STREAM_H  /* 头文件保护宏，防止字节流接口重复声明。 */

#include "main.h"
#include <stdarg.h>

/**
 * @brief  字节流底层写入回调类型。
 * @param  data 待发送数据指针。
 * @param  length 待发送字节数。
 * @return 实际写入的字节数。
 */
typedef uint16_t (*ByteStreamWriteFn)(const uint8_t *data,
                                      uint16_t length);

/**
 * @brief  通用字节流对象。
 * @note   write 由具体通信模块提供，串口适配非阻塞缓冲，
 *         蓝牙适配阻塞发送。
 */
typedef struct {
    ByteStreamWriteFn write;
} ByteStream_t;

/**
 * @brief  调用底层写回调发送一段数据。
 * @param  stream 字节流对象指针。
 * @param  data 待发送数据指针。
 * @param  length 待发送字节数。
 * @return 实际写入的字节数。
 * @note   空对象、空指针或零长度请求直接返回 0。
 */
uint16_t ByteStream_Write(ByteStream_t *stream,
                          const uint8_t *data,
                          uint16_t length);

/**
 * @brief  发送单个字节。
 * @param  stream 字节流对象指针。
 * @param  byte 待发送字节。
 */
void ByteStream_SendByte(ByteStream_t *stream, uint8_t byte);

/**
 * @brief  发送字节数组。
 * @param  stream 字节流对象指针。
 * @param  data 数据指针。
 * @param  length 数据长度，单位字节。
 */
void ByteStream_SendArray(ByteStream_t *stream,
                          const uint8_t *data,
                          uint16_t length);

/**
 * @brief  发送以 '\0' 结尾的字符串。
 * @param  stream 字节流对象指针。
 * @param  string 待发送字符串。
 * @note   长度超过 0xFFFF 时截断。
 */
void ByteStream_SendString(ByteStream_t *stream, const char *string);

/**
 * @brief  发送固定长度的十进制无符号整数。
 * @param  stream 字节流对象指针。
 * @param  number 待发送整数。
 * @param  length 固定输出位数。
 */
void ByteStream_SendNumber(ByteStream_t *stream,
                           uint32_t number,
                           uint8_t length);

/**
 * @brief  使用 va_list 进行有界格式化输出。
 * @param  stream 字节流对象指针。
 * @param  format printf 风格格式字符串。
 * @param  arguments 可变参数列表。
 */
void ByteStream_VPrintf(ByteStream_t *stream,
                        const char *format,
                        va_list arguments);

/**
 * @brief  使用可变参数进行有界格式化输出。
 * @param  stream 字节流对象指针。
 * @param  format printf 风格格式字符串。
 * @param  ... 可变参数。
 * @note   内部使用固定 128 字节缓冲区。
 */
void ByteStream_Printf(ByteStream_t *stream, const char *format, ...);

#endif

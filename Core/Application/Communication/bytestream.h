/* 通用字节流输出接口。 */

#ifndef __BYTE_STREAM_H
#define __BYTE_STREAM_H  /* 头文件保护宏，防止字节流接口重复声明。 */

#include "main.h"
#include <stdarg.h>

/* 字节流底层写入回调类型。返回实际写入字节数。 */
typedef uint16_t (*ByteStreamWriteFn)(const uint8_t *data,
                                      uint16_t length);

/* 通用字节流对象。write 由具体通信模块提供：串口适配非阻塞缓冲，蓝牙适配阻塞发送。 */
typedef struct {
    ByteStreamWriteFn write;
} ByteStream_t;

/* 调用底层写回调发送一段数据。空对象/空指针/零长度直接返回 0。 */
uint16_t ByteStream_Write(ByteStream_t *stream,
                          const uint8_t *data,
                          uint16_t length);

/* 发送单个字节。 */
void ByteStream_SendByte(ByteStream_t *stream, uint8_t byte);

/* 发送字节数组。 */
void ByteStream_SendArray(ByteStream_t *stream,
                          const uint8_t *data,
                          uint16_t length);

/* 发送以 '\0' 结尾的字符串。长度超 0xFFFF 截断。 */
void ByteStream_SendString(ByteStream_t *stream, const char *string);

/* 发送固定长度的十进制无符号整数。 */
void ByteStream_SendNumber(ByteStream_t *stream,
                           uint32_t number,
                           uint8_t length);

/* 使用 va_list 进行有界格式化输出（128 字节栈缓冲，超长由 vsnprintf 截断）。 */
void ByteStream_VPrintf(ByteStream_t *stream,
                        const char *format,
                        va_list arguments);

/* 使用可变参数进行有界格式化输出。 */
void ByteStream_Printf(ByteStream_t *stream, const char *format, ...);

#endif

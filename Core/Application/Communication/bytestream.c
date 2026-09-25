/**
 * @file    bytestream.c
 * @brief   通用字节流输出实现。
 */

#include "bytestream.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* 字节流格式化输出临时缓冲区大小，单位字节。 */
#define BYTE_STREAM_PRINTF_BUFFER_SIZE 128U

/**
 * @brief  调用底层写回调发送一段数据。
 * @param  stream 字节流对象指针。
 * @param  data 待发送数据指针。
 * @param  length 待发送字节数。
 * @return 实际写入的字节数。
 */
uint16_t ByteStream_Write(ByteStream_t *stream,
                          const uint8_t *data,
                          uint16_t length)
{
    if (stream == NULL || stream->write == NULL
        || data == NULL || length == 0U) {
        return 0U;
    }

    return stream->write(data, length);
}

/**
 * @brief  发送单个字节。
 * @param  stream 字节流对象指针。
 * @param  byte 待发送字节。
 */
void ByteStream_SendByte(ByteStream_t *stream, uint8_t byte)
{
    (void)ByteStream_Write(stream, &byte, 1U);
}

/**
 * @brief  发送字节数组。
 * @param  stream 字节流对象指针。
 * @param  data 数据指针。
 * @param  length 数据长度，单位字节。
 */
void ByteStream_SendArray(ByteStream_t *stream,
                          const uint8_t *data,
                          uint16_t length)
{
    (void)ByteStream_Write(stream, data, length);
}

/**
 * @brief  发送以 '\0' 结尾的字符串。
 * @param  stream 字节流对象指针。
 * @param  string 待发送字符串。
 */
void ByteStream_SendString(ByteStream_t *stream, const char *string)
{
    size_t length;

    if (string == NULL) {
        return;
    }

    length = strlen(string);
    if (length > 0xFFFFU) {
        length = 0xFFFFU;
    }

    (void)ByteStream_Write(stream,
                           (const uint8_t *)string,
                           (uint16_t)length);
}

/**
 * @brief  发送固定长度的十进制无符号整数。
 * @param  stream 字节流对象指针。
 * @param  number 待发送整数。
 * @param  length 固定输出位数。
 */
void ByteStream_SendNumber(ByteStream_t *stream,
                           uint32_t number,
                           uint8_t length)
{
    uint32_t divisor = 1U;

    if (length == 0U) {
        return;
    }

    for (uint8_t i = 1U; i < length; i++) {
        divisor *= 10U;
    }

    for (uint8_t i = 0U; i < length; i++) {
        ByteStream_SendByte(stream, (uint8_t)(number / divisor % 10U + '0'));
        divisor /= 10U;
    }
}

/**
 * @brief  使用 va_list 进行有界格式化输出。
 * @param  stream 字节流对象指针。
 * @param  format printf 风格格式字符串。
 * @param  arguments 可变参数列表。
 * @note   固定使用 128 字节栈缓冲区，超长内容由 vsnprintf 截断。
 */
void ByteStream_VPrintf(ByteStream_t *stream,
                        const char *format,
                        va_list arguments)
{
    char string[BYTE_STREAM_PRINTF_BUFFER_SIZE];

    if (format == NULL) {
        return;
    }

    (void)vsnprintf(string, sizeof(string), format, arguments);
    ByteStream_SendString(stream, string);
}

/**
 * @brief  使用可变参数进行有界格式化输出。
 * @param  stream 字节流对象指针。
 * @param  format printf 风格格式字符串。
 * @param  ... 可变参数。
 */
void ByteStream_Printf(ByteStream_t *stream, const char *format, ...)
{
    va_list arguments;

    va_start(arguments, format);
    ByteStream_VPrintf(stream, format, arguments);
    va_end(arguments);
}

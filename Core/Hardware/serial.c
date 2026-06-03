/**
 * @file    serial.c
 * @brief   UART 串口调试/VOFA+ 通信模块
 * @note    使用 USART3 (PD8=TX, PD9=RX), 115200bps
 *          通过 fputc 重定向 printf 到串口
 *          VOFA+ FireWater 协议: CSV 格式数据帧
 */

#include "serial.h"
#include <stdarg.h>

extern UART_HandleTypeDef huart3;

/**
 * @brief  串口初始化 (USART3 已在 CubeMX 中配置完成)
 */
void Serial_Init(void)
{
    /* CubeMX 已配置 USART3, 此处预留扩展 */
}

/**
 * @brief  通过串口发送一个字节
 * @param  Byte 待发送字节
 */
void Serial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart3, &Byte, 1, 10);
}

/**
 * @brief  通过串口发送字节数组
 * @param  Array  数据缓冲区
 * @param  Length 发送长度
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit(&huart3, Array, Length, 1000);
}

/**
 * @brief  通过串口发送字符串 (以 '\0' 结尾)
 * @param  String 字符串指针
 */
void Serial_SendString(char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++)
        Serial_SendByte(String[i]);
}

/**
 * @brief  整数的幂运算 (用于数字转字符串)
 * @param  X 底数
 * @param  Y 指数
 * @return X 的 Y 次方
 */
static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

/**
 * @brief  通过串口发送无符号整数 (按指定位数, 高位补0)
 * @param  Number 待发送数值
 * @param  Length 显示位数
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
}

/**
 * @brief  printf 重定向: 将 stdout 输出重定向到串口
 * @param  ch  待输出字符
 * @param  f   文件指针 (未使用)
 * @return 输出的字符
 */
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

/**
 * @brief  格式化串口打印 (类似 printf)
 * @param  format 格式化字符串
 * @param  ...    可变参数列表
 * @note   使用 vsprintf 格式化到 100 字节缓冲区, 注意溢出风险
 */
void Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}

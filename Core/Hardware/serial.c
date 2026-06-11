/**
 * @file    serial.c
 * @brief   UART 串口调试/VOFA+ 通信模块 (非阻塞 TX, 中断环形缓冲区)
 * @note    使用 USART3 (PD8=TX, PD9=RX), 115200bps
 *          TX 通过 USART3 TXE 中断 + 环形缓冲区发送, 不阻塞 HAL RX 中断
 *          fputc 重定向 printf 到串口
 *          VOFA+ FireWater 协议: CSV 格式数据帧
 */

#include "serial.h"
#include <stdarg.h>

extern UART_HandleTypeDef huart3;

/* TX 环形缓冲区 (中断发送, 不占 HAL 状态机) */
#define TX_BUF_SIZE  256
static uint8_t  tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;

/**
 * @brief  USART3 TXE 中断处理: 从环形缓冲区取字节写入 DR
 * @note   在 USART3_IRQHandler 中 HAL_UART_IRQHandler 之后调用
 */
void Serial_TXE_IRQ(void)
{
    if (USART3->SR & USART_SR_TXE) {
        if (tx_head != tx_tail) {
            USART3->DR = tx_buf[tx_tail];
            tx_tail = (tx_tail + 1) % TX_BUF_SIZE;
        } else {
            USART3->CR1 &= ~USART_CR1_TXEIE;   /* 发完, 关 TXE 中断 */
        }
    }
}

/**
 * @brief  串口初始化
 */
void Serial_Init(void)
{
    /* CubeMX 已配置 USART3, 此处预留扩展 */
}

/**
 * @brief  通过串口发送一个字节 (非阻塞, 写入环形缓冲区)
 */
void Serial_SendByte(uint8_t Byte)
{
    uint16_t next = (tx_head + 1) % TX_BUF_SIZE;
    while (next == tx_tail);   /* 缓冲区满则等待 (256 字节够大, 不应触发) */
    tx_buf[tx_head] = Byte;
    tx_head = next;
    USART3->CR1 |= USART_CR1_TXEIE;   /* 启动 TXE 中断发送 */
}

/**
 * @brief  通过串口发送字节数组 (非阻塞)
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++) {
        uint16_t next = (tx_head + 1) % TX_BUF_SIZE;
        while (next == tx_tail);
        tx_buf[tx_head] = Array[i];
        tx_head = next;
    }
    USART3->CR1 |= USART_CR1_TXEIE;
}

/**
 * @brief  通过串口发送字符串
 */
void Serial_SendString(char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++)
        Serial_SendByte(String[i]);
}

/**
 * @brief  整数的幂运算
 */
static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

/**
 * @brief  通过串口发送无符号整数
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
}

/**
 * @brief  printf 重定向
 */
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

/**
 * @brief  格式化串口打印
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

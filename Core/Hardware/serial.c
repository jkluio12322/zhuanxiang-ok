/**
 * @file    serial.c
 * @brief   UART3 串口调试/VOFA+ 通信模块 (DMA TX, 非阻塞)
 * @note    USART3: PD8=TX, PD9=RX, 115200bps
 *
 *          ┌──────────┐     ┌──────────┐     ┌────────────┐
 *          │ fputc /   │────→│ TX Ring  │────→│ DMA buffer │──→ USART3 TX
 *          │ Printf    │     │ (1024B)  │     │ (256B)     │
 *          └──────────┘     └──────────┘     └────────────┘
 *                               ↑                  ↑
 *                          非阻塞写入        DMA Complete ISR
 *                          (满则丢弃)        自动接力下一块
 *
 *          优先级: 速度 > 完整 — 若TX环满则静默丢弃, 保证控制循环不阻塞
 */

#include "serial.h"
#include "ringbuf.h"
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;

/* TX 环形缓冲区 (生产者: 主循环/任务, 消费者: DMA Complete ISR) */
#define TX_RING_SIZE  1024
static uint8_t tx_ring_buf[TX_RING_SIZE];
static RingBuf  tx_ring;

/* DMA 线性发送缓冲 */
#define DMA_BUF_SIZE  256
static uint8_t  dma_buf[DMA_BUF_SIZE];
static volatile uint8_t dma_tx_busy = 0;

/* ---- 内部: 从 ring buf 取数据启动 DMA ---- */
static void Serial_StartDMA(void)
{
    if (dma_tx_busy) return;

    uint16_t avail = RingBuf_Available(&tx_ring);
    if (avail == 0) return;

    uint16_t n = (avail < DMA_BUF_SIZE) ? avail : DMA_BUF_SIZE;
    for (uint16_t i = 0; i < n; i++) {
        RingBuf_Get(&tx_ring, &dma_buf[i]);
    }

    dma_tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart3, dma_buf, n);
}

/**
 * @brief  串口初始化: 初始化 TX 环形缓冲区
 */
void Serial_Init(void)
{
    RingBuf_Init(&tx_ring, tx_ring_buf, TX_RING_SIZE);
    dma_tx_busy = 0;
}

/**
 * @brief  批量写入数据到 TX 环形缓冲区 (非阻塞)
 * @param  data 数据指针
 * @param  len  字节数
 * @return 实际写入字节数, -1 表示缓冲区满
 */
int Serial_Write(const uint8_t *data, uint16_t len)
{
    uint16_t written = 0;
    for (uint16_t i = 0; i < len; i++) {
        if (RingBuf_PutNB(&tx_ring, data[i]) < 0) break;
        written++;
    }
    Serial_StartDMA();
    return written;
}

/**
 * @brief  发送单字节 (非阻塞, 写入 ring buf)
 */
void Serial_SendByte(uint8_t Byte)
{
    RingBuf_PutNB(&tx_ring, Byte);
    Serial_StartDMA();
}

/**
 * @brief  发送字节数组 (非阻塞)
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    Serial_Write(Array, Length);
}

/**
 * @brief  发送字符串 (非阻塞)
 */
void Serial_SendString(char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++)
        if (RingBuf_PutNB(&tx_ring, (uint8_t)String[i]) < 0) break;
    Serial_StartDMA();
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
 * @brief  发送无符号整数
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
}

/**
 * @brief  printf 重定向 → 非阻塞 Serial_Write
 */
int fputc(int ch, FILE *f)
{
    Serial_SendByte((uint8_t)ch);
    return ch;
}

/**
 * @brief  格式化串口打印 (vsnprintf 防栈溢出)
 */
void Serial_Printf(char *format, ...)
{
    char String[128];
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    Serial_SendString(String);
}

/**
 * @brief  DMA TX 完成回调 (从 USART3 DMA TX ISR 调用)
 * @note   检查 ring buf 中是否有待发数据, 有则接力启动下一次 DMA
 */
void Serial_DMA_TX_Complete(void)
{
    dma_tx_busy = 0;
    Serial_StartDMA();
}

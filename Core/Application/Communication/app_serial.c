/* USART1 串口调试/VOFA 通信：fputc/printf -> TX 环形缓冲(1024B) -> HAL IT 缓冲(256B)
   -> TX Complete ISR 自动接力下一块。生产者主循环/任务，消费者发送完成中断，多生产者靠
   PRIMASK 临界区串行化；环满静默丢弃，保证控制循环不被串口阻塞。USART1: PA9=TX PA10=RX
   115200bps。 */

#include "app_serial.h"
#include "app_console.h"
#include "bytestream.h"
#include "ringbuf.h"
#include <stdarg.h>

static UART_HandleTypeDef *serial_uart = NULL;

/* 单字节接收中断的落点，由 HAL_UART_RxCpltCallback 转交给命令行模块。 */
static uint8_t serialRxByte = 0U;

/* 串口发送环形缓冲区容量，单位字节。 */
#define TX_RING_SIZE  1024
static uint8_t tx_ring_buf[TX_RING_SIZE];
static RingBuf  tx_ring;

/* 单次 HAL UART 中断发送使用的线性缓冲区容量，单位字节。 */
#define TX_LINEAR_BUF_SIZE  256
static uint8_t  txLinearBuf[TX_LINEAR_BUF_SIZE];
static volatile uint8_t txBusy = 0;

/* USART1 通用字节流适配器。 */
static ByteStream_t serialByteStream;

/* 将通用字节流写入 USART1 非阻塞环形发送缓冲。 */
static uint16_t Serial_ByteStreamWrite(const uint8_t *data, uint16_t length)
{
    int written = Serial_Write(data, length);

    return (written > 0) ? (uint16_t)written : 0U;
}

/* 关中断并返回进入临界区前的 PRIMASK 状态。 */
static uint32_t Serial_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/* 按保存的 PRIMASK 状态恢复中断（原为 0 才重新开）。 */
static void Serial_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* 从发送环形缓冲取一批数据并启动下一次中断发送。当前发送未完成时直接返回；
   调用方须持串口临界区，保证环形缓冲读取与发送状态更新不被并发修改。 */
static void Serial_StartTransmit(void)
{
    if (txBusy) return;

    uint16_t avail = RingBuf_Available(&tx_ring);
    if (avail == 0) return;

    uint16_t n = (avail < TX_LINEAR_BUF_SIZE)
                 ? avail
                 : TX_LINEAR_BUF_SIZE;
    for (uint16_t i = 0; i < n; i++) {
        RingBuf_Get(&tx_ring, &txLinearBuf[i]);
    }

    txBusy = 1;
    HAL_UART_Transmit_IT(serial_uart, txLinearBuf, n);
}

/* 初始化 TX 环形缓冲并启动单字节接收中断。 */
void Serial_Init(UART_HandleTypeDef *huart)
{
    serial_uart = huart;
    serialByteStream.write = Serial_ByteStreamWrite;
    RingBuf_Init(&tx_ring, tx_ring_buf, TX_RING_SIZE);
    txBusy = 0;

    /* 中断里只缓存一个字节，解析在主循环里做。 */
    HAL_UART_Receive_IT(serial_uart, &serialRxByte, 1);
}

/* UART 接收完成回调（中断上下文）。本工程只用 USART1；USART2 保留给蓝牙未使用。
   只转交字节并重新装填，不做解析。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        AppConsole_RxByte(serialRxByte);
        HAL_UART_Receive_IT(huart, &serialRxByte, 1);
    }
}

/* UART 发送完成回调（中断上下文）：TX 完成后接力发送环形缓冲剩余数据。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Serial_TX_Done();
    }
}

/* 批量写入数据到 TX 环形缓冲（非阻塞）。返回实际写入字节数 0~len，空间不足时返回已写入数。 */
int Serial_Write(const uint8_t *data, uint16_t len)
{
    uint16_t written = 0;
    uint32_t primask = Serial_EnterCritical();

    for (uint16_t i = 0; i < len; i++) {
        if (RingBuf_PutNB(&tx_ring, data[i]) < 0) break;
        written++;
    }
    Serial_StartTransmit();
    Serial_ExitCritical(primask);
    return written;
}

/* 发送单字节（非阻塞，写环形缓冲）。缓冲满时静默丢弃。 */
void Serial_SendByte(uint8_t Byte)
{
    ByteStream_SendByte(&serialByteStream, Byte);
}

/* 发送字节数组（非阻塞）。 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    ByteStream_SendArray(&serialByteStream, Array, Length);
}

/* 发送字符串（非阻塞）。环形缓冲满时在当前位置停止，剩余字符不发送。 */
void Serial_SendString(char *String)
{
    ByteStream_SendString(&serialByteStream, String);
}

/* 发送无符号整数（固定位数）。 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    ByteStream_SendNumber(&serialByteStream, Number, Length);
}

/* 格式化串口打印（经 ByteStream 用 128 字节有界缓冲）。 */
void Serial_Printf(char *format, ...)
{
    va_list arg;

    va_start(arg, format);
    ByteStream_VPrintf(&serialByteStream, format, arg);
    va_end(arg);
}

/* printf 重定向 -> 非阻塞 Serial_Write。 */
int fputc(int ch, FILE *f)
{
    (void)f;
    Serial_SendByte((uint8_t)ch);
    return ch;
}

/* TX 完成回调：清 busy 并检查环形缓冲有无待发数据，有则接力启动下一批（中断上下文）。 */
void Serial_TX_Done(void)
{
    uint32_t primask = Serial_EnterCritical();

    txBusy = 0;
    Serial_StartTransmit();
    Serial_ExitCritical(primask);
}

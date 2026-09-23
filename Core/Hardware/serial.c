/**
 * @file    serial.c
 * @brief   USART1 串口调试/VOFA+ 通信模块 (中断 TX, 非阻塞)
 * @note    USART1: PA9=TX, PA10=RX, 115200bps
 *
 *          数据流:
 *            fputc/printf -> TX RingBuf (1024B) -> HAL IT buffer (256B)
 *            -> USART1 TX -> TX Complete ISR -> 自动接力下一块
 *
 *          生产者为主循环或周期任务，消费者为 USART1 发送完成中断。
 *          多个生产者通过 PRIMASK 临界区串行化，避免并发修改环形缓冲区。
 *          环形缓冲区满时静默丢弃剩余字节，保证控制循环不被串口阻塞。
 */

#include "serial.h"
#include "ringbuf.h"
#include <stdarg.h>
#include <string.h>

static UART_HandleTypeDef *serial_uart = NULL;

/* TX 环形缓冲区 (生产者: 主循环/任务, 消费者: UART TX Complete ISR)。 */
/* 串口发送环形缓冲区容量，单位字节。 */
#define TX_RING_SIZE  1024
static uint8_t tx_ring_buf[TX_RING_SIZE];
static RingBuf  tx_ring;

/* 单次 HAL UART 中断发送使用的线性缓冲区容量，单位字节。 */
#define TX_LINEAR_BUF_SIZE  256
static uint8_t  txLinearBuf[TX_LINEAR_BUF_SIZE];
static volatile uint8_t txBusy = 0;

/**
 * @brief  关闭中断并返回进入临界区前的 PRIMASK 状态。
 * @return 进入临界区前的 PRIMASK 值。
 */
static uint32_t Serial_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief  根据保存的 PRIMASK 状态恢复中断。
 * @param  primask Serial_EnterCritical 返回的 PRIMASK 值。
 */
static void Serial_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* ---- 内部: 从环形缓冲区取数据并启动下一批中断发送 ---- */
/**
 * @brief  从发送环形缓冲区取出一批数据并启动下一次中断发送。
 * @note   使用 HAL_UART_Transmit_IT，不配置发送 DMA；当前发送未完成时直接返回。
 *         调用方必须持有串口临界区，确保环形缓冲区读取和发送状态更新不会被
 *         其它生产者或发送完成中断并发修改。
 */
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

/**
 * @brief  串口初始化: 初始化 TX 环形缓冲区
 * @param  huart 用于发送的 UART 句柄。
 */
void Serial_Init(UART_HandleTypeDef *huart)
{
    serial_uart = huart;
    RingBuf_Init(&tx_ring, tx_ring_buf, TX_RING_SIZE);
    txBusy = 0;
}

/**
 * @brief  批量写入数据到 TX 环形缓冲区 (非阻塞)
 * @param  data 数据指针
 * @param  len  字节数
 * @return 实际写入的字节数，范围为 0~len；空间不足时返回已写入数量。
 */
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

/**
 * @brief  发送单字节 (非阻塞, 写入 ring buf)
 * @param  Byte 待发送字节。
 * @note   缓冲区满时静默丢弃该字节。
 */
void Serial_SendByte(uint8_t Byte)
{
    (void)Serial_Write(&Byte, 1U);
}

/**
 * @brief  发送字节数组 (非阻塞)
 * @param  Array 数据指针。
 * @param  Length 数据长度，单位字节。
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    Serial_Write(Array, Length);
}

/**
 * @brief  发送字符串 (非阻塞)
 * @param  String 以 '\0' 结尾的字符串。
 * @note   环形缓冲区满时在当前位置停止，剩余字符不会发送。
 */
void Serial_SendString(char *String)
{
    size_t length = strlen(String);

    if (length > 0xFFFFU) {
        length = 0xFFFFU;
    }
    (void)Serial_Write((const uint8_t *)String, (uint16_t)length);
}

/**
 * @brief  整数的幂运算
 * @param  X 底数。
 * @param  Y 指数。
 * @return X 的 Y 次幂。
 */
static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

/**
 * @brief  发送无符号整数
 * @param  Number 待发送数值。
 * @param  Length 固定输出位数。
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
}

/**
 * @brief  printf 重定向 → 非阻塞 Serial_Write
 * @param  ch 待输出字符。
 * @param  f 标准流指针，函数不使用。
 * @return 传入的字符值。
 */
int fputc(int ch, FILE *f)
{
    Serial_SendByte((uint8_t)ch);
    return ch;
}

/**
 * @brief  格式化串口打印 (vsnprintf 防栈溢出)
 * @param  format printf 风格格式字符串。
 * @param  ... 可变参数。
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
 * @brief  TX 完成回调 (USART1 TX Complete ISR → HAL_UART_TxCpltCallback 调用)
 * @note   检查 ring buf 中是否有待发数据, 有则接力启动下一次发送
 *         该函数运行在中断上下文，只允许执行缓冲区操作和启动下一次发送。
 */
void Serial_TX_Done(void)
{
    uint32_t primask = Serial_EnterCritical();

    txBusy = 0;
    Serial_StartTransmit();
    Serial_ExitCritical(primask);
}

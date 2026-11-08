/* 串口收发：USART1 115200 8N1。
   收：HAL_UARTEx_ReceiveToIdle_DMA —— 64 B DMA 缓冲 + IDLE 事件，回到 IDLE 就把
       已收到的字节推入环形缓冲并重新挂起。115200 下一个字节 86.8 µs，逐字节中断
       在遥测/显示高峰会丢字节；DMA 把中断频率从 11.5 kHz 降到"每包一次"。
   发：保持 HAL_UART_Transmit_IT 分块（128 B）。F103 的 DMA1 通道 4/5 是
       USART1_TX/RX 与 SPI2/I2C2 共用的，Ch4 已经分给 OLED 的 I2C2_TX，
       所以 TX 走中断；一次 128 B 约 11 ms 后台发送，主循环不阻塞。 */

#include "serial.h"
#include "ringbuf.h"
#include <string.h>

#define SERIAL_TX_CAPACITY   512U
#define SERIAL_RX_CAPACITY   256U
#define SERIAL_TX_CHUNK      128U
#define SERIAL_DMA_RX_SIZE    64U

static UART_HandleTypeDef *s_huart = 0;
static uint8_t s_tx_storage[SERIAL_TX_CAPACITY];
static uint8_t s_rx_storage[SERIAL_RX_CAPACITY];
static RingBuf s_tx;
static RingBuf s_rx;

static uint8_t s_dma_rx[SERIAL_DMA_RX_SIZE];
static volatile uint8_t s_rx_armed = 0U;

static uint8_t s_tx_chunk[SERIAL_TX_CHUNK];
static volatile uint8_t s_tx_busy = 0U;

/* 关掉 DMA 半传输中断。HAL_UARTEx_ReceiveToIdle_DMA 会开 HT：64 B 缓冲收满一半
   就回调，而回调里重新挂 DMA 会 abort 掉当前这笔，后半包的字节直接丢。
   本工程只要「整包/IDLE」两种事件，所以每次挂起后把 HT 关掉。 */
static void serial_rx_ht_off(void)
{
    if (s_huart->hdmarx != 0) {
        __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
    }
}

/* 把一次 DMA 接收交给 HAL。失败说明上一次没收干净（溢出/错误后 RxState 卡在
   BUSY_RX），先 AbortReceive 复位状态再立刻重挂，否则会永久收不到数据。 */
static void serial_rx_arm(void)
{
    if (s_huart == 0) {
        return;
    }
    s_rx_armed = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_rx, SERIAL_DMA_RX_SIZE) == HAL_OK) {
        serial_rx_ht_off();
        s_rx_armed = 1U;
        return;
    }
    (void)HAL_UART_AbortReceive(s_huart);
    if (HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_rx, SERIAL_DMA_RX_SIZE) == HAL_OK) {
        serial_rx_ht_off();
        s_rx_armed = 1U;
    }
}

void serial_init(UART_HandleTypeDef *huart)
{
    if (huart == 0) {
        return;
    }
    s_huart = huart;
    ringbuf_init(&s_tx, s_tx_storage, SERIAL_TX_CAPACITY);
    ringbuf_init(&s_rx, s_rx_storage, SERIAL_RX_CAPACITY);
    s_tx_busy = 0U;
    serial_rx_arm();
}

void serial_task(void)
{
    uint16_t len;

    if (s_huart == 0) {
        return;
    }
    if (s_rx_armed == 0U) {
        serial_rx_arm();
    }
    if (s_tx_busy != 0U) {
        return;
    }

    len = ringbuf_get(&s_tx, s_tx_chunk, SERIAL_TX_CHUNK);
    if (len == 0U) {
        return;
    }

    s_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(s_huart, s_tx_chunk, len) != HAL_OK) {
        /* 发送没启动，把刚取出的字节退回，下一轮重试，不丢遥测 */
        ringbuf_unget(&s_tx, len);
        s_tx_busy = 0U;
    }
}

uint16_t serial_tx_free(void)
{
    if (s_huart == 0) {
        return 0U;
    }
    return ringbuf_free(&s_tx);
}

void serial_write_bytes(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0U)) {
        return;
    }
    /* 队列满时丢尾部并计数，绝不阻塞在中断或控制周期里 */
    (void)ringbuf_put(&s_tx, data, len);
}

void serial_write(const char *text)
{
    if (text == 0) {
        return;
    }
    serial_write_bytes((const uint8_t *)text, (uint16_t)strlen(text));
}

uint8_t serial_getc(uint8_t *byte)
{
    return ringbuf_get_byte(&s_rx, byte);
}

uint32_t serial_tx_overflow(void)
{
    return s_tx.overflow;
}

uint32_t serial_rx_overflow(void)
{
    return s_rx.overflow;
}

/* IDLE 事件（收到一包）或 DMA 计满 64 B 都会进这里：Size 是本包有效字节数。
   推完立刻重新挂起，中间不做任何阻塞操作。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if ((s_huart == 0) || (huart->Instance != s_huart->Instance)) {
        return;
    }
    if ((Size > 0U) && (Size <= SERIAL_DMA_RX_SIZE)) {
        (void)ringbuf_put(&s_rx, s_dma_rx, Size);
    }
    serial_rx_arm();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart != 0) && (huart->Instance == s_huart->Instance)) {
        s_tx_busy = 0U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart != 0) && (huart->Instance == s_huart->Instance)) {
        s_tx_busy = 0U;
        /* 溢出/帧错后 DMA 与 UART 都可能停在半路，交给 serial_task 重挂 */
        (void)HAL_UART_AbortReceive(s_huart);
        s_rx_armed = 0U;
    }
}

/* USART1（115200）收发：中断接收单字节进环形缓冲，中断发送分块出队。
 * 所有 HAL 回调都在本文件实现，别的模块不碰 HAL 串口 API。 */
#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"

void serial_init(UART_HandleTypeDef *huart);
void serial_task(void);                                   /* 主循环调用：推进发送队列 */
void serial_write(const char *text);
void serial_write_bytes(const uint8_t *data, uint16_t len);
uint8_t serial_getc(uint8_t *byte);
uint16_t serial_tx_free(void);   /* 发送队列剩余字节数：整帧发送前先问，避免半帧 */
uint32_t serial_tx_overflow(void);
uint32_t serial_rx_overflow(void);

#endif /* __SERIAL_H */

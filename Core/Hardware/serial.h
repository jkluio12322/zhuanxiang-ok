/**
 * @file    serial.h
 * @brief   UART3 串口调试/VOFA+ 通信模块 (DMA TX, 非阻塞)
 * @note    fputc 重定向 printf → Serial_Write → RingBuf → DMA TX
 *          DMA TX Complete ISR 自动接力发送下一块数据
 */

#ifndef __BSP_SERIAL_H
#define __BSP_SERIAL_H
#include "main.h"
#include <stdio.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
int  Serial_Write(const uint8_t *data, uint16_t len);  /* non-blocking bulk write */
void Serial_DMA_TX_Complete(void);                     /* ISR callback */

#endif

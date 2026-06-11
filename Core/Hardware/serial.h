/**
 * @file    serial.h
 * @brief   UART3 串口调试/VOFA+ 通信模块头文件
 * @note    通过 fputc 重定向 printf 到串口
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
void Serial_TXE_IRQ(void);

#endif

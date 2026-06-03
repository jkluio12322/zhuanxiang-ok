/**
 * @file    blueserial.h
 * @brief   HC-05 蓝牙模块 UART 通信 + 协议解析头文件
 * @note    使用 USART2 (PA2=TX, PA3=RX), 9600bps, 中断接收 + 环形缓冲
 *
 *          协议格式:
 *            [joystick,<LH>,<LV>,<RH>,<RV>]  — 摇杆数据
 *            [key,<N>,<up/down>]             — 按键事件
 */

#ifndef __BLUE_SERIAL_H
#define __BLUE_SERIAL_H
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/** 蓝牙协议解析结果: 6 个字段 × 20 字符 */
extern char BlueSerial_StringArray[6][20];

void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_Printf(char *format, ...);
uint8_t BlueSerial_ReceiveFlag(void);  /**< 检测是否收到完整帧 */
void BlueSerial_Receive(void);         /**< 解析蓝牙数据帧 */
void BlueSerial_ClearBuffer(void);

#endif

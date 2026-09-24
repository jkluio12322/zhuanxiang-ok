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
#define __BLUE_SERIAL_H  /* 头文件保护宏，防止蓝牙串口接口重复声明。 */
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/** 蓝牙协议解析结果: 6 个字段 × 20 字符 */
extern char BlueSerial_StringArray[6][20];

/**
 * @brief  初始化蓝牙串口并启动中断接收。
 */
void BlueSerial_Init(void);

/**
 * @brief  清空蓝牙接收环形缓冲区。
 */
void BlueSerial_ClearBuffer(void);

/**
 * @brief  通过蓝牙串口发送一个字节。
 * @param  Byte 待发送字节。
 */
void BlueSerial_SendByte(uint8_t Byte);

/**
 * @brief  通过蓝牙串口发送一段字节数组。
 * @param  Array 数据指针。
 * @param  Length 数据长度。
 */
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);

/**
 * @brief  通过蓝牙串口发送字符串。
 * @param  String 以 '\0' 结尾的字符串。
 */
void BlueSerial_SendString(char *String);

/**
 * @brief  通过蓝牙串口发送格式化字符串。
 * @param  format printf 风格格式字符串。
 * @param  ... 可变参数列表。
 */
void BlueSerial_Printf(char *format, ...);

/**
 * @brief  检查接收缓冲区中是否已经存在完整数据帧。
 * @return 1 表示存在完整帧，0 表示尚未收到完整帧。
 */
uint8_t BlueSerial_ReceiveFlag(void);

/**
 * @brief  从接收缓冲区提取并解析一帧蓝牙数据。
 */
void BlueSerial_Receive(void);

#endif

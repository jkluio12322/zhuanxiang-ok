/**
 * @file    oled.h
 * @brief   SSD1306 0.96" OLED 128×64 I2C 显示屏驱动 (帧缓冲 + 非阻塞 IT)
 */

#ifndef __BSP_OLED_H
#define __BSP_OLED_H  /* 头文件保护宏，防止 OLED 接口重复声明。 */
#include "main.h"

/* SSD1306 在 8 位 I2C 地址格式下的设备地址。 */
#define OLED_ADDR  0x78

/**
 * @brief  初始化 OLED 显示屏和帧缓冲。
 */
void OLED_Init(void);

/**
 * @brief  清空 OLED 帧缓冲。
 */
void OLED_Clear(void);

/**
 * @brief  在指定行列显示一个字符。
 * @param  Line 页行号，1~8。
 * @param  Column 字符列号，1 起始。
 * @param  Char 要显示的字符。
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);

/**
 * @brief  在指定行列显示字符串。
 * @param  Line 页行号，1~8。
 * @param  Column 起始字符列号，1 起始。
 * @param  String 以 '\0' 结尾的字符串。
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

/**
 * @brief  在指定行列显示一个 6x8 小字符。
 * @param  Line 页行号，1~8。
 * @param  Column 字符列号，1 起始。
 * @param  Char 要显示的字符。
 */
void OLED_ShowCharSmall(uint8_t Line, uint8_t Column, char Char);

/**
 * @brief  在指定行列显示 6x8 小字符串。
 * @param  Line 页行号，1~8。
 * @param  Column 起始字符列号，1 起始。
 * @param  String 以 '\0' 结尾的字符串。
 */
void OLED_ShowStringSmall(uint8_t Line, uint8_t Column, char *String);

/**
 * @brief  显示固定长度的无符号十进制数。
 * @param  Line 行号，1 起始。
 * @param  Column 列号，1 起始。
 * @param  Number 待显示数值。
 * @param  Length 十进制位数。
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief  在指定行列显示有符号十进制数。
 * @param  Line 行号，1 起始。
 * @param  Column 列号，1 起始。
 * @param  Number 待显示数值。
 * @param  Length 十进制位数。
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);

/**
 * @brief  在指定行列显示十六进制数。
 * @param  Line 行号，1 起始。
 * @param  Column 列号，1 起始。
 * @param  Number 待显示数值。
 * @param  Length 十六进制位数。
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief  在指定行列显示二进制数。
 * @param  Line 行号，1 起始。
 * @param  Column 列号，1 起始。
 * @param  Number 待显示数值。
 * @param  Length 二进制位数。
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief  非阻塞刷新帧缓冲到 SSD1306。
 */
void OLED_Flush(void);

#endif

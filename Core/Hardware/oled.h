/**
 * @file    oled.h
 * @brief   SSD1306 0.96" OLED 128×64 I2C 显示屏驱动 (帧缓冲 + 非阻塞 IT)
 */

#ifndef __BSP_OLED_H
#define __BSP_OLED_H
#include "main.h"

#define OLED_ADDR  0x78

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_Flush(void);  /* 非阻塞: 将帧缓冲发送到 SSD1306 */

#endif

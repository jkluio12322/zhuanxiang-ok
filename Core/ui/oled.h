/* SSD1306 128×64 I2C 显示屏驱动 (帧缓冲 + 非阻塞 IT) */

#ifndef __BSP_OLED_H
#define __BSP_OLED_H  /* 头文件保护 */
#include "main.h"

/* SSD1306 8 位 I2C 地址 */
#define OLED_ADDR  0x78

void oled_init_begin(void);                                         /* 启动非阻塞初始化序列 */
uint8_t oled_init_poll(void);                                       /* 推进初始化，1 = 已结束 */
void oled_recover_begin(void);                                      /* 主动启动 I2C 总线恢复 */
void oled_recover_poll(void);                                       /* 推进非阻塞 I2C 总线恢复 */
void oled_clear(void);                                              /* 清帧缓冲 */
void oled_show_char(uint8_t Line, uint8_t Column, char Char);        /* 写 8×16 字符 */
void oled_show_string(uint8_t Line, uint8_t Column, char *String);   /* 写 8×16 字符串 */
void oled_show_char_small(uint8_t Line, uint8_t Column, char Char);   /* 写 6×8 小字符 (Line 1~8) */
void oled_show_string_small(uint8_t Line, uint8_t Column, char *String);  /* 写 6×8 小字符串 */
void oled_show_num(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);       /* 定长无符号十进制 */
void oled_show_signed(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);  /* 带符号十进制 */
void oled_show_hex(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);    /* 定长十六进制 */
void oled_show_bin(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);    /* 定长二进制 */
void oled_flush(void);                                              /* 帧缓冲 → SSD1306 (非阻塞 DMA) */
void oled_dma_init(void);                                           /* 把 DMA1_Channel4 接到 I2C2_TX，幂等 */
extern DMA_HandleTypeDef hdma_i2c2_tx;                              /* 供 isr.c 的 DMA1_Channel4_IRQHandler 使用 */

#endif

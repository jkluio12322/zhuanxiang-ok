/* SSD1306 128×64 I2C 驱动：写屏 = RAM 帧缓冲(瞬时) + I2C IT 突发(非阻塞)。
   I2C2: PB10=SCL, PB11=SDA, 地址 0x78；8×16 字模 4 行×16 列，128 列×8 页。 */

#include "oled.h"
#include "oled_font.h"
#include "oled_font_small.h"
#include "i2c.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c2;

/* OLED I2C 控制字节：0x00=命令 */
#define OLED_CMD  0x00

/* OLED I2C 控制字节：0x40=数据 */
#define OLED_DATA 0x40

/* 帧缓冲: 128 列 × 8 页 = 1024 字节 */
static uint8_t oled_fb[1024];
static uint8_t oled_dirty = 0;   /* 待刷新标志 */

/* IT 发送缓冲: [0]=0x40 数据标识 + [1..1024]=帧缓冲 */
static uint8_t oled_tx_buf[1 + 1024];

/* 软件翻转 SCL 释放被从机拉死的总线 */
static void I2C_BusRecover(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_I2C2_CLK_DISABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    for (uint8_t i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C2_CLK_ENABLE();
    MX_I2C2_Init();
}

/* 阻塞 I2C 写命令，仅初始化/刷新窗口用，不可进中断 */
static HAL_StatusTypeDef oled_write_command(uint8_t Command)
{
    uint8_t buf[2] = {OLED_CMD, Command};
    HAL_StatusTypeDef status;
    uint8_t retry = 3;
    do {
        status = HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, buf, 2, 100);
        if (status == HAL_OK) return HAL_OK;
        if (status == HAL_BUSY) I2C_BusRecover();
    } while (--retry > 0);
    return status;
}

/* 按指定进制显示定长无符号整数 (Base 支持 2/10/16) */
static void oled_show_uint_base(uint8_t Line, uint8_t Column,
                              uint32_t Number, uint8_t Length,
                              uint32_t Base)
{
    uint32_t divisor = 1U;

    if ((Length == 0U) || (Base < 2U) || (Base > 16U)) return;

    for (uint8_t i = 1U; i < Length; i++) {
        divisor *= Base;
    }

    for (uint8_t i = 0; i < Length; i++) {
        uint8_t digit = (uint8_t)(Number / divisor % Base);

        oled_show_char(Line, Column + i,
                      digit < 10U ? (char)(digit + '0')
                                  : (char)(digit - 10U + 'A'));
        divisor /= Base;
    }
}

/* 初始化 SSD1306 并清帧缓冲；阻塞，仅启动时调用一次 */
void oled_init(void)
{
    HAL_Delay(100);

    if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_ADDR, 3, 100) != HAL_OK)
        return;

    oled_write_command(0xAE);  /* 关闭显示 */
    oled_write_command(0xD5); oled_write_command(0x80);
    oled_write_command(0xA8); oled_write_command(0x3F);
    oled_write_command(0xD3); oled_write_command(0x00);
    oled_write_command(0x40);
    oled_write_command(0xA1);
    oled_write_command(0xC8);
    oled_write_command(0xDA); oled_write_command(0x12);
    oled_write_command(0x81); oled_write_command(0xCF);
    oled_write_command(0xD9); oled_write_command(0xF1);
    oled_write_command(0xDB); oled_write_command(0x30);
    oled_write_command(0xA4);
    oled_write_command(0xA6);
    oled_write_command(0x8D); oled_write_command(0x14);
    oled_write_command(0x20); oled_write_command(0x00);  /* 水平寻址模式 */
    oled_write_command(0xAF);  /* 开启显示 */

    memset(oled_fb, 0, sizeof(oled_fb));   /* 清帧缓冲 */

    /* 首次全屏刷新 (阻塞) */
    oled_tx_buf[0] = OLED_DATA;
    memset(oled_tx_buf + 1, 0, 1024);
    oled_write_command(0x21); oled_write_command(0); oled_write_command(127);
    oled_write_command(0x22); oled_write_command(0); oled_write_command(7);
    HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, oled_tx_buf, sizeof(oled_tx_buf), 100);

    oled_dirty = 0;
}

void oled_clear(void)   /* 清帧缓冲并标记待刷新 */
{
    memset(oled_fb, 0, sizeof(oled_fb));
    oled_dirty = 1;
}

void oled_show_char(uint8_t Line, uint8_t Column, char Char)   /* 写 8×16 字符到帧缓冲 (只改 RAM，不碰 I2C) */
{
    uint8_t page = (Line - 1) * 2;               /* 每字符占 2 页 */
    uint8_t col  = (Column - 1) * 8;             /* 每字符 8 像素宽 */

    for (uint8_t i = 0; i < 8; i++) {
        oled_fb[ page       * 128 + col + i] = OLED_F8x16[Char - ' '][i];
        oled_fb[(page + 1)  * 128 + col + i] = OLED_F8x16[Char - ' '][i + 8];
    }
    oled_dirty = 1;
}

void oled_show_string(uint8_t Line, uint8_t Column, char *String)   /* 写 8×16 字符串 */
{
    for (uint8_t i = 0; String[i] != '\0'; i++)
        oled_show_char(Line, Column + i, String[i]);
}

void oled_show_char_small(uint8_t Line, uint8_t Column, char Char)   /* 写 6×8 小字符，范围外显示空格 */
{
    uint8_t index;
    uint8_t page;
    uint8_t col;

    if ((Line < 1U) || (Line > 8U) || (Column < 1U)) return;

    page = (uint8_t)(Line - 1U);
    col  = (uint8_t)((Column - 1U) * 6U);
    if ((uint16_t)col + 5U >= 128U) return;

    if ((Char < ' ') || (Char > '~')) Char = ' ';
    index = (uint8_t)(Char - ' ');

    for (uint8_t i = 0; i < 6U; i++) {
        oled_fb[(uint16_t)page * 128U + col + i] = OLED_F6x8[index][i];
    }

    oled_dirty = 1;
}

void oled_show_string_small(uint8_t Line, uint8_t Column, char *String)   /* 写 6×8 小字符串 */
{
    for (uint8_t i = 0; String[i] != '\0'; i++) {
        oled_show_char_small(Line, (uint8_t)(Column + i), String[i]);
    }
}

void oled_show_num(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)   /* 定长无符号十进制 */
{
    oled_show_uint_base(Line, Column, Number, Length, 10U);
}

void oled_show_signed(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)   /* 带符号十进制 */
{
    uint32_t Number1;
    if (Number >= 0) { oled_show_char(Line, Column, '+'); Number1 = (uint32_t)Number; }
    else             { oled_show_char(Line, Column, '-'); Number1 = 0U - (uint32_t)Number; }
    oled_show_uint_base(Line, (uint8_t)(Column + 1U), Number1, Length, 10U);
}

void oled_show_hex(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)   /* 定长十六进制 */
{
    oled_show_uint_base(Line, Column, Number, Length, 16U);
}

void oled_show_bin(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)   /* 定长二进制 */
{
    oled_show_uint_base(Line, Column, Number, Length, 2U);
}

/* 帧缓冲 → SSD1306 (非阻塞 IT)。dirty=1 且 I2C2 空闲才发；1025 字节 ≈ 23ms @400kHz */
void oled_flush(void)
{
    if (!oled_dirty) return;
    if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) return;

    /* 设置写入窗口: 全屏 128×64 */
    oled_write_command(0x21); oled_write_command(0);   oled_write_command(127);
    oled_write_command(0x22); oled_write_command(0);   oled_write_command(7);

    /* 组装发送缓冲: [0]=数据标识, [1..1024]=帧缓冲 */
    oled_tx_buf[0] = OLED_DATA;
    memcpy(oled_tx_buf + 1, oled_fb, 1024);

    HAL_I2C_Master_Transmit_IT(&hi2c2, OLED_ADDR, oled_tx_buf, sizeof(oled_tx_buf));
    oled_dirty = 0;
}

/* I2C 发送完成回调 (ISR)：HAL 已置 READY，无需额外操作 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
}

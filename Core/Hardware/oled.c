/**
 * @file    oled.c
 * @brief   SSD1306 0.96" OLED 128×64 I2C 驱动 (帧缓冲 + 非阻塞 IT)
 * @note    写屏 = RAM 帧缓冲 (瞬时) + I2C IT 突发 (非阻塞)
 *          I2C2: PB10=SCL, PB11=SDA, address 0x78
 *          字模: 8×16 像素, 4 行 × 16 列
 *          显示区域: 128 列 × 8 页 (每页 8 像素行)
 */

#include "oled.h"
#include "oled_font.h"
#include "oled_font_small.h"
#include "i2c.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c2;

/* OLED I2C 控制字节：后续数据为命令。 */
#define OLED_CMD  0x00

/* OLED I2C 控制字节：后续数据为显示数据。 */
#define OLED_DATA 0x40

/* ── 帧缓冲: 128 列 × 8 页 = 1024 字节 ─────────────── */
static uint8_t oled_fb[1024];
static uint8_t oled_dirty = 0;

/* IT 发送缓冲: [0]=0x40(数据标识) + [1..1024]=帧缓冲 */
static uint8_t oled_tx_buf[1 + 1024];

/* ════════════════════════════════════════════════════════
 * I2C 总线死锁恢复
 * ════════════════════════════════════════════════════════ */

/**
 * @brief  通过软件翻转 SCL 恢复 I2C 总线死锁。
 * @note   当从机将 SDA 持续拉低时，主机发送若干时钟脉冲释放总线。
 */
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

/* ════════════════════════════════════════════════════════
 * 阻塞命令写 (仅初始化用, 不可进 ISR)
 * ════════════════════════════════════════════════════════ */

static HAL_StatusTypeDef OLED_WriteCommand(uint8_t Command)
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

/* ════════════════════════════════════════════════════════
 * 初始化 (阻塞, 仅启动时调用一次)
 * ════════════════════════════════════════════════════════ */

void OLED_Init(void)
{
    HAL_Delay(100);

    if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_ADDR, 3, 100) != HAL_OK)
        return;

    OLED_WriteCommand(0xAE);  /* 关闭显示 */
    OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14);
    OLED_WriteCommand(0x20); OLED_WriteCommand(0x00);  /* 水平寻址模式 */
    OLED_WriteCommand(0xAF);  /* 开启显示 */

    /* 清帧缓冲 */
    memset(oled_fb, 0, sizeof(oled_fb));

    /* 首次全屏刷新 (阻塞) */
    oled_tx_buf[0] = OLED_DATA;
    memset(oled_tx_buf + 1, 0, 1024);
    OLED_WriteCommand(0x21); OLED_WriteCommand(0); OLED_WriteCommand(127);
    OLED_WriteCommand(0x22); OLED_WriteCommand(0); OLED_WriteCommand(7);
    HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, oled_tx_buf, sizeof(oled_tx_buf), 100);

    oled_dirty = 0;
}

/* ════════════════════════════════════════════════════════
 * 帧缓冲字符渲染 (RAM 操作, 瞬时, ISR 安全)
 * ════════════════════════════════════════════════════════ */

void OLED_Clear(void)
{
    memset(oled_fb, 0, sizeof(oled_fb));
    oled_dirty = 1;
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t page = (Line - 1) * 2;               /* 每字符占 2 页 */
    uint8_t col  = (Column - 1) * 8;             /* 每字符 8 像素宽 */

    for (uint8_t i = 0; i < 8; i++) {
        oled_fb[ page       * 128 + col + i] = OLED_F8x16[Char - ' '][i];
        oled_fb[(page + 1)  * 128 + col + i] = OLED_F8x16[Char - ' '][i + 8];
    }
    oled_dirty = 1;
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++)
        OLED_ShowChar(Line, Column + i, String[i]);
}

/* ── 数字显示 (基于 OLED_ShowChar, 均为帧缓冲操作) ── */

void OLED_ShowCharSmall(uint8_t Line, uint8_t Column, char Char)
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

void OLED_ShowStringSmall(uint8_t Line, uint8_t Column, char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++) {
        OLED_ShowCharSmall(Line, (uint8_t)(Column + i), String[i]);
    }
}

static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i,
            Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint32_t Number1;
    if (Number >= 0) { OLED_ShowChar(Line, Column, '+'); Number1 = (uint32_t)Number; }
    else             { OLED_ShowChar(Line, Column, '-'); Number1 = 0U - (uint32_t)Number; }
    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i + 1,
            Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t SingleNumber;
    for (uint8_t i = 0; i < Length; i++) {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        OLED_ShowChar(Line, Column + i,
            SingleNumber < 10 ? SingleNumber + '0' : SingleNumber - 10 + 'A');
    }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i,
            Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
}

/* ════════════════════════════════════════════════════════
 * 非阻塞刷新: 帧缓冲 → SSD1306 (I2C IT)
 * ════════════════════════════════════════════════════════ */

/**
 * @brief  将帧缓冲发送到 SSD1306 (非阻塞 IT, 主循环任意频率调用)
 * @note   仅在 oled_dirty=1 且 I2C2 空闲时启动传输
 *         1025 字节 ≈ 23ms @ 400kHz, 完成后 I2C 自动恢复 READY
 */
void OLED_Flush(void)
{
    if (!oled_dirty) return;
    if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) return;

    /* 设置写入窗口: 全屏 128×64 */
    OLED_WriteCommand(0x21); OLED_WriteCommand(0);   OLED_WriteCommand(127);
    OLED_WriteCommand(0x22); OLED_WriteCommand(0);   OLED_WriteCommand(7);

    /* 组装发送缓冲: [0]=数据标识, [1..1024]=帧缓冲 */
    oled_tx_buf[0] = OLED_DATA;
    memcpy(oled_tx_buf + 1, oled_fb, 1024);

    HAL_I2C_Master_Transmit_IT(&hi2c2, OLED_ADDR, oled_tx_buf, sizeof(oled_tx_buf));
    oled_dirty = 0;
}

/**
 * @brief I2C 主机发送完成回调 (HAL 框架, I2C ISR 上下文)
 * @note  仅清除 busy 状态, 下一帧由 OLED_Flush 自动启动
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* I2C2 传输完成, HAL 已自动将 State 置为 READY, 无需额外操作 */
    (void)hi2c;
}

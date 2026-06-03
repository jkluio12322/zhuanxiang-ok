/**
 * @file    oled.c
 * @brief   SSD1306 0.96" OLED 显示屏 I2C 驱动 (128x64 像素)
 * @note    使用 I2C1 (PB6=SCL, PB7=SDA), 地址 0x78
 *          字模: 8x16 像素 (OLED_F8x16), 每行 16 字符, 共 4 行
 *          显示区域: 128 列 × 8 页 (每页 8 行)
 *
 *          I2C 总线死锁恢复: 当 HAL 返回 BUSY 时, 通过 GPIO 模拟
 *          9 个 SCL 时钟脉冲 + STOP 条件解除从设备死锁
 */

#include "oled.h"
#include "oled_font.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;

/** OLED I2C 写命令/数据标识 */
#define OLED_CMD  0x00  /**< 后续字节为命令 */
#define OLED_DATA 0x40  /**< 后续字节为数据 */

/**
 * @brief  I2C 总线死锁恢复
 * @note   当从设备 SDA 持续拉低时, 通过 GPIO 模拟 9 个 SCL 脉冲 +
 *         STOP 条件来释放总线, 然后重新初始化 I2C1
 */
static void I2C_BusRecover(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 禁用 I2C1 时钟, 将 SCL/SDA 切换为 GPIO 开漏输出 */
    __HAL_RCC_I2C1_CLK_DISABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 发送 9 个 SCL 脉冲释放 SDA (SDA=高, SCL=翻转) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);  /* SDA=高 */
    for (uint8_t i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   /* SCL=1 */
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); /* SCL=0 */
        HAL_Delay(1);
    }

    /* 发送 STOP 条件: SDA 从低变高 (当时钟为高) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    /* 恢复 I2C1 为复用开漏模式并重新初始化 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C1_CLK_ENABLE();
    MX_I2C1_Init();
}

/**
 * @brief  向 OLED 写入命令字节 (带重试)
 * @param  Command 命令字节
 * @return HAL 状态
 * @note   I2C BUSY 时自动进行总线恢复, 最多重试 3 次
 */
static HAL_StatusTypeDef OLED_WriteCommand(uint8_t Command)
{
    uint8_t buf[2] = {OLED_CMD, Command};  /* 第1字节: 命令标识, 第2字节: 命令 */
    HAL_StatusTypeDef status;
    uint8_t retry = 3;
    do {
        status = HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 100);
        if (status == HAL_OK) return HAL_OK;
        if (status == HAL_BUSY) I2C_BusRecover();  /* 总线死锁时恢复 */
    } while (--retry > 0);
    return status;
}

/**
 * @brief  向 OLED 写入数据字节 (带重试)
 * @param  Data 数据字节
 * @return HAL 状态
 */
static HAL_StatusTypeDef OLED_WriteData(uint8_t Data)
{
    uint8_t buf[2] = {OLED_DATA, Data};  /* 第1字节: 数据标识, 第2字节: 数据 */
    HAL_StatusTypeDef status;
    uint8_t retry = 3;
    do {
        status = HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 100);
        if (status == HAL_OK) return HAL_OK;
        if (status == HAL_BUSY) I2C_BusRecover();
    } while (--retry > 0);
    return status;
}

/**
 * @brief  设置 OLED 光标位置 (页寻址模式)
 * @param  Y 页号 (0~7, 每页 8 像素行)
 * @param  X 列号 (0~127)
 */
static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                      /* 设置页地址 */
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));      /* 设置列地址高 4 位 */
    OLED_WriteCommand(0x00 | (X & 0x0F));              /* 设置列地址低 4 位 */
}

/**
 * @brief  清屏: 将全部 128×64 像素写 0
 */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++) {          /* 遍历 8 页 */
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++)       /* 每页 128 列 */
            OLED_WriteData(0x00);
    }
}

/**
 * @brief  在指定位置显示一个字符 (8x16 像素)
 * @param  Line   行号 (1~4)
 * @param  Column 列号 (1~16)
 * @param  Char   要显示的 ASCII 字符
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    /* 每个字符占 2 页 (16 像素高), 第1页上半部分 */
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);

    /* 第2页下半部分 */
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
}

/**
 * @brief  在指定位置显示字符串
 * @param  Line   起始行号 (1~4)
 * @param  Column 起始列号 (1~16)
 * @param  String 字符串指针 (以 '\0' 结尾)
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++)
        OLED_ShowChar(Line, Column + i, String[i]);
}

/**
 * @brief  整数的幂运算 (用于数字按位分解)
 */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

/**
 * @brief  显示无符号整数 (十进制, 高位补0)
 * @param  Line/Column 起始位置
 * @param  Number 待显示数值
 * @param  Length 显示位数
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

/**
 * @brief  显示有符号整数 (十进制, 带正负号)
 * @param  Line/Column 起始位置
 * @param  Number 待显示数值
 * @param  Length 数值部分的位数 (不含符号位)
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint32_t Number1;
    if (Number >= 0) { OLED_ShowChar(Line, Column, '+'); Number1 = (uint32_t)Number; }
    else             { OLED_ShowChar(Line, Column, '-'); Number1 = 0U - (uint32_t)Number; }
    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

/**
 * @brief  显示十六进制数
 * @param  Line/Column 起始位置
 * @param  Number 待显示数值
 * @param  Length 显示位数
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t SingleNumber;
    for (uint8_t i = 0; i < Length; i++) {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        OLED_ShowChar(Line, Column + i, SingleNumber < 10 ? SingleNumber + '0' : SingleNumber - 10 + 'A');
    }
}

/**
 * @brief  显示二进制数
 * @param  Line/Column 起始位置
 * @param  Number 待显示数值
 * @param  Length 显示位数
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
}

/**
 * @brief  初始化 SSD1306 OLED 显示屏
 * @note   配置: 128×64, 内部电荷泵, 水平寻址, 正常显示
 *          上电等待 100ms 确保电源稳定
 */
void OLED_Init(void)
{
    HAL_Delay(100);  /* 等待 OLED 上电稳定 */

    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR, 3, 100) != HAL_OK)
        return;  /* I2C 无设备应答 — 检查接线 PB6/PB7, OLED 供电 */

    /* --- SSD1306 初始化序列 --- */
    OLED_WriteCommand(0xAE);  /* 关闭显示 */
    OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);  /* 设置时钟分频/振荡频率 */
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);  /* 设置多路复用比 64 */
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);  /* 设置显示偏移 0 */
    OLED_WriteCommand(0x40);  /* 设置起始行 0 */
    OLED_WriteCommand(0xA1);  /* 段重映射 (左右翻转) */
    OLED_WriteCommand(0xC8);  /* COM 扫描方向 (上下翻转) */
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);  /* 设置 COM 引脚硬件配置 */
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);  /* 设置对比度 0xCF */
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);  /* 设置预充电周期 */
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x30);  /* 设置 VCOMH 取消选择电平 */
    OLED_WriteCommand(0xA4);  /* 全局显示开启 (跟随 RAM) */
    OLED_WriteCommand(0xA6);  /* 正常显示 (不反相) */
    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14);  /* 使能电荷泵 */
    OLED_WriteCommand(0xAF);  /* 开启显示 */

    OLED_Clear();
}

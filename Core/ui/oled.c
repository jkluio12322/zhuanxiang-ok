/* SSD1306 128×64 I2C 驱动：写屏 = RAM 帧缓冲(瞬时) + I2C IT 突发(非阻塞)。
   I2C2: PB10=SCL, PB11=SDA, 地址 0x78；8×16 字模 4 行×16 列，128 列×8 页。 */

#include "oled.h"
#include "oled_font.h"
#include "oled_font_small.h"
#include "i2c.h"
#include "config.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c2;

/* OLED 帧发送用的 DMA 通道（DMA1_Channel4 = I2C2_TX）。非 static：
   isr.c 的 DMA1_Channel4_IRQHandler 要拿它调 HAL_DMA_IRQHandler。 */
DMA_HandleTypeDef hdma_i2c2_tx;

/* 把 DMA1_Channel4 接到 I2C2 的发送请求上。只做一次：
   通道是固定映射（F1 没有 DMAMUX），这里只配方向/对齐/优先级。 */
void oled_dma_init(void)
{
    static uint8_t inited = 0U;

    if (inited != 0U) {
        return;
    }
    inited = 1U;

    hdma_i2c2_tx.Instance                 = DMA1_Channel4;
    hdma_i2c2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i2c2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2c2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2c2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_i2c2_tx.Init.Mode                = DMA_NORMAL;
    hdma_i2c2_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    (void)HAL_DMA_Init(&hdma_i2c2_tx);
    __HAL_LINKDMA(&hi2c2, hdmatx, hdma_i2c2_tx);
}

/* OLED I2C 控制字节：0x00=命令 */
#define OLED_CMD  0x00

/* OLED I2C 控制字节：0x40=数据 */
#define OLED_DATA 0x40

/* 帧缓冲: 128 列 × 8 页 = 1024 字节 */
static uint8_t oled_fb[1024];
static uint8_t oled_dirty = 0;   /* 待刷新标志 */

/* SSD1306 uses a separate I2C control byte for commands and display data. */
#define OLED_WINDOW_TX_SIZE   7U
#define OLED_FRAME_TX_SIZE    (1U + 1024U)
static uint8_t oled_window_buf[OLED_WINDOW_TX_SIZE];
static uint8_t oled_frame_buf[OLED_FRAME_TX_SIZE];

/* 上一次 IT 传输是否出错：HAL 错误回调置位，发起新传输时清除。 */
static volatile uint8_t oledTxError = 0U;

/* SSD1306 上电初始化命令序列，一条 I2C 事务发完。 */
static const uint8_t oledInitCmds[] = {
    0xAE,               /* 关闭显示 */
    0x2E,               /* 关闭硬件滚动，避免复位前滚动状态残留 */
    0xD5, 0x80,         /* 时钟分频 */
    0xA8, 0x3F,         /* 复用率 1/64 */
    0xD3, 0x00,         /* 显示偏移 0 */
    0x40,               /* 起始行 0 */
    0xA1,               /* 段重映射 */
    0xC8,               /* COM 扫描方向 */
    0xDA, 0x12,         /* COM 引脚配置 */
    0x81, 0xCF,         /* 对比度 */
    0xD9, 0xF1,         /* 预充电周期 */
    0xDB, 0x30,         /* VCOMH */
    0xA4,               /* 恢复显存显示 */
    0xA6,               /* 正常显示（非反色） */
    0x8D, 0x14,         /* 电荷泵开 */
    0x20, 0x00,         /* 水平寻址模式 */
    0xAF                /* 开启显示 */
};
static uint8_t oled_init_buf[1U + sizeof(oledInitCmds)];

/* I2C2 是否正忙（上一次 IT 传输还没回）。 */
static uint8_t oled_busy(void)
{
    return (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) ? 1U : 0U;
}

/* 发起一次非阻塞 I2C 写。返回 1 = 已交给 HAL；0 = 忙或启动失败（下一轮再试）。 */
static uint8_t oled_start_tx(uint8_t *data, uint16_t length)
{
    if (oled_busy() != 0U) {
        return 0U;
    }

    oledTxError = 0U;
#if (OLED_DMA_TX != 0U)
    /* DMA 发送：1025 B 帧数据 CPU 完全不参与，中断只在整帧结束时来一次 */
    if (HAL_I2C_Master_Transmit_DMA(&hi2c2, OLED_ADDR, data, length) != HAL_OK) {
        oledTxError = 1U;
        return 0U;
    }
#else
    if (HAL_I2C_Master_Transmit_IT(&hi2c2, OLED_ADDR, data, length) != HAL_OK) {
        oledTxError = 1U;
        return 0U;
    }
#endif
    return 1U;
}

/* Build the window and framebuffer as separate I2C transfers. */
static void oled_build_window(void)
{
    oled_window_buf[0] = OLED_CMD;
    oled_window_buf[1] = 0x21;  oled_window_buf[2] = 0x00;  oled_window_buf[3] = 0x7FU;
    oled_window_buf[4] = 0x22;  oled_window_buf[5] = 0x00;  oled_window_buf[6] = 0x07U;
}

static void oled_build_frame(void)
{
    oled_frame_buf[0] = OLED_DATA;
    memcpy(&oled_frame_buf[1], oled_fb, sizeof(oled_fb));
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

/* 初始化状态机：等上电稳定 → 发命令序列 → 清屏。全程非阻塞，由主循环驱动，
   每轮只推进一小步；原来的实现是一串阻塞 I2C 命令 + HAL_Delay，会把主循环拖住。 */
typedef enum {
    OLED_ST_IDLE = 0,   /* 未开始或已完成 */
    OLED_ST_RECOVER,    /* 复位后先释放可能被从机拉住的 I2C 总线 */
    OLED_ST_DELAY,      /* 等 OLED 电源稳定 */
    OLED_ST_CMDS,       /* 发初始化命令 */
    OLED_ST_CLEAR,      /* 设置清屏窗口 */
    OLED_ST_CLEAR_DATA, /* 发送清屏数据 */
    OLED_ST_WAIT_DONE,  /* 等清屏传输结束 */
    OLED_ST_FAILED      /* I2C 无应答：放弃显示，但不影响遥测 */
} oledInitState_t;

static oledInitState_t oledInitState  = OLED_ST_IDLE;
static uint32_t        oledInitStart  = 0U;
static uint32_t        oledInitBeginAt = 0U;
static uint8_t         oledInitRetries = 0U;

static void oled_recover_blocking(void);

static void oled_recover_blocking(void)
{
    GPIO_InitTypeDef gpio = {0};

    (void)HAL_I2C_DeInit(&hi2c2);
    __HAL_RCC_I2C2_CLK_DISABLE();

    gpio.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1U);

    for (uint8_t i = 0U; i < 10U; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1U);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1U);

    gpio.Mode  = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    __HAL_RCC_I2C2_CLK_ENABLE();
    MX_I2C2_Init();
    oled_dma_init();
    HAL_Delay(2U);
    oledTxError = 0U;
}

void oled_init_begin(void)
{
    memset(oled_fb, 0, sizeof(oled_fb));
    oled_dirty    = 0U;
    oledTxError   = 0U;
    oledInitStart = HAL_GetTick();
    oledInitBeginAt = oledInitStart;
    oledInitRetries = 0U;
    oled_recover_blocking();
    oledInitState = OLED_ST_DELAY;
}

/* 返回 1 表示初始化结束（成功或已放弃）。 */
uint8_t oled_init_poll(void)
{
    uint32_t now = HAL_GetTick();

    if ((oledInitState != OLED_ST_IDLE) && (oledInitState != OLED_ST_FAILED)
        && ((uint32_t)(now - oledInitBeginAt) >= OLED_INIT_TIMEOUT_MS)) {
        oledInitState = OLED_ST_FAILED;
        return 1U;
    }

    switch (oledInitState) {
    case OLED_ST_RECOVER:
        oled_recover_blocking();
        oledInitStart = HAL_GetTick();
        oledInitState = OLED_ST_DELAY;
        return 0U;

    case OLED_ST_DELAY:
        if ((uint32_t)(HAL_GetTick() - oledInitStart) < OLED_BOOT_DELAY_MS) {
            return 0U;
        }
        oledInitState = OLED_ST_CMDS;
        return 0U;

    case OLED_ST_CMDS:
        if (oledTxError != 0U) {
            if (++oledInitRetries >= 3U) {
                oledInitState = OLED_ST_FAILED;
                return 1U;
            }
            oledInitState = OLED_ST_RECOVER;
            return 0U;
        }
        if (oled_busy() != 0U) {
            return 0U;
        }
        oled_init_buf[0] = OLED_CMD;
        memcpy(&oled_init_buf[1], oledInitCmds, sizeof(oledInitCmds));
        if (oled_start_tx(oled_init_buf, (uint16_t)sizeof(oled_init_buf)) == 0U) {
            return 0U;
        }
        oledInitState = OLED_ST_CLEAR;
        return 0U;

    case OLED_ST_CLEAR:
        if (oledTxError != 0U) {
            if (++oledInitRetries >= 3U) {
                oledInitState = OLED_ST_FAILED;
                return 1U;
            }
            oledInitState = OLED_ST_RECOVER;
            return 0U;
        }
        if (oled_busy() != 0U) {
            return 0U;
        }
        oled_build_window();
        if (oled_start_tx(oled_window_buf, (uint16_t)sizeof(oled_window_buf)) == 0U) {
            return 0U;
        }
        oledInitState = OLED_ST_CLEAR_DATA;
        return 0U;

    case OLED_ST_CLEAR_DATA:
        if (oledTxError != 0U) {
            if (++oledInitRetries >= 3U) {
                oledInitState = OLED_ST_FAILED;
                return 1U;
            }
            oledInitState = OLED_ST_RECOVER;
            return 0U;
        }
        if (oled_busy() != 0U) {
            return 0U;
        }
        oled_build_frame();
        if (oled_start_tx(oled_frame_buf, (uint16_t)sizeof(oled_frame_buf)) == 0U) {
            return 0U;
        }
        oledInitState = OLED_ST_WAIT_DONE;
        return 0U;

    case OLED_ST_WAIT_DONE:
        if (oledTxError != 0U) {
            if (++oledInitRetries >= 3U) {
                oledInitState = OLED_ST_FAILED;
                return 1U;
            }
            oledInitState = OLED_ST_RECOVER;
            return 0U;
        }
        if (oled_busy() != 0U) {
            return 0U;
        }
        oledInitState = OLED_ST_IDLE;
        return 1U;

    case OLED_ST_FAILED:
        return 1U;

    case OLED_ST_IDLE:
    default:
        return 1U;
    }
}

/* 非阻塞总线恢复：PB10/PB11 临时切成开漏 GPIO，手动打 10 个 SCL 脉冲把被从机
   拉死的 SDA 放回来，再恢复 I2C2 外设。每轮主循环推进 1 ms 一步。 */
typedef enum {
    REC_IDLE = 0,
    REC_CLOCK,
    REC_STOP_LOW,
    REC_STOP_HIGH,
    REC_RESTORE
} oledRecoverState_t;

static oledRecoverState_t recState          = REC_IDLE;
static uint32_t           recLastTick       = 0U;
static uint8_t            recHalfStep       = 0U;
static volatile uint8_t   oledRecoverRequest = 0U;   /* 中断里置位，主循环消费 */

void oled_recover_begin(void)
{
    GPIO_InitTypeDef gpio = {0};

    if (recState != REC_IDLE) {
        return;                     /* 已经在恢复流程里，不重入 */
    }

    oledRecoverRequest = 0U;
    oledTxError = 0U;
    (void)HAL_I2C_DeInit(&hi2c2);
    __HAL_RCC_I2C2_CLK_DISABLE();

    gpio.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);

    recHalfStep = 0U;
    recLastTick = HAL_GetTick();
    recState    = REC_CLOCK;
}

/* 由主循环反复调用；没有需要恢复的总线时立刻返回。 */
void oled_recover_poll(void)
{
    GPIO_InitTypeDef gpio = {0};

    if ((recState == REC_IDLE) && (oledRecoverRequest != 0U)) {
        oledRecoverRequest = 0U;
        oled_recover_begin();
    }

    if (recState == REC_IDLE) {
        return;
    }
    if ((uint32_t)(HAL_GetTick() - recLastTick) < 1U) {
        return;
    }
    recLastTick = HAL_GetTick();

    switch (recState) {
    case REC_CLOCK:
        /* 半周期计数：偶数拉低 SCL、奇数拉高，20 个半周期 = 10 个脉冲。 */
        if ((recHalfStep & 1U) == 0U) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        }
        recHalfStep++;
        if (recHalfStep >= 20U) {
            recState = REC_STOP_LOW;
        }
        break;

    case REC_STOP_LOW:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);   /* SDA 低 */
        recState = REC_STOP_HIGH;
        break;

    case REC_STOP_HIGH:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);     /* SCL 高 */
        recState = REC_RESTORE;
        break;

    case REC_RESTORE:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);     /* SDA 高 = STOP */

        gpio.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
        gpio.Mode  = GPIO_MODE_AF_OD;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &gpio);

        __HAL_RCC_I2C2_CLK_ENABLE();
        MX_I2C2_Init();

        recState = REC_IDLE;
        break;

    default:
        recState = REC_IDLE;
        break;
    }
}

void oled_clear(void)   /* 清帧缓冲并标记待刷新 */
{
    memset(oled_fb, 0, sizeof(oled_fb));
    oled_dirty = 1;
}

void oled_show_char(uint8_t Line, uint8_t Column, char Char)   /* ? 8?16 ?????? (?? RAM??? I2C) */
{
    uint16_t page;
    uint16_t col;
    uint8_t index;

    if ((Line < 1U) || (Line > 4U) || (Column < 1U) || (Column > 16U)) return;
    if ((Char < ' ') || (Char > '~')) Char = ' ';

    page = (uint16_t)(Line - 1U) * 2U;
    col  = (uint16_t)(Column - 1U) * 8U;
    index = (uint8_t)(Char - ' ');

    for (uint8_t i = 0U; i < 8U; i++) {
        oled_fb[page * 128U + col + i] = OLED_F8x16[index][i];
        oled_fb[(page + 1U) * 128U + col + i] = OLED_F8x16[index][i + 8U];
    }
    oled_dirty = 1U;
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

/* 帧缓冲 → SSD1306：窗口命令与帧数据合并成一次非阻塞 IT 传输（1032 B ≈ 23 ms @400 kHz）。
   启动失败时保留 dirty，下一轮主循环重试。 */
void oled_flush(void)
{
    static uint8_t flushState;

    if (flushState == 1U) {
        if (oledTxError != 0U) {
            flushState = 0U;
            oled_dirty = 1U;
            return;
        }
        if (oled_busy() != 0U) return;

        oled_build_frame();
        if (oled_start_tx(oled_frame_buf, (uint16_t)sizeof(oled_frame_buf)) != 0U) {
            oled_dirty = 0U;
            flushState = 2U;
        }
        return;
    }

    if (flushState == 2U) {
        if (oledTxError != 0U) {
            flushState = 0U;
            oled_dirty = 1U;
        } else if (oled_busy() == 0U) {
            flushState = 0U;
        }
        return;
    }

    if ((oled_dirty == 0U) || (oled_busy() != 0U)) return;

    oled_build_window();
    if (oled_start_tx(oled_window_buf, (uint16_t)sizeof(oled_window_buf)) != 0U) {
        flushState = 1U;
    }
}

/* I2C 发送完成回调 (ISR)：HAL 已把状态置回 READY，无需额外操作。 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
}

/* I2C 错误回调 (ISR)：只置标志，真正的总线恢复交给主循环的 oled_recover_poll()，
   避免在中断里关/开外设时钟、重配 GPIO。 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) {
        return;
    }
    oledTxError = 1U;
    oledRecoverRequest = 1U;
}

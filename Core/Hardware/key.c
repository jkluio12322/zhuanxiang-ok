/**
 * @file    key.c
 * @brief   非阻塞按键状态机 (移植自平衡车项目, 适配 HAL 库)
 * @note    Key_Tick() 每 1ms 调用 (由 TIM6 中断驱动)
 *
 *          引脚映射:
 *            KEY1: PC7 (ADD/前进)
 *            KEY2: PC6 (DEC/后退)
 *            KEY3: PG8 (SET/确认)
 *
 *          状态机周期 20ms → 消抖 20ms
 *          长按阈值: 1000ms
 *          连发间隔: 100ms
 */

#include "key.h"

#define KEY_PRESSED     1
#define KEY_UNPRESSED   0

#define KEY_TIME_DOUBLE  0
#define KEY_TIME_LONG    1000
#define KEY_TIME_REPEAT  100

uint8_t Key_Flag[KEY_COUNT];

/**
 * @brief  按键 GPIO 初始化 (上拉输入)
 * @note   GPIO 时钟已在 MX_GPIO_Init 中使能, 此处再次初始化以防万一
 */
void Key_Init(void)
{
    /* PC7(KEY1), PC6(KEY2), PG8(KEY3) 已在 CubeMX MX_GPIO_Init 中配置为输入上拉 */
}

/**
 * @brief  读取单个按键的物理电平
 * @param  n 按键编号 (0=KEY1, 1=KEY2, 2=KEY3)
 * @return KEY_PRESSED(1) 或 KEY_UNPRESSED(0)
 */
static uint8_t Key_GetState(uint8_t n)
{
    if (n == KEY_1) {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET)  /* PC7 低电平=按下 */
            return KEY_PRESSED;
    }
    else if (n == KEY_2) {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_RESET)  /* PC6 */
            return KEY_PRESSED;
    }
    else if (n == KEY_3) {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET)  /* PC8, CubeMX 已配上拉 */
            return KEY_PRESSED;
    }
    return KEY_UNPRESSED;
}

/**
 * @brief  查询按键事件
 * @param  n    按键编号
 * @param  Flag 事件类型 (KEY_SINGLE/KEY_LONG/KEY_HOLD...)
 * @return 1=事件发生, 0=未发生
 * @note   除 KEY_HOLD 外, 其他事件读取后自动清除
 */
uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
    if (Key_Flag[n] & Flag) {
        if (Flag != KEY_HOLD) {
            Key_Flag[n] &= ~Flag;
        }
        return 1;
    }
    return 0;
}

/**
 * @brief  清除所有按键事件标志
 */
void Key_Clear(void)
{
    uint8_t i;
    for (i = 0; i < KEY_COUNT; i++) {
        Key_Flag[i] = 0;
    }
}

/**
 * @brief  按键状态机 (每 1ms 调用一次)
 * @note   状态机说明:
 *         S=0: 空闲, 等待按下
 *         S=1: 按下消抖中, 超时 → KEY_LONG
 *         S=2: 释放等待, 超时 → KEY_SINGLE, 再次按下 → KEY_DOUBLE
 *         S=3: 双击后等待释放
 *         S=4: 长按连发, 超时 → KEY_REPEAT
 */
void Key_Tick(void)
{
    static uint8_t Count, i;
    static uint8_t CurrState[KEY_COUNT], PrevState[KEY_COUNT];
    static uint8_t S[KEY_COUNT];
    static uint16_t Time[KEY_COUNT];

    /* 计时器递减 */
    for (i = 0; i < KEY_COUNT; i++) {
        if (Time[i] > 0) Time[i]--;
    }

    /* 每 20ms 采样一次 (消抖) */
    Count++;
    if (Count >= 20) {
        Count = 0;

        for (i = 0; i < KEY_COUNT; i++) {
            PrevState[i] = CurrState[i];
            CurrState[i] = Key_GetState(i);

            /* 持续按住标志 */
            if (CurrState[i] == KEY_PRESSED) //0000 0001
                Key_Flag[i] |= KEY_HOLD;     //0000 0001
            else
                Key_Flag[i] &= ~KEY_HOLD;    //0000 0000

            /* 按下边沿 */
            if (CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED)
                Key_Flag[i] |= KEY_DOWN;

            /* 抬起边沿 */
            if (CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED)
                Key_Flag[i] |= KEY_UP;

            /* --- 状态机 --- */
            if (S[i] == 0) {
                /* 空闲 → 检测按下 */
                if (CurrState[i] == KEY_PRESSED) {
                    Time[i] = KEY_TIME_LONG;
                    S[i] = 1;
                }
            }
            else if (S[i] == 1) {
                /* 按住中 → 等待释放或超时 */
                if (CurrState[i] == KEY_UNPRESSED) {
                    Time[i] = KEY_TIME_DOUBLE;
                    S[i] = 2;
                } else if (Time[i] == 0) {
                    Time[i] = KEY_TIME_REPEAT;
                    Key_Flag[i] |= KEY_LONG;
                    S[i] = 4;
                }
            }
            else if (S[i] == 2) {
                /* 已释放 → 等待双击确认或超时判定单击 */
                if (CurrState[i] == KEY_PRESSED) {
                    Key_Flag[i] |= KEY_DOUBLE;
                    S[i] = 3;
                } else if (Time[i] == 0) {
                    Key_Flag[i] |= KEY_SINGLE;
                    S[i] = 0;
                }
            }
            else if (S[i] == 3) {
                /* 双击后 → 等待释放回空闲 */
                if (CurrState[i] == KEY_UNPRESSED)
                    S[i] = 0;
            }
            else if (S[i] == 4) {
                /* 长按中 → 连发或释放 */
                if (CurrState[i] == KEY_UNPRESSED)
                    S[i] = 0;
                else if (Time[i] == 0) {
                    Time[i] = KEY_TIME_REPEAT;
                    Key_Flag[i] |= KEY_REPEAT;
                    S[i] = 4;
                }
            }
        }
    }
}

/*============================================================
 * 兼容旧接口: 阻塞式 Key_GetNum() (仅开机画面使用)
 * 返回 0=无按键, 1=KEY1(PC7), 2=KEY2(PC6), 3=KEY3(PG8)
 *============================================================*/
uint8_t Key_GetNum(void)
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 0) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 0); HAL_Delay(20);
        return 1;
    }
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 0) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 0); HAL_Delay(20);
        return 2;
    }
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 0) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 0); HAL_Delay(20);
        return 3;
    }
    return 0;
}

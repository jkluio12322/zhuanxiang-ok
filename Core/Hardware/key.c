/**
 * @file    key.c
 * @brief   非阻塞按键状态机 (EXTI 边沿触发 + 计时状态机)
 * @note    按下/抬起: 由 EXTI 中断即时捕获
 *          长按/双击/连发: 由 Key_Tick() 20ms 状态机处理
 *          引脚: KEY1=PC7, KEY2=PC6, KEY3=PC8, KEY4=PC9
 *          按下 = 低电平, 上拉输入
 */

#include "key.h"

#define KEY_TIME_DOUBLE  0
#define KEY_TIME_LONG    1000
#define KEY_TIME_REPEAT  100

uint8_t Key_Flag[KEY_COUNT];

void Key_Init(void) {}

uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
    if (Key_Flag[n] & Flag) {
        if (Flag != KEY_HOLD)
            Key_Flag[n] &= ~Flag;
        return 1;
    }
    return 0;
}

void Key_Clear(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
        Key_Flag[i] = 0;
}

/* ════════════════════════════════════════════════════════
 * EXTI 回调: 捕获按键边沿 (I2C 引脚 GPIOC 中断上下文)
 * ════════════════════════════════════════════════════════ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t n;
    if      (GPIO_Pin == GPIO_PIN_7) n = KEY_1;
    else if (GPIO_Pin == GPIO_PIN_6) n = KEY_2;
    else if (GPIO_Pin == GPIO_PIN_8) n = KEY_3;
    else if (GPIO_Pin == GPIO_PIN_9) n = KEY_4;
    else return;

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET) {
        Key_Flag[n] |= KEY_DOWN;
        Key_Flag[n] |= KEY_HOLD;
    } else {
        Key_Flag[n] &= ~KEY_HOLD;
        Key_Flag[n] |= KEY_UP;
    }
}

/* ════════════════════════════════════════════════════════
 * 计时状态机: 20ms 调用一次 (不读 GPIO, 仅计时)
 * ════════════════════════════════════════════════════════ */

void Key_Tick(void)
{
    static uint8_t  S[KEY_COUNT];
    static uint16_t Time[KEY_COUNT];

    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        if (Time[i] > 0) Time[i]--;

        uint8_t pressed  = (Key_Flag[i] & KEY_HOLD) != 0;
        uint8_t just_dn  = (Key_Flag[i] & KEY_DOWN) != 0;
        uint8_t just_up  = (Key_Flag[i] & KEY_UP)   != 0;

        if (just_dn) Key_Flag[i] &= ~KEY_DOWN;
        if (just_up) Key_Flag[i] &= ~KEY_UP;

        switch (S[i]) {
        case 0:  /* 空闲 */
            if (just_dn) { Time[i] = KEY_TIME_LONG; S[i] = 1; }
            break;
        case 1:  /* 按住中 */
            if (just_up) { Time[i] = KEY_TIME_DOUBLE; S[i] = 2; }
            else if (Time[i] == 0) {
                Time[i] = KEY_TIME_REPEAT;
                Key_Flag[i] |= KEY_LONG;
                S[i] = 4;
            }
            break;
        case 2:  /* 已释放, 等待双击 */
            if (just_dn) { Key_Flag[i] |= KEY_DOUBLE; S[i] = 3; }
            else if (Time[i] == 0) { Key_Flag[i] |= KEY_SINGLE; S[i] = 0; }
            break;
        case 3:  /* 双击后等待释放 */
            if (just_up) S[i] = 0;
            break;
        case 4:  /* 长按连发 */
            if (just_up) S[i] = 0;
            else if (Time[i] == 0) {
                Time[i] = KEY_TIME_REPEAT;
                Key_Flag[i] |= KEY_REPEAT;
            }
            break;
        }
    }
}

/* ════════════════════════════════════════════════════════
 * 阻塞式按键读取 (仅开机画面使用)
 * ════════════════════════════════════════════════════════ */

uint8_t Key_GetNum(void)
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET); HAL_Delay(20);
        return 1;
    }
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_RESET) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_RESET); HAL_Delay(20);
        return 2;
    }
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET); HAL_Delay(20);
        return 3;
    }
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET); HAL_Delay(20);
        return 4;
    }
    return 0;
}

/**
 * @file    key.c
 * @brief   非阻塞按键输入模块
 * @note    状态机: IDLE → DEBOUNCE(20ms) → PRESS → HOLD → DEBOUNCE_RELEASE(20ms) → IDLE
 *          每 10ms 调用 Key_Scan() 推进状态机
 *          引脚: PC8=SET, PC7=ADD, PC6=DEC (上拉输入, 低电平有效)
 */

#include "key.h"

/** 按键引脚定义 */
#define KEY1_PORT  GPIOC
#define KEY2_PORT  GPIOC
#define KEY3_PORT  GPIOC
#define KEY1_PIN   GPIO_PIN_8  /**< SET */
#define KEY2_PIN   GPIO_PIN_7  /**< ADD */
#define KEY3_PIN   GPIO_PIN_6  /**< DEC */

/** 按键状态枚举 */
typedef enum {
    KS_IDLE = 0,          /**< 空闲, 等待按下 */
    KS_DEBOUNCE_PRESS,    /**< 按下消抖 (等 20ms) */
    KS_PRESS,             /**< 确认按下 (单次触发) */
    KS_HOLD,              /**< 持续按住 */
    KS_DEBOUNCE_RELEASE,  /**< 释放消抖 (等 20ms) */
} KeyState;

/** 单个按键状态机 */
typedef struct {
    KeyState state;       /**< 当前状态 */
    uint8_t  pressed : 1; /**< 边沿触发标志 (按下瞬间置1, 读取后清零) */
    uint8_t  held    : 1; /**< 持续按住标志 */
    uint8_t  debounce_cnt; /**< 消抖计数器 (×10ms) */
    GPIO_TypeDef *port;
    uint16_t pin;
} Key;

static Key key1, key2, key3;

/**
 * @brief  初始化单个按键对象
 */
static void Key_InitOne(Key *k, GPIO_TypeDef *port, uint16_t pin)
{
    k->state = KS_IDLE;
    k->pressed = 0;
    k->held = 0;
    k->debounce_cnt = 0;
    k->port = port;
    k->pin = pin;
}

/**
 * @brief  按键初始化
 */
void Key_Init(void)
{
    Key_InitOne(&key1, KEY1_PORT, KEY1_PIN);
    Key_InitOne(&key2, KEY2_PORT, KEY2_PIN);
    Key_InitOne(&key3, KEY3_PORT, KEY3_PIN);
}

/**
 * @brief  单个按键状态机扫描
 * @note   消抖时间 = 2 * 10ms = 20ms (press + release 各 20ms)
 */
static void Key_ScanOne(Key *k)
{
    uint8_t pin_now = (HAL_GPIO_ReadPin(k->port, k->pin) == GPIO_PIN_RESET) ? 0 : 1;
    /* 低电平 = 按下(0), 高电平 = 释放(1) */

    switch (k->state) {
    case KS_IDLE:
        if (pin_now == 0) {  /* 检测到低电平 → 开始消抖 */
            k->state = KS_DEBOUNCE_PRESS;
            k->debounce_cnt = 2;  /* 2 × 10ms = 20ms */
        }
        break;

    case KS_DEBOUNCE_PRESS:
        if (pin_now == 0) {
            if (--k->debounce_cnt == 0) {
                k->state = KS_PRESS;
                k->pressed = 1;  /* 边沿触发标志 */
                k->held = 1;     /* 持续按住标志 */
            }
        } else {
            k->state = KS_IDLE;  /* 抖动, 退回空闲 */
        }
        break;

    case KS_PRESS:
        k->pressed = 0;  /* 单次触发, 下一周期清零 */
        if (pin_now == 0) {
            k->state = KS_HOLD;
        } else {
            k->state = KS_DEBOUNCE_RELEASE;
            k->debounce_cnt = 2;
        }
        break;

    case KS_HOLD:
        if (pin_now == 0) {
            k->held = 1;
        } else {
            k->state = KS_DEBOUNCE_RELEASE;
            k->debounce_cnt = 2;
        }
        break;

    case KS_DEBOUNCE_RELEASE:
        if (pin_now == 0) {
            k->state = KS_HOLD;  /* 又按下去了, 抖动 */
        } else {
            if (--k->debounce_cnt == 0) {
                k->state = KS_IDLE;
                k->held = 0;
            }
        }
        break;
    }
}

/**
 * @brief  扫描全部按键 (每 10ms 调用一次)
 */
void Key_Scan(void)
{
    Key_ScanOne(&key1);
    Key_ScanOne(&key2);
    Key_ScanOne(&key3);
}

/** @brief SET 键刚按下 (边沿触发, 读取后自动清零) */
uint8_t Key1_Pressed(void) { return key1.pressed; }

/** @brief ADD 键刚按下 */
uint8_t Key2_Pressed(void) { return key2.pressed; }

/** @brief DEC 键刚按下 */
uint8_t Key3_Pressed(void) { return key3.pressed; }

/** @brief SET 键持续按住中 */
uint8_t Key1_Held(void) { return key1.held; }

/** @brief ADD 键持续按住中 */
uint8_t Key2_Held(void) { return key2.held; }

/** @brief DEC 键持续按住中 */
uint8_t Key3_Held(void) { return key3.held; }

/*============================================================
 * 兼容旧接口: 阻塞式 Key_GetNum() (仅开机画面使用)
 *============================================================*/
uint8_t Key_GetNum(void)
{
    uint8_t KeyNum = 0;

    if (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == 0) {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == 0);
        HAL_Delay(20);
        KeyNum = 1;
    }
    if (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == 0) {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == 0);
        HAL_Delay(20);
        KeyNum = 2;
    }
    if (HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN) == 0) {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN) == 0);
        HAL_Delay(20);
        KeyNum = 3;
    }
    return KeyNum;
}

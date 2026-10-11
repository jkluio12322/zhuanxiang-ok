/**
 * @file    key.c
 * @brief   非阻塞按键状态机 (EXTI 边沿触发 + 计时状态机)
 * @note    按下/抬起: 由 EXTI 中断即时捕获
 *          长按/双击/连发: 由 Key_Tick() 20ms 状态机处理
 *          引脚: KEY1=PC8, KEY2=PC7, KEY3=PC6, KEY4=PC9
 *          MPU6050 INT=PA12 (上升沿 → 数据就绪)
 *          按下 = 低电平, 上拉输入
 */

#include "key.h"

/* 双击等待时间，单位为按键扫描周期。
   注意这个 0 是**有意**的：窗口为零，抬起的那一拍就直接判单击，
   所以双击事件实际上不可达。而且即便产生也没有消费者——
   全工程没有任何 Key_Check(_, KEY_DOUBLE)（Key_Flag 里那个位只在
   Key_Tick() 里被置起，随后被 Key_Check() 的清除逻辑顺手抹掉）。
   要启用双击：把这里改成非零的窗口拍数，并在需要的地方查 KEY_DOUBLE。
   当前没有任何功能依赖双击，所以维持现状。 */
#define KEY_TIME_DOUBLE  0

/* 长按判定时间，单位为毫秒对应的扫描计数。 */
#define KEY_TIME_LONG    1000

/* 长按后的连发周期，单位为毫秒对应的扫描计数。 */
#define KEY_TIME_REPEAT  100

/* 按键事件标志，跨上下文访问，所以是 volatile：
   写它的是 EXTI 中断（置 KEY_DOWN/KEY_UP）与 20 ms 的 Key_Tick()（生成
   短按/长按/双击/连发）；读并清零它的既有 Key_Tick()，也有主循环里的
   Key_Check()/Key_CheckSingleOrLong()（开机页直接用它们轮询）。
   没有 volatile 时主循环里的 `if (Key_Flag[n] & Flag)` 可能被优化成只读一次。 */
volatile uint8_t Key_Flag[KEY_COUNT];
volatile uint8_t mpu_int_ready = 0;   /**< MPU6050 INT 数据就绪标志 */

/**
 * @brief  初始化按键模块。
 * @note   当前无需额外初始化，GPIO/EXTI 已由 CubeMX 配置。
 */
void Key_Init(void) {}

/**
 * @brief  查询指定按键事件。
 * @param  n 按键编号。
 * @param  Flag 要查询的事件标志。
 * @return 1 表示事件有效，0 表示没有该事件。
 * @note   除 KEY_HOLD 外，读取后自动清除事件，避免重复执行。
 */
uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
    if (Key_Flag[n] & Flag) {
        if (Flag != KEY_HOLD)
            Key_Flag[n] &= ~Flag;
        return 1;
    }
    return 0;
}

/**
 * @brief  取出一次短按或长按事件。
 * @param  n 按键编号 KEY_1-KEY_4。
 * @return 1 表示取得一次事件，0 表示没有短按或长按。
 * @note   一次调用只消费短按或长按中的一个，并清除连发标志。
 */
uint8_t Key_CheckSingleOrLong(uint8_t n)
{
    uint8_t event = 0U;

    if (n >= KEY_COUNT) return 0U;

    if (Key_Flag[n] & KEY_SINGLE) {
        Key_Flag[n] &= ~KEY_SINGLE;
        event = 1U;
    } else if (Key_Flag[n] & KEY_LONG) {
        Key_Flag[n] &= ~KEY_LONG;
        event = 1U;
    }

    Key_Flag[n] &= ~KEY_REPEAT;
    return event;
}

/**
 * @brief  清除所有按键事件标志。
 * @note   常用于模式切换或初始化阶段，防止旧事件残留。
 */
void Key_Clear(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
        Key_Flag[i] = 0;
}

/* ════════════════════════════════════════════════════════
 * 计时状态机: 20ms 调用一次 (不读 GPIO, 仅计时)
 * ════════════════════════════════════════════════════════ */

/**
 * @brief  按键计时状态机。
 * @note   根据 EXTI 捕获的边沿事件生成短按、双击、长按和连发事件。
 */
void Key_Tick(void)
{
    static uint8_t  S[KEY_COUNT];
    static uint16_t Time[KEY_COUNT];

    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        if (Time[i] > 0) Time[i]--;

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

/**
 * @brief  阻塞式读取当前按下的按键编号。
 * @return 1~4 表示按键编号，0 表示无按键。
 * @note   通过延时和等待松手实现消抖，仅用于开机画面。
 */
uint8_t Key_GetNum(void)
{
    if (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin) == GPIO_PIN_RESET); HAL_Delay(20);
        return 1;
    }
    if (HAL_GPIO_ReadPin(key2_GPIO_Port, key2_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(key2_GPIO_Port, key2_Pin) == GPIO_PIN_RESET); HAL_Delay(20);
        return 2;
    }
    if (HAL_GPIO_ReadPin(key3_GPIO_Port, key3_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(key3_GPIO_Port, key3_Pin) == GPIO_PIN_RESET); HAL_Delay(20);
        return 3;
    }
    if (HAL_GPIO_ReadPin(key4_GPIO_Port, key4_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); while (HAL_GPIO_ReadPin(key4_GPIO_Port, key4_Pin) == GPIO_PIN_RESET); HAL_Delay(20);
        return 4;
    }
    return 0;
}

/* ════════════════════════════════════════════════════════
 * EXTI 回调: 捕获按键边沿 + MPU6050 数据就绪中断
 * ════════════════════════════════════════════════════════ */

/**
 * @brief  GPIO EXTI 中断回调。
 * @param  GPIO_Pin 触发中断的 GPIO 引脚。
 * @note   MPU6050 中断设置数据就绪标志；按键中断捕获按下/抬起边沿。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* MPU6050 INT: PA12 上升沿 = 数据就绪 */
    if (GPIO_Pin == MPU6050_INT_Pin) {
        mpu_int_ready = 1;
        return;
    }

    uint8_t n;
    if      (GPIO_Pin == key1_Pin) n = KEY_1;
    else if (GPIO_Pin == key2_Pin) n = KEY_2;
    else if (GPIO_Pin == key3_Pin) n = KEY_3;
    else if (GPIO_Pin == key4_Pin) n = KEY_4;
    else return;

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET) {
        Key_Flag[n] |= KEY_DOWN;
        Key_Flag[n] |= KEY_HOLD;
    } else {
        Key_Flag[n] &= ~KEY_HOLD;
        Key_Flag[n] |= KEY_UP;
    }
}

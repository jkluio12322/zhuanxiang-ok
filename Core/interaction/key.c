/* 非阻塞按键状态机：EXTI 捕获按下/抬起，key_tick() 20ms 生成短按/长按/双击/连发。
   引脚 KEY1=PC8, KEY2=PC7, KEY3=PC6, KEY4=PC9，MPU6050 INT=PA12(上升沿=就绪)，按下=低电平。 */

#include "key.h"

/* 双击窗口拍数：0 = 双击不可达（有意禁用，全工程无 KEY_DOUBLE 消费者） */
#define KEY_TIME_DOUBLE  0

/* 长按判定时间（扫描计数） */
#define KEY_TIME_LONG    1000

/* 长按连发周期（扫描计数） */
#define KEY_TIME_REPEAT  100

/* 跨上下文访问须 volatile：EXTI 中断与 20ms key_tick() 写，主循环 key_check() 读 */
volatile uint8_t key_flag[KEY_COUNT];
volatile uint8_t mpu_int_ready = 0;   /* MPU6050 INT 数据就绪标志 */

void key_init(void) {}   /* 无需初始化，GPIO/EXTI 由 CubeMX 配置 */

uint8_t key_check(uint8_t n, uint8_t Flag)   /* 查询事件；除 KEY_HOLD 外读后自动清除 */
{
    if (key_flag[n] & Flag) {
        if (Flag != KEY_HOLD)
            key_flag[n] &= ~Flag;
        return 1;
    }
    return 0;
}

uint8_t key_check_single_or_long(uint8_t n)   /* 取一次短按或长按，并清除连发标志 */
{
    uint8_t event = 0U;

    if (n >= KEY_COUNT) return 0U;

    if (key_flag[n] & KEY_SINGLE) {
        key_flag[n] &= ~KEY_SINGLE;
        event = 1U;
    } else if (key_flag[n] & KEY_LONG) {
        key_flag[n] &= ~KEY_LONG;
        event = 1U;
    }

    key_flag[n] &= ~KEY_REPEAT;
    return event;
}

void key_clear(void)   /* 清除所有事件标志 */
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
        key_flag[i] = 0;
}

/* 计时状态机，20ms 调用一次（不读 GPIO，仅计时） */
void key_tick(void)
{
    static uint8_t  S[KEY_COUNT];
    static uint16_t Time[KEY_COUNT];

    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        if (Time[i] > 0) Time[i]--;

        uint8_t just_dn  = (key_flag[i] & KEY_DOWN) != 0;
        uint8_t just_up  = (key_flag[i] & KEY_UP)   != 0;

        if (just_dn) key_flag[i] &= ~KEY_DOWN;
        if (just_up) key_flag[i] &= ~KEY_UP;

        switch (S[i]) {
        case 0:  /* 空闲 */
            if (just_dn) { Time[i] = KEY_TIME_LONG; S[i] = 1; }
            break;
        case 1:  /* 按住中 */
            if (just_up) { Time[i] = KEY_TIME_DOUBLE; S[i] = 2; }
            else if (Time[i] == 0) {
                Time[i] = KEY_TIME_REPEAT;
                key_flag[i] |= KEY_LONG;
                S[i] = 4;
            }
            break;
        case 2:  /* 已释放, 等待双击 */
            if (just_dn) { key_flag[i] |= KEY_DOUBLE; S[i] = 3; }
            else if (Time[i] == 0) { key_flag[i] |= KEY_SINGLE; S[i] = 0; }
            break;
        case 3:  /* 双击后等待释放 */
            if (just_up) S[i] = 0;
            break;
        case 4:  /* 长按连发 */
            if (just_up) S[i] = 0;
            else if (Time[i] == 0) {
                Time[i] = KEY_TIME_REPEAT;
                key_flag[i] |= KEY_REPEAT;
            }
            break;
        }
    }
}

/* 阻塞读按键（延时消抖 + 等松手），仅开机页用 */
uint8_t key_get_num(void)
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

/* EXTI 回调：MPU6050 就绪置标志，按键捕获按下/抬起边沿 */
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
        key_flag[n] |= KEY_DOWN;
        key_flag[n] |= KEY_HOLD;
    } else {
        key_flag[n] &= ~KEY_HOLD;
        key_flag[n] |= KEY_UP;
    }
}

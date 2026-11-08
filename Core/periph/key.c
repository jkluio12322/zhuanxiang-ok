/* 四路按键扫描：5 ms 一拍，连续 3 拍（15 ms）电平一致才认，机械抖动不会重复触发。 */

#include "key.h"
#include "main.h"

/* 15 ms 去抖窗口；按键低有效，读到 RESET 即按下 */
#define KEY_DEBOUNCE_TICKS   3U

static const struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} s_pins[KEY_COUNT] = {
    {K1_GPIO_Port, K1_Pin},   /* PG2  */
    {K2_GPIO_Port, K2_Pin},   /* PG3  */
    {K3_GPIO_Port, K3_Pin},   /* PD14 */
    {K4_GPIO_Port, K4_Pin},   /* PD15 */
};

static uint8_t s_stable;              /* 去抖后的稳定电平，bit=1 表示按下 */
static uint8_t s_cnt[KEY_COUNT];      /* 每个键的电平变化计数 */
static uint8_t s_pending;             /* 待消费的按下沿事件 */

void key_init(void)
{
    uint8_t i;

    s_stable = 0U;
    s_pending = 0U;
    for (i = 0U; i < KEY_COUNT; i++) {
        s_cnt[i] = 0U;
    }
}

void key_tick(void)
{
    uint8_t i;

    for (i = 0U; i < KEY_COUNT; i++) {
        uint8_t raw = (HAL_GPIO_ReadPin(s_pins[i].port, s_pins[i].pin) == GPIO_PIN_RESET) ? 1U : 0U;
        uint8_t cur = (uint8_t)((s_stable >> i) & 0x01U);

        if (raw != cur) {
            s_cnt[i]++;
            if (s_cnt[i] >= KEY_DEBOUNCE_TICKS) {
                s_cnt[i] = 0U;
                s_stable ^= (uint8_t)(1U << i);
                if (raw != 0U) {
                    s_pending |= (uint8_t)(1U << i);   /* 只在按下沿发事件 */
                }
            }
        } else {
            s_cnt[i] = 0U;
        }
    }
}

uint8_t key_take_press(void)
{
    uint8_t mask;

    /* 事件由 5 ms 中断写、主循环读：摘取和清零之间不能被中断插进来，否则丢一次按键 */
    __disable_irq();
    mask = s_pending;
    s_pending = 0U;
    __enable_irq();

    return mask;
}
/* 板载按键 K1~K4：低有效，CubeMX 已配成内部上拉输入（gpio.c）。
 * 扫描放在 5 ms 控制拍里（key_tick），去抖按拍计数，不占专用定时器。
 * 消费侧拿到的是「按下沿」事件位掩码，取走即清，按住不放不会连发。 */
#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

typedef enum {
    KEY_1 = 0,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_COUNT
} KeyId;

#define KEY_BIT(id)   ((uint8_t)(1U << (id)))

void key_init(void);            /* 清状态，上电调一次 */
void key_tick(void);            /* 5 ms 控制拍调用：采样 + 去抖 + 生成按下事件 */
uint8_t key_take_press(void);   /* 取走并清空按下事件，bit0=K1 … bit3=K4 */

#endif /* __KEY_H */
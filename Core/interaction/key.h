/* 非阻塞按键状态机：短按/长按/双击/持续按住/按下/抬起；key_tick() 20ms 调用，key_check() 查询 */

#ifndef __BSP_KEY_H
#define __BSP_KEY_H  /* 头文件保护 */
#include "main.h"

#define KEY_COUNT       4

#define KEY_1           0   /* PC8 */
#define KEY_2           1   /* PC7 */
#define KEY_3           2   /* PC6 */
#define KEY_4           3   /* PC9 */

#define KEY_HOLD        0x01  /* 持续按住 */
#define KEY_DOWN        0x02  /* 刚按下(边沿) */
#define KEY_UP          0x04  /* 刚抬起(边沿) */
#define KEY_SINGLE      0x08  /* 短按 */
#define KEY_DOUBLE      0x10  /* 双击 */
#define KEY_LONG        0x20  /* 长按(>1s) */
#define KEY_REPEAT      0x40  /* 连发 */

extern volatile uint8_t key_flag[];
extern volatile uint8_t mpu_int_ready;  /* MPU6050 INT 数据就绪，ISR 置 1 */

void key_init(void);                                    /* 初始化（GPIO/EXTI 由 CubeMX 完成） */
uint8_t key_check(uint8_t n, uint8_t Flag);             /* 查询事件；KEY_HOLD 不清除，其余读后清除 */
uint8_t key_check_single_or_long(uint8_t n);               /* 取一次短按或长按，并清除连发 */
void key_clear(void);                                   /* 清除所有事件标志 */
void key_tick(void);                                    /* 计时状态机，20ms 调用一次 */
uint8_t key_get_num(void);                               /* 阻塞读按键编号，仅开机页用 */

#endif

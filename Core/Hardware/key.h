/**
 * @file    key.h
 * @brief   非阻塞按键状态机 (移植自平衡车项目)
 * @note    支持: 短按/长按/双击/持续按住/按下/抬起
 *          Key_Tick() 每 1ms 调用一次, Key_Check() 查询按键事件
 */

#ifndef __BSP_KEY_H
#define __BSP_KEY_H
#include "main.h"

#define KEY_COUNT       3

#define KEY_1           0   /**< 按键1: ADD/前进 */
#define KEY_2           1   /**< 按键2: DEC/后退 */
#define KEY_3           2   /**< 按键3: SET/确认 */

#define KEY_HOLD        0x01  /**< 持续按住 */
#define KEY_DOWN        0x02  /**< 刚按下 (边沿) 0000 0010*/ 
#define KEY_UP          0x04  /**< 刚抬起 (边沿) */
#define KEY_SINGLE      0x08  /**< 短按 (按下+抬起) */
#define KEY_DOUBLE      0x10  /**< 双击 */
#define KEY_LONG        0x20  /**< 长按 (>1s) */
#define KEY_REPEAT      0x40  /**< 连发 (长按持续触发) */

extern uint8_t Key_Flag[];

void Key_Init(void);
void Key_Tick(void);                    /**< 状态机驱动, 每 1ms 调用 */
uint8_t Key_Check(uint8_t n, uint8_t Flag);  /**< 查询按键事件 */
void Key_Clear(void);                  /**< 清除所有按键标志 */

#endif

/**
 * @file    key.h
 * @brief   非阻塞按键输入模块 (3 键)
 * @note    需要每 10ms 在主循环中调用 Key_Scan()
 *          使用边沿触发: KeyX_Pressed() 按下瞬间返回 1(自动清零)
 *          KeyX_Held() 持续按住返回 1
 */

#ifndef __BSP_KEY_H
#define __BSP_KEY_H
#include "main.h"

/** 按键 ID */
#define KEY_SET   1
#define KEY_ADD   2
#define KEY_DEC   3

void Key_Init(void);
void Key_Scan(void);             /**< 状态机扫描, 每 10ms 调用一次 */

uint8_t Key1_Pressed(void);      /**< SET 键刚按下 (边沿触发, 自动清零) */
uint8_t Key2_Pressed(void);      /**< ADD 键刚按下 */
uint8_t Key3_Pressed(void);      /**< DEC 键刚按下 */

uint8_t Key1_Held(void);         /**< SET 键持续按住 (>20ms 消抖后) */
uint8_t Key2_Held(void);         /**< ADD 键持续按住 */
uint8_t Key3_Held(void);         /**< DEC 键持续按住 */

/* 兼容旧接口 (阻塞式, 仅开机画面使用) */
uint8_t Key_GetNum(void);

#endif

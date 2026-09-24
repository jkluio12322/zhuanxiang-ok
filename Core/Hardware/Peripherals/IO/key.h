/**
 * @file    key.h
 * @brief   非阻塞按键状态机 (移植自平衡车项目)
 * @note    支持: 短按/长按/双击/持续按住/按下/抬起
 *          Key_Tick() 每 20 ms 调用一次, Key_Check() 查询按键事件
 */

#ifndef __BSP_KEY_H
#define __BSP_KEY_H  /* 头文件保护宏，防止按键接口重复声明。 */
#include "main.h"

/* 系统实际使用的按键数量。 */
#define KEY_COUNT       4

#define KEY_1           0   /**< 按键1: PC8 */
#define KEY_2           1   /**< 按键2: PC7 */
#define KEY_3           2   /**< 按键3: PC6 */
#define KEY_4           3   /**< 按键4: PC9 */

#define KEY_HOLD        0x01  /**< 持续按住 */
#define KEY_DOWN        0x02  /**< 刚按下 (边沿) 0000 0010*/ 
#define KEY_UP          0x04  /**< 刚抬起 (边沿) */
#define KEY_SINGLE      0x08  /**< 短按 (按下+抬起) */
#define KEY_DOUBLE      0x10  /**< 双击 */
#define KEY_LONG        0x20  /**< 长按 (>1s) */
#define KEY_REPEAT      0x40  /**< 连发 (长按持续触发) */

extern uint8_t Key_Flag[];
extern volatile uint8_t mpu_int_ready;  /**< MPU6050 INT 数据就绪, ISR 置 1 */

/**
 * @brief  初始化按键模块。
 * @note   当前 GPIO 和 EXTI 配置由 CubeMX 的 MX_GPIO_Init 完成。
 */
void Key_Init(void);

/**
 * @brief  查询并可选清除某个按键事件。
 * @param  n 按键编号 KEY_1~KEY_4。
 * @param  Flag 要查询的事件标志。
 * @return 1 表示事件存在，0 表示无事件。
 * @note   KEY_HOLD 查询时不清除，其余事件读取后自动清除。
 */
uint8_t Key_Check(uint8_t n, uint8_t Flag);

/**
 * @brief  查询并清除一次短按或长按事件。
 * @param  n 按键编号 KEY_1~KEY_4。
 * @return 1 表示发生了一次短按或长按，0 表示无事件。
 * @note   长按只在首次达到长按阈值时返回一次，同时清除连发标志，
 *         适合模式切换等每次按压只执行一次的操作。
 */
uint8_t Key_CheckSingleOrLong(uint8_t n);

/**
 * @brief  清除所有按键事件标志。
 */
void Key_Clear(void);

/**
 * @brief  执行一次按键计时状态机。
 * @note   固定 20 ms 调用一次，用于产生短按、长按、双击和连发事件。
 */
void Key_Tick(void);

/**
 * @brief  阻塞式读取按下的按键编号。
 * @return 1~4 表示按键编号，0 表示当前没有按键按下。
 * @note   仅用于开机页等待启动按键，不建议在周期任务中调用。
 */
uint8_t Key_GetNum(void);

#endif

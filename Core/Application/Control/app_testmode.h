/**
 * @file    app_testmode.h
 * @brief   两种开环测试模式的记账：PWM 速度测试与 PWM 循迹测试。
 * @note    本模块只管相位、计时、角度和基础 PWM 这些状态；
 *          "谁来设目标、谁来复位 PID、谁来回传遥测" 由 50 ms 控制任务决定，
 *          避免 app_testmode 和 app_speedloop 互相依赖。
 */
#ifndef __APP_TESTMODE_H
#define __APP_TESTMODE_H

#include <stdint.h>
#include "app_types.h"

/**
 * @brief  初始化测试模式状态。
 */
void AppTestMode_Init(void);

/* ---------------- PWM 速度测试：左右轮同向同速，定时换向 ---------------- */

/**
 * @brief  复位 PWM 速度测试，相位回到正转。
 */
void AppTestMode_ResetPwmSpeed(void);

/**
 * @brief  推进 PWM 速度测试，50 ms 调用一次。
 * @return 1 表示本周期相位发生了翻转。
 */
uint8_t AppTestMode_TickPwmSpeed(void);

/**
 * @brief  获取当前相位对应的方向符号。
 * @return +1 正转，-1 反转。
 */
int8_t AppTestMode_GetPwmSpeedSign(void);

/* ---------------- PWM 循迹测试：开环差速 + 可选模拟角度 ---------------- */

/**
 * @brief  复位 PWM 循迹测试，相位回到左偏，计时清零。
 */
void AppTestMode_ResetPwmTrack(void);

/**
 * @brief  推进 PWM 循迹测试，20 ms 调用一次。
 */
void AppTestMode_Task20ms(void);

/**
 * @brief  设置模拟角度，不在允许表内的值会被忽略。
 */
void AppTestMode_SetAngle(uint8_t angleDeg);

/**
 * @brief  在允许的模拟角度表里循环切换到下一个。
 */
void AppTestMode_NextAngle(void);

/**
 * @brief  获取当前模拟角度。
 */
uint8_t AppTestMode_GetAngle(void);

/**
 * @brief  调整 PWM 循迹的基础占空比，范围 0..100。
 */
void AppTestMode_StepBasePwm(int16_t step);

/**
 * @brief  获取当前基础占空比。
 */
int16_t AppTestMode_GetBasePwm(void);

/**
 * @brief  开关模拟角度输入。
 */
void AppTestMode_SetSimEnabled(uint8_t enabled);

/**
 * @brief  查询模拟角度输入是否开启。
 */
uint8_t AppTestMode_GetSimEnabled(void);

/**
 * @brief  获取 PWM 速度测试的当前相位。
 */
PwmTestPhase_t AppTestMode_GetSpeedPhase(void);

/**
 * @brief  获取 PWM 循迹测试的当前相位。
 */
PwmTestPhase_t AppTestMode_GetTrackPhase(void);

#endif /* __APP_TESTMODE_H */

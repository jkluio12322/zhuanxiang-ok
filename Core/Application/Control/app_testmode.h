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
 * @brief  复位 PWM 速度测试，相位回到正转，并**恢复自动换向**。
 * @note   也是手动转速模式的退出点：串口发了 l/r/t 之后自动相位被关掉，
 *         下次 stop 或模式切换会走到这里，自动换向随之恢复。
 */
void AppTestMode_ResetPwmSpeed(void);

/**
 * @brief  开关 PWM 速度测试的自动换向。
 * @param  enabled 非 0 恢复 5 秒自动换向，0 交出手动控制权。
 * @note   发 l/r/t 手动转速命令时会关掉它，否则跑满 5 秒目标就被自动相位覆盖了。
 *         只需关，不必手动开——AppTestMode_ResetPwmSpeed() 会恢复。
 */
void AppTestMode_SetAutoPhase(uint8_t enabled);

/**
 * @brief  推进 PWM 速度测试，50 ms 调用一次。
 * @return 1 表示本周期相位发生了翻转；自动换向已关闭时恒返回 0。
 */
uint8_t AppTestMode_TickPwmSpeed(void);

/**
 * @brief  获取当前相位对应的方向符号。
 * @return +1 正转，-1 反转。
 */
int8_t AppTestMode_GetPwmSpeedSign(void);

/* ---------------- PWM 循迹测试：开环差速 + 模拟图案 ---------------- */

/* 模拟图案的位数，等于八路通道数。AppTestMode_GetPattern() 的返回值按
   bit0..bit(n-1) 对应 IR1..IRn 展开（bit0 是最左边那一路），压黑置 1。
   显示模块按它展开成 "11110000" 这种字符串。 */
#define PATTERN_BITS 8U

/**
 * @brief  复位 PWM 循迹测试，相位回到左偏，计时清零。
 */
void AppTestMode_ResetPwmTrack(void);

/**
 * @brief  推进 PWM 循迹测试，20 ms 调用一次。
 */
void AppTestMode_Task20ms(void);

/**
 * @brief  设置要模拟的八路图案，不在允许表内的值会被忽略。
 * @note   当前没有调用者，图案由 KEY2 轮换；保留为「直接指定图案」的入口。
 */
void AppTestMode_SetPattern(uint8_t pattern);

/**
 * @brief  在允许的图案表里循环切换到下一个。
 */
void AppTestMode_NextPattern(void);

/**
 * @brief  获取当前实际注入的图案（已按相位镜像）。
 * @return bit0..7 对应 IR1..8，压黑置 1。
 */
uint8_t AppTestMode_GetPattern(void);

/**
 * @brief  调整 PWM 循迹的基础占空比，范围 0..100。
 */
void AppTestMode_StepBasePwm(int16_t step);

/**
 * @brief  获取当前基础占空比。
 */
int16_t AppTestMode_GetBasePwm(void);

/**
 * @brief  获取 PWM 速度测试的当前相位。
 */
PwmTestPhase_t AppTestMode_GetSpeedPhase(void);

/**
 * @brief  获取 PWM 循迹测试的当前相位。
 */
PwmTestPhase_t AppTestMode_GetTrackPhase(void);

/**
 * @brief  相位名，供 OLED 和串口命令行共用。
 */
const char *AppPhaseName(PwmTestPhase_t phase);

#endif /* __APP_TESTMODE_H */

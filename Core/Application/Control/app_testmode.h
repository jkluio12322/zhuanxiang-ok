/* 两种开环测试模式的记账：PWM 速度测试与 PWM 循迹测试。本模块只管状态，设目标/复位/遥测由 50 ms 任务决定。 */
#ifndef __APP_TESTMODE_H
#define __APP_TESTMODE_H

#include <stdint.h>
#include "app_types.h"

/* 初始化测试模式状态。 */
void AppTestMode_Init(void);

/* PWM 速度测试：左右轮同向同速，定时换向 */

/* 复位 PWM 速度测试：相位回正转、恢复自动换向。也是手动转速模式的退出点。 */
void AppTestMode_ResetPwmSpeed(void);

/* 开关自动换向。发 l/r/t 手动转速命令时会关掉它，否则跑满 5 秒目标被自动相位覆盖；只需关，不必手动开。 */
void AppTestMode_SetAutoPhase(uint8_t enabled);

/* 推进 PWM 速度测试，50 ms 一次。返回 1 表示本周期相位翻转；自动换向关闭时恒返回 0。 */
uint8_t AppTestMode_TickPwmSpeed(void);

/* 当前相位方向符号：+1 正转，-1 反转。 */
int8_t AppTestMode_GetPwmSpeedSign(void);

/* PWM 循迹测试：开环差速 + 模拟图案 */

/* 模拟图案位数（= 八路通道数）。AppTestMode_GetPattern() 返回值按 bit0..bit(n-1) 对应
   IR1..IRn（bit0 是最左一路），压黑置 1。显示模块按它展开成 "11110000" 字符串。 */
#define PATTERN_BITS 8U

/* 复位 PWM 循迹测试：相位回左偏、计时清零。 */
void AppTestMode_ResetPwmTrack(void);

/* 推进 PWM 循迹测试，20 ms 一次。 */
void AppTestMode_Task20ms(void);

/* 设置模拟图案，不在允许表内的值被忽略。当前无调用者，保留为「直接指定图案」的入口。 */
void AppTestMode_SetPattern(uint8_t pattern);

/* 在允许的图案表里循环切到下一个。 */
void AppTestMode_NextPattern(void);

/* 取当前实际注入的图案（已按相位镜像），bit0..7 对应 IR1..8，压黑置 1。 */
uint8_t AppTestMode_GetPattern(void);

/* 调整基础占空比，范围 0..100。 */
void AppTestMode_StepBasePwm(int16_t step);

/* 取当前基础占空比。 */
int16_t AppTestMode_GetBasePwm(void);

/* 取 PWM 速度测试当前相位。 */
PwmTestPhase_t AppTestMode_GetSpeedPhase(void);

/* 取 PWM 循迹测试当前相位。 */
PwmTestPhase_t AppTestMode_GetTrackPhase(void);

/* 取相位名，供 OLED 和串口共用。 */
const char *AppPhaseName(PwmTestPhase_t phase);

#endif /* __APP_TESTMODE_H */

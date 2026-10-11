/* 两种开环测试模式的记账：PWM 速度测试与 PWM 循迹测试。本模块只管状态，设目标/复位/遥测由 50 ms 任务决定。 */
#ifndef __TESTMODE_H
#define __TESTMODE_H

#include <stdint.h>
#include "types.h"

/* 初始化测试模式状态。 */
void testmode_init(void);

/* PWM 速度测试：左右轮同向同速，定时换向 */

/* 复位 PWM 速度测试：相位回正转、恢复自动换向。也是手动转速模式的退出点。 */
void testmode_reset_pwm_speed(void);

/* 开关自动换向。发 l/r/t 手动转速命令时会关掉它，否则跑满 5 秒目标被自动相位覆盖；只需关，不必手动开。 */
void testmode_set_auto_phase(uint8_t enabled);

/* 推进 PWM 速度测试，50 ms 一次。返回 1 表示本周期相位翻转；自动换向关闭时恒返回 0。 */
uint8_t testmode_tick_pwm_speed(void);

/* 当前相位方向符号：+1 正转，-1 反转。 */
int8_t testmode_get_pwm_speed_sign(void);

/* PWM 循迹测试：开环差速 + 模拟图案 */

/* 模拟图案位数（= 八路通道数）。testmode_get_pattern() 返回值按 bit0..bit(n-1) 对应
   IR1..IRn（bit0 是最左一路），压黑置 1。显示模块按它展开成 "11110000" 字符串。 */
#define PATTERN_BITS 8U

/* 复位 PWM 循迹测试：相位回左偏、计时清零。 */
void testmode_reset_pwm_track(void);

/* 推进 PWM 循迹测试，20 ms 一次。 */
void testmode_task(void);

/* 设置模拟图案，不在允许表内的值被忽略。当前无调用者，保留为「直接指定图案」的入口。 */
void testmode_set_pattern(uint8_t pattern);

/* 在允许的图案表里循环切到下一个。 */
void testmode_next_pattern(void);

/* 取当前实际注入的图案（已按相位镜像），bit0..7 对应 IR1..8，压黑置 1。 */
uint8_t testmode_get_pattern(void);

/* 调整基础占空比，范围 0..100。 */
void testmode_step_base_pwm(int16_t step);

/* 取当前基础占空比。 */
int16_t testmode_get_base_pwm(void);

/* 取 PWM 速度测试当前相位。 */
PwmTestPhase_t testmode_get_speed_phase(void);

/* 取 PWM 循迹测试当前相位。 */
PwmTestPhase_t testmode_get_track_phase(void);

/* 取相位名，供 OLED 和串口共用。 */
const char *phase_name(PwmTestPhase_t phase);

#endif /* __TESTMODE_H */

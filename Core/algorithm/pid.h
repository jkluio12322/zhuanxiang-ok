/* 增量式 PI（速度环专用）。
 *
 * 为什么是增量式：抗饱和天然成立 —— 状态就是输出本身，输出被限幅后下一次累加
 * 自然从限幅值继续，不会像位置式那样积分项独自跑到天上、退饱和要等好几秒。
 *
 *   du  = kp * (e - e_prev) + ki * e * dt_s
 *   out = clamp(out + du, out_min, out_max)
 *
 * kp / ki 是「每拍增量」量纲：kp 作用在误差变化量上、ki 作用在误差上，
 * 与位置式的换算关系是 kp = kp_pos、ki = ki_pos * dt（dt 取标称采样周期）。
 * 没有 D 项：差分对测速量化噪声太敏感，加了只会让闭环变硬、电机发热异响。
 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;

    float prev_error;   /* 上一拍的误差，增量式只需要这一个历史量 */

    float out_min;
    float out_max;

    float target;       /* 最近一次的设定值，供显示/遥测 */
    float actual;       /* 最近一次的实际值，供显示/遥测 */
    float out;          /* 最近一次的输出（即累加后的占空比），供显示/遥测 */
} Pid;

void pid_init(Pid *p, float kp, float ki, float out_min, float out_max);
void pid_set_gains(Pid *p, float kp, float ki);
void pid_reset(Pid *p);
float pid_update(Pid *p, float target, float actual, float dt_s);

#endif /* __PID_H */

/* 速度环：左右轮各一个增量式 PI，把目标转速换成 PWM 交给 motion 层。 */
#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>
#include "pid.h"

void control_init(void);

/* 2 ms 采样拍调用（SPEED_LOOP_IN_SAMPLE_ISR=1 时是唯一入口）：
   测速快照 → 目标斜坡 → 增量 PI → 电机。dt_us 是本拍实测间隔。 */
void control_speed_task(uint32_t dt_us);

/* 5 ms 控制拍调用，仅在 SPEED_LOOP_IN_SAMPLE_ISR=0 时有效（兼容旧时序）。 */
void control_task(uint32_t dt_us);

/* 只写左右轮目标（原子），不改运行模式。
   循迹/全白转角都走这里 —— 它比直接改 st->target_* 安全：
   控制拍写目标的同时采样拍正在读，两个 float 必须一起更新。 */
void control_apply_targets(float left_rpm, float right_rpm);

/* 串口手动轮速：额外把模式切到 MANUAL。 */
void control_set_manual(float left_rpm, float right_rpm);

/* 运行期整定速度环（增量式，两个参数）。 */
void control_set_gains(float kp, float ki);

/* 只读诊断：把当前速度环增益取出来，串口 d 页用来查「增益是不是被清成 0」。 */
void control_get_gains(float *kp, float *ki);

/* 立即停：目标清零 + 清 PID + 短路制动，不走斜坡。
   转角到点用它，斜坡会让轮子在「已经到位」之后还推 0.1 s，车身又滑 1 度。 */
void control_hard_stop(void);

const Pid *control_pid_left(void);
const Pid *control_pid_right(void);

#endif /* __CONTROL_H */

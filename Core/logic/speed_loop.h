/* 快速速度控制、模式分发和电机输出。这是唯一直接驱动电机的模块。 */
#ifndef __SPEED_LOOP_H
#define __SPEED_LOOP_H

#include <stdint.h>
#include "pid.h"

/* 速度环通道。只有左右两个环，没有平均环。 */
typedef enum {
    SPEED_CH_LEFT = 0,
    SPEED_CH_RIGHT
} SpeedChannel_t;

/* 供 show 命令打印的速度环信息。 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float actual;
    float out;
} SpeedPidInfo_t;

/* 初始化左右两个速度环。 */
void speed_loop_Init(void);

/* 执行一次 5 ms 控制任务。TIM6 中断里调用。 */
void speed_loop_task(void);

/* 推进测试阶段等低频状态，每 50 ms 由 TIM6 调用。 */
void speed_loop_slow_task(void);

/* 清空速度环运行时状态，保留增益。 */
void speed_loop_ResetRuntime(void);

/* 停车：清运行标志、清目标、复位速度环、关电机。 */
void speed_loop_ResetMotion(void);

/* 复位整个模式：停车 + 复位循迹 + 复位测试模式。 */
void speed_loop_ResetModeState(void);

/* 读一个速度环的增益与运行状态。只读，无在线改增益接口。 */
void speed_loop_GetInfo(SpeedChannel_t channel, SpeedPidInfo_t *out);

/* 停车整定：修改左/右速度环的一个增益并清运行时状态。 */
uint8_t speed_loop_SetGain(SpeedChannel_t channel, PidField_t field, float value);

#endif /* __SPEED_LOOP_H */

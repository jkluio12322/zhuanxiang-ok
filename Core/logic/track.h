/* 八路循迹：归一化质心 + 单个 PD 位置环 + 计时计圈。误差取归一化质心 e_y，单位置式 PID 出差速，目标居中。 */
#ifndef __TRACK_H
#define __TRACK_H

#include <stdint.h>
#include "pid.h"

/* 循迹状态机状态。 */
typedef enum {
    TRK_ACQUIRE = 0,   /* 起步直行，直到线路居中或超时。 */
    TRK_FOLLOW,        /* 正常循迹。 */
    TRK_LOST,          /* 确认丢线，保持丢线前的舵量。 */
    TRK_STOPPED        /* 已停车。 */
} TrackState_t;

/* 只读循迹快照，字段直接读。 */
typedef struct {
    float        base_rpm;   /* 本周期使用的基础速度，即当前档位速度。 */
    float        steer_rpm;  /* 循迹环输出的差速量，单位 RPM。 */
    float        e_y;        /* 当前归一化横向偏差，范围约 ±7。 */
    TrackState_t state;      /* 当前状态机状态。 */
    uint8_t      pattern;    /* 八路黑线掩码，bit0..7 对应 IR1..8，压黑置 1。转发自驱动层，
                               供显示模块画红外状态，免得显示直接读 ir_data。 */
} TrackOutput_t;

/* 取只读循迹快照。 */
const TrackOutput_t *track_output(void);

/* 初始化循迹环。 */
void track_init(void);

/* 启动循迹，进入起步直行阶段。 */
void track_start(void);

/* 复位状态机、PID 和计时器。 */
void track_reset_state(void);

/* 执行一次 20 ms 循迹任务。TIM6 中断里调用。 */
void track_task(void);

/* 读循迹环增益。只读，无在线改增益接口。 */
float track_get_gain(PidField_t field);

/* 设「差速 RPM 换算 PWM 百分比」增益，供 PWM 循迹测试用。 */
void track_set_steer_rpm_to_pwm_gain(float gain);

/* 读上面的换算增益。 */
float track_get_steer_rpm_to_pwm_gain(void);

/* PWM 循迹测试一步：把一段八路图案当传感器读数跑一遍循迹。返回 1 表示本周期没算差速。
   与实车共用同一 sample/状态机/确认计数/PID 限幅/lastSteer，只是不读 GPIO、不过弯计圈。TIM6 20 ms 时隙。 */
uint8_t track_test_step20ms(uint8_t pattern);

/* 写本周期差速输出，供 PWM 循迹测试发布用。 */
void track_set_pwm_steer(float steerRpm);

/* 读当前差速输出。 */
float track_get_steer(void);

#endif /* __TRACK_H */

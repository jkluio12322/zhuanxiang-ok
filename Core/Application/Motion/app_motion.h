/* 底盘运动：编码器测速、中值滤波、电机输出。A=物理右轮 / B=物理左轮 交叉映射只在本模块出现。 */
#ifndef __APP_MOTION_H
#define __APP_MOTION_H

#include <stdint.h>

/* 只读运动视图，字段直接读，不另配 getter。 */
typedef struct {
    float   left_rpm;        /* 左轮速度，中值滤波后，单位 RPM。 */
    float   right_rpm;       /* 右轮速度，中值滤波后，单位 RPM。 */
    float   left_rpm_raw;    /* 左轮速度，原始值，供遥测对比。 */
    float   right_rpm_raw;   /* 右轮速度，原始值，供遥测对比。 */
    int16_t left_pwm;        /* 上一次实际输出的左轮 PWM。 */
    int16_t right_pwm;       /* 上一次实际输出的右轮 PWM。 */
} MotionView_t;

/* 取只读运动视图。 */
const MotionView_t *AppMotion(void);

/* 初始化编码器对象。 */
void AppMotion_Init(void);

/* 读编码器并做中值滤波。50 ms 控制任务里调；滤波只此一处。 */
void AppMotion_Update(void);

/* 限幅后输出到电机。内部完成 ±MOTOR_OUTPUT 限幅和 A/B 交叉映射。 */
void AppMotion_Apply(int16_t left_pwm, int16_t right_pwm);

/* 立即停车。 */
void AppMotion_Stop(void);

#endif /* __APP_MOTION_H */

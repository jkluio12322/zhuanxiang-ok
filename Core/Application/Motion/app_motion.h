/**
 * @file    app_motion.h
 * @brief   底盘运动：编码器测速、中值滤波、电机输出。
 * @note    驱动层的 A=物理右轮 / B=物理左轮 交叉映射只在本模块出现一次。
 */
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

/**
 * @brief  获取只读运动视图。
 */
const MotionView_t *AppMotion(void);

/**
 * @brief  初始化编码器对象。
 */
void AppMotion_Init(void);

/**
 * @brief  读取编码器并做中值滤波。
 * @note   在 50 ms 控制任务里调用；滤波只此一处。
 */
void AppMotion_Update(void);

/**
 * @brief  限幅后输出到电机。
 * @param  left_pwm  左轮 PWM 百分比。
 * @param  right_pwm 右轮 PWM 百分比。
 * @note   内部完成 ±MOTOR_OUTPUT 限幅和 A/B 交叉映射。
 */
void AppMotion_Apply(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief  立即停车。
 */
void AppMotion_Stop(void);

#endif /* __APP_MOTION_H */

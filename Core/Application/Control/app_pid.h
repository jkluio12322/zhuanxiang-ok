/**
 * @file    app_pid.h
 * @brief   统一的 PID 实现，增量式（速度环）与位置式（循迹环）合一。
 * @note    取代原来的 pid.c：那里是 PID_t 和 TrackPid_t 两套并行的
 *          init / set / reset / update，现在塌缩成一套。
 */
#ifndef __APP_PID_H
#define __APP_PID_H

#include <stdint.h>

/* PID 的工作方式。 */
typedef enum {
    PID_MODE_INCREMENTAL = 0,   /* 增量式：输出累加增量，用于速度环 */
    PID_MODE_POSITIONAL         /* 位置式：输出绝对值，用于循迹转角环 */
} PidMode_t;

/* 可在线整定的增益字段。 */
typedef enum {
    PID_FIELD_KP = 0,
    PID_FIELD_KI,
    PID_FIELD_KD
} PidField_t;

/* PID 的固定配置。 */
typedef struct {
    float     kp;
    float     ki;
    float     kd;
    float     dt;                       /* 位置式积分步长，单位秒。 */
    float     out_min;
    float     out_max;
    float     zero_target_hold;         /* |目标| <= 此值时输出强制为 0。 */
    float     out_offset;               /* 输出非零时同向叠加的静摩擦补偿。 */
    PidMode_t mode;
    uint8_t   reset_on_sign_change;     /* 目标反号时清运行时状态。 */
    uint8_t   derivative_on_measurement;/* 1 = 微分作用于测量值，避免目标突变冲击。 */
} PidConfig_t;

/* PID 运行时对象。 */
typedef struct {
    PidConfig_t cfg;
    float       target;
    float       actual;
    float       actual_prev;
    float       e0;
    float       e1;
    float       e2;
    float       integral;
    float       out;
} Pid_t;

/**
 * @brief  用配置初始化 PID 并清空运行时状态。
 */
void Pid_Init(Pid_t *p, const PidConfig_t *cfg);

/**
 * @brief  清空运行时状态，保留增益与配置。
 */
void Pid_Reset(Pid_t *p);

/**
 * @brief  在线设置一个增益。
 */
void Pid_SetGain(Pid_t *p, PidField_t field, float value);

/**
 * @brief  读取一个增益。
 */
float Pid_GetGain(const Pid_t *p, PidField_t field);

/**
 * @brief  修改输出限幅，用于按模式放开或禁止反转。
 */
void Pid_SetLimits(Pid_t *p, float out_min, float out_max);

/**
 * @brief  按 目标/实测 更新一次 PID。
 * @param  p  PID 对象。
 * @param  target 目标值。
 * @param  actual 实测值。
 * @return 本次输出。
 */
float Pid_Update(Pid_t *p, float target, float actual);

/**
 * @brief  位置式的便捷入口，直接喂误差。
 * @param  p  PID 对象。
 * @param  error 误差值。
 * @return 本次输出。
 * @note   内部把 target 设为 error、actual 设为 0，微分项仍然按配置走。
 */
float Pid_UpdatePosition(Pid_t *p, float error);

#endif /* __APP_PID_H */

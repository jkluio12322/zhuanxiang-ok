/* 统一 PID：增量式（速度环）与位置式（循迹环）合一，取代原 pid.c 里两套并行实现。 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* PID 的工作方式。 */
typedef enum {
    PID_MODE_INCREMENTAL = 0,   /* 增量式：输出累加增量，用于速度环 */
    PID_MODE_POSITIONAL         /* 位置式：输出绝对值，用于循迹转角环 */
} PidMode_t;

/* 增益字段选取器，供状态显示和停车整定命令使用。 */
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

/* 用配置初始化 PID 并清空运行时状态。 */
void pid_init(Pid_t *p, const PidConfig_t *cfg);

/* 清空运行时状态，保留增益与配置。 */
void pid_reset(Pid_t *p);

/* 读一个增益；越界返回 0。 */
float pid_get_gain(const Pid_t *p, PidField_t field);

/* 修改一个非负增益；字段非法或数值非有限非负值时返回 0。 */
uint8_t pid_set_gain(Pid_t *p, PidField_t field, float value);

/* 改输出限幅，用于按模式放开或禁止反转。 */
void pid_set_limits(Pid_t *p, float out_min, float out_max);

/* 按 目标/实测 更新一次 PID（速度环用），返回本次输出。中断上下文。 */
float pid_update(Pid_t *p, float target, float actual);

/* 增量式速度环更新；integral_scale 按参考周期缩放 Ki 累积量。 */
float pid_update_scaled(Pid_t *p, float target, float actual,
                        float integral_scale);

/* 位置式便捷入口，直接喂误差（循迹环用）。内部把 target 设为 error、actual 设为 0。 */
float pid_update_position(Pid_t *p, float error);

#endif /* __PID_H */

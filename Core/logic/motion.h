/* 底盘运动：编码器测速、中值滤波、电机输出。A=物理右轮 / B=物理左轮 交叉映射只在本模块出现。 */
#ifndef __MOTION_H
#define __MOTION_H

#include <stdint.h>

/* 只读运动视图，字段直接读，不另配 getter。 */
typedef struct {
    int16_t left_encoder_delta;
    int16_t right_encoder_delta;
    float   left_rpm;        /* 左轮速度，50 ms 滑动计数窗，单位 RPM。 */
    float   right_rpm;       /* 右轮速度，50 ms 滑动计数窗，单位 RPM。 */
    float   left_rpm_raw;    /* 左轮 5 ms 原始速度，供遥测对比。 */
    float   right_rpm_raw;   /* 右轮 5 ms 原始速度，供遥测对比。 */
    float   left_angle_deg;  /* 左轮输出端等效累计角度。 */
    float   right_angle_deg; /* 右轮输出端等效累计角度。 */
    float   left_pwm;        /* 上一次实际输出的左轮 PWM 百分比。 */
    float   right_pwm;       /* 上一次实际输出的右轮 PWM 百分比。 */
} MotionView_t;

/* 取只读运动视图。 */
const MotionView_t *motion_view(void);

/* 初始化编码器对象。 */
void motion_init(void);

/* 读编码器、更新角度与 50 ms 滑动测速。每 5 ms 调用。 */
void motion_update(uint16_t elapsed_ms);
uint8_t motion_set_measure_window_ms(uint8_t window_ms);
uint8_t motion_get_measure_window_ms(void);

/* 把左右轮累计角度清零，供位置测试建立相对原点。 */
void motion_reset_position(void);

/* 限幅后输出到电机。内部完成 ±MOTOR_OUTPUT 限幅和 A/B 交叉映射。 */
void motion_apply(float left_pwm, float right_pwm);

/* 立即停车。 */
void motion_stop(void);

/* 运行期翻转极性：电机两路或编码器两路各翻一次（命令行 mot / enc）。
   翻完必须复位速度环，否则带着旧积分继续跑。 */
void motion_flip_motors(void);
void motion_flip_encoders(void);

/* 读回四路极性开关（±1），传 0 表示不关心该路。 */
void motion_get_signs(int8_t *motorLeft, int8_t *motorRight,
                      int8_t *encoderLeft, int8_t *encoderRight);

#endif /* __MOTION_H */

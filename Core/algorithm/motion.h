/* 底盘运动：编码器测速、时间加权测速窗、电机输出。
   A=物理右轮 / B=物理左轮 交叉映射只在本模块出现。 */
#ifndef __MOTION_H
#define __MOTION_H

#include <stdint.h>

/* 只读运动视图，字段直接读，不另配 getter。 */
typedef struct {
    int16_t left_encoder_delta;   /* 本采样拍的脉冲增量 */
    int16_t right_encoder_delta;
    float   left_rpm;             /* 左轮速度：时间加权滑动窗（默认 20 ms），单位 RPM */
    float   right_rpm;
    float   left_rpm_raw;         /* 左轮速度：本拍瞬时值，仅供对比/诊断 */
    float   right_rpm_raw;
    float   left_angle_deg;       /* 左轮输出端等效累计角度 */
    float   right_angle_deg;
    float   left_pwm;             /* 上一次实际输出的左轮 PWM 百分比 */
    float   right_pwm;
    float   distance_m;           /* 左右轮平均里程，取绝对值累加，单位 m */
    uint32_t window_us;           /* 当前测速窗长度（微秒），便于显示核对 */
} MotionView_t;

/* 取只读运动视图（指向最近一次已发布的快照）。单字段读取用它；
   要一次读多个字段且保证互相自洽，用 motion_snapshot()。 */
const MotionView_t *motion_view(void);

/* 取一份自洽的运动快照（seqlock）。
   返回 1 = 拷贝成功；返回 0 = 连读 3 次都被采样拍打断，此时 *out 退回上一份
   已知一致的快照，同时 state()->win_retry 自增，供显示确认真实发生过几次撕裂。 */
uint8_t motion_snapshot(MotionView_t *out);

/* 初始化编码器对象。 */
void motion_init(void);

/* 读编码器并更新测速窗。每 2 ms 采样拍调用一次。
   now_us = 本次采样时间戳，dt_us = 与上次采样之间的真实间隔（都由 sched 提供）。 */
void motion_update(uint32_t now_us, uint32_t dt_us);

/* 运行期改测速窗长度，单位 ms（基础窗；低速时内部会自动扩到 MAX）。
   返回 0 表示超范围未生效。 */
uint8_t motion_set_measure_window_ms(uint16_t window_ms);
uint16_t motion_get_measure_window_ms(void);

/* 运行期改速度低通截止频率 Hz（0 = 不滤波，上限 100）。返回 0 表示超范围。 */
uint8_t motion_set_filter_hz(uint16_t hz);
uint16_t motion_get_filter_hz(void);

/* 把左右轮累计角度与里程清零，供位置测试建立相对原点。 */
void motion_reset_position(void);

/* 上电以来的累计里程（m，取绝对值，倒车也继续加）。 */
float motion_distance_m(void);

/* 限幅后输出到电机。内部完成 ±MOTOR_OUTPUT 限幅和 A/B 交叉映射。 */
void motion_apply(float left_pwm, float right_pwm);

/* 立即停车。 */
void motion_stop(void);

/* 运行期翻转极性：电机两路或编码器两路各翻一次。
   翻完必须复位速度环，否则带着旧积分继续跑。 */
void motion_flip_motors(void);
void motion_flip_encoders(void);

/* 读回四路极性开关（±1），传 0 表示不关心该路。 */
void motion_get_signs(int8_t *motorLeft, int8_t *motorRight,
                      int8_t *encoderLeft, int8_t *encoderRight);

#endif /* __MOTION_H */

/* 左右轮、正反方向的开环 RPM-LUT 与死区补偿。 */
#ifndef __SPEED_CALIBRATION_H
#define __SPEED_CALIBRATION_H

#include <stdint.h>

typedef enum {
    SPEED_CAL_LEFT = 0,
    SPEED_CAL_RIGHT
} SpeedCalSide_t;

/* 目标 RPM -> 前馈 PWM 百分比；包含方向和死区，输出限制 ±100%。 */
float speed_calibration_feedforward(float target_rpm, SpeedCalSide_t side);

/* 标定表预测：给定带符号 PWM，返回该侧预期 RPM。 */
float speed_calibration_predict_rpm(float signed_duty, SpeedCalSide_t side);

/* OPEN-EQ 使用：按请求占空比先求左右原始平均 RPM，
   再反解成该侧需要输出的占空比，使左右预测 RPM 接近。 */
float speed_calibration_compensated_duty(float requested_duty,
                                         int8_t direction,
                                         SpeedCalSide_t side);

float speed_calibration_start_deadzone(int8_t direction, SpeedCalSide_t side);
float speed_calibration_sustain_deadzone(int8_t direction, SpeedCalSide_t side);

#endif

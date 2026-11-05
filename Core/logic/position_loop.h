/* 轮端位置 PD：角度误差 -> 目标 RPM，由内层速度 PI 执行。 */
#ifndef __POSITION_LOOP_H
#define __POSITION_LOOP_H

#include <stdint.h>

typedef enum {
    POSITION_LEFT = 0,
    POSITION_RIGHT
} PositionSide_t;

typedef struct {
    float target_deg;
    float reference_deg;
    float actual_deg;
    float error_deg;
    float velocity_ff_rpm;
    float actual_rpm;
    float rpm_cmd;
    uint8_t holding;
} PositionAxisView_t;

typedef struct {
    PositionAxisView_t axis[2];
} PositionLoopView_t;

void position_loop_init(void);

/* 开始一次相对位置运动；目标是在当前清零原点下的累计角度。 */
void position_loop_begin(float target_left_deg, float target_right_deg);

/* 开始连续轨迹跟踪，把位置参考对齐到当前轮端角度。 */
void position_loop_track_begin(float actual_left_deg, float actual_right_deg);

/* 复位目标和保持状态，不驱动电机。 */
void position_loop_reset(void);
float position_loop_get_track_kp(PositionSide_t side);
float position_loop_get_track_kd(PositionSide_t side);
uint8_t position_loop_set_track_gains(PositionSide_t side,
                                      float kp, float kd);

/* 5 ms 连续轨迹：期望轮速积分成位置参考，再由位置 PD 修正 RPM。 */
void position_loop_track_step(float desired_left_rpm, float desired_right_rpm,
                              float actual_left_deg, float actual_right_deg,
                              float actual_left_rpm, float actual_right_rpm,
                              float out_min, float out_max);

/* 定点位置控制。reference/velocity 来自阶跃或梯形规划器。 */
void position_loop_step(float reference_left_deg, float reference_right_deg,
                        float velocity_left_rpm, float velocity_right_rpm,
                        float actual_left_deg, float actual_right_deg,
                        float actual_left_rpm, float actual_right_rpm,
                        float output_limit_rpm, float approach_accel_rpm_s,
                        float brake_accel_rpm_s);

const PositionLoopView_t *position_loop_view(void);

#endif

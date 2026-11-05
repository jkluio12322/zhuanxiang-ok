/* 八路循迹：稳定分母归一化质心、转向比例 PD、陀螺仪阻尼和计圈。 */
#ifndef __TRACK_H
#define __TRACK_H

#include <stdint.h>
#include "pid.h"

typedef enum {
    TRK_FOLLOW = 1,
    TRK_LOST = 2,
    TRK_STOPPED = 3
} TrackState_t;

typedef struct {
    float        base_rpm;
    float        steer_ratio;
    float        steer_rpm;
    float        line_error;
    float        yaw_rate_dps;
    float        desired_left_rpm;
    float        desired_right_rpm;
    float        target_left_rpm;
    float        target_right_rpm;
    float        lost_yaw_deg;
    TrackState_t state;
    uint8_t      pattern;
    uint8_t      imu_active;
} TrackOutput_t;

const TrackOutput_t *track_output(void);

void track_init(void);
void track_start(void);
void track_reset_state(void);
void track_task(void);

float track_get_gain(PidField_t field);
uint8_t track_set_gain(PidField_t field, float value);
float track_get_gyro_gain(void);
uint8_t track_set_gyro_gain(float value);

void track_set_steer_rpm_to_pwm_gain(float gain);
float track_get_steer_rpm_to_pwm_gain(void);

uint8_t track_test_step(uint8_t pattern);
void track_set_pwm_steer(float steerRpm);
float track_get_steer(void);
float track_get_steer_ratio(void);
void track_wheel_targets_from_ratio(float base_rpm, float ratio,
                                    float *left_rpm, float *right_rpm);

/* Shared normalized-IR and steering entry points for EIGHT arcs. */
uint8_t track_sample_line(float *line_error, uint8_t *pattern);
float track_follow_update(float line_error);
void track_follow_reset(void);

#endif

/**
 * @file    imu_mahony.h
 * @brief   Mahony 互补滤波姿态解算 (从 MSPM0 ICM42688 移植, 适配 MPU6050)
 * @note    输入: gx/gy/gz (rad/s), ax/ay/az (g), dt (秒)
 *          输出: roll/pitch/yaw (度) — 四元数 + 重力矢量修正
 *
 *          Yaw 轴不修正 (重力矢量无法观测 Yaw), 纯靠陀螺积分 + 零偏校准
 */
#ifndef __IMU_MAHONY_H
#define __IMU_MAHONY_H

#include <stdint.h>
#include <math.h>

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngle_t;

extern EulerAngle_t chassis_angle;

typedef struct {
    float Kp;
    float Ki;
    float g_deadband;    /* 陀螺死区 rad/s (默认 0.05 = ~2.9 deg/s) */
    float bias_x;        /* 陀螺零偏 (原始 ADC 值, 量程转换前减去) */
    float bias_y;
    float bias_z;
} Mahony_Param_t;

extern Mahony_Param_t mahony_param;

void IMU_Calibrate_Zero_Attitude(void);
void IMU_Attitude_Update(float gx, float gy, float gz,
                         float ax, float ay, float az, float dt);
void IMU_Attitude_Reset(void);

/**
 * @brief 陀螺仪零偏校准 (上电静止时调用, 阻塞, 耗时约 samples*2ms)
 * @param samples  采样数 (建议 200)
 * @param delay_ms 每次采样间隔 (建议 2~5ms)
 */
void IMU_GyroBias_Calibrate(uint16_t samples, uint16_t delay_ms);

#endif

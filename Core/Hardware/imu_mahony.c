/**
 * @file    imu_mahony.c
 * @brief   Mahony 互补滤波 — 从 MSPM0 ICM42688 移植, 适配 MPU6050
 *
 *          算法流程 (每次采样):
 *            1. 陀螺死区过滤           |w| < deadband → 0
 *            2. 四元数 → 重力方向估计   (vx, vy, vz)
 *            3. 加速度归一化           (单位向量)
 *            4. 叉积求误差             (仅 Roll/Pitch, 不修正 Yaw)
 *            5. PI 修正陀螺            Kp=0.8, Ki=0.0001
 *            6. 一阶龙格-库塔更新四元数
 *            7. 四元数归一化
 *            8. 四元数 → 欧拉角 (度)
 *            9. 减去零点姿态
 */
#include "imu_mahony.h"
#include "mpu6050.h"

EulerAngle_t  chassis_angle = {0};
Mahony_Param_t mahony_param = {
    .Kp = 0.8f, .Ki = 0.0001f, .g_deadband = 0.05f,
    .bias_x = 0, .bias_y = 0, .bias_z = 0,
};

static EulerAngle_t ZeroAttitude = {0};
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float eInt[3] = {0};

void IMU_Calibrate_Zero_Attitude(void)
{
    ZeroAttitude.roll  = chassis_angle.roll;
    ZeroAttitude.pitch = chassis_angle.pitch;
    ZeroAttitude.yaw   = chassis_angle.yaw;
}

void IMU_Attitude_Reset(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    eInt[0] = 0; eInt[1] = 0; eInt[2] = 0;
}

void IMU_Attitude_Update(float gx, float gy, float gz,
                         float ax, float ay, float az, float dt)
{
    float halftime = dt * 0.5f;

    /* 1. 陀螺死区 */
    if (fabsf(gx) < mahony_param.g_deadband) gx = 0.0f;
    if (fabsf(gy) < mahony_param.g_deadband) gy = 0.0f;
    if (fabsf(gz) < mahony_param.g_deadband) gz = 0.0f;

    /* 2. 四元数 → 重力方向估计 */
    float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

    float vx = 2.0f * (q1q3 - q0q2);
    float vy = 2.0f * (q0q1 + q2q3);
    float vz = 1.0f - 2.0f * (q1q1 + q2q2);

    /* 3. 加速度归一化 */
    float norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= norm; ay *= norm; az *= norm;

    /* 4. 叉积求误差 (仅 Roll/Pitch) */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;

    /* 5. PI 修正陀螺 */
    eInt[0] += ex;
    eInt[1] += ey;

    gx += mahony_param.Kp * ex + mahony_param.Ki * eInt[0];
    gy += mahony_param.Kp * ey + mahony_param.Ki * eInt[1];

    /* 6. 一阶龙格-库塔更新四元数 */
    float q0_t = (-q1 * gx - q2 * gy - q3 * gz) * halftime;
    float q1_t = ( q0 * gx + q2 * gz - q3 * gy) * halftime;
    float q2_t = ( q0 * gy - q1 * gz + q3 * gx) * halftime;
    float q3_t = ( q0 * gz + q1 * gy - q2 * gx) * halftime;

    q0 += q0_t; q1 += q1_t; q2 += q2_t; q3 += q3_t;

    /* 7. 四元数归一化 */
    norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;

    /* 8. 四元数 → 欧拉角 (度) */
    chassis_angle.roll  = atan2f(2.0f * (q2 * q3 + q0 * q1),
                                 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.2957795f;
    chassis_angle.pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * 57.2957795f;
    chassis_angle.yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3),
                                 q0q0 + q1q1 - q2q2 - q3q3) * 57.2957795f;

    /* 9. 减去零点姿态 */
    chassis_angle.roll  -= ZeroAttitude.roll;
    chassis_angle.pitch -= ZeroAttitude.pitch;
    chassis_angle.yaw   -= ZeroAttitude.yaw;
}

void IMU_GyroBias_Calibrate(uint16_t samples, uint16_t delay_ms)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int16_t ax, ay, az, gx, gy, gz;

    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        if (i % 50 == 0) HAL_Delay(delay_ms);
    }
    mahony_param.bias_x = sum_gx / samples;
    mahony_param.bias_y = sum_gy / samples;
    mahony_param.bias_z = sum_gz / samples;
}

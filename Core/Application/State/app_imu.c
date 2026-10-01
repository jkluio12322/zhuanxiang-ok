/**
 * @file    app_imu.c
 * @brief   MPU6050 采样与姿态解算任务的实现。
 */

#include "app_imu.h"
#include "app_config.h"
#include "key.h"

extern I2C_HandleTypeDef hi2c1;

/* 姿态对象只在本文件里定义，外部通过 AppImu_Get() 只读访问。 */
static MPU6050 g_imu;

void AppImu_Init(void)
{
    MPU6050_init(&hi2c1);
    MPU6050_ReadAsync_Start();
}

const MPU6050 *AppImu_Get(void)
{
    return &g_imu;
}

void AppImu_Task10ms(void)
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;

    if (!MPU6050_ReadAsync_Done()) {
        return;
    }

    MPU6050_ReadAsync_Get(&ax, &ay, &az, &gx, &gy, &gz);

    g_imu.AccX = ax;
    g_imu.AccY = ay;
    g_imu.AccZ = az;
    g_imu.GyroX = gx;
    g_imu.GyroY = gy;
    g_imu.GyroZ = gz;

    MPU6050_AHRS_Update(&g_imu, (float)APP_SENSOR_PERIOD_MS / 1000.0f);

    mpu_int_ready = 0;
    MPU6050_ReadAsync_Start();
}

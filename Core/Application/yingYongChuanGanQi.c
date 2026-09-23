/**
 * @file    yingYongChuanGanQi.c
 * @brief   应用层 MPU6050 姿态采样任务。
 * @note    异步 I2C 完成中断只提供数据，姿态解算在 10 ms 任务中执行。
 */

#include "yingYongChuanGanQi.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "key.h"
#include "mpu6050.h"

/**
 * @brief  执行一次 MPU6050 姿态采样任务。
 * @note   异步 I2C 数据未就绪时直接返回；数据就绪后更新原始轴数据并运行 AHRS，
 *         最后无条件启动下一次异步读取。PA12 中断标志只用于诊断。
 */
void appSensorTask10ms(void)
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;

    /* 异步读取完成后再更新 AHRS，避免使用旧数据。 */
    if (!MPU6050_ReadAsync_Done()) return;

    MPU6050_ReadAsync_Get(&ax, &ay, &az, &gx, &gy, &gz);

    imu.AccX = ax;
    imu.AccY = ay;
    imu.AccZ = az;
    imu.GyroX = gx;
    imu.GyroY = gy;
    imu.GyroZ = gz;

    /* 使用固定 10 ms 周期作为姿态解算的采样时间。 */
    MPU6050_AHRS_Update(&imu, (float)APP_SENSOR_PERIOD_MS / 1000.0f);

    /*
     * 100 Hz 传感器任务负责维持异步读取链。PA12 中断只作为数据就绪诊断，
     * 即使丢失边沿也不能停止后续采样。
     */
    mpu_int_ready = 0;
    MPU6050_ReadAsync_Start();
}

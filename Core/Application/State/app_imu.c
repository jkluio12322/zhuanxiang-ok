/* MPU6050 采样与姿态解算任务。

   陀螺仪只供遥测：解算出的 yaw 由 app_telemetry.c 填进遥测快照，
   不参与任何控制（本工程没有航向闭环）。
   执行上下文：AppImu_Task10ms() 在 TIM6 中断里，10 ms 一次；姿态解算是浮点运算。 */

#include "app_imu.h"
#include "app_config.h"
#include "key.h"

extern I2C_HandleTypeDef hi2c1;

static MPU6050 g_imu;   /* 只在本文件定义，外部只能通过 AppImu_Get() 只读访问 */

/* 一次 10 ms 姿态采样。异步读取未完成时直接返回，不等待总线。
   流程：查异步读是否完成 -> 取原始六轴 -> 写入对象 -> 跑 AHRS -> 清就绪标志 -> 启动下一次读。
   无条件启动下一次读取，保证采样节奏不因某次超时而中断。 */
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

    /* 积分步长取采样周期常数，不测实际间隔——周期由 10 ms 时隙保证。 */
    MPU6050_AHRS_Update(&g_imu, (float)APP_SENSOR_PERIOD_MS / 1000.0f);

    mpu_int_ready = 0;
    MPU6050_ReadAsync_Start();
}

/* 初始化 MPU6050 并立即启动第一次异步读取。I2C1 必须先由 MX_I2C1_Init() 初始化。
   注意：驱动内部有 3 次重试，I2C 出错还会走 MPU6050_I2C_BusRecover()（时钟脉冲 + 毫秒级延时），
   IMU 未接时会把开机拖慢；函数返回 void，没有状态可查。 */
void AppImu_Init(void)
{
    MPU6050_init(&hi2c1);
    MPU6050_ReadAsync_Start();
}

/* 取只读姿态对象。yaw 由 10 ms 任务更新，读到的可能是上一周期的值。 */
const MPU6050 *AppImu_Get(void)
{
    return &g_imu;
}

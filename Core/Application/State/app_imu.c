/**
 * @file    app_imu.c
 * @brief   MPU6050 采样与姿态解算任务。
 *
 * @note    文件内排序：核心任务放最上方，对外接口放下面。
 *
 * @note    陀螺仪**只供遥测**：解算出的 yaw 由 app_telemetry.c 填进遥测快照的
 *          yawDeg 字段，不参与任何控制。本工程没有航向闭环，
 *          那个功能在 README §10「已知未实现项」里有说明。
 *
 * @note    采样用异步 I2C（MPU6050_ReadAsync_*），中断里只做「取结果 + 跑姿态解算
 *          + 启动下一次」，不阻塞等待总线。
 *
 * @note    执行上下文：AppImu_Task10ms() 在 TIM6 中断里，10 ms 一次。
 *          姿态解算是浮点运算，这是中断里少数几处浮点之一；
 *          它跑在 10 ms 时隙上，单次耗时可接受。
 */

#include "app_imu.h"
#include "app_config.h"
#include "key.h"

extern I2C_HandleTypeDef hi2c1;

/* 姿态对象只在本文件里定义，外部通过 AppImu_Get() 只读访问。 */
static MPU6050 g_imu;

/* ══════════════ 核心任务 ══════════════ */

/**
 * @brief  执行一次 10 ms 姿态采样任务 —— 本文件核心。
 * @note   在 TIM6 中断里执行。异步读取未完成时直接返回，不等待。
 * @note   流程：查异步读取是否完成 -> 取原始六轴 -> 写入对象 ->
 *         跑 AHRS -> 清就绪标志 -> 启动下一次异步读取。
 *         这里**无条件**启动下一次读取，保证采样节奏不因某次超时而中断。
 * @note   `mpu_int_ready` 由 MPU6050 的 EXTI 中断置位，在这里清零；
 *         但本任务的推进条件是 MPU6050_ReadAsync_Done() 而不是这个标志，
 *         所以它是「供外部观察/预留」的，不是流程必需——不确定：是否有
 *         别处依赖该标志做判断。
 */
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

    /* 积分步长取采样周期常数，不测实际间隔——本任务由 10 ms 时隙保证周期。 */
    MPU6050_AHRS_Update(&g_imu, (float)APP_SENSOR_PERIOD_MS / 1000.0f);

    mpu_int_ready = 0;
    MPU6050_ReadAsync_Start();
}

/* ══════════════ 对外接口 ══════════════ */

/**
 * @brief  初始化 MPU6050 并立即启动第一次异步读取。
 * @note   I2C1 必须先由 MX_I2C1_Init() 初始化。
 * @warning 初始化内部有 3 次重试，I2C 出错时还会走 MPU6050_I2C_BusRecover()
 *          （GPIO 时钟脉冲 + 毫秒级延时）。若 IMU 未接，开机会被拖慢。
 *          本函数返回 void，没有状态可查——这是驱动层的既有接口。
 */
void AppImu_Init(void)
{
    MPU6050_init(&hi2c1);
    MPU6050_ReadAsync_Start();
}

/**
 * @brief  获取只读的姿态对象。
 * @return 指向内部 MPU6050 对象的指针，调用方只读。
 * @note   浮点字段 yaw 由 10 ms 任务更新，读到的可能是上一周期的值，
 *         作为遥测显示无影响。
 */
const MPU6050 *AppImu_Get(void)
{
    return &g_imu;
}

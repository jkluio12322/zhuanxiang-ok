/**
 * @file    timer.c
 * @brief   系统毫秒计时和应用周期任务调度。
 * @note    TIM5 提供 1 ms 时间戳，TIM6 分频执行传感器、按键、循迹、
 *          速度控制和 VOFA 遥测任务。
 */

#include "timer.h"
#include "yingYongKongZhi.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "yingYongXunJi.h"
#include "yingYongYaoCe.h"
#include "key.h"
#include "mpu6050.h"

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim5;

volatile uint64_t milliseconds = 0;

/**
 * @brief  执行一次 10 ms MPU6050 姿态采样任务。
 * @note   异步 I2C 数据未就绪时直接返回；数据就绪后更新原始轴数据并运行 AHRS，
 *         最后无条件启动下一次异步读取。
 */
static void appSensorTask10ms(void)
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;

    if (!MPU6050_ReadAsync_Done()) return;

    MPU6050_ReadAsync_Get(&ax, &ay, &az, &gx, &gy, &gz);

    imu.AccX = ax;
    imu.AccY = ay;
    imu.AccZ = az;
    imu.GyroX = gx;
    imu.GyroY = gy;
    imu.GyroZ = gz;

    MPU6050_AHRS_Update(&imu, (float)APP_SENSOR_PERIOD_MS / 1000.0f);

    mpu_int_ready = 0;
    MPU6050_ReadAsync_Start();
}

/**
 * @brief  启动 TIM6 中断，用于调度应用层周期任务。
 */
void Timer_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief  启动 TIM5 中断，用于 1 ms 系统毫秒计时。
 */
void TIM5_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief  HAL 定时器周期中断统一回调。
 * @param  htim 触发中断的定时器句柄。
 * @note   TIM5 用于毫秒计数；TIM6 用于分频调度 MPU6050、按键、
 *         循迹状态机、控制循环和 VOFA 遥测发送。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* TIM5 每 1 ms 累加系统时间。 */
    if (htim->Instance == TIM5) {
        milliseconds++;
    }

    /* TIM6 以 1 ms 为基准分频出传感器、按键、循迹和控制任务。 */
    if (htim->Instance == TIM6) {
        static uint8_t sensorTick = 0;
        static uint8_t trackingTick = 0;
        static uint8_t controlTick = 0;
        static uint8_t keyTick = 0;

        /* 每 10 ms 执行一次 MPU6050 采样。 */
        if (++sensorTick >= APP_SENSOR_PERIOD_MS) {
            sensorTick = 0;

            /* 按键状态机固定 20 ms 执行一次。 */
            if (++keyTick >= (APP_KEY_TICK_PERIOD_MS
                              / APP_SENSOR_PERIOD_MS)) {
                keyTick = 0;
                Key_Tick();
            }

            appSensorTask10ms();
        }

        /* 每 20 ms 生成循迹目标，供后续 50 ms 速度控制任务使用。 */
        if (++trackingTick >= APP_TRACK_PERIOD_MS) {
            trackingTick = 0;
            appTrackingTask20ms();
        }

        if (++controlTick >= APP_CONTROL_PERIOD_MS) {
            controlTick = 0;
            appControlTask50ms();
            appTelemetrySend();
        }
    }
}

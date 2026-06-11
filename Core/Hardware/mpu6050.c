/**
 * @file    mpu6050.c
 * @brief   MPU6050 六轴传感器 I2C 驱动 (阻塞 + 非阻塞双模式)
 * @note    阻塞模式: 仅用于启动校准, HAL 阻塞 I2C, 不可进 ISR
 *          非阻塞模式: ISR 安全, 通过 HAL_I2C_Mem_Read_IT + 回调完成
 *          I2C2: PB10=SCL, PB11=SDA, 地址 0xD0
 *          量程: 加速度 ±16g, 陀螺仪 ±2000°/s
 *          采样率: 1kHz/(1+7) = 125Hz
 */

#include "mpu6050.h"
#include "mpu6050_reg.h"

extern I2C_HandleTypeDef hi2c2;

/* ── 非阻塞模式状态 ──────────────────────────────────── */
static uint8_t  mpu_async_buf[14];
static int16_t  mpu_async_ax, mpu_async_ay, mpu_async_az;
static int16_t  mpu_async_gx, mpu_async_gy, mpu_async_gz;
static volatile uint8_t mpu_async_ready = 0;   /* 1=数据就绪 */
static volatile uint8_t mpu_async_busy  = 0;   /* 1=I2C 传输中 */

/* ════════════════════════════════════════════════════════
 * 阻塞模式 (启动校准用)
 * ════════════════════════════════════════════════════════ */

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, RegAddress,
                      I2C_MEMADD_SIZE_8BIT, &Data, 1, 10);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, RegAddress,
                     I2C_MEMADD_SIZE_8BIT, &Data, 1, 10);
    return Data;
}

void MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x00);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
 * @brief 阻塞读取六轴原始数据 (仅用于启动校准, 不可进 ISR)
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT, Data, 14, 10);

    *AccX = (Data[0] << 8) | Data[1];
    *AccY = (Data[2] << 8) | Data[3];
    *AccZ = (Data[4] << 8) | Data[5];
    *GyroX = (Data[8]  << 8) | Data[9];
    *GyroY = (Data[10] << 8) | Data[11];
    *GyroZ = (Data[12] << 8) | Data[13];
}

/* ════════════════════════════════════════════════════════
 * 非阻塞模式 (ISR 安全)
 * ════════════════════════════════════════════════════════ */

/**
 * @brief 启动异步 I2C 读取, 立即返回, 完成后由回调解析
 * @note  可在 ISR 中调用, 如果上一次传输未完成则跳过
 */
void MPU6050_ReadAsync_Start(void)
{
    if (mpu_async_busy) return;           /* 上一帧还未传完, 跳过 */
    mpu_async_busy = 1;
    HAL_I2C_Mem_Read_IT(&hi2c2, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                        I2C_MEMADD_SIZE_8BIT, mpu_async_buf, 14);
}

/**
 * @brief 查询是否有新数据就绪
 * @retval 1=就绪, 0=等待中或出错
 */
uint8_t MPU6050_ReadAsync_Done(void)
{
    return mpu_async_ready;
}

/**
 * @brief 取出最新异步读取的六轴数据
 * @note  调用后自动清除 ready 标志
 */
void MPU6050_ReadAsync_Get(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                           int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    *AccX = mpu_async_ax;  *AccY = mpu_async_ay;  *AccZ = mpu_async_az;
    *GyroX = mpu_async_gx; *GyroY = mpu_async_gy; *GyroZ = mpu_async_gz;
    mpu_async_ready = 0;
}

/**
 * @brief I2C 接收完成回调 (HAL 框架调用, I2C ISR 上下文)
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        mpu_async_ax = (mpu_async_buf[0] << 8) | mpu_async_buf[1];
        mpu_async_ay = (mpu_async_buf[2] << 8) | mpu_async_buf[3];
        mpu_async_az = (mpu_async_buf[4] << 8) | mpu_async_buf[5];
        mpu_async_gx = (mpu_async_buf[8]  << 8) | mpu_async_buf[9];
        mpu_async_gy = (mpu_async_buf[10] << 8) | mpu_async_buf[11];
        mpu_async_gz = (mpu_async_buf[12] << 8) | mpu_async_buf[13];
        mpu_async_ready = 1;
        mpu_async_busy  = 0;
    }
}

/**
 * @brief I2C 错误回调 — 清除忙标志, 下一周期重试
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        mpu_async_busy = 0;
    }
}

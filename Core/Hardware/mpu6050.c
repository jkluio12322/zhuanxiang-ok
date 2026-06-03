/**
 * @file    mpu6050.c
 * @brief   MPU6050 六轴传感器 I2C 驱动 (加速度计 + 陀螺仪)
 * @note    使用 I2C2 (PB10=SCL, PB11=SDA), 地址 0xD0
 *          量程: 加速度 ±16g, 陀螺仪 ±2000°/s
 *          采样率: 1kHz / (1+9) = 100Hz
 */

#include "mpu6050.h"
#include "mpu6050_reg.h"

extern I2C_HandleTypeDef hi2c2;

/**
 * @brief  向 MPU6050 指定寄存器写入一个字节
 * @param  RegAddress 寄存器地址
 * @param  Data       写入数据
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, RegAddress,
                      I2C_MEMADD_SIZE_8BIT, &Data, 1, 10);
}

/**
 * @brief  从 MPU6050 指定寄存器读取一个字节
 * @param  RegAddress 寄存器地址
 * @return 读取的数据
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, RegAddress,
                     I2C_MEMADD_SIZE_8BIT, &Data, 1, 10);
    return Data;
}

/**
 * @brief  初始化 MPU6050
 * @note   配置: 唤醒 → 关闭电源管理 → 采样率分频=9 → DLPF=6(5Hz) →
 *          陀螺仪 ±2000°/s → 加速度计 ±16g
 */
void MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   /* 唤醒, 时钟选 X 轴陀螺仪 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   /* 全部传感器使能 */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);   /* 采样率 = 1kHz/(1+9) = 100Hz */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);        /* DLPF = 6 (5Hz 带宽) */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);   /* 陀螺仪 ±2000°/s */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);  /* 加速度 ±16g */
}

/**
 * @brief  读取 MPU6050 设备 ID
 * @return WHO_AM_I 寄存器值 (应为 0x68)
 */
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
 * @brief  读取 MPU6050 六轴原始数据
 * @param[out] AccX/AccY/AccZ  三轴加速度原始值
 * @param[out] GyroX/GyroY/GyroZ 三轴陀螺仪原始值
 * @note    加速度原始值需除以 2048 得到 g 值 (±16g 量程)
 *          陀螺仪原始值需除以 16.4 得到 °/s (±2000°/s 量程)
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH, DataL;

    /* 加速度 X 轴 (高字节在前) */
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *AccX = (DataH << 8) | DataL;

    /* 加速度 Y 轴 */
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY = (DataH << 8) | DataL;

    /* 加速度 Z 轴 */
    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = (DataH << 8) | DataL;

    /* 陀螺仪 X 轴 */
    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;

    /* 陀螺仪 Y 轴 */
    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;

    /* 陀螺仪 Z 轴 */
    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;
}

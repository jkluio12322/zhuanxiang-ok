/**
 * @file    mpu6050.c
 * @brief   MPU6050 六轴传感器 I2C 驱动 (参考平衡车测试程序-V1.0)
 * @note    使用 I2C2 (PB10=SCL, PB11=SDA), 地址 0xD0
 *          量程: 加速度 ±16g, 陀螺仪 ±2000°/s
 *          采样率: 1kHz/(1+7) = 125Hz, DLPF 关闭
 */

#include "mpu6050.h"
#include "mpu6050_reg.h"

extern I2C_HandleTypeDef hi2c2;

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
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   /* 唤醒, 时钟选 X 轴陀螺仪 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   /* 全部传感器使能 */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);   /* 采样率 = 1kHz/(1+7) = 125Hz */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x00);        /* DLPF 关闭, 带宽 256Hz */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);   /* 陀螺仪 ±2000°/s */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);  /* 加速度 ±16g */
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
 * @brief  连续读取 MPU6050 六轴原始数据 (单次 I2C 突发读取 14 字节)
 * @note    一次性读取 ACCEL_XOUT_H ~ GYRO_ZOUT_L, 保证数据来自同一采样时刻
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

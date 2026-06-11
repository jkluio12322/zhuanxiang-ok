/**
 * @file    mpu6050.h
 * @brief   MPU6050 六轴传感器驱动头文件 (阻塞 + 非阻塞双模式)
 */

#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H
#include "main.h"

/** MPU6050 I2C 从设备地址 (AD0=0) */
#define MPU6050_ADDR  0xD0

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);

/* 阻塞模式 (仅启动校准用, 不可进 ISR) */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/* 非阻塞模式 (ISR 安全) */
void MPU6050_ReadAsync_Start(void);
uint8_t MPU6050_ReadAsync_Done(void);
void MPU6050_ReadAsync_Get(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                           int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif

/**
 * @file    mpu6050.h
 * @brief   MPU6050 六轴传感器驱动 (移植自 bilibili 小努班 V1.7)
 * @note    I2C 地址 0xD0 (AD0=0)
 *          - 陀螺 ±2000°/s, 加速度 ±16g
 *          - Madgwick 自适应 AHRS (MPU6050_Get_Angle_Plus)
 *          - 支持异步 I2C (ISR 安全)
 */

#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H
#include "main.h"

/* ── 滤波器开关 (依赖 filter.c/h, 默认关闭) ─────────── */
#define MPU6050_USE_Filter 1

/* ── I2C 地址 ─────────────────────────────────────── */
#define MPU6050_ADDRESS  0x68   /* 7-bit → HAL 调用时左移1位 */

/* ── 数据结构 ─────────────────────────────────────── */
typedef struct {
    int16_t AccX, AccY, AccZ;   /* 加速度原始 ADC */
    int16_t GyroX, GyroY, GyroZ; /* 陀螺仪原始 ADC */
    int16_t rawTemp;             /* 温度原始 ADC */
    float yaw, roll, pitch;      /* 欧拉角 (°) */
    float temp;                  /* 温度 (°C) */
    float q0, q1, q2, q3;       /* 四元数 */
} MPU6050;

/* ── 配置枚举 ─────────────────────────────────────── */
typedef enum {
    Band_256Hz = 0x00, Band_186Hz, Band_96Hz, Band_43Hz,
    Band_21Hz, Band_10Hz, Band_5Hz
} Filter_Typedef;

typedef enum {
    gyro_250  = 0x00, gyro_500  = 0x08,
    gyro_1000 = 0x10, gyro_2000 = 0x18
} GYRO_CONFIG_Typedef;

typedef enum {
    acc_2g  = 0x00, acc_4g  = 0x08,
    acc_8g  = 0x10, acc_16g = 0x18
} ACCEL_CONFIG_Typedef;

typedef enum {
    Data_Ready_EN     = 0x01,
    I2C_Master_EN     = 0x08,
    FIFO_overFolow_EN = 0x10,
    Motion_EN         = 0x40,
} INT_EN_Typedef;

typedef struct {
    uint16_t SMPLRT_Rate;
    Filter_Typedef Filter;
    GYRO_CONFIG_Typedef gyro_range;
    ACCEL_CONFIG_Typedef acc_range;
    uint8_t FIFO_EN;
    INT_EN_Typedef INT;
} MPU6050_InitTypeDef;

/* ── API ──────────────────────────────────────────── */
void MPU6050_init(I2C_HandleTypeDef *hi2c);           /* 初始化 + 校准 (阻塞) */
void MPU6050_Get_Angle_Plus(MPU6050 *mpu);            /* AHRS 解算 (含 I2C 读取) */
void MPU6050_Get_Angle(MPU6050 *mpu);                 /* 互补滤波解算 (含 I2C) */
void MPU6050_Set_Angle0(MPU6050 *mpu);                /* 当前角度归零 */
uint8_t MPU6050_ID(void);                             /* 读 WHO_AM_I */

/* ── 异步 I2C (ISR 安全) ──────────────────────────── */
void     MPU6050_ReadAsync_Start(void);
uint8_t  MPU6050_ReadAsync_Done(void);
void     MPU6050_ReadAsync_Get(int16_t *AX, int16_t *AY, int16_t *AZ,
                               int16_t *GX, int16_t *GY, int16_t *GZ);

/* ── AHRS 纯数学 (无 I2C, ISR 安全) ────────────────── */
void MPU6050_AHRS_Update(MPU6050 *mpu, float dt);     /* 用 mpu 中已有的原始数据 + dt 解算角度 */

#endif

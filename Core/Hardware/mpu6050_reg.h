/**
 * @file    mpu6050_reg.h
 * @brief   MPU6050 寄存器地址宏定义
 */

#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H

/* --- 配置寄存器 --- */
#define MPU6050_SMPLRT_DIV      0x19  /**< 采样率分频器 */
#define MPU6050_CONFIG          0x1A  /**< DLPF 配置 */
#define MPU6050_GYRO_CONFIG     0x1B  /**< 陀螺仪量程 */
#define MPU6050_ACCEL_CONFIG    0x1C  /**< 加速度计量程 */

/* --- 加速度计数据 (高/低字节) --- */
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41  /**< 温度传感器数据高字节 */
#define MPU6050_TEMP_OUT_L      0x42

/* --- 陀螺仪数据 (高/低字节) --- */
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48

/* --- 电源管理与 ID --- */
#define MPU6050_PWR_MGMT_1      0x6B  /**< 电源管理 1 (唤醒/睡眠/时钟) */
#define MPU6050_PWR_MGMT_2      0x6C  /**< 电源管理 2 (传感器使能) */
#define MPU6050_WHO_AM_I        0x75  /**< 设备 ID (应为 0x68) */

#endif

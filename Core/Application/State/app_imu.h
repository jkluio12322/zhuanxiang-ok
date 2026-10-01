/**
 * @file    app_imu.h
 * @brief   MPU6050 采样与姿态解算任务。
 * @note    陀螺仪只用于遥测的 yawDeg 字段，不参与任何控制。
 */
#ifndef __APP_IMU_H
#define __APP_IMU_H

#include "mpu6050.h"

/**
 * @brief  初始化 MPU6050 并启动第一次异步读取。
 */
void AppImu_Init(void);

/**
 * @brief  执行一次 10 ms 姿态采样任务。
 * @note   在 TIM6 中断里调用。
 */
void AppImu_Task10ms(void);

/**
 * @brief  获取只读的姿态对象。
 * @return 指向内部 MPU6050 对象的指针，调用方只读。
 */
const MPU6050 *AppImu_Get(void);

#endif /* __APP_IMU_H */

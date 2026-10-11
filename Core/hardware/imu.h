/* MPU6050 采样与姿态解算任务。陀螺仪只用于遥测的 yawDeg 字段，不参与任何控制。 */
#ifndef __IMU_H
#define __IMU_H

#include "mpu6050.h"

void imu_init(void);             /* 初始化 MPU6050 并启动第一次异步读取 */
void imu_task(void);         /* 一次 10 ms 姿态采样，在 TIM6 中断里调用 */
const MPU6050 *imu_get(void);    /* 取只读姿态对象 */

#endif

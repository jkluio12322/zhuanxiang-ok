/* IMU963RA（LSM6DSR + 传感器集线器上的 QMC5883L）SPI2 采样与 yaw 积分。
   对外六轴数据已重映射为车辆坐标：+X 车头、+Y 车左、+Z 车顶。
   引脚：SPC=PB13 SDI=PB15 SDO=PB14，CS=PB12（软件片选，低有效），INT=PE15（未用中断）。 */
#ifndef __IMU_H
#define __IMU_H

#include "main.h"

typedef enum
{
    IMU_CAL_NO_SENSOR = 0,
    IMU_CAL_SETTLING,
    IMU_CAL_COLLECTING,
    IMU_CAL_READY,
    IMU_CAL_FAILED
} imu_cal_status_t;

typedef struct
{
    float   acc_g[3];       /* 车辆坐标加速度，g（±4g 量程） */
    float   gyro_raw_dps[3];/* 车辆坐标原始角速度，未减零偏，°/s */
    float   gyro_dps[3];    /* 车辆坐标角速度，已减零偏，±1000 dps 量程 */
    float   gyro_bias[3];   /* 车辆坐标零偏，°/s */
    float   yaw;            /* 航向角，度，-180..+180（telemetry.c 用这个字段名） */
    uint32_t sample_ms;     /* 最近一次有效陀螺仪样本的 HAL tick */
    uint32_t sample_dt_ms;  /* 与上一个有效样本的实际间隔 */
    float   roll_deg;       /* 横滚角，度：用加速度重力方向算，静止时可信 */
    float   pitch_deg;      /* 俯仰角，度：同上 */
    uint8_t ok;             /* 1 = 在线且零偏标定完成 */
    imu_cal_status_t cal_status;
    uint8_t mag_ok;         /* 1 = QMC5883L 已配置且芯片 ID 正确 */
    uint8_t mag_ready;      /* 1 = 最近一次后台轮询读到新数据 */
    uint8_t mag_id;         /* QMC5883L CHIP_ID，正常为 0xFF */
    uint8_t mag_step;       /* 初始化失败步骤，0 = 未开始/已成功 */
    uint8_t mag_status;     /* 最近一次 STATUS_MASTER 原始值 */
    int16_t mag_raw[3];     /* QMC 模块原生 XYZ，仅供诊断，不参与任何控制 */
} imu_data_t;

void imu_init(void);                    /* 主上下文调用：探测 + 软复位，随后由任务完成配置与标定 */
void imu_task(void);                    /* 10 ms 周期任务，由主循环的延后任务表调用（不在中断里） */
const imu_data_t *imu_get(void);        /* 取只读数据 */
uint8_t  imu_whoami(void);              /* 读 WHO_AM_I，正常为 0x6B */
uint32_t imu_err_count(void);
/* 调试：读回 CTRL1_XL/CTRL2_G/CTRL3_C 与 OUTX_L_G 起 12 字节；返回 1=总线正常 */
uint8_t imu_dbg_snapshot(uint8_t ctrl[3], uint8_t raw[12]);
uint8_t imu_dbg_burst(uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t imu_dbg_reg(uint8_t reg, uint8_t *val);           /* SPI 错误/坏帧累计次数 */
void     imu_yaw_reset(void);
uint8_t  imu_recalibrate(void);        /* 1=已开始重标定；调用方须先确认车辆静止 */

#endif

/**
 * @file    mpu6050.h
 * @brief   MPU6050 六轴姿态传感器 — 通用可移植模块
 *
 *          移植到新项目只需:
 *             1. 配置下方 [硬件配置] 区域的引脚宏
 *             2. main.c: MPU605O_init(&你的I2C句柄)
 *             3. (可选) 在 HAL_I2C_MemRxCpltCallback 中转发本模块回调
 *
 *          文件清单 (复制到新项目):
 *             mpu6050.c / mpu6050.h  — 驱动 + 卡尔曼 + Mahony PI 姿态解算
 *             filter.c  / filter.h   — 卡尔曼 / PT1 / PT2 滤波器
 */

#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H  /* 头文件保护宏，防止 MPU6050 接口重复声明。 */
#include "main.h"

/* ═══════════════════════════════════════════════════════
 * 硬件配置 — 移植时只需修改此处
 * ═══════════════════════════════════════════════════════ */

/* I2C 死锁恢复开关：STM32F1 使用 1，F4/H7/G0 可设为 0。 */
#define MPU6050_I2C_BUS_RECOVER 1

/* 是否由本模块自动定义 HAL I2C 回调；单 I2C 设备时设为 1。 */
#define MPU6050_DEFINE_I2C_CALLBACKS 1

#if MPU6050_I2C_BUS_RECOVER
/* I2C1 SCL 恢复时使用的 GPIO 端口。 */
#define MPU6050_I2C_SCL_PORT   GPIOB

/* I2C1 SCL 恢复时使用的 GPIO 引脚。 */
#define MPU6050_I2C_SCL_PIN    GPIO_PIN_6

/* I2C1 SDA 恢复时使用的 GPIO 端口。 */
#define MPU6050_I2C_SDA_PORT   GPIOB

/* I2C1 SDA 恢复时使用的 GPIO 引脚。 */
#define MPU6050_I2C_SDA_PIN    GPIO_PIN_7
#endif

/* ═══════════════════════════════════════════════════════
 * 滤波器 / I2C 地址
 * ═══════════════════════════════════════════════════════ */

/* 姿态滤波选择：1=启用卡尔曼滤波，0=直接使用原始角度。 */
#define MPU6050_USE_Filter 1

/* MPU6050 7 位 I2C 地址，AD0 接 GND 时为 0x68。 */
#define MPU6050_ADDRESS    0x68

/* ── 数据结构 ─────────────────────────────────────────── */
typedef struct {
    int16_t AccX, AccY, AccZ;      /* 三轴加速度原始值。 */
    int16_t GyroX, GyroY, GyroZ;   /* 三轴角速度原始值。 */
    int16_t rawTemp;               /* 温度传感器原始值。 */
    float yaw, roll, pitch;        /* 姿态角，单位度。 */
    float temp;                    /* 温度，单位摄氏度。 */
    float q0, q1, q2, q3;          /* Mahony PI 姿态四元数。 */
} MPU6050;

/* ── 配置枚举 ─────────────────────────────────────────── */
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
    FIFO_Disable,
    Acc_OUT   = 0x08,
    Gyro_zOUT = 0x10,
    Gyro_yOUT = 0x20,
    Gyro_xOUT = 0x40,
    Temp_OUT  = 0x80,
} FIFO_EN_Typedef;

typedef enum {
    interrupt_Disable,
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
    FIFO_EN_Typedef FIFO_EN;
    INT_EN_Typedef INT;
} MPU6050_InitTypeDef;

#ifdef HAL_I2C_MODULE_ENABLED

/* ── 阻塞式 API ───────────────────────────────────────── */
/**
 * @brief  初始化 MPU6050 和关联 I2C 总线。
 * @param  hi2c 使用的 I2C 句柄。
 */
void    MPU6050_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  连续读取并更新姿态角。
 * @param  mpu MPU6050 对象。
 */
void    MPU6050_Get_Angle_Plus(MPU6050 *mpu);

/**
 * @brief  读取一次姿态角。
 * @param  mpu MPU6050 对象。
 */
void    MPU6050_Get_Angle(MPU6050 *mpu);

/**
 * @brief  将当前姿态角设置为零点。
 * @param  mpu MPU6050 对象。
 */
void    MPU6050_Set_Angle0(MPU6050 *mpu);

/**
 * @brief  读取 MPU6050 WHO_AM_I 寄存器。
 * @return 设备 ID。
 */
uint8_t MPU6050_ID(void);

/**
 * @brief  读取并换算芯片温度。
 * @param  mpu MPU6050 对象。
 * @return 温度值，单位摄氏度。
 */
float   MPU6050_GetTemp(MPU6050 *mpu);

/**
 * @brief  执行 MPU6050 自检。
 * @param  accel_g 输出三轴加速度偏差。
 * @param  gyro_dps 输出三轴角速度偏差。
 */
void    MPU6050_SelfTest(float accel_g[3], float gyro_dps[3]);

/* ── 异步 I2C (ISR 安全) ──────────────────────────────── */
/**
 * @brief  启动一次异步原始数据读取。
 */
void    MPU6050_ReadAsync_Start(void);

/**
 * @brief  查询异步读取是否完成。
 * @return 1 表示完成，0 表示仍在读取。
 */
uint8_t MPU6050_ReadAsync_Done(void);

/**
 * @brief  获取最近一次异步读取的原始数据。
 */
void    MPU6050_ReadAsync_Get(int16_t *AX, int16_t *AY, int16_t *AZ,
                              int16_t *GX, int16_t *GY, int16_t *GZ);

/* ── AHRS 纯数学 (ISR 安全) ───────────────────────────── */
/**
 * @brief  使用 Mahony PI 互补滤波更新姿态角。
 * @param  mpu MPU6050 对象。
 * @param  dt 采样周期，单位秒。
 */
void    MPU6050_AHRS_Update(MPU6050 *mpu, float dt);

/* ── I2C 回调 (由 HAL_I2C_MemRxCpltCallback / HAL_I2C_ErrorCallback 转发) ── */
/**
 * @brief  处理 MPU6050 异步读取完成事件。
 * @param  hi2c 完成读取的 I2C 句柄。
 */
void    MPU6050_I2C_RxCplt(I2C_HandleTypeDef *hi2c);

/**
 * @brief  处理 MPU6050 异步读取错误事件。
 * @param  hi2c 发生错误的 I2C 句柄。
 */
void    MPU6050_I2C_Error(I2C_HandleTypeDef *hi2c);

#endif /* HAL_I2C_MODULE_ENABLED */
#endif /* __BSP_MPU6050_H */

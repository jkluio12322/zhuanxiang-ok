/* MPU6050 六轴姿态传感器 — 通用可移植模块。
   移植：改下方[硬件配置]引脚宏 → main.c 调 MPU6050_init() → (可选) 转发 I2C 回调。
   文件：mpu6050.c/h(驱动+卡尔曼+Mahony PI) + filter.c/h(卡尔曼/PT1/PT2)。 */

#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H  /* 头文件保护 */
#include "main.h"

/* 硬件配置（移植时只改这里） */

/* I2C 死锁恢复开关：STM32F1=1，F4/H7/G0=0 */
#define MPU6050_I2C_BUS_RECOVER 1

/* 是否由本模块自动定义 HAL I2C 回调；单设备=1 */
#define MPU6050_DEFINE_I2C_CALLBACKS 1

#if MPU6050_I2C_BUS_RECOVER
#define MPU6050_I2C_SCL_PORT   GPIOB      /* SCL 端口 */
#define MPU6050_I2C_SCL_PIN    GPIO_PIN_6 /* SCL 引脚 */
#define MPU6050_I2C_SDA_PORT   GPIOB      /* SDA 端口 */
#define MPU6050_I2C_SDA_PIN    GPIO_PIN_7 /* SDA 引脚 */
#endif

/* 姿态滤波选择：1=卡尔曼，0=原始角度 */
#define MPU6050_USE_Filter 1

/* MPU6050 7 位 I2C 地址，AD0 接 GND = 0x68 */
#define MPU6050_ADDRESS    0x68

/* 数据结构 */
typedef struct {
    int16_t AccX, AccY, AccZ;      /* 三轴加速度原始值 */
    int16_t GyroX, GyroY, GyroZ;   /* 三轴角速度原始值 */
    int16_t rawTemp;               /* 温度原始值 */
    float yaw, roll, pitch;        /* 姿态角(°) */
    float temp;                    /* 温度(℃) */
    float q0, q1, q2, q3;          /* Mahony PI 四元数 */
} MPU6050;

/* 配置枚举 */
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

/* 阻塞式 API（不可进 ISR） */
void    MPU6050_init(I2C_HandleTypeDef *hi2c);          /* 初始化 + 校准 + 滤波器初始化 */
void    MPU6050_Get_Angle_Plus(MPU6050 *mpu);           /* 读原始 + Mahony 解算 */
void    MPU6050_Get_Angle(MPU6050 *mpu);                /* 读原始 + 互补滤波解算 */
void    MPU6050_Set_Angle0(MPU6050 *mpu);               /* 置当前姿态为零点 */
uint8_t MPU6050_ID(void);                               /* 读 WHO_AM_I */
float   MPU6050_GetTemp(MPU6050 *mpu);                  /* 读温度(℃) */
void    MPU6050_SelfTest(float accel_g[3], float gyro_dps[3]);  /* 自检输出均值 */

/* 异步 I2C（ISR 安全） */
void    MPU6050_ReadAsync_Start(void);                  /* 启动 14 字节异步读 */
uint8_t MPU6050_ReadAsync_Done(void);                   /* 查询是否完成 */
void    MPU6050_ReadAsync_Get(int16_t *AX, int16_t *AY, int16_t *AZ,
                              int16_t *GX, int16_t *GY, int16_t *GZ);  /* 取异步数据 */

/* AHRS 纯数学（ISR 安全） */
void    MPU6050_AHRS_Update(MPU6050 *mpu, float dt);    /* Mahony PI 更新姿态 */

/* I2C 回调（由 HAL_I2C_MemRxCpltCallback / HAL_I2C_ErrorCallback 转发） */
void    MPU6050_I2C_RxCplt(I2C_HandleTypeDef *hi2c);    /* 异步读完成处理 */
void    MPU6050_I2C_Error(I2C_HandleTypeDef *hi2c);     /* 异步读错误处理 */

#endif /* HAL_I2C_MODULE_ENABLED */
#endif /* __BSP_MPU6050_H */

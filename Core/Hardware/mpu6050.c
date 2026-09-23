/**
 * @file    mpu6050.c
 * @brief   MPU6050 驱动、卡尔曼滤波和 Mahony PI 互补姿态解算。
 * @note    阻塞 I2C: MPU6050_init / MPU6050_Get_Angle_Plus (不可进 ISR)
 *          异步 I2C: MPU6050_ReadAsync_Start/Done/Get (ISR 安全)
 *          AHRS 纯数学: MPU6050_AHRS_Update (ISR 安全, 配合异步 I2C)
 *          滤波器: 卡尔曼 (Acc: q=1,r=10; Gyro: q=1,r=0.1)
 *          I2C 总线死锁自动恢复 (STM32F1 errata)
 */

#include "mpu6050.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

#ifdef HAL_I2C_MODULE_ENABLED

/* ═══════════════════════════════════════════════════════
 * 第0层: I2C 总线死锁恢复 (STM32F1 errata)
 * ═══════════════════════════════════════════════════════ */

I2C_HandleTypeDef *MPU6050_I2C = NULL;

/**
 * @brief  通过 GPIO 时钟脉冲恢复 I2C 总线死锁。
 * @note   STM32F1 I2C 外设异常时使用；函数包含毫秒级阻塞延时，只能在初始化
 *         或错误重试路径调用，不可在中断上下文中调用。
 */
static void MPU6050_I2C_BusRecover(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. HAL 反初始化 I2C (关闭时钟 + 释放 GPIO + 关 NVIC) */
    HAL_I2C_DeInit(MPU6050_I2C);

    /* 2. 引脚改为 GPIO 开漏输出, 模拟时钟释放总线 */
    GPIO_InitStruct.Pin   = MPU6050_I2C_SCL_PIN | MPU6050_I2C_SDA_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MPU6050_I2C_SCL_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(MPU6050_I2C_SDA_PORT, MPU6050_I2C_SDA_PIN, GPIO_PIN_SET);
    for (uint8_t i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(MPU6050_I2C_SCL_PORT, MPU6050_I2C_SCL_PIN, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(MPU6050_I2C_SCL_PORT, MPU6050_I2C_SCL_PIN, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    /* 3. STOP 条件 */
    HAL_GPIO_WritePin(MPU6050_I2C_SDA_PORT, MPU6050_I2C_SDA_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(MPU6050_I2C_SCL_PORT, MPU6050_I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(MPU6050_I2C_SDA_PORT, MPU6050_I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    /* 4. HAL 重新初始化 I2C (开时钟 + 配 GPIO AF_OD + 开 NVIC) */
    HAL_I2C_Init(MPU6050_I2C);
}

/* ═══════════════════════════════════════════════════════
 * 第1层: I2C 底层 (含重试 + 死锁恢复)
 * ═══════════════════════════════════════════════════════ */

/**
 * @brief  向 MPU6050 寄存器写入一个字节。
 * @param  reg 寄存器地址。
 * @param  data 待写入数据。
 * @note   使用阻塞 I2C，失败时最多重试 3 次。
 */
static void mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tmpdata = data;
    HAL_StatusTypeDef status;
    uint8_t retry = 3;
    do {
        status = HAL_I2C_Mem_Write(MPU6050_I2C, MPU6050_ADDRESS << 1, reg,
                                   I2C_MEMADD_SIZE_8BIT, &tmpdata, 1, 200);
        if (status == HAL_OK) return;
        if (status == HAL_BUSY) MPU6050_I2C_BusRecover();
    } while (--retry > 0);
}

/**
 * @brief  读取 MPU6050 单个寄存器。
 * @param  reg 寄存器地址。
 * @return 寄存器值；连续失败时返回 0。
 */
static uint8_t mpu6050_read_reg(uint8_t reg)
{
    uint8_t tmpdata = 0;
    HAL_StatusTypeDef status;
    uint8_t retry = 3;
    do {
        status = HAL_I2C_Mem_Read(MPU6050_I2C, MPU6050_ADDRESS << 1, reg,
                                 I2C_MEMADD_SIZE_8BIT, &tmpdata, 1, 200);
        if (status == HAL_OK) return tmpdata;
        if (status == HAL_BUSY) MPU6050_I2C_BusRecover();
    } while (--retry > 0);
    return 0;
}

/**
 * @brief  从指定寄存器连续读取多个字节。
 * @param  reg 起始寄存器地址。
 * @param  len 读取长度，单位字节。
 * @param  Data 接收缓冲区。
 * @note   读取失败时最多重试 3 次，最终仍失败则把缓冲区清零。
 */
static void mpu6050_read_reg_continue(uint8_t reg, uint8_t len, uint8_t *Data)
{
    HAL_StatusTypeDef status;
    uint8_t retry = 3;
    do {
        status = HAL_I2C_Mem_Read(MPU6050_I2C, MPU6050_ADDRESS << 1, reg,
                                  I2C_MEMADD_SIZE_8BIT, Data, len, 100);
        if (status == HAL_OK) return;
        if (status == HAL_BUSY) MPU6050_I2C_BusRecover();
    } while (--retry > 0);
    memset(Data, 0, len);
}

/* 便捷宏 */
#define MPU6050_DELAY_MS(ms)                        HAL_Delay(ms)
#define MPU6050WRITE_REG(reg, data)                 mpu6050_write_reg(reg, data)
#define MPU6050READ_REG(reg)                        mpu6050_read_reg(reg)
#define MPU6050_READ_REG_CONTINUE(reg, len, P_data) mpu6050_read_reg_continue(reg, len, P_data)

/* ═══════════════════════════════════════════════════════
 * 第2层: 寄存器地址
 * ═══════════════════════════════════════════════════════ */

#define MPU6050_SMPLRT_DIV        0x19
#define MPU6050_CONFIG            0x1A
#define MPU6050_GYRO_CONFIG       0x1B
#define MPU6050_ACCEL_CONFIG      0x1C
#define MPU6050_FIFO_EN           0x23
#define MPU6050_INTBP_CFG_REG     0x37
#define MPU6050_INT_ENABLE        0x38
#define MPU6050_ACCEL_XOUT_H      0x3B
#define MPU6050_TEMP_OUT_H        0x41
#define MPU6050_GYRO_XOUT_H       0x43
#define MPU6050_SIGNAL_PATH_RESET 0x68
#define MPU6050_USER_CTRL         0x6A
#define MPU6050_PWR_MGMT_1        0x6B
#define MPU6050_WHO_AM_I          0x75

/* ═══════════════════════════════════════════════════════
 * 第3层: 静态变量 (量程 / 零偏 / 滤波器 / AHRS 状态)
 * ═══════════════════════════════════════════════════════ */

static float   mpu6050_dt;
static float   gyroScale;   /* rad/s / LSB */
static float   accelScale;  /* g / LSB */
static int16_t gyro_zero_x, gyro_zero_y, gyro_zero_z;

static float angle_yaw   = 0;
static float angle_roll  = 0;
static float angle_pitch = 0;

/* 卡尔曼滤波器 */
#if MPU6050_USE_Filter
#include "filter.h"
static KalmanFilter Kal_filter_x, Kal_filter_y, Kal_filter_z;
static KalmanFilter Kal_filter_gx, Kal_filter_gy, Kal_filter_gz;
#endif

/* AHRS 内部状态 */
static float ahrs_q0 = 1.0f, ahrs_q1 = 0.0f, ahrs_q2 = 0.0f, ahrs_q3 = 0.0f;
static float integralX = 0, integralY = 0, integralZ = 0;
static uint16_t ahrs_init_cnt = 0;
static float unwrapped_yaw = 0.0f;
static uint8_t ahrs_first_yaw = 1;

/* ═══════════════════════════════════════════════════════
 * 第4层: 初始化 + 校准
 * ═══════════════════════════════════════════════════════ */

/**
 * @brief  采集静止陀螺仪数据并计算三轴零偏。
 * @param  calibration_samples 每轮校准采样次数；必须大于 0。
 * @return 0 表示校准后残余偏差符合要求，-1 表示校准失败。
 * @note   校准期间必须保持传感器静止，函数包含阻塞延时。
 */
static int8_t MPU6050_SoftCalibrate_Z(uint16_t calibration_samples)
{
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t GyroX, GyroY, GyroZ;
    uint8_t temp_buffer[6];

    for (uint16_t i = 0; i < calibration_samples; i++) {
        MPU6050_READ_REG_CONTINUE(MPU6050_GYRO_XOUT_H, 6, temp_buffer);
        GyroX = ((int16_t)(temp_buffer[0] << 8) | temp_buffer[1]);
        GyroY = ((int16_t)(temp_buffer[2] << 8) | temp_buffer[3]);
        GyroZ = ((int16_t)(temp_buffer[4] << 8) | temp_buffer[5]);
        gx_sum += GyroX;
        gy_sum += GyroY;
        gz_sum += GyroZ;
        MPU6050_DELAY_MS((uint32_t)(mpu6050_dt * 1000));
    }
    gyro_zero_x = (int16_t)(gx_sum / calibration_samples);
    gyro_zero_y = (int16_t)(gy_sum / calibration_samples);
    gyro_zero_z = (int16_t)(gz_sum / calibration_samples);

    /* 验证校准 */
    gx_sum = gy_sum = gz_sum = 0;
    for (uint16_t i = 0; i < 100; i++) {
        MPU6050_READ_REG_CONTINUE(MPU6050_GYRO_XOUT_H, 6, temp_buffer);
        GyroX = ((int16_t)(temp_buffer[0] << 8) | temp_buffer[1]) - gyro_zero_x;
        GyroY = ((int16_t)(temp_buffer[2] << 8) | temp_buffer[3]) - gyro_zero_y;
        GyroZ = ((int16_t)(temp_buffer[4] << 8) | temp_buffer[5]) - gyro_zero_z;
        gx_sum += GyroX;
        gy_sum += GyroY;
        gz_sum += GyroZ;
        MPU6050_DELAY_MS((uint32_t)(mpu6050_dt * 1000));
    }
    if (abs((int)(gx_sum / 100)) >= 2 ||
        abs((int)(gy_sum / 100)) >= 2 ||
        abs((int)(gz_sum / 100)) >= 2)
        return -1;
    return 0;
}

/**
 * @brief  配置 MPU6050 采样率、量程、滤波器和中断。
 * @param  cfg 初始化参数结构体。
 */
static void MPU6050_Register_init(MPU6050_InitTypeDef *cfg)
{
    float temp_acc_scale, temp_gyro_scale;
    switch (cfg->acc_range) {
        case acc_2g:  temp_acc_scale = 16384.f; break;
        case acc_4g:  temp_acc_scale = 8192.f;  break;
        case acc_8g:  temp_acc_scale = 4096.f;  break;
        default:      temp_acc_scale = 2048.f;  break;
    }
    switch (cfg->gyro_range) {
        case gyro_250:  temp_gyro_scale = 131.f;   break;
        case gyro_500:  temp_gyro_scale = 65.5f;   break;
        case gyro_1000: temp_gyro_scale = 32.8f;   break;
        default:        temp_gyro_scale = 16.4f;   break;
    }
    accelScale = 1.f / temp_acc_scale;
    gyroScale  = 0.0174533f / temp_gyro_scale;  /* deg→rad 内置 */

    MPU6050WRITE_REG(MPU6050_PWR_MGMT_1, 0x80);
    MPU6050_DELAY_MS(100);
    MPU6050WRITE_REG(MPU6050_PWR_MGMT_1, 0x00);

    uint16_t rate = cfg->SMPLRT_Rate;
    if (rate > 1000) rate = 1000;
    if (rate < 4)    rate = 4;
    uint8_t SMPLRT_DIV = (uint8_t)(1000.f / rate - 1);
    MPU6050WRITE_REG(MPU6050_SMPLRT_DIV, SMPLRT_DIV);
    MPU6050WRITE_REG(MPU6050_INTBP_CFG_REG, 0x00);
    MPU6050WRITE_REG(MPU6050_INT_ENABLE, cfg->INT);
    MPU6050WRITE_REG(MPU6050_CONFIG,     cfg->Filter);
    MPU6050WRITE_REG(MPU6050_GYRO_CONFIG,cfg->gyro_range);
    MPU6050WRITE_REG(MPU6050_ACCEL_CONFIG,cfg->acc_range);
    MPU6050WRITE_REG(MPU6050_FIFO_EN,    cfg->FIFO_EN);

    uint8_t temp = 0x00;
    if (cfg->FIFO_EN != 0x00)
        temp = 0x40;
    if ((cfg->INT & 0x01) == 0)
        temp |= 0x08;
    MPU6050WRITE_REG(MPU6050_USER_CTRL, temp);
    MPU6050WRITE_REG(MPU6050_PWR_MGMT_1, 0x01);  /* X轴陀螺时钟源 */
}

/**
 * @brief  初始化 MPU6050、校准陀螺零偏并初始化滤波器。
 * @param  hi2c 使用的 I2C 句柄，本工程为 I2C1。
 * @note   函数包含约数百毫秒的阻塞延时，只能在系统初始化阶段调用。
 */
void MPU6050_init(I2C_HandleTypeDef *hi2c)
{
    MPU6050_I2C = hi2c;

    MPU6050_InitTypeDef cfg;
    cfg.SMPLRT_Rate = 100;
    cfg.Filter      = Band_21Hz;
    cfg.gyro_range  = gyro_2000;
    cfg.acc_range   = acc_4g;
    cfg.FIFO_EN     = FIFO_Disable;
    cfg.INT         = Data_Ready_EN;
    MPU6050_Register_init(&cfg);
    mpu6050_dt = 1.f / (float)cfg.SMPLRT_Rate;

    MPU6050_DELAY_MS(300);
    for (int i = 0; i < 3; i++)
        if (MPU6050_SoftCalibrate_Z(64) == 0)
            break;

#if MPU6050_USE_Filter
    KalmanFilter_Init(&Kal_filter_x,  1, 10,  0, 0);   /* Accel: 平滑优先 */
    KalmanFilter_Init(&Kal_filter_y,  1, 10,  0, 0);
    KalmanFilter_Init(&Kal_filter_z,  1, 10,  0, 0);
    KalmanFilter_Init(&Kal_filter_gx, 1, 0.1, 0, 0);   /* Gyro:  响应优先 */
    KalmanFilter_Init(&Kal_filter_gy, 1, 0.1, 0, 0);
    KalmanFilter_Init(&Kal_filter_gz, 1, 0.1, 0, 0);
#endif
}

/* ═══════════════════════════════════════════════════════
 * 第5层: 原始数据读取 (仅读寄存器 + 减零偏, 不含滤波)
 * ═══════════════════════════════════════════════════════ */

/**
 * @brief  读取加速度、温度和陀螺仪原始数据并应用陀螺零偏。
 * @param  mpu MPU6050 对象。
 * @note   使用阻塞 I2C，不可在中断中调用。
 */
static void MPU6050_Get_Raw(MPU6050 *mpu)
{
    static uint8_t temp_buffer[14];
    MPU6050_READ_REG_CONTINUE(MPU6050_ACCEL_XOUT_H, 14, temp_buffer);
    mpu->AccX    = ((int16_t)temp_buffer[0]  << 8) | temp_buffer[1];
    mpu->AccY    = ((int16_t)temp_buffer[2]  << 8) | temp_buffer[3];
    mpu->AccZ    = ((int16_t)temp_buffer[4]  << 8) | temp_buffer[5];
    mpu->rawTemp = ((int16_t)temp_buffer[6]  << 8) | temp_buffer[7];
    mpu->GyroX   = ((int16_t)temp_buffer[8]  << 8) | temp_buffer[9];
    mpu->GyroY   = ((int16_t)temp_buffer[10] << 8) | temp_buffer[11];
    mpu->GyroZ   = ((int16_t)temp_buffer[12] << 8) | temp_buffer[13];

    mpu->GyroX -= gyro_zero_x;
    mpu->GyroY -= gyro_zero_y;
    mpu->GyroZ -= gyro_zero_z;
}

/* ═══════════════════════════════════════════════════════
 * 第6层: 快速平方根倒数 + Mahony PI AHRS (含卡尔曼, ISR 安全)
 * ═══════════════════════════════════════════════════════ */

/**
 * @brief  使用快速平方根倒数算法计算 1/sqrt(x)。
 * @param  x 正浮点数。
 * @return x 的平方根倒数近似值。
 */
static inline float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y     = x;
    long  i     = *(long *)&y;
    i           = 0x5f3759df - (i >> 1);
    y           = *(float *)&i;
    y           = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief  使用 Mahony PI 互补滤波更新四元数和欧拉角。
 * @param  mpu MPU6050 对象，函数会更新四元数、roll、pitch 和 yaw。
 * @param  dt 采样周期，单位秒，必须大于 0。
 * @note   纯数学滤波，适合在任务上下文调用；输入数据应来自异步 I2C 读取。
 */
void MPU6050_AHRS_Update(MPU6050 *mpu, float dt)
{
#if MPU6050_USE_Filter
    /* 卡尔曼滤波: 6 通道独立滤波 */
    mpu->AccX  = (int16_t)KalmanFilter_Update(&Kal_filter_x,  (float)mpu->AccX);
    mpu->AccY  = (int16_t)KalmanFilter_Update(&Kal_filter_y,  (float)mpu->AccY);
    mpu->AccZ  = (int16_t)KalmanFilter_Update(&Kal_filter_z,  (float)mpu->AccZ);
    mpu->GyroX = (int16_t)KalmanFilter_Update(&Kal_filter_gx, (float)mpu->GyroX);
    mpu->GyroY = (int16_t)KalmanFilter_Update(&Kal_filter_gy, (float)mpu->GyroY);
    mpu->GyroZ = (int16_t)KalmanFilter_Update(&Kal_filter_gz, (float)mpu->GyroZ);
#endif

    /* ADC → 物理量 */
    float gx = (float)mpu->GyroX * gyroScale;
    float gy = (float)mpu->GyroY * gyroScale;
    float gz = (float)mpu->GyroZ * gyroScale;
    float ax = (float)mpu->AccX  * accelScale;
    float ay = (float)mpu->AccY  * accelScale;
    float az = (float)mpu->AccZ  * accelScale;

    /* 加速度幅值 → 自适应增益 */
    float accMag = ax * ax + ay * ay + az * az;
    float Kp, Ki;
    if (ahrs_init_cnt < 400) {
        ahrs_init_cnt++;
        Kp = 8.0f;   Ki = 0.002f;
    } else {
        if (accMag > 1.44f || accMag < 0.64f)
            { Kp = 3.6f; Ki = 0.001f; }
        else
            { Kp = 4.8f; Ki = 0.0015f; }
    }

    /* 加速度归一化 + 重力矢量 Mahony PI 修正。 */
    if (accMag > 0.01f) {
        float recip = invSqrt(accMag);
        ax *= recip; ay *= recip; az *= recip;

        float vx = 2.0f * (ahrs_q1 * ahrs_q3 - ahrs_q0 * ahrs_q2);
        float vy = 2.0f * (ahrs_q0 * ahrs_q1 + ahrs_q2 * ahrs_q3);
        float vz = ahrs_q0 * ahrs_q0 - ahrs_q1 * ahrs_q1
                 - ahrs_q2 * ahrs_q2 + ahrs_q3 * ahrs_q3;

        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        if (Ki > 0.0f) {
            integralX += ex * dt;
            integralY += ey * dt;
            integralZ += ez * dt;
            gx += Ki * integralX;
            gy += Ki * integralY;
            gz += Ki * integralZ;
        }
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    /* 四元数更新 */
    float halfT = dt * 0.5f;
    float qDot0 = (-ahrs_q1 * gx - ahrs_q2 * gy - ahrs_q3 * gz) * halfT;
    float qDot1 = ( ahrs_q0 * gx + ahrs_q2 * gz - ahrs_q3 * gy) * halfT;
    float qDot2 = ( ahrs_q0 * gy - ahrs_q1 * gz + ahrs_q3 * gx) * halfT;
    float qDot3 = ( ahrs_q0 * gz + ahrs_q1 * gy - ahrs_q2 * gx) * halfT;
    ahrs_q0 += qDot0; ahrs_q1 += qDot1;
    ahrs_q2 += qDot2; ahrs_q3 += qDot3;

    /* 四元数归一化 */
    float norm = invSqrt(ahrs_q0 * ahrs_q0 + ahrs_q1 * ahrs_q1
                       + ahrs_q2 * ahrs_q2 + ahrs_q3 * ahrs_q3);
    ahrs_q0 *= norm; ahrs_q1 *= norm;
    ahrs_q2 *= norm; ahrs_q3 *= norm;

    mpu->q0 = ahrs_q0; mpu->q1 = ahrs_q1;
    mpu->q2 = ahrs_q2; mpu->q3 = ahrs_q3;

    /* 四元数 → 欧拉角 */
    mpu->roll  = atan2f(2.0f * (ahrs_q0 * ahrs_q1 + ahrs_q2 * ahrs_q3),
                        1.0f - 2.0f * (ahrs_q1 * ahrs_q1 + ahrs_q2 * ahrs_q2)) * 57.29578f;
    mpu->pitch = asinf(2.0f * (ahrs_q0 * ahrs_q2 - ahrs_q3 * ahrs_q1)) * 57.29578f;
    float current_yaw = atan2f(2.0f * (ahrs_q0 * ahrs_q3 + ahrs_q1 * ahrs_q2),
                               1.0f - 2.0f * (ahrs_q2 * ahrs_q2 + ahrs_q3 * ahrs_q3)) * 57.29578f;

    /* Yaw 解包裹 */
    if (ahrs_first_yaw) {
        unwrapped_yaw = current_yaw;
        ahrs_first_yaw = 0;
    } else {
        float diff = current_yaw - unwrapped_yaw;
        if      (diff >  180.0f) unwrapped_yaw += diff - 360.0f;
        else if (diff < -180.0f) unwrapped_yaw += diff + 360.0f;
        else                     unwrapped_yaw  = current_yaw;
    }

    mpu->roll  -= angle_roll;
    mpu->pitch -= angle_pitch;
    mpu->yaw    = unwrapped_yaw - angle_yaw;
}

/* ═══════════════════════════════════════════════════════
 * 第7层: 顶层 API (阻塞 I2C)
 * ═══════════════════════════════════════════════════════ */

/**
 * @brief  阻塞读取一帧原始数据并运行 Mahony 姿态解算。
 * @param  mpu MPU6050 对象。
 * @note   包含阻塞 I2C 读取，不可在中断上下文调用。
 */
void MPU6050_Get_Angle_Plus(MPU6050 *mpu)
{
    MPU6050_Get_Raw(mpu);
    MPU6050_AHRS_Update(mpu, mpu6050_dt);
}

/**
 * @brief  使用加速度计和陀螺仪互补融合更新姿态角。
 * @param  mpu MPU6050 对象。
 * @note   该接口与 Mahony 四元数路径二选一使用；包含阻塞 I2C 读取。
 */
void MPU6050_Get_Angle(MPU6050 *mpu)
{
    MPU6050_Get_Raw(mpu);

    float ax = (float)mpu->AccX  * accelScale;
    float ay = (float)mpu->AccY  * accelScale;
    float az = (float)mpu->AccZ  * accelScale;
    float gx = (float)mpu->GyroX * gyroScale * mpu6050_dt;
    float gy = (float)mpu->GyroY * gyroScale * mpu6050_dt;
    float gz = (float)mpu->GyroZ * gyroScale * mpu6050_dt;

#if MPU6050_USE_Filter
    /* 互补滤波前先卡尔曼平滑 */
    ax = KalmanFilter_Update(&Kal_filter_x,  ax);
    ay = KalmanFilter_Update(&Kal_filter_y,  ay);
    az = KalmanFilter_Update(&Kal_filter_z,  az);
    gx = KalmanFilter_Update(&Kal_filter_gx, gx);
    gy = KalmanFilter_Update(&Kal_filter_gy, gy);
    gz = KalmanFilter_Update(&Kal_filter_gz, gz);
#endif

    float absAcc = sqrtf(ax * ax + ay * ay + az * az);
    float weight = (absAcc > 1.2f) ? 0.8f : 0.98f;

    static float Gyroscope_roll  = 0.0f;
    static float Gyroscope_pitch = 0.0f;
    Gyroscope_roll  += gy;
    Gyroscope_pitch += gx;
    mpu->roll  = weight * atan2f(ay, az) * 57.29578f + (1.f - weight) * Gyroscope_roll;
    mpu->pitch = -(weight * atan2f(ax, az) * 57.29578f + (1.f - weight) * Gyroscope_pitch);
    mpu->yaw  += gz * 57.29578f;

    mpu->roll  -= angle_roll;
    mpu->pitch -= angle_pitch;
    mpu->yaw   -= angle_yaw;
}

/**
 * @brief  将当前姿态角设置为新的零点和解包裹基点。
 * @param  mpu 当前姿态对象。
 * @note   函数包含短时间空循环，应在车辆静止时调用。
 */
void MPU6050_Set_Angle0(MPU6050 *mpu)
{
    for (uint32_t i = 0; i < 1000000; i++);
    angle_yaw   = mpu->yaw;
    angle_roll  = mpu->roll;
    angle_pitch = mpu->pitch;
    ahrs_first_yaw = 1;  /* 强制下一帧重设解包裹基点 */
}

/**
 * @brief  读取 MPU6050 WHO_AM_I 寄存器。
 * @return 设备 ID；通信失败时返回 0。
 */
uint8_t MPU6050_ID(void)
{
    return MPU6050READ_REG(MPU6050_WHO_AM_I);
}

/**
 * @brief  将原始温度值换算为摄氏度。
 * @param  mpu MPU6050 对象。
 * @return 温度值，单位摄氏度。
 */
float MPU6050_GetTemp(MPU6050 *mpu)
{
    mpu->temp = (float)mpu->rawTemp / 340.f + 36.53f;
    return mpu->temp;
}

/**
 * @brief  采集静止数据并输出加速度、角速度平均值。
 * @param  accel_g 输出三轴平均值，单位 g。
 * @param  gyro_dps 输出三轴平均值，单位度/秒。
 * @note   函数包含 50 次采样和阻塞延时，仅用于调试或自检。
 */
void MPU6050_SelfTest(float accel_g[3], float gyro_dps[3])
{
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint8_t buf[14];
    int16_t raw;

    for (uint8_t n = 0; n < 50; n++) {
        mpu6050_read_reg_continue(MPU6050_ACCEL_XOUT_H, 14, buf);

        raw = ((int16_t)buf[0] << 8) | buf[1];
        ax_sum += raw;
        raw = ((int16_t)buf[2] << 8) | buf[3];
        ay_sum += raw;
        raw = ((int16_t)buf[4] << 8) | buf[5];
        az_sum += raw;

        raw = ((int16_t)buf[8] << 8) | buf[9];
        gx_sum += raw;
        raw = ((int16_t)buf[10] << 8) | buf[11];
        gy_sum += raw;
        raw = ((int16_t)buf[12] << 8) | buf[13];
        gz_sum += raw;

        HAL_Delay(5);
    }

    accel_g[0] = (ax_sum / 50) * accelScale;
    accel_g[1] = (ay_sum / 50) * accelScale;
    accel_g[2] = (az_sum / 50) * accelScale;

    gyro_dps[0] = (gx_sum / 50) * gyroScale * 57.29578f;  /* rad/s → °/s */
    gyro_dps[1] = (gy_sum / 50) * gyroScale * 57.29578f;
    gyro_dps[2] = (gz_sum / 50) * gyroScale * 57.29578f;
}

/* ═══════════════════════════════════════════════════════
 * 第8层: 异步 I2C (ISR 安全)
 * ═══════════════════════════════════════════════════════ */

static uint8_t  mpu_async_buf[14];
static int16_t  mpu_async_ax, mpu_async_ay, mpu_async_az;
static int16_t  mpu_async_gx, mpu_async_gy, mpu_async_gz;
static volatile uint8_t mpu_async_ready = 0;
static volatile uint8_t mpu_async_busy  = 0;

/**
 * @brief  启动一次 14 字节 MPU6050 异步读取。
 * @note   上一次读取未完成时直接返回；I2C 完成回调会清除 busy 标志。
 */
void MPU6050_ReadAsync_Start(void)
{
    HAL_StatusTypeDef status;

    if (mpu_async_busy) return;
    mpu_async_busy = 1;
    status = HAL_I2C_Mem_Read_IT(MPU6050_I2C,
                                 MPU6050_ADDRESS << 1,
                                 MPU6050_ACCEL_XOUT_H,
                                 I2C_MEMADD_SIZE_8BIT,
                                 mpu_async_buf,
                                 14);
    if (status != HAL_OK) {
        mpu_async_busy = 0;
    }
}

/**
 * @brief  查询异步读取是否已经完成。
 * @return 1 表示新数据就绪，0 表示仍在读取或读取失败。
 */
uint8_t MPU6050_ReadAsync_Done(void)
{
    return mpu_async_ready;
}

/**
 * @brief  取出最近一次异步读取的六轴数据并清除就绪标志。
 * @param  AX 输出 X 轴加速度原始值。
 * @param  AY 输出 Y 轴加速度原始值。
 * @param  AZ 输出 Z 轴加速度原始值。
 * @param  GX 输出 X 轴角速度零偏补偿值。
 * @param  GY 输出 Y 轴角速度零偏补偿值。
 * @param  GZ 输出 Z 轴角速度零偏补偿值。
 * @note   应在 HAL_I2C 完成回调之后、下一次启动读取之前调用。
 */
void MPU6050_ReadAsync_Get(int16_t *AX, int16_t *AY, int16_t *AZ,
                           int16_t *GX, int16_t *GY, int16_t *GZ)
{
    *AX = mpu_async_ax;
    *AY = mpu_async_ay;
    *AZ = mpu_async_az;
    *GX = mpu_async_gx - gyro_zero_x;  /* 异步路径: 应用零偏 */
    *GY = mpu_async_gy - gyro_zero_y;
    *GZ = mpu_async_gz - gyro_zero_z;
    mpu_async_ready = 0;
}

/* ── I2C 回调转发 (用户从 HAL_I2C_MemRxCpltCallback 中调用) ── */
/**
 * @brief  处理 MPU6050 异步读取完成事件。
 * @param  hi2c 完成读取的 I2C 句柄。
 * @note   在 HAL I2C 中断上下文调用，只复制缓冲区和更新 volatile 标志。
 */
void MPU6050_I2C_RxCplt(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == MPU6050_I2C) {
        mpu_async_ax = (mpu_async_buf[0]  << 8) | mpu_async_buf[1];
        mpu_async_ay = (mpu_async_buf[2]  << 8) | mpu_async_buf[3];
        mpu_async_az = (mpu_async_buf[4]  << 8) | mpu_async_buf[5];
        mpu_async_gx = (mpu_async_buf[8]  << 8) | mpu_async_buf[9];
        mpu_async_gy = (mpu_async_buf[10] << 8) | mpu_async_buf[11];
        mpu_async_gz = (mpu_async_buf[12] << 8) | mpu_async_buf[13];
        mpu_async_ready = 1;
        mpu_async_busy  = 0;
    }
}

/**
 * @brief  处理 MPU6050 异步 I2C 错误。
 * @param  hi2c 发生错误的 I2C 句柄。
 * @note   在 HAL I2C 中断上下文调用，只清除 busy 和 ready 状态。
 */
void MPU6050_I2C_Error(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == MPU6050_I2C) {
        mpu_async_ready = 0;
        mpu_async_busy = 0;
    }
}

#if MPU6050_DEFINE_I2C_CALLBACKS
/* 单设备便捷模式: 直接覆盖 HAL 弱回调 */
/**
 * @brief  HAL I2C 内存读取完成回调转发。
 * @param  hi2c 完成读取的 I2C 句柄。
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) { MPU6050_I2C_RxCplt(hi2c); }

/**
 * @brief  HAL I2C 错误回调转发。
 * @param  hi2c 发生错误的 I2C 句柄。
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)     { MPU6050_I2C_Error(hi2c); }
#endif

#endif /* HAL_I2C_MODULE_ENABLED */

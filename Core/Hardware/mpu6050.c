/**
 * @file    mpu6050.c
 * @brief   MPU6050 驱动 — I2C + Madgwick AHRS (移植自 bilibili 小努班 V1.7)
 * @note    阻塞 I2C: MPU6050_init / MPU6050_Get_Angle_Plus (不可进 ISR)
 *          异步 I2C: MPU6050_ReadAsync_Start/Done/Get (ISR 安全)
 *          AHRS 纯数学: MPU6050_AHRS_Update (ISR 安全, 配合异步 I2C)
 */

#include "mpu6050.h"
#include <math.h>
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c2;

/* ── 滤波器 (可选) ────────────────────────────────── */
#if MPU6050_USE_Filter
#include "filter.h"
static PT1Filter_t pt1_filter_x, pt1_filter_y, pt1_filter_z;
static PT1Filter_t pt1_filter_gx, pt1_filter_gy, pt1_filter_gz;
#endif

/* ═══════════════════════════════════════════════════════
 * 第1层: I2C 底层 (适配 hi2c2)
 * ═══════════════════════════════════════════════════════ */

static I2C_HandleTypeDef *mpu_i2c = NULL;

static void mpu_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t d = data;
    HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDRESS << 1, reg,
                      I2C_MEMADD_SIZE_8BIT, &d, 1, 200);
}

static uint8_t mpu_read_reg(uint8_t reg)
{
    uint8_t d = 0;
    HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDRESS << 1, reg,
                     I2C_MEMADD_SIZE_8BIT, &d, 1, 200);
    return d;
}

static void mpu_read_buf(uint8_t reg, uint8_t len, uint8_t *buf)
{
    HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDRESS << 1, reg,
                     I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/* 便捷宏 (参考项目风格) */
#define MPU_DELAY(ms)               HAL_Delay(ms)
#define MPU_WRITE(reg, data)        mpu_write_reg(reg, data)
#define MPU_READ(reg)               mpu_read_reg(reg)
#define MPU_READ_BUF(reg, len, buf) mpu_read_buf(reg, len, buf)

/* ═══════════════════════════════════════════════════════
 * 第2层: 寄存器地址
 * ═══════════════════════════════════════════════════════ */
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_FIFO_EN       0x23
#define REG_INT_PIN_CFG   0x37
#define REG_INT_ENABLE    0x38
#define REG_ACCEL_XOUT_H  0x3B
#define REG_TEMP_OUT_H    0x41
#define REG_GYRO_XOUT_H   0x43
#define REG_USER_CTRL     0x6A
#define REG_PWR_MGMT_1    0x6B
#define REG_PWR_MGMT_2    0x6C
#define REG_WHO_AM_I      0x75

/* ═══════════════════════════════════════════════════════
 * 第3层: 初始化 + 校准
 * ═══════════════════════════════════════════════════════ */

static float   gyroScale;   /* °/s / LSB → rad/s / LSB */
static float   accelScale;  /* g / LSB */
static float   mpu_dt;      /* 采样周期 (s) */
static int16_t gyro_bias_x, gyro_bias_y, gyro_bias_z;

/**
 * @brief  陀螺零偏校准 (阻塞, 上电时调用)
 * @return 0=成功, -1=失败
 */
static int8_t MPU6050_SoftCalibrate_Z(uint16_t samples)
{
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint8_t buf[6];

    for (uint16_t i = 0; i < samples; i++) {
        MPU_READ_BUF(REG_GYRO_XOUT_H, 6, buf);
        int16_t gx = ((int16_t)buf[0] << 8) | buf[1];
        int16_t gy = ((int16_t)buf[2] << 8) | buf[3];
        int16_t gz = ((int16_t)buf[4] << 8) | buf[5];
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        MPU_DELAY((uint32_t)(mpu_dt * 1000));
    }
    gyro_bias_x = (int16_t)(gx_sum / samples);
    gyro_bias_y = (int16_t)(gy_sum / samples);
    gyro_bias_z = (int16_t)(gz_sum / samples);

    /* 验证 */
    gx_sum = gy_sum = gz_sum = 0;
    for (uint16_t i = 0; i < 100; i++) {
        MPU_READ_BUF(REG_GYRO_XOUT_H, 6, buf);
        int16_t gx = ((int16_t)buf[0] << 8) | buf[1] - gyro_bias_x;
        int16_t gy = ((int16_t)buf[2] << 8) | buf[3] - gyro_bias_y;
        int16_t gz = ((int16_t)buf[4] << 8) | buf[5] - gyro_bias_z;
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        MPU_DELAY((uint32_t)(mpu_dt * 1000));
    }
    if (abs((int)(gx_sum / 100)) >= 2 &&
        abs((int)(gy_sum / 100)) >= 2 &&
        abs((int)(gz_sum / 100)) >= 2)
        return -1;
    return 0;
}

static void MPU6050_Register_init(MPU6050_InitTypeDef *cfg)
{
    float acc_s, gyro_s;
    switch (cfg->acc_range) {
        case acc_2g:  acc_s = 16384.f; break;
        case acc_4g:  acc_s = 8192.f;  break;
        case acc_8g:  acc_s = 4096.f;  break;
        default:      acc_s = 2048.f;  break;
    }
    switch (cfg->gyro_range) {
        case gyro_250:  gyro_s = 131.f;   break;
        case gyro_500:  gyro_s = 65.5f;   break;
        case gyro_1000: gyro_s = 32.8f;   break;
        default:        gyro_s = 16.4f;   break;
    }
    accelScale = 1.f / acc_s;
    gyroScale  = 0.0174533f / gyro_s;  /* deg→rad 转换内置 */

    MPU_WRITE(REG_PWR_MGMT_1, 0x80); MPU_DELAY(100);
    MPU_WRITE(REG_PWR_MGMT_1, 0x00);

    uint16_t rate = cfg->SMPLRT_Rate;
    if (rate > 1000) rate = 1000;
    if (rate < 4)    rate = 4;
    uint8_t div = (uint8_t)(1000.f / rate - 1);
    MPU_WRITE(REG_SMPLRT_DIV, div);
    MPU_WRITE(REG_INT_PIN_CFG, 0x00);
    MPU_WRITE(REG_INT_ENABLE,  cfg->INT);
    MPU_WRITE(REG_CONFIG,      cfg->Filter);
    MPU_WRITE(REG_GYRO_CONFIG, cfg->gyro_range);
    MPU_WRITE(REG_ACCEL_CONFIG,cfg->acc_range);
    MPU_WRITE(REG_FIFO_EN,     cfg->FIFO_EN);

    uint8_t user = 0x00;
    if (cfg->FIFO_EN != 0x00) user |= 0x40;
    if ((cfg->INT & 0x01) == 0) user |= 0x08;
    MPU_WRITE(REG_USER_CTRL, user);
    MPU_WRITE(REG_PWR_MGMT_1, 0x01);  /* X轴陀螺为时钟源 */
}

void MPU6050_init(I2C_HandleTypeDef *hi2c)
{
    mpu_i2c = hi2c;

    MPU6050_InitTypeDef cfg;
    cfg.SMPLRT_Rate = 200;
    cfg.Filter      = Band_21Hz;
    cfg.gyro_range  = gyro_2000;
    cfg.acc_range   = acc_16g;
    cfg.FIFO_EN     = 0x00;
    cfg.INT         = Data_Ready_EN;
    MPU6050_Register_init(&cfg);
    mpu_dt = 1.f / (float)cfg.SMPLRT_Rate;

    MPU_DELAY(300);
    for (int i = 0; i < 3; i++)
        if (MPU6050_SoftCalibrate_Z(64) == 0)
            break;

#if MPU6050_USE_Filter
    PT1Filter_InitWithFreq(&pt1_filter_x,  48, cfg.SMPLRT_Rate);
    PT1Filter_InitWithFreq(&pt1_filter_y,  48, cfg.SMPLRT_Rate);
    PT1Filter_InitWithFreq(&pt1_filter_z,  48, cfg.SMPLRT_Rate);
    PT1Filter_InitWithFreq(&pt1_filter_gx, 88, cfg.SMPLRT_Rate);
    PT1Filter_InitWithFreq(&pt1_filter_gy, 88, cfg.SMPLRT_Rate);
    PT1Filter_InitWithFreq(&pt1_filter_gz, 88, cfg.SMPLRT_Rate);
#endif
}

/* ═══════════════════════════════════════════════════════
 * 第4层: 原始数据读取
 * ═══════════════════════════════════════════════════════ */

static void MPU6050_Get_Raw(MPU6050 *mpu)
{
    uint8_t buf[14];
    MPU_READ_BUF(REG_ACCEL_XOUT_H, 14, buf);
    mpu->AccX    = ((int16_t)buf[0]  << 8) | buf[1];
    mpu->AccY    = ((int16_t)buf[2]  << 8) | buf[3];
    mpu->AccZ    = ((int16_t)buf[4]  << 8) | buf[5];
    mpu->rawTemp = ((int16_t)buf[6]  << 8) | buf[7];
    mpu->GyroX   = ((int16_t)buf[8]  << 8) | buf[9];
    mpu->GyroY   = ((int16_t)buf[10] << 8) | buf[11];
    mpu->GyroZ   = ((int16_t)buf[12] << 8) | buf[13];

    mpu->GyroX -= gyro_bias_x;
    mpu->GyroY -= gyro_bias_y;
    mpu->GyroZ -= gyro_bias_z;
}

/* ═══════════════════════════════════════════════════════
 * 第5层: Madgwick 自适应 AHRS (参考项目 MPU6050_Get_Angle_Plus)
 * ═══════════════════════════════════════════════════════ */

/* ── 快速平方根倒数 ───────────────────────────────── */
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

/* AHRS 内部状态 */
static float ahrs_q0 = 1.0f, ahrs_q1 = 0.0f, ahrs_q2 = 0.0f, ahrs_q3 = 0.0f;
static float integralX = 0, integralY = 0, integralZ = 0;
static uint16_t ahrs_init_cnt = 0;
static float unwrapped_yaw = 0.0f;
static uint8_t ahrs_first_yaw = 1;
static float zero_yaw = 0, zero_roll = 0, zero_pitch = 0;

/**
 * @brief  Madgwick AHRS 纯数学解算 (无 I2C, ISR 安全)
 * @note   mpu 中已有的 AccX/Y/Z, GyroX/Y/Z 转换为物理量后完成四元数更新
 *         dt = 采样周期 (s)
 */
void MPU6050_AHRS_Update(MPU6050 *mpu, float dt)
{
#if MPU6050_USE_Filter
    /* PT1 低通滤波器: 对原始 ADC 值滤波后再转换 */
    mpu->AccX  = (int16_t)PT1Filter_Apply(&pt1_filter_x,  (float)mpu->AccX);
    mpu->AccY  = (int16_t)PT1Filter_Apply(&pt1_filter_y,  (float)mpu->AccY);
    mpu->AccZ  = (int16_t)PT1Filter_Apply(&pt1_filter_z,  (float)mpu->AccZ);
    mpu->GyroX = (int16_t)PT1Filter_Apply(&pt1_filter_gx, (float)mpu->GyroX);
    mpu->GyroY = (int16_t)PT1Filter_Apply(&pt1_filter_gy, (float)mpu->GyroY);
    mpu->GyroZ = (int16_t)PT1Filter_Apply(&pt1_filter_gz, (float)mpu->GyroZ);
#endif

    /* 原始 ADC → 物理量 */
    float gx = (float)mpu->GyroX * gyroScale;   /* rad/s */
    float gy = (float)mpu->GyroY * gyroScale;
    float gz = (float)mpu->GyroZ * gyroScale;
    float ax = (float)mpu->AccX * accelScale;    /* g */
    float ay = (float)mpu->AccY * accelScale;
    float az = (float)mpu->AccZ * accelScale;

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

    /* 加速度归一化 + 重力矢量 PI 修正陀螺 */
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

    /* 归一化 */
    float norm = invSqrt(ahrs_q0 * ahrs_q0 + ahrs_q1 * ahrs_q1
                       + ahrs_q2 * ahrs_q2 + ahrs_q3 * ahrs_q3);
    ahrs_q0 *= norm; ahrs_q1 *= norm;
    ahrs_q2 *= norm; ahrs_q3 *= norm;

    /* 四元数 → 欧拉角 */
    mpu->q0 = ahrs_q0; mpu->q1 = ahrs_q1;
    mpu->q2 = ahrs_q2; mpu->q3 = ahrs_q3;

    mpu->roll  = atan2f(2.0f * (ahrs_q0 * ahrs_q1 + ahrs_q2 * ahrs_q3),
                        1.0f - 2.0f * (ahrs_q1 * ahrs_q1 + ahrs_q2 * ahrs_q2)) * 57.29578f;
    mpu->pitch = asinf(2.0f * (ahrs_q0 * ahrs_q2 - ahrs_q3 * ahrs_q1)) * 57.29578f;
    float cy   = atan2f(2.0f * (ahrs_q0 * ahrs_q3 + ahrs_q1 * ahrs_q2),
                        1.0f - 2.0f * (ahrs_q2 * ahrs_q2 + ahrs_q3 * ahrs_q3)) * 57.29578f;

    /* Yaw 解包裹 */
    if (ahrs_first_yaw) {
        unwrapped_yaw = cy;
        ahrs_first_yaw = 0;
    } else {
        float diff = cy - unwrapped_yaw;
        if      (diff >  180.0f) unwrapped_yaw += diff - 360.0f;
        else if (diff < -180.0f) unwrapped_yaw += diff + 360.0f;
        else                     unwrapped_yaw  = cy;
    }

    mpu->roll  -= zero_roll;
    mpu->pitch -= zero_pitch;
    mpu->yaw    = unwrapped_yaw - zero_yaw;
}

/**
 * @brief  Madgwick 自适应 AHRS (含阻塞 I2C 读取, 不进 ISR)
 */
void MPU6050_Get_Angle_Plus(MPU6050 *mpu)
{
    MPU6050_Get_Raw(mpu);
    MPU6050_AHRS_Update(mpu, mpu_dt);
}

/* ═══════════════════════════════════════════════════════
 * 第6层: 互补滤波 (参考项目 MPU6050_Get_Angle)
 * ═══════════════════════════════════════════════════════ */

void MPU6050_Get_Angle(MPU6050 *mpu)
{
    MPU6050_Get_Raw(mpu);

    float ax = (float)mpu->AccX * accelScale;
    float ay = (float)mpu->AccY * accelScale;
    float az = (float)mpu->AccZ * accelScale;
    float gx = (float)mpu->GyroX * gyroScale * mpu_dt;
    float gy = (float)mpu->GyroY * gyroScale * mpu_dt;
    float gz = (float)mpu->GyroZ * gyroScale * mpu_dt;

    float absAcc = sqrtf(ax * ax + ay * ay + az * az);
    float weight = (absAcc > 1.2f) ? 0.8f : 0.98f;

    static float gyro_roll = 0, gyro_pitch = 0;
    gyro_roll  += gy;
    gyro_pitch += gx;
    mpu->roll   = weight * atan2f(ay, az) * 57.29578f + (1.f - weight) * gyro_roll;
    mpu->pitch  = -(weight * atan2f(ax, az) * 57.29578f + (1.f - weight) * gyro_pitch);
    mpu->yaw   += gz * 57.29578f;

    mpu->roll  -= zero_roll;
    mpu->pitch -= zero_pitch;
    mpu->yaw   -= zero_yaw;
}

/* ═══════════════════════════════════════════════════════
 * 第7层: 角度归零 + ID + 温度
 * ═══════════════════════════════════════════════════════ */

void MPU6050_Set_Angle0(MPU6050 *mpu)
{
    for (volatile uint32_t i = 0; i < 1000000; i++);
    zero_yaw   = unwrapped_yaw;
    zero_roll  = mpu->roll  + zero_roll;
    zero_pitch = mpu->pitch + zero_pitch;
    ahrs_first_yaw = 1;  /* 强制下一帧重设解包裹基点 */
}

uint8_t MPU6050_ID(void)
{
    return MPU_READ(REG_WHO_AM_I);
}

/* ═══════════════════════════════════════════════════════
 * 第8层: 异步 I2C (ISR 安全, 保留 bluecar 原有实现)
 * ═══════════════════════════════════════════════════════ */

static uint8_t  mpu_async_buf[14];
static int16_t  mpu_async_ax, mpu_async_ay, mpu_async_az;
static int16_t  mpu_async_gx, mpu_async_gy, mpu_async_gz;
static volatile uint8_t mpu_async_ready = 0;
static volatile uint8_t mpu_async_busy  = 0;

void MPU6050_ReadAsync_Start(void)
{
    if (mpu_async_busy) return;
    mpu_async_busy = 1;
    HAL_I2C_Mem_Read_IT(mpu_i2c, MPU6050_ADDRESS << 1, REG_ACCEL_XOUT_H,
                        I2C_MEMADD_SIZE_8BIT, mpu_async_buf, 14);
}

uint8_t MPU6050_ReadAsync_Done(void)
{
    return mpu_async_ready;
}

void MPU6050_ReadAsync_Get(int16_t *AX, int16_t *AY, int16_t *AZ,
                           int16_t *GX, int16_t *GY, int16_t *GZ)
{
    *AX = mpu_async_ax;  *AY = mpu_async_ay;  *AZ = mpu_async_az;
    *GX = mpu_async_gx;  *GY = mpu_async_gy;  *GZ = mpu_async_gz;
    mpu_async_ready = 0;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
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

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        mpu_async_busy = 0;
    }
}

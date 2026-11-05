/* IMU963RA（LSM6DSR + QMC5883L）驱动：六轴标定/yaw 积分 + 磁力计只读诊断。

   安装：模块从车尾向车头看时，丝印 +X 朝车左、+Y 朝下、+Z 朝车尾。
         驱动统一输出车辆右手坐标 +X 车头、+Y 车左、+Z 车顶。
   硬件：J3 IMU963RA，SPI2 Mode0 / 9 MHz（72 MHz APB1 / 4），
         SPC=PB13 SDI=PB15 SDO=PB14，CS=PB12 软件片选（低有效），INT=PE15（不用中断，轮询）。

   寄存器：CTRL3_C=0x44（BDU=1 读期间寄存器不更新，IF_INC=1 支持连读）
           CTRL1_XL=0x48（ODR 104 Hz，±4g，0.122 mg/LSB）
           CTRL2_G =0x48（ODR 104 Hz，±1000 dps，35 mdps/LSB）
           连读起点 0x22，一次 12 字节 = 陀螺 xyz + 加速度 xyz
           QMC5883L 位于 LSM6DSR 内部 I2C sensor hub，地址 0x0D。磁力计仅初始化、
           校验和采集原始值，不参与 yaw、姿态估计或运动控制。

   ★ 执行上下文：imu_task() 由 TIM6 每 10 ms 置 DEFERRED_IMU、在主循环里执行
     （见 sched.c），不在中断里，不会拖长中断关闭窗口。
     一次采样 12 字节 @9 MHz 约 11 us，仍是忙等；要彻底非阻塞只能上 DMA
     （SPI2_RX=DMA1_Ch4、SPI2_TX=DMA1_Ch5 + 双缓冲），本工程 10 ms 周期下收益约 0.1%，
     故不引入。
     软复位后的寄存器配置拆到下一拍完成，静止校准收集 200 帧（约 2 s）。

   校准要求编码器静止、陀螺波动小且加速度稳定；通过后固定零偏，不运行时追零。
   失败处理：SPI 传输超时（裸寄存器忙等，有固定循环上限，永不挂死）或 |a| 不在 [0.5, 1.5] g -> 丢帧 + err++；
             在线运行丢帧会清除校准状态，并按 IMU_REPROBE_MS 限频重探。 */

#include <math.h>
#include "imu.h"
#include "board.h"
#include "config.h"
#include "motion.h"

/* ---------------- LSM6DSR 寄存器 ---------------- */
#define IMU_REG_WHO_AM_I    0x0Fu
#define IMU_REG_FUNC_CFG    0x01u
#define IMU_REG_CTRL1_XL    0x10u
#define IMU_REG_CTRL2_G     0x11u
#define IMU_REG_CTRL3_C     0x12u
#define IMU_REG_OUTX_L_G    0x22u

/* 以下地址仅在 FUNC_CFG_ACCESS=0x40 的 sensor-hub 页面有效。 */
#define IMU_REG_SENSOR_HUB_1   0x02u
#define IMU_REG_MASTER_CONFIG  0x14u
#define IMU_REG_SLV0_ADD       0x15u
#define IMU_REG_SLV0_SUBADD    0x16u
#define IMU_REG_SLV0_CONFIG    0x17u
#define IMU_REG_DATAWRITE_SLV0 0x21u
#define IMU_REG_STATUS_MASTER  0x22u

#define IMU_FUNC_CFG_HUB       0x40u
#define IMU_MASTER_RESET       0x80u
#define IMU_MASTER_WRITE_ONCE  0x4Cu
#define IMU_MASTER_AUTO_READ   0x6Cu

#define QMC5883L_ADDR          0x0Du
#define QMC5883L_REG_OUTX_L    0x00u
#define QMC5883L_REG_CONTROL1  0x09u
#define QMC5883L_REG_CONTROL2  0x0Au
#define QMC5883L_REG_FBR       0x0Bu
#define QMC5883L_REG_CHIP_ID   0x0Du
#define QMC5883L_CHIP_ID       0xFFu
#define QMC5883L_CTRL1_8G      0x19u

#define IMU_WHOAMI_VAL      0x6Bu
#define IMU_SPI_READ        0x80u
#define IMU_SPI_AUTOINC     0x40u

#define IMU_CTRL3_RESET     0x01u   /* SW_RESET = 1 */
#define IMU_CTRL3_CFG       0x44u   /* BDU = 1，IF_INC = 1 */
#define IMU_CTRL1_XL_104HZ  0x48u   /* ODR 104 Hz + FS ±4g */
#define IMU_CTRL2_G_104HZ   0x48u   /* ODR 104 Hz + FS ±1000 dps */

/* 驱动内部状态：OFF 等模块 -> RESET 已发软复位 -> SETTLE 沉降 -> CAL 采零偏 -> RUN 输出 */
enum
{
    IMU_ST_OFF = 0,
    IMU_ST_RESET,
    IMU_ST_SETTLE,
    IMU_ST_CAL,
    IMU_ST_CAL_FAILED,
    IMU_ST_RUN
};

static imu_data_t s_data;
static uint8_t    s_st;
static uint32_t   s_last_ms;      /* 上一次采样的毫秒时间戳 */
static uint32_t   s_last_probe;   /* 上一次 WHO_AM_I 重探的时间戳 */
static uint16_t   s_cal_n;
static uint32_t   s_cal_start_ms;
static float      s_cal_sum[3];
static float      s_cal_sum_sq[3];
static float      s_acc_sum[3];
static float      s_acc_sum_sq[3];
static int16_t    s_cal_samples[IMU_BIAS_SAMPLES][3];
static float      s_cal_last_left_deg;
static float      s_cal_last_right_deg;
static uint8_t    s_cal_encoder_initialized;
static uint8_t    s_run_fail_n;
static uint32_t   s_err_cnt;
static uint32_t   s_last_mag_ms;

/* HAL tick 提供 32 位原子毫秒时间基准，按无符号差值处理回绕。 */
static uint32_t imu_ms(void)
{
    return HAL_GetTick();
}

static void imu_map_to_vehicle(const float sensor[3], float vehicle[3])
{
    vehicle[0] = IMU_VEHICLE_X_SIGN * sensor[IMU_VEHICLE_X_SOURCE];
    vehicle[1] = IMU_VEHICLE_Y_SIGN * sensor[IMU_VEHICLE_Y_SOURCE];
    vehicle[2] = IMU_VEHICLE_Z_SIGN * sensor[IMU_VEHICLE_Z_SOURCE];
}

static void imu_cal_reset_window(void)
{
    uint8_t axis;

    s_cal_n = 0u;
    s_cal_encoder_initialized = 0u;
    for (axis = 0u; axis < 3u; axis++) {
        s_cal_sum[axis] = 0.0f;
        s_cal_sum_sq[axis] = 0.0f;
        s_acc_sum[axis] = 0.0f;
        s_acc_sum_sq[axis] = 0.0f;
    }
}

static uint8_t imu_cal_encoder_still(void)
{
    const MotionView_t *motion = motion_view();
    uint8_t still = 1u;

    if ((fabsf(motion->left_rpm_raw) > IMU_CAL_ENCODER_RPM_MAX)
        || (fabsf(motion->right_rpm_raw) > IMU_CAL_ENCODER_RPM_MAX)) {
        still = 0u;
    }

    /* Keep the window-start angles so slow drift cannot hide behind per-sample steps. */
    if (s_cal_encoder_initialized == 0u) {
        s_cal_last_left_deg = motion->left_angle_deg;
        s_cal_last_right_deg = motion->right_angle_deg;
        s_cal_encoder_initialized = 1u;
    } else {
        if ((fabsf(motion->left_angle_deg - s_cal_last_left_deg)
             > IMU_CAL_ENCODER_ANGLE_EPS_DEG)
            || (fabsf(motion->right_angle_deg - s_cal_last_right_deg)
                > IMU_CAL_ENCODER_ANGLE_EPS_DEG)) {
            still = 0u;
        }
    }

    return still;
}

static void imu_sort_i16_range(int16_t *values, int16_t low, int16_t high)
{
    int16_t i = low;
    int16_t j = high;
    int16_t pivot = values[low + ((high - low) / 2)];

    while (i <= j) {
        int16_t value;
        while (values[i] < pivot) i++;
        while (values[j] > pivot) j--;
        if (i <= j) {
            value = values[i];
            values[i] = values[j];
            values[j] = value;
            j--;
            i++;
        }
    }

    if (low < j) imu_sort_i16_range(values, low, j);
    if (i < high) imu_sort_i16_range(values, i, high);
}

static uint8_t imu_cal_finish(float bias[3])
{
    uint8_t axis;
    uint16_t i;
    uint16_t trim = (uint16_t)(s_cal_n / 10u);

    for (axis = 0u; axis < 3u; axis++) {
        float mean = s_cal_sum[axis] / (float)s_cal_n;
        float variance = (s_cal_sum_sq[axis] / (float)s_cal_n)
                         - (mean * mean);
        float acc_mean = s_acc_sum[axis] / (float)s_cal_n;
        float acc_variance = (s_acc_sum_sq[axis] / (float)s_cal_n)
                             - (acc_mean * acc_mean);
        int16_t sorted[IMU_BIAS_SAMPLES];
        float trimmed_sum = 0.0f;
        float max_deviation = 0.0f;

        if (variance < 0.0f) variance = 0.0f;
        if (acc_variance < 0.0f) acc_variance = 0.0f;

        if ((fabsf(mean) > IMU_CAL_MAX_GYRO_MEAN_DPS)
            || (sqrtf(variance) > IMU_CAL_MAX_GYRO_STD_DPS)
            || (sqrtf(acc_variance) > IMU_CAL_MAX_ACCEL_STD_G)) {
            return 0u;
        }

        for (i = 0u; i < s_cal_n; i++) {
            float sample = (float)s_cal_samples[i][axis] * IMU_GYRO_LSB_DPS;
            float deviation = fabsf(sample - mean);
            sorted[i] = s_cal_samples[i][axis];
            if (deviation > max_deviation) max_deviation = deviation;
        }
        if (max_deviation > IMU_CAL_MAX_GYRO_DEV_DPS) {
            return 0u;
        }

        imu_sort_i16_range(sorted, 0, (int16_t)(s_cal_n - 1u));
        for (i = trim; i < (uint16_t)(s_cal_n - trim); i++) {
            trimmed_sum += (float)sorted[i];
        }
        bias[axis] = (trimmed_sum / (float)(s_cal_n - (2u * trim)))
                     * IMU_GYRO_LSB_DPS;
    }

    return 1u;
}

/* ------------ 底层：片选（低 = 选中） ------------ */
static void cs_low(void)
{
    /* 本仓库这份 STM32F1 HAL 的 HAL_SPI_Init() 只写 CR1、不置 SPE 位（SPE 是在
       HAL_SPI_Transmit / TransmitReceive 内部才置的），而本驱动是裸寄存器收发，
       所以每次发起传输前必须自己确保 SPI2 已使能，否则 SR 一直为 0、必然超时。 */
    if ((SPI2->CR1 & SPI_CR1_SPE) == 0U)
    {
        SET_BIT(SPI2->CR1, SPI_CR1_SPE);
    }
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}
static void cs_high(void) { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); }

/* ------------ 底层：SPI2 裸寄存器收发 ------------
 * 不用 HAL_SPI_TransmitReceive：它的超时靠 HAL_GetTick 递增，一旦被更高优先级
 * 中断或关中断（调度器、DMA）挡住就会死等；这里直接读写 SPI2->SR/DR，
 * 超时用固定循环次数，任何硬件状态下都保证返回。 */
#define IMU_SPI_GUARD   20000u

static uint8_t s_spi_bad;   /* 最近一次 spi_xfer 是否超时 */

static uint8_t spi_xfer(uint8_t out)
{
    uint32_t g;

    g = IMU_SPI_GUARD;
    while ((SPI2->SR & SPI_SR_TXE) == 0u)
    {
        if (--g == 0u) { s_spi_bad = 1u; return 0xFFu; }
    }
    *(volatile uint8_t *)&SPI2->DR = out;

    g = IMU_SPI_GUARD;
    while ((SPI2->SR & SPI_SR_RXNE) == 0u)
    {
        if (--g == 0u) { s_spi_bad = 1u; return 0xFFu; }
    }
    return *(volatile uint8_t *)&SPI2->DR;
}

/* 返回 1 = 成功；0 = SPI 出错（已计入 err_cnt） */
static uint8_t imu_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t v;

    cs_low();
    s_spi_bad = 0u;
    (void)spi_xfer((uint8_t)(reg | IMU_SPI_READ));
    v = spi_xfer(0xFFu);
    cs_high();

    if (s_spi_bad != 0u) { s_err_cnt++; return 0u; }
    *val = v;
    return 1u;
}

static uint8_t imu_write_reg(uint8_t reg, uint8_t val)
{
    cs_low();
    s_spi_bad = 0u;
    (void)spi_xfer((uint8_t)(reg & 0x7Fu));
    (void)spi_xfer(val);
    cs_high();

    if (s_spi_bad != 0u) { s_err_cnt++; return 0u; }
    return 1u;
}

/* 连读 len 字节（自动地址递增）；返回 1 = 成功 */
static uint8_t imu_read_burst(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    cs_low();
    s_spi_bad = 0u;
    (void)spi_xfer((uint8_t)(reg | IMU_SPI_READ | IMU_SPI_AUTOINC));
    for (i = 0u; i < len; i++)
    {
        buf[i] = spi_xfer(0xFFu);
    }
    cs_high();

    if (s_spi_bad != 0u) { s_err_cnt++; return 0u; }
    return 1u;
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((int16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]));
}

static uint8_t imu_hub_wait_status(uint8_t mask)
{
    uint32_t start = HAL_GetTick();
    uint8_t status;

    do {
        if (imu_read_reg(IMU_REG_STATUS_MASTER, &status) == 0u) {
            return 0u;
        }
        s_data.mag_status = status;
        if ((status & mask) != 0u) {
            return 1u;
        }
    } while ((uint32_t)(HAL_GetTick() - start) < IMU_MAG_TIMEOUT_MS);

    return 0u;
}

/* Caller must hold the sensor-hub register page. */
static uint8_t imu_hub_write_mag(uint8_t reg, uint8_t value)
{
    if ((imu_write_reg(IMU_REG_SLV0_CONFIG, 0x00u) == 0u)
        || (imu_write_reg(IMU_REG_SLV0_ADD, (uint8_t)(QMC5883L_ADDR << 1)) == 0u)
        || (imu_write_reg(IMU_REG_SLV0_SUBADD, reg) == 0u)
        || (imu_write_reg(IMU_REG_DATAWRITE_SLV0, value) == 0u)
        || (imu_write_reg(IMU_REG_MASTER_CONFIG, IMU_MASTER_WRITE_ONCE) == 0u)) {
        return 0u;
    }

    return imu_hub_wait_status(0x80u);
}

static uint8_t imu_hub_write_mag_retry(uint8_t reg, uint8_t value)
{
    uint8_t retry;

    for (retry = 0u; retry < 3u; retry++) {
        if (imu_hub_write_mag(reg, value) != 0u) {
            return 1u;
        }
        HAL_Delay(2u);
    }
    return 0u;
}

/* Caller must hold the sensor-hub register page. */
static uint8_t imu_hub_read_mag(uint8_t reg, uint8_t *value)
{
    if ((imu_write_reg(IMU_REG_SLV0_ADD,
                       (uint8_t)((QMC5883L_ADDR << 1) | 0x01u)) == 0u)
        || (imu_write_reg(IMU_REG_SLV0_SUBADD, reg) == 0u)
        || (imu_write_reg(IMU_REG_SLV0_CONFIG, 0x01u) == 0u)
        || (imu_write_reg(IMU_REG_MASTER_CONFIG, IMU_MASTER_WRITE_ONCE) == 0u)) {
        return 0u;
    }
    if (imu_hub_wait_status(0x01u) == 0u) {
        return 0u;
    }
    return imu_read_reg(IMU_REG_SENSOR_HUB_1, value);
}

static uint8_t imu_mag_init(void)
{
    uint8_t attempt;
    uint8_t retry;
    uint8_t value = 0u;
    uint8_t ok = 0u;

    s_data.mag_ok = 0u;
    s_data.mag_ready = 0u;
    s_data.mag_id = 0u;
    s_data.mag_step = 1u;
    s_data.mag_status = 0u;
    s_data.mag_raw[0] = 0;
    s_data.mag_raw[1] = 0;
    s_data.mag_raw[2] = 0;

#if IMU_MAG_ENABLE
    if (imu_write_reg(IMU_REG_FUNC_CFG, IMU_FUNC_CFG_HUB) == 0u) {
        return 0u;
    }

    s_data.mag_step = 2u;
    if (imu_write_reg(IMU_REG_MASTER_CONFIG, IMU_MASTER_RESET) != 0u) {
        HAL_Delay(2u);
        (void)imu_write_reg(IMU_REG_MASTER_CONFIG, 0x00u);
        HAL_Delay(2u);

        for (attempt = 0u; attempt < 3u; attempt++) {
            s_data.mag_step = 3u;
            if (imu_hub_write_mag_retry(QMC5883L_REG_CONTROL2, 0x80u) == 0u) {
                continue;
            }
            HAL_Delay(20u);

            s_data.mag_step = 4u;
            if (imu_hub_write_mag_retry(QMC5883L_REG_CONTROL2, 0x00u) == 0u) {
                continue;
            }
            HAL_Delay(5u);

            for (retry = 0u; retry < 4u; retry++) {
                if ((imu_hub_read_mag(QMC5883L_REG_CHIP_ID, &value) != 0u)) {
                    s_data.mag_id = value;
                    if (value == QMC5883L_CHIP_ID) {
                        ok = 1u;
                        break;
                    }
                }
                HAL_Delay(5u);
            }
            if (ok != 0u) {
                break;
            }
        }

        if (ok != 0u) {
            s_data.mag_step = 5u;
            HAL_Delay(2u);
            ok = (uint8_t)(
                (imu_hub_write_mag_retry(QMC5883L_REG_CONTROL1,
                                         QMC5883L_CTRL1_8G) != 0u)
                && (imu_hub_write_mag_retry(QMC5883L_REG_FBR, 0x01u) != 0u));
            if (ok != 0u) {
                s_data.mag_step = 6u;
                ok = (uint8_t)(
                    (imu_write_reg(
                        IMU_REG_SLV0_ADD,
                        (uint8_t)((QMC5883L_ADDR << 1) | 0x01u)) != 0u)
                    && (imu_write_reg(IMU_REG_SLV0_SUBADD,
                                      QMC5883L_REG_OUTX_L) != 0u)
                    && (imu_write_reg(IMU_REG_SLV0_CONFIG, 0x06u) != 0u)
                    && (imu_write_reg(IMU_REG_MASTER_CONFIG,
                                      IMU_MASTER_AUTO_READ) != 0u));
            }
        }
    }

    (void)imu_write_reg(IMU_REG_FUNC_CFG, 0x00u);
    s_data.mag_ok = ok;
    if (ok != 0u) {
        s_data.mag_step = 0u;
    }
#endif
    return ok;
}

static uint8_t imu_mag_read_raw(void)
{
    uint8_t status;
    uint8_t raw[6];
    uint8_t i;
    uint8_t ok = 0u;

    if (s_data.mag_ok == 0u) {
        return 0u;
    }
    if (imu_write_reg(IMU_REG_FUNC_CFG, IMU_FUNC_CFG_HUB) == 0u) {
        return 0u;
    }
    if ((imu_read_reg(IMU_REG_STATUS_MASTER, &status) != 0u)
        && ((status & 0x01u) != 0u)) {
        ok = 1u;
        for (i = 0u; i < 6u; i++) {
            if (imu_read_reg((uint8_t)(IMU_REG_SENSOR_HUB_1 + i), &raw[i]) == 0u) {
                ok = 0u;
                break;
            }
        }
        if (ok != 0u) {
            s_data.mag_raw[0] = le16(&raw[0]);
            s_data.mag_raw[1] = le16(&raw[2]);
            s_data.mag_raw[2] = le16(&raw[4]);
        }
    }
    (void)imu_write_reg(IMU_REG_FUNC_CFG, 0x00u);
    s_data.mag_ready = ok;
    return ok;
}

uint8_t imu_whoami(void)
{
    uint8_t v = 0u;

    (void)imu_read_reg(IMU_REG_WHO_AM_I, &v);
    return v;
}

uint8_t imu_dbg_reg(uint8_t reg, uint8_t *val)
{
    return imu_read_reg(reg, val);
}

uint8_t imu_dbg_burst(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return imu_read_burst(reg, buf, len);
}

uint8_t imu_dbg_snapshot(uint8_t ctrl[3], uint8_t raw[12])
{
    uint8_t ok = 1u;

    ok = (uint8_t)(ok & imu_read_reg(IMU_REG_CTRL1_XL, &ctrl[0]));
    ok = (uint8_t)(ok & imu_read_reg(IMU_REG_CTRL2_G,  &ctrl[1]));
    ok = (uint8_t)(ok & imu_read_reg(IMU_REG_CTRL3_C,  &ctrl[2]));
    ok = (uint8_t)(ok & imu_read_burst(IMU_REG_OUTX_L_G, raw, 12u));
    return ok;
}

uint32_t imu_err_count(void)
{
    return s_err_cnt;
}

uint8_t imu_recalibrate(void)
{
    if ((s_st != IMU_ST_RUN) && (s_st != IMU_ST_CAL_FAILED)) {
        return 0u;
    }

    imu_cal_reset_window();
    s_data.ok = 0u;
    s_data.cal_status = IMU_CAL_SETTLING;
    s_data.yaw = 0.0f;
    s_data.sample_ms = 0u;
    s_data.sample_dt_ms = 0u;
    s_run_fail_n = 0u;
    s_st = IMU_ST_SETTLE;
    s_last_ms = imu_ms();
    s_cal_start_ms = s_last_ms;
    return 1u;
}

void imu_yaw_reset(void)
{
    s_data.yaw = 0.0f;
}

const imu_data_t *imu_get(void)
{
    return &s_data;
}

/* 读一帧并做量程合理性检查；成功返回 1，坏帧返回 0（已计 err） */
static uint8_t imu_read_frame(uint8_t *buf)
{
    float a[3];
    float mod2;
    uint8_t i;
    uint8_t attempt;
    uint8_t read_ok;

    /* 逐字节读：本模块（逐飞 IMU963RA，带电平转换缓冲）在 9 MHz 下做 12 字节
       自动递增连读时只有头一两个字节有效、其余全是 0x00，实测不可用；
       单字节读完全正常（静止时 |a|≈1.00 g），12 次单读约 25 µs，10 ms 一帧足够。 */
    read_ok = 0u;
    for (attempt = 0u; attempt < IMU_FRAME_RETRY_COUNT; attempt++) {
        read_ok = 1u;
        for (i = 0u; i < 12u; i++) {
            if (imu_read_reg((uint8_t)(IMU_REG_OUTX_L_G + i), &buf[i]) == 0u) {
                read_ok = 0u;
                break;
            }
        }
        if (read_ok != 0u) {
            break;
        }
    }
    if (read_ok == 0u) {
        return 0u;
    }

    for (i = 0u; i < 3u; i++)
    {
        a[i] = (float)le16(&buf[6u + 2u * i]) * IMU_ACC_LSB_G;
    }

    /* 静止/匀速时 |a| 必须≈1g；越界说明 SPI 错位或模块异常 */
    mod2 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    if ((mod2 < (IMU_ACC_MOD_MIN_G * IMU_ACC_MOD_MIN_G)) ||
        (mod2 > (IMU_ACC_MOD_MAX_G * IMU_ACC_MOD_MAX_G)))
    {
        s_err_cnt++;
        return 0u;
    }
    return 1u;
}

/* 主上下文调用（MX_SPI2_Init 之后、sched_init 之前）：探测 + 软复位。
   零偏标定不在这里做，交给 imu_task() 的 CAL 状态逐帧完成。 */
void imu_init(void)
{
    uint8_t v = 0u;

    s_data.ok        = 0u;
    s_data.cal_status = IMU_CAL_NO_SENSOR;
    s_data.yaw       = 0.0f;
    s_data.sample_ms = 0u;
    s_data.sample_dt_ms = 0u;
    s_data.mag_ok    = 0u;
    s_data.mag_ready = 0u;
    s_data.mag_id    = 0u;
    s_data.mag_step  = 0u;
    s_data.mag_status = 0u;
    s_last_ms        = 0u;
    s_last_probe     = 0u;
    s_last_mag_ms    = 0u;
    s_cal_start_ms   = 0u;
    s_run_fail_n     = 0u;
    imu_cal_reset_window();
    for (uint8_t i = 0u; i < 3u; i++) {
        s_data.acc_g[i]     = 0.0f;
        s_data.gyro_raw_dps[i] = 0.0f;
        s_data.gyro_dps[i]  = 0.0f;
        s_data.gyro_bias[i] = 0.0f;
        s_data.mag_raw[i]   = 0;
    }

    cs_high();                         /* 先放开片选，模块未接时也不会选中 */
    s_st = IMU_ST_OFF;

    /* 不再 HAL_Delay(20)：imu_init() 现在排在 OLED 非阻塞初始化之后，模块上电早已超过
       20 ms；若模块仍缺席，imu_task() 的 IMU_ST_OFF 分支会按 IMU_REPROBE_MS 限频重探。 */

    if ((imu_read_reg(IMU_REG_WHO_AM_I, &v) == 0u) || (v != IMU_WHOAMI_VAL))
    {
        return;                        /* 未接/接线错：保持 OFF，imu_task 限频重探 */
    }

    (void)imu_write_reg(IMU_REG_CTRL3_C, IMU_CTRL3_RESET);
    s_st      = IMU_ST_RESET;          /* 下一次 tick 写配置（间隔 10 ms） */
    s_last_ms = imu_ms();
}

void imu_task(void)
{
    uint32_t now = imu_ms();
    uint32_t dt_ms;
    uint8_t  buf[12];
    uint8_t  i;
    float    sensor_gyro[3];
    float    sensor_acc[3];
    float    vehicle_gyro[3];
    float    vehicle_acc[3];
    float    bias[3];
    float    sensor_bias[3];

    switch (s_st)
    {
    case IMU_ST_OFF:
        /* 模块缺席：每 IMU_REPROBE_MS 才探一次，避免在中断里空转 SPI */
        if ((now - s_last_probe) < IMU_REPROBE_MS)
        {
            return;
        }
        s_last_probe = now;
        if (imu_whoami() == IMU_WHOAMI_VAL)
        {
            (void)imu_write_reg(IMU_REG_CTRL3_C, IMU_CTRL3_RESET);
            s_st      = IMU_ST_RESET;
            s_last_ms = now;
        }
        return;

    case IMU_ST_RESET:
        /* 软复位已过至少 10 ms：写工作配置，进入零偏标定 */
        (void)imu_write_reg(IMU_REG_CTRL3_C, IMU_CTRL3_CFG);
        (void)imu_write_reg(IMU_REG_CTRL1_XL, IMU_CTRL1_XL_104HZ);
        (void)imu_write_reg(IMU_REG_CTRL2_G,  IMU_CTRL2_G_104HZ);
        (void)imu_mag_init();
        imu_cal_reset_window();
        s_data.ok = 0u;
        s_run_fail_n = 0u;
        s_data.cal_status = IMU_CAL_SETTLING;
        s_data.sample_ms = 0u;
        s_cal_start_ms = now;
        /* 先"沉降"再标定：LSM6DSR 上电后零偏要 1~2 s 才稳定，立刻标定会留下 ~1.8 dps 残差。 */
        s_last_ms = now;
        s_st      = IMU_ST_SETTLE;
        return;

    case IMU_ST_SETTLE:
        if ((uint32_t)(now - s_cal_start_ms) >= IMU_CAL_TIMEOUT_MS) {
            s_data.ok = 0u;
            s_data.cal_status = IMU_CAL_FAILED;
            s_st = IMU_ST_CAL_FAILED;
            break;
        }
        /* 上电后先等 IMU_SETTLE_MS 让陀螺零偏沉降，再开始采样标定 */
        if ((now - s_last_ms) < IMU_SETTLE_MS)
        {
            return;
        }
        imu_cal_reset_window();
        s_last_ms = now;
        s_st      = IMU_ST_CAL;
        s_data.cal_status = IMU_CAL_COLLECTING;
        return;

    case IMU_ST_CAL:
        if ((uint32_t)(now - s_cal_start_ms) >= IMU_CAL_TIMEOUT_MS) {
            s_data.ok = 0u;
            s_data.cal_status = IMU_CAL_FAILED;
            s_st = IMU_ST_CAL_FAILED;
            break;
        }
        if ((now - s_last_ms) < IMU_SAMPLE_MS)
        {
            return;
        }
        dt_ms = (s_data.sample_ms == 0u) ? IMU_SAMPLE_MS
                                         : (uint32_t)(now - s_data.sample_ms);
        s_last_ms = now;

        if (imu_read_frame(buf) == 0u)
        {
            imu_cal_reset_window();
            return;
        }

        if (dt_ms > IMU_CAL_MAX_SAMPLE_GAP_MS) {
            imu_cal_reset_window();
        }
        if (imu_cal_encoder_still() == 0u) {
            imu_cal_reset_window();
            return;
        }

        for (i = 0u; i < 3u; i++)
        {
            int16_t raw_gyro = le16(&buf[2u * i]);

            s_cal_samples[s_cal_n][i] = raw_gyro;
            sensor_gyro[i] = (float)raw_gyro * IMU_GYRO_LSB_DPS;
            sensor_acc[i] =
                (float)le16(&buf[6u + 2u * i]) * IMU_ACC_LSB_G;
            s_cal_sum[i] += sensor_gyro[i];
            s_cal_sum_sq[i] += sensor_gyro[i] * sensor_gyro[i];
            s_acc_sum[i] += sensor_acc[i];
            s_acc_sum_sq[i] += sensor_acc[i] * sensor_acc[i];
        }
        imu_map_to_vehicle(sensor_gyro, vehicle_gyro);
        imu_map_to_vehicle(sensor_acc, vehicle_acc);
        for (i = 0u; i < 3u; i++) {
            s_data.gyro_raw_dps[i] = vehicle_gyro[i];
            s_data.gyro_dps[i] = vehicle_gyro[i] - s_data.gyro_bias[i];
            s_data.acc_g[i] = vehicle_acc[i];
        }
        s_cal_n++;
        s_data.sample_dt_ms = dt_ms;
        s_data.sample_ms = now;

        if (s_cal_n >= IMU_BIAS_SAMPLES)
        {
            if (imu_cal_finish(sensor_bias) != 0u) {
                imu_map_to_vehicle(sensor_bias, bias);
                for (i = 0u; i < 3u; i++) {
                    s_data.gyro_bias[i] = bias[i];
                    s_data.gyro_dps[i] = s_data.gyro_raw_dps[i] - bias[i];
                }
                s_data.yaw = 0.0f;
                s_data.ok = 1u;
                s_data.cal_status = IMU_CAL_READY;
                s_run_fail_n = 0u;
                s_st = IMU_ST_RUN;
                s_last_ms = now;
            } else {
                imu_cal_reset_window();
            }
        }
        return;

    case IMU_ST_CAL_FAILED:
    case IMU_ST_RUN:
    default:
        break;
    }

    /* RUN 与校准超时状态继续采样，后者仅供诊断，不输出已校准姿态。 */
    if ((now - s_last_ms) < IMU_SAMPLE_MS)
    {
        return;
    }
    dt_ms = (s_data.sample_ms == 0u) ? 0u
                                     : (uint32_t)(now - s_data.sample_ms);
    s_last_ms = now;

    if (imu_read_frame(buf) == 0u)
    {
        if (++s_run_fail_n >= IMU_RUN_FAIL_LIMIT) {
            s_data.ok = 0u;
            s_data.cal_status = IMU_CAL_NO_SENSOR;
            s_st = IMU_ST_OFF;
        }
        return;
    }
    s_run_fail_n = 0u;

    if ((s_st == IMU_ST_RUN)
        && ((uint32_t)(now - s_last_mag_ms) >= IMU_MAG_SAMPLE_MS)) {
        s_last_mag_ms = now;
        (void)imu_mag_read_raw();
    }

    for (i = 0u; i < 3u; i++)
    {
        sensor_gyro[i] = (float)le16(&buf[2u * i]) * IMU_GYRO_LSB_DPS;
        sensor_acc[i] =
            (float)le16(&buf[6u + 2u * i]) * IMU_ACC_LSB_G;
    }
    imu_map_to_vehicle(sensor_gyro, vehicle_gyro);
    imu_map_to_vehicle(sensor_acc, vehicle_acc);
    for (i = 0u; i < 3u; i++) {
        s_data.gyro_raw_dps[i] = vehicle_gyro[i];
        s_data.gyro_dps[i] = vehicle_gyro[i] - s_data.gyro_bias[i];
        s_data.acc_g[i] = vehicle_acc[i];
    }

    /* 车辆坐标 +X 车头、+Y 车左、+Z 车顶；平放时加速度为 (0,0,+1g)。 */
    {
        float flat = sqrtf((s_data.acc_g[1] * s_data.acc_g[1])
                           + (s_data.acc_g[2] * s_data.acc_g[2]));
        float rollRaw  = atan2f(s_data.acc_g[1], s_data.acc_g[2]) * 57.29578f;
        float pitchRaw = atan2f(-s_data.acc_g[0], flat) * 57.29578f;

        s_data.roll_deg  = rollRaw  - IMU_ROLL_OFFSET;
        s_data.pitch_deg = pitchRaw - IMU_PITCH_OFFSET;
    }

    s_data.sample_dt_ms = dt_ms;
    s_data.sample_ms = now;
    s_data.ok = (s_st == IMU_ST_RUN) ? 1u : 0u;

    if (s_st == IMU_ST_RUN) {
        s_data.yaw += s_data.gyro_dps[2] * ((float)dt_ms * 0.001f);
        s_data.yaw = fmodf(s_data.yaw, 360.0f);
        if (s_data.yaw > 180.0f) s_data.yaw -= 360.0f;
        if (s_data.yaw < -180.0f) s_data.yaw += 360.0f;
    }
}

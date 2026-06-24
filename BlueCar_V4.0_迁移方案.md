# BlueCar V4.0 — MPU6050 Mahony 姿态解算 + 8路循迹 + 通用迁移方案

> **来源**: 融合 `2026_MSPM0G3507_CAR` (ICM42688 + Mahony + PID) 算法到 `BlueCar V3.0` (STM32F103ZET6 + MPU6050 + 裸机协程)
> **原则**: 算法与硬件解耦, 可直接迁移到 STM32/GD32/MM32/AT32/ESP32 等平台

---

## 目录

1. [硬件总览 (从实际代码确认)](#1-硬件总览)
2. [现有架构分析](#2-现有架构分析)
3. [PID 简化: 去掉 PulsePID](#3-pid-简化-去掉-pulsepid)
4. [MPU6050 Mahony 姿态解算](#4-mpu6050-mahony-姿态解算)
5. [8路循迹模块接口设计](#5-8路循迹模块接口设计)
6. [蓝牙协议保留与扩展](#6-蓝牙协议保留与扩展)
7. [文件修改清单与代码接口](#7-文件修改清单与代码接口)
8. [迁移检查清单](#8-迁移检查清单)
9. [附录: 关键公式速查](#9-附录关键公式速查)

---

## 1. 硬件总览

### 1.1 STM32F103ZET6 引脚分配 (从实际 CubeMX 代码确认)

| 模块 | 型号/参数 | STM32 引脚 | 外设实例 |
|------|-----------|-----------|----------|
| **MCU** | STM32F103ZET6, 72MHz | — | — |
| **左电机** | TB6612 A通道 | PWM: PE11 (TIM1_CH2), DIR: PF1/PF2 | TIM1 |
| **右电机** | TB6612 B通道 | PWM: PE9 (TIM1_CH1), DIR: PF3/PF4 | TIM1 |
| **左编码器** | JGB37-520, 30:1减速比 | PA6 (CH1), PA7 (CH2) | TIM3 编码器模式 |
| **右编码器** | JGB37-520, 30:1减速比 | PA0 (CH1), PA1 (CH2) | TIM2 编码器模式 |
| **MPU6050** | I2C, AD0=0, 地址 0xD0 | PB10=SCL, PB11=SDA | **I2C2** |
| **OLED** | SSD1306, 128×64 | PB6=SCL, PB7=SDA | **I2C1** |
| **蓝牙 HC-05** | UART, 9600bps | PA2=TX, PA3=RX | **USART2** |
| **调试串口** | USB-TTL, 115200bps | PA9=TX, PA10=RX | **USART1** |
| **按键×4** | 上拉, 双边沿 EXTI | PC6/PC7/PC8/PC9 | EXTI9_5 |
| **蜂鸣器** | 有源 | PG6 | GPIO |
| **系统时基** | 1kHz | — | TIM5 (millis) + TIM6 (任务调度) |

### 1.2 编码器参数

| 参数 | 值 | 说明 |
|------|-----|------|
| PPR | 11 | 编码器线数 (每转脉冲) |
| 减速比 | 30:1 | 电机轴:输出轴 |
| 倍频 | 4× | TIM 编码器模式 TI12 |
| 轮转一圈脉冲 | 1320 | `11 × 4 × 30` |
| 轮半径 | 0.065m | — |

### 1.3 PWM 参数

```
TIM1: 72MHz / 72 / 4096 ≈ 244Hz  (PSC=71, ARR=4095)
CCR = Speed × 41  (0~100 → 0~4096 线性映射)
```

---

## 2. 现有架构分析

### 2.1 协程任务调度 (裸机, 无 RTOS)

```
TIM6 ISR (1kHz):
  ├── 每 1ms: tick 计数
  ├── 每 10ms:
  │     ├── Key_Tick()          按键状态机
  │     ├── Task_Sensor_10ms()   MPU6050 非阻塞 I2C 读取 + 陀螺积分
  │     └── Task_Pulse_10ms()    [将被删除, 见第3章]
  └── 每 50ms:
        ├── Task_Control_50ms()  外环 SpeedPID/YawPID + 命令队列消费
        └── Task_VOFA_Send()     串口遥测 CSV 输出

Main Loop (while 1):
  ├── Cmd_ProcessBluetooth()     蓝牙 → cmd_queue
  ├── Cmd_ProcessKeys()          按键 → cmd_queue
  ├── Cmd_ProcessPIDTune()       串口命令 → 改 PID / cmd_queue
  ├── OLED_Flush()               I2C 非阻塞帧刷新
  └── Task_OLED_Update()         每 200ms 更新显示内容
```

### 2.2 命令解耦架构

```
┌──────────────┐   CmdMsg    ┌──────────────────┐   VofaMsg   ┌──────────────┐
│ BT / Key /   │───→ Q ──→  │ Task_Control_50ms │───→ 快照 ──→│ Task_VOFA    │
│ PIDTune      │  (SPSC=8)   │ (命令消费+PID运算) │            │ _Send (CSV)  │
└──────────────┘             └──────────────────┘            └──────────────┘
```

### 2.3 当前 MPU6050 姿态解算 (需升级)

当前方案非常简单, 仅做陀螺 Z 轴积分 + 加速度 atan2 倾角:

```c
gz = (GZ - GZ_Offset) / 32768.0 * 2000.0;           // °/s
YawAngle += gz * 0.01;                               // 纯积分 (每10ms)
AngleAcc = -atan2(ax_cali, az_cali) * 180/PI;         // 加速度倾角
TiltFilter = 0.8*TiltFilter + 0.2*AngleAcc;           // EMA 低通
```

**问题**: 无陀螺仪 Roll/Pitch 积分, 动态时倾角不准; 无互补滤波, 振动噪声直接耦合。

---

## 3. PID 简化: 去掉 PulsePID

### 3.1 当前三层架构 → 简化二层

```
【当前三层】                            【简化二层】
SpeedPID (50ms) → 脉冲修正              SpeedPID (50ms) → PWM
    ↓                                       ↓ + 前馈 + 死区补偿
PulsePID (10ms) → PWM (±100)              Motor PWM
    ↓
Motor PWM
```

### 3.2 为什么去掉 PulsePID

| 理由 | 说明 |
|------|------|
| **MSPM0 项目已验证** | ICM42688 项目只用 SpeedPID → PWM, 循迹过直角弯、位置环都跑得稳 |
| **前馈承担了大部分输出** | PulsePID.Kf=2.5 已提供约 85% 的稳态输出, PID 只是微调, 没有必要单独一层 |
| **电机机械时间常数 >> 10ms** | JGB37-520 带 30:1 减速箱, 50ms 闭环完全够用, 10ms 内环是过度设计 |
| **减少调参维度** | 从 8 个参数 (SpeedPID 4 + PulsePID 4) 降到 4 个参数 (SpeedPID 4) |
| **百分比空间归一化已经很聪明** | SpeedPID.Target/Actual 都归一化到 `% of spd_max`, 各档位自适应, 不需要额外内环 |

### 3.3 简化后的 Task_Control_50ms (核心)

```c
void Task_Control_50ms(void)
{
    /*---- 读取编码器 ----*/
    updateEncoderLoopSimpleVersion(&ecd_left,  50, TIM3);
    updateEncoderLoopSimpleVersion(&ecd_right, 50, TIM2);

    /* PWM 设置模式 ----*/
    if (Mode == MODE_PWM_SET) {
        Motor_SetSpeedA(PWMDuty);
        Motor_SetSpeedB(PWMDuty);
        return;
    }

    /*---- 处理命令队列 (不变) ----*/
    while (CmdQueue_Available()) {
        CmdMsg msg;
        if (CmdQueue_Get(&msg) < 0) break;
        switch (msg.type) {
            /* ... 全部保留, 移除 PulsePID 相关清零 ... */
        }
    }

    float ave = (ecd_left.velocity.angular + ecd_right.velocity.angular) / 2.0f;

    /* 停车 ----*/
    if (!RunFlag) {
        ecd_left.velocity.angular  = 0;
        ecd_right.velocity.angular = 0;
        Motor_SetSpeedA(0); Motor_SetSpeedB(0);
        /* 填充 vofa_data ... */
        return;
    }

    /*---- SpeedPID (50ms): 百分比空间, 输出直接 = PWM ----*/

    /* EMA 滤波 RPM */
    static float rpm_filtered = 0;
    rpm_filtered = 0.5f * rpm_raw + 0.5f * rpm_filtered;

    float spd_max = SpeedTable[SpeedLevel - 1];
    if (spd_max < 1.0f) spd_max = 1.0f;

    /* 目标斜坡 (每 50ms 最多变 16 RPM) */
    static float ramp_target = 0;
    float delta = SpeedPID.Target - ramp_target;
    if      (delta >  16.0f) ramp_target += 16.0f;
    else if (delta < -16.0f) ramp_target -= 16.0f;
    else                      ramp_target = SpeedPID.Target;

    /* SpeedPID 归一化到 % of spd_max */
    SpeedPID.Actual = rpm_filtered / spd_max * 100.0f;
    float save_tgt = SpeedPID.Target;
    SpeedPID.Target = ramp_target / spd_max * 100.0f;
    PID_Update(&SpeedPID);
    SpeedPID.Target = save_tgt;

    /* ★ 前馈 + PID 输出直接作为 PWM 百分比 ★ */
    float ff_out = ramp_target / spd_max * 100.0f * 0.65f;  /* 前馈: Kf=0.65 */
    float pid_out = SpeedPID.Out;                            /* PID 修正 */
    float out = ff_out + pid_out;

    /* TB6612 死区补偿 */
    if (out > 0) out += 15.0f; else if (out < 0) out -= 15.0f;
    if (out > 100.0f) out = 100.0f;
    if (out < -100.0f) out = -100.0f;

    /*---- YawPID (100ms) ----*/
    static uint8_t outer_cycle = 0;
    float diff_yaw = 0;
    if (++outer_cycle >= 2) {
        outer_cycle = 0;
        YawPID.Actual = chassis_angle.yaw;
        PID_Update(&YawPID);
        diff_yaw = YawPID.Out;
    }

    /* 差速叠加 */
    float diff = TurnTarget + diff_yaw;
    int16_t L = (int16_t)(out + diff);
    int16_t R = (int16_t)(out - diff);
    if (L >  100) L =  100; if (L < -100) L = -100;
    if (R >  100) R =  100; if (R < -100) R = -100;

    Motor_SetSpeedA(L);
    Motor_SetSpeedB(R);

    /* 填充 vofa_data (字段见附录) */
}
```

### 3.4 SpeedPID 推荐参数

```c
PID_t SpeedPID = {
    .Kp = 0.5f,  .Ki = 0.06f, .Kd = 0.1f,
    .OutMax =  30,   .OutMin = -30,      /* PID 修正量 ±30%, 前馈 + PID 最多 ±95% */
    .OutOffset       = 0,                 /* 死区补偿移到外部 */
    .IntegralLimit   = 35.0f,
    .Deadband        = 3.0f,
    .OutputRateLimit = 12.0f,
};
```

### 3.5 删除 Task_Pulse_10ms

`timer.c` 的 `HAL_TIM_PeriodElapsedCallback` 中删除 `Task_Pulse_10ms()` 调用:

```c
if (++tick_10ms >= 10) {
    tick_10ms = 0;
    if (++tick_20ms >= 2) { tick_20ms = 0; Key_Tick(); }
    flag_10ms = 1;
    Task_Sensor_10ms();     /* MPU6050 传感器, 非阻塞 I2C */
    // Task_Pulse_10ms();   /* ← 删除此行 */
}
```

### 3.6 删除 PulsePID 相关代码

`main.c` 中删除:

```c
// 删除这行声明
void Task_Pulse_10ms(void);

// 删除 PID_Init + PID_SetParam
PID_Init(&PulsePID);
PID_SetParam(&PulsePID, 2.0f, 0.1f, 0.3f);

// 删除 CMD_STOP 中的 PulsePID 清零
// PulsePID.Out = 0; PulsePID.IOut = 0; 等

// 删除整个 Task_Pulse_10ms 函数体
```

---

## 4. MPU6050 Mahony 姿态解算

### 4.1 新增文件: `imu_mahony.h`

```c
/**
 * @file    imu_mahony.h
 * @brief   Mahony 互补滤波姿态解算 (从 MSPM0 ICM42688 移植, 适配 MPU6050)
 * @note    输入: gx/gy/gz (rad/s), ax/ay/az (g), dt (秒)
 *          输出: roll/pitch/yaw (度)
 *
 *          Yaw 轴不修正 (重力矢量无法观测 Yaw), 纯靠陀螺积分 + 零偏校准
 */
#ifndef __IMU_MAHONY_H
#define __IMU_MAHONY_H

#include <stdint.h>
#include <math.h>

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngle_t;

extern EulerAngle_t chassis_angle;

typedef struct {
    float Kp;
    float Ki;
    float g_deadband;    /* 陀螺死区 rad/s (默认 0.05 = ~2.9°/s) */
    float bias_x;        /* 陀螺零偏 (原始值, 量程转换前减去) */
    float bias_y;
    float bias_z;
} Mahony_Param_t;

extern Mahony_Param_t mahony_param;

void IMU_Calibrate_Zero_Attitude(void);
void IMU_Attitude_Update(float gx, float gy, float gz,
                         float ax, float ay, float az, float dt);
void IMU_Attitude_Reset(void);

#endif
```

### 4.2 新增文件: `imu_mahony.c`

```c
/**
 * @file    imu_mahony.c
 * @brief   Mahony 互补滤波 — 从 MSPM0 移植, 适配 MPU6050
 *
 *          算法流程:
 *            1. 陀螺死区        |ω| < deadband → ω = 0
 *            2. 四元数 → 重力方向估计 (vx,vy,vz)
 *            3. 加速度归一化
 *            4. 叉积求误差 (仅 Roll/Pitch, 不修正 Yaw)
 *            5. PI 修正陀螺
 *            6. 一阶龙格-库塔更新四元数
 *            7. 四元数归一化
 *            8. 四元数 → 欧拉角 (度)
 *            9. 减去零点姿态
 */
#include "imu_mahony.h"

EulerAngle_t  chassis_angle = {0};
Mahony_Param_t mahony_param = {
    .Kp = 0.8f, .Ki = 0.0001f, .g_deadband = 0.05f,
    .bias_x = 0, .bias_y = 0, .bias_z = 0,
};

static EulerAngle_t ZeroAttitude = {0};
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float eInt[3] = {0};

void IMU_Calibrate_Zero_Attitude(void)
{
    ZeroAttitude.roll  = chassis_angle.roll;
    ZeroAttitude.pitch = chassis_angle.pitch;
    ZeroAttitude.yaw   = chassis_angle.yaw;
}

void IMU_Attitude_Reset(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    eInt[0] = 0; eInt[1] = 0; eInt[2] = 0;
}

void IMU_Attitude_Update(float gx, float gy, float gz,
                         float ax, float ay, float az, float dt)
{
    float halftime = dt * 0.5f;

    /* 1. 陀螺死区 */
    if (fabsf(gx) < mahony_param.g_deadband) gx = 0.0f;
    if (fabsf(gy) < mahony_param.g_deadband) gy = 0.0f;
    if (fabsf(gz) < mahony_param.g_deadband) gz = 0.0f;

    /* 2. 四元数 → 重力方向 */
    float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

    float vx = 2.0f * (q1q3 - q0q2);
    float vy = 2.0f * (q0q1 + q2q3);
    float vz = 1.0f - 2.0f * (q1q1 + q2q2);

    /* 3. 加速度归一化 */
    float norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= norm; ay *= norm; az *= norm;

    /* 4. 叉积误差 (仅 Roll/Pitch) */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;

    /* 5. PI 修正陀螺 */
    eInt[0] += ex;
    eInt[1] += ey;

    gx += mahony_param.Kp * ex + mahony_param.Ki * eInt[0];
    gy += mahony_param.Kp * ey + mahony_param.Ki * eInt[1];

    /* 6. 一阶龙格-库塔更新四元数 */
    float q0_t = (-q1 * gx - q2 * gy - q3 * gz) * halftime;
    float q1_t = ( q0 * gx + q2 * gz - q3 * gy) * halftime;
    float q2_t = ( q0 * gy - q1 * gz + q3 * gx) * halftime;
    float q3_t = ( q0 * gz + q1 * gy - q2 * gx) * halftime;

    q0 += q0_t; q1 += q1_t; q2 += q2_t; q3 += q3_t;

    /* 7. 四元数归一化 */
    norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;

    /* 8. 四元数 → 欧拉角 (度) */
    chassis_angle.roll  = atan2f(2.0f*(q2*q3 + q0*q1), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 57.2957795f;
    chassis_angle.pitch = asinf(-2.0f*(q1*q3 - q0*q2)) * 57.2957795f;
    chassis_angle.yaw   = atan2f(2.0f*(q1*q2 + q0*q3), q0q0 + q1q1 - q2q2 - q3q3) * 57.2957795f;

    /* 9. 减去零点 */
    chassis_angle.roll  -= ZeroAttitude.roll;
    chassis_angle.pitch -= ZeroAttitude.pitch;
    chassis_angle.yaw   -= ZeroAttitude.yaw;
}
```

### 4.3 集成到 Task_Sensor_10ms

替换 `main.c` 中 `MPU6050_Process()`:

```c
static void MPU6050_Process(void)
{
    if (!MPU6050_ReadAsync_Done()) return;

    int16_t AX, AY, AZ, GX, GY, GZ;
    MPU6050_ReadAsync_Get(&AX, &AY, &AZ, &GX, &GY, &GZ);

    /* 量程转换 + 减去零偏 → rad/s 和 g */
    float gx = (float)(GX - mahony_param.bias_x) / 16.4f * 0.0174533f;
    float gy = (float)(GY - mahony_param.bias_y) / 16.4f * 0.0174533f;
    float gz = (float)(GZ - mahony_param.bias_z) / 16.4f * 0.0174533f;

    float ax = (float)(AX) / 2048.0f;
    float ay = (float)(AY) / 2048.0f;
    float az = (float)(AZ) / 2048.0f;

    IMU_Attitude_Update(gx, gy, gz, ax, ay, az, 0.01f);

    MPU6050_ReadAsync_Start();
}
```

### 4.4 启动校准

替换 `main.c` 中 `MPU6050_Calibrate()`:

```c
static void MPU6050_Calibrate(void)
{
    /* 采集 200 样本求零偏均值 */
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    for (int i = 0; i < 200; i++) {
        int16_t AX, AY, AZ, GX, GY, GZ;
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        sum_gx += GX; sum_gy += GY; sum_gz += GZ;
        if (i % 50 == 0) HAL_Delay(5);
    }
    mahony_param.bias_x = sum_gx / 200;
    mahony_param.bias_y = sum_gy / 200;
    mahony_param.bias_z = sum_gz / 200;

    /* 50+800 周期让姿态收敛 */
    MPU6050_ReadAsync_Start();
    for (int phase = 0; phase < 2; phase++) {
        int loops = (phase == 0) ? 50 : 800;
        for (int i = 0; i < loops; i++) {
            uint32_t t0 = millis;
            while (!MPU6050_ReadAsync_Done()) {
                if (millis - t0 > 10) break;
            }
            int16_t AX, AY, AZ, GX, GY, GZ;
            MPU6050_ReadAsync_Get(&AX, &AY, &AZ, &GX, &GY, &GZ);

            float gx = (float)(GX - mahony_param.bias_x) / 16.4f * 0.0174533f;
            float gy = (float)(GY - mahony_param.bias_y) / 16.4f * 0.0174533f;
            float gz = (float)(GZ - mahony_param.bias_z) / 16.4f * 0.0174533f;
            float ax = (float)(AX) / 2048.0f;
            float ay = (float)(AY) / 2048.0f;
            float az = (float)(AZ) / 2048.0f;

            IMU_Attitude_Update(gx, gy, gz, ax, ay, az, 0.005f);
            MPU6050_ReadAsync_Start();
        }
    }

    IMU_Calibrate_Zero_Attitude();
}
```

### 4.5 Mahony 调参命令 (pid_tune.c 新增)

```c
else if (strcmp(tok, "mkp") == 0 && n >= 2) {
    mahony_param.Kp = val;
    tune_rsp("[Mahony] Kp=%.4f\r\n", mahony_param.Kp);
}
else if (strcmp(tok, "mki") == 0 && n >= 2) {
    mahony_param.Ki = val;
    tune_rsp("[Mahony] Ki=%.6f\r\n", mahony_param.Ki);
}
else if (strcmp(tok, "mdb") == 0 && n >= 2) {
    mahony_param.g_deadband = val;
    tune_rsp("[Mahony] Deadband=%.4f\r\n", mahony_param.g_deadband);
}
```

### 4.6 需删除的旧全局变量

```c
// main.c 中删除:
int16_t GZ_Offset, AX_Offset, AZ_Offset;  // → mahony_param.bias_*
float YawAngle;                             // → chassis_angle.yaw
float TiltFilter;                           // → chassis_angle.roll / .pitch
```

**兼容技巧**: 用宏映射避免改遍所有旧代码:

```c
/* MPU6050 + Mahony ----------------------------------------------------------*/
static int16_t AX, AY, AZ, GX, GY, GZ;
#define YawAngle  chassis_angle.yaw   /* 兼容旧代码, 映射到 Mahony 输出 */
```

这样 VOFA、OLED、YawPID 里所有的 `YawAngle` 引用**完全不用改**，编译器自动展开为 `chassis_angle.yaw`。

---

## 5. 8路循迹模块接口设计

### 5.1 硬件接口预留

| 信号 | 建议引脚 | GPIO Port | 说明 |
|------|---------|-----------|------|
| IR1 | PE0 | GPIOE | 最左侧 |
| IR2 | PE1 | GPIOE | |
| IR3 | PE2 | GPIOE | |
| IR4 | PE3 | GPIOE | 中线左侧 |
| IR5 | PE4 | GPIOE | 中线右侧 |
| IR6 | PE5 | GPIOE | |
| IR7 | PE6 | GPIOE | |
| IR8 | PD12 | GPIOD | 最右侧 |

> 如使用模拟灰度传感器, 改用 ADC1 多通道扫描 (PA0~PA7)。

### 5.2 接口文件: `irtrack.h`

```c
/**
 * @file    irtrack.h
 * @brief   8路红外循迹传感器驱动接口
 *
 *          传感器排列 (车头视角):
 *            IR8 IR7 IR6 IR5 IR4 IR3 IR2 IR1
 *            ← 右                         左 →
 *
 *          权重: 检测到黑线=1, 白=0
 *            weight = Σ(sensor[i] × w[i])
 *            weight < 0 → 偏左需右转, weight > 0 → 偏右需左转
 */

#ifndef __IRTRACK_H
#define __IRTRACK_H
#include "main.h"

#define IRTRACK_CHANNELS  8

typedef enum {
    IR_MODE_DIGITAL = 0,
    IR_MODE_ANALOG  = 1,
} IR_Mode_t;

/* 十字路口/弯道状态 */
typedef enum {
    TRACK_NORMAL     = 0,
    TRACK_HARD_LEFT  = 1,   /* 左直角弯 */
    TRACK_HARD_RIGHT = 2,   /* 右直角弯 */
    TRACK_CROSS      = 3,   /* 十字路口 */
    TRACK_LOST       = 4,   /* 全部丢线 */
} IR_State_t;

typedef struct {
    uint8_t  raw[IRTRACK_CHANNELS];    /* 数字量: 0=白/1=黑 */
    int8_t   weight_sum;               /* 加权和 */
    uint8_t  black_count;              /* 检测到黑线数量 */
    IR_State_t state;                  /* 路口状态 */
} IRTrack_Data_t;

extern IRTrack_Data_t ir_data;

void IRTrack_Init(IR_Mode_t mode);
void IRTrack_Read(void);
void IRTrack_Compute(void);
IR_State_t IRTrack_DetectCrossroad(void);

#endif
```

### 5.3 权重表

```c
/* 方案 A: 等差数列 (适用 8 路等间距) */
static const int8_t ir_weight[IRTRACK_CHANNELS] = {
    -7, -5, -3, -1,  1,  3,  5,  7
};

/* 方案 B: 按物理位置 (mm) 归一化, 用于非等间距排列 */
// static const int8_t ir_weight[IRTRACK_CHANNELS] = {
//     -4, -3, -2, -1,  1,  2,  3,  4
// };
```

### 5.4 GPIO 初始化 (在 MX_GPIO_Init 中追加)

```c
static void IRTrack_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;    /* 上拉: 未触发=H, 触发=L → 逻辑取反后 黑=1 */
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
           |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
    HAL_GPIO_Init(GPIOE, &g);

    g.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOD, &g);
}
```

### 5.5 循迹控制集成 (在 Task_Control_50ms 中)

```c
/* 循迹模式 */
if (Mode == MODE_TRACK) {
    IRTrack_Read();
    IRTrack_Compute();

    IR_State_t cross = ir_data.state;

    switch (cross) {
    case TRACK_HARD_LEFT: {
        /* 前冲 → 差速过弯 → 消抖退出 */
        int32_t start = total_pulse_left;
        while ((total_pulse_left - start) < 500) {  /* 前冲 500 脉冲 */
            Motor_SetSpeedA(base_rpm);
            Motor_SetSpeedB(base_rpm);
            vTaskDelay_eqv(10);  /* 见下文裸机替代 */
        }
        uint8_t debounce = 0;
        while (1) {
            Motor_SetSpeedA(10);   /* 内轮慢 */
            Motor_SetSpeedB(50);   /* 外轮快 */
            IRTrack_Read(); IRTrack_Compute();
            if (ir_data.black_count > 0 && ir_data.black_count <= 3
                && abs(ir_data.weight_sum) <= 15) {
                if (++debounce >= 3) break;
            } else { debounce = 0; }
            HAL_Delay(10);  /* 注意: 仅循迹初始阶段可用阻赛延时, 正式版改为状态机 */
        }
        ir_data.state = TRACK_NORMAL;
        break;
    }

    case TRACK_NORMAL:
    default:
        if (ir_data.black_count > 0) {
            TrackPID.Target = 0.0f;
            TrackPID.Actual = (float)ir_data.weight_sum;
            PID_Update(&TrackPID);
            float diff = TrackPID.Out;
            Motor_SetSpeedA((int16_t)(base_rpm - diff));
            Motor_SetSpeedB((int16_t)(base_rpm + diff));
        } else {
            Motor_SetSpeedA(0); Motor_SetSpeedB(0);  /* 丢线停车 */
        }
        break;
    }
    return;
}
```

### 5.6 循迹 PID 参数

```c
PID_t TrackPID = {
    .Kp = 0.65f, .Ki = 0.0f, .Kd = 0.1f,
    .OutMax =  50, .OutMin = -50,
    .IntegralLimit   = 20.0f,
    .Deadband        = 3.0f,
    .OutputRateLimit = 15.0f,
};
```

---

## 6. 蓝牙协议保留与扩展

### 6.1 现有协议 (全部保留)

```
[joystick, LH, LV, RH, RV]  摇杆: LV=前后(-100~100), RH=转向(-100~100)
[key, 1, up]                 启停切换
[key, 2, up]                 档位-1
[key, 3, up]                 档位+1
```

### 6.2 新增协议

```
[mode, track]                切换到循迹模式
[mode, manual]               切换到手动蓝牙控制
[mode, pwm]                  切换到 PWM 直接调试模式
[tune, skp, 0.5]             在线调 SpeedPID.Kp
[tune, ykp, 0.3]             在线调 YawPID.Kp
[tune, mkp, 0.8]             在线调 Mahony.Kp
[imu, calib]                 触发陀螺零偏重校准
[imu, zero]                  触发姿态零点标定
[gear, N]                    直接设档位 N (1~6)
```

### 6.3 解析代码 (在 Cmd_ProcessBluetooth 中追加)

```c
/* 新增: mode 协议 */
if (strcmp(BlueSerial_StringArray[0], "mode") == 0) {
    if (strcmp(BlueSerial_StringArray[1], "track") == 0) {
        msg.type = CMD_MODE_TRACK; msg.val = 0; CmdQueue_Put(msg);
    } else if (strcmp(BlueSerial_StringArray[1], "manual") == 0) {
        msg.type = CMD_MODE_RUN; msg.val = 0; CmdQueue_Put(msg);
    } else if (strcmp(BlueSerial_StringArray[1], "pwm") == 0) {
        msg.type = CMD_MODE_PWM; msg.val = 0; CmdQueue_Put(msg);
    }
}
/* 新增: tune 协议 (蓝牙远程调 PID) */
else if (strcmp(BlueSerial_StringArray[0], "tune") == 0) {
    float v = atof(BlueSerial_StringArray[2]);
    if (strcmp(BlueSerial_StringArray[1], "skp") == 0) {
        msg.type = CMD_SET_SPEED_KP; msg.val = v; CmdQueue_Put(msg);
    } else if (strcmp(BlueSerial_StringArray[1], "ykp") == 0) {
        msg.type = CMD_SET_YAW_KP; msg.val = v; CmdQueue_Put(msg);
    }
    /* ... ski/skd/ykd/mkp 等类似 ... */
}
/* 新增: imu 协议 */
else if (strcmp(BlueSerial_StringArray[0], "imu") == 0) {
    if (strcmp(BlueSerial_StringArray[1], "calib") == 0) {
        msg.type = CMD_IMU_CALIB; msg.val = 0; CmdQueue_Put(msg);
    } else if (strcmp(BlueSerial_StringArray[1], "zero") == 0) {
        msg.type = CMD_IMU_ZERO; msg.val = 0; CmdQueue_Put(msg);
    }
}
/* 新增: gear 协议 */
else if (strcmp(BlueSerial_StringArray[0], "gear") == 0) {
    int8_t g = atoi(BlueSerial_StringArray[1]);
    if (g >= 1 && g <= 6) SpeedLevel = g;
}
```

---

## 7. 文件修改清单与代码接口

### 7.1 新增文件

| 文件 | 路径 | 说明 |
|------|------|------|
| `imu_mahony.c` | `Core/Hardware/` | Mahony 互补滤波姿态解算 (见 4.2) |
| `imu_mahony.h` | `Core/Hardware/` | 姿态解算头文件 (见 4.1) |
| `irtrack.c` | `Core/Hardware/` | 8路循迹传感器驱动 (见 5.2~5.4) |
| `irtrack.h` | `Core/Hardware/` | 循迹传感器接口 |

### 7.2 修改文件

| 文件 | 修改内容 |
|------|---------|
| **main.c** | ① `#include "imu_mahony.h"` + `#include "irtrack.h"` |
| | ② 删除 `YawAngle`/`TiltFilter`/`GZ_Offset`/`AX_Offset`/`AZ_Offset` |
| | ③ 删除 `PulsePID` 全局变量 + `PID_Init(&PulsePID)` |
| | ④ 删除 `Task_Pulse_10ms()` 整个函数 |
| | ⑤ 替换 `MPU6050_Process()` → Mahony 版本 (4.3) |
| | ⑥ 替换 `MPU6050_Calibrate()` → 三段式校准 (4.4) |
| | ⑦ 重写 `Task_Control_50ms()` → SpeedPID 直接输出 PWM (3.3) |
| | ⑧ 添加循迹分支 `MODE_TRACK` (5.5) |
| | ⑨ 处理 `CMD_IMU_CALIB`/`CMD_IMU_ZERO`/`CMD_MODE_TRACK` 命令 |
| **main.h** | ① `Mode_t` 新增 `MODE_TRACK` |
| | ② `CmdType` 新增 `CMD_IMU_CALIB`/`CMD_IMU_ZERO`/`CMD_MODE_TRACK`/`CMD_MODE_RUN`/`CMD_MODE_PWM` |
| **timer.c** | 删除 `Task_Pulse_10ms()` 调用 (3.5) |
| **pid_tune.c** | 新增 `mkp`/`mki`/`mdb` 命令 |
| **blueserial.c** | 新增 `mode`/`tune`/`imu`/`gear` 协议解析 |
| **mpu6050.h** | 添加量程转换宏 `MPU6050_RAW_TO_G()`/`MPU6050_RAW_TO_DPS()`/`MPU6050_RAW_TO_RADS()` |

### 7.3 无需修改的文件

| 文件 | 原因 |
|------|------|
| `motor.c/h` | 电机驱动完全不变 |
| `encoder.c/h` | 编码器接口不变 |
| `pid.c/h` | PID 算法不变 (已足够完善) |
| `serial.c/h` | 串口非阻塞 TX 架构不变 |
| `ringbuf.c/h` | 环形缓冲区不变 |
| `key.c/h` | 按键不变 |
| `oled.c/h` | OLED 不变 |
| `buzzer.c/h` | 蜂鸣器不变 |
| `mpu6050.c` | 驱动不变 (Mahony 只消费 `MPU6050_ReadAsync_Get` 的输出) |
| `mpu6050_reg.h` | 寄存器宏不变 |
| `stm32f1xx_it.c` | 中断处理不变 |

---

## 8. 迁移检查清单

### 8.1 PID 简化

- [ ] 删除 `main.c` 中 `PulsePID` 变量和初始化代码
- [ ] 删除 `main.c` 中 `Task_Pulse_10ms()` 函数
- [ ] 删除 `timer.c` 中 `Task_Pulse_10ms()` 调用
- [ ] 重写 `Task_Control_50ms()`: SpeedPID 直接输出 PWM (3.3)
- [ ] `CMD_STOP` 中移除 PulsePID 相关清零
- [ ] 调参: SpeedPID 参数按 3.4 调整

### 8.2 Mahony 姿态解算

- [ ] 创建 `imu_mahony.c` / `imu_mahony.h`
- [ ] 修改 `main.c` 的 `MPU6050_Process()` 为 Mahony 版本
- [ ] 修改 `main.c` 的 `MPU6050_Calibrate()` 为三段式校准
- [ ] 删除旧的 `YawAngle`/`TiltFilter`/`*_Offset` 全局变量
- [ ] 同步修改 VOFA 输出和 OLED 显示 (YawAngle → chassis_angle.yaw)
- [ ] `pid_tune.c` 添加 `mkp`/`mki`/`mdb` 命令

### 8.3 8路循迹

- [ ] CubeMX 配置 PE0~PE6 + PD12 为 GPIO Input (Pull-up)
- [ ] 创建 `irtrack.c` (GPIO 读取 + 权重计算 + 路口检测)
- [ ] 创建 `irtrack.h`
- [ ] `main.c` 添加 `MODE_TRACK` 分支
- [ ] 添加 `TrackPID` 控制器
- [ ] 添加 `CMD_MODE_TRACK` 等命令

### 8.4 蓝牙

- [ ] `main.h` 添加新的 `CmdType` 枚举值
- [ ] `Cmd_ProcessBluetooth()` 添加 `mode`/`tune`/`imu`/`gear` 协议解析
- [ ] `Task_Control_50ms()` 添加对应命令处理

### 8.5 参数调优顺序

| 步骤 | 操作 | 预期 |
|------|------|------|
| 1 | 上电, `mkp 0.8`, `mki 0.0001` | 静止时 Roll/Pitch/Yaw 不漂 |
| 2 | 晃动车身 | Roll/Pitch 应快速跟踪, Yaw 积分正确 |
| 3 | `t 50` 直行 | RPM 平稳, 稳态误差小 |
| 4 | `skp`/`ski` 微调 | 加速不震荡, 匀速不抖动 |
| 5 | 接上循迹, `track` 模式 | 直线不摆头, 入弯不拖尾 |

---

## 9. 附录: 关键公式速查

### MPU6050 量程转换

```
加速度 (g)   = raw / 2048           (±16g, 2048 LSB/g)
陀螺 (°/s)   = raw / 16.4           (±2000°/s, 16.4 LSB/(°/s))
陀螺 (rad/s) = raw / 16.4 × π/180
             = raw × 0.0010642
```

### 编码器 RPM

```
RPM = count / (4 × 30 × 11) × (1000 / T_ms) × 60
    = count × 60000 / (1320 × T_ms)
    = count × 45.45 / T_ms          (50ms 周期: count × 0.909)
```

### Mahony 互补滤波参数关系

```
截止频率 ≈ Kp / (2π × dt) = 0.8 / (2π × 0.01) ≈ 12.7 Hz
时间常数 ≈ 1 / 12.7 ≈ 79 ms
Roll/Pitch 在 ~80ms 内收敛到加速度计观测值
```

### SpeedPID 输出 → PWM

```
pwm_out = ff_out + pid_out + deadband
        = (target/spd_max × 100 × 0.65) + SpeedPID.Out + sign(out)×15

L = clamp(pwm_out + diff, ±100)
R = clamp(pwm_out - diff, ±100)
```

### 循迹权重 → 差速 RPM

```
weight = Σ(sensor[i] × w[i])        // 例: w = [-7,-5,-3,-1,1,3,5,7]
diff   = PID_Update(&TrackPID)      // Target=0, Actual=weight
       ≈ Kp × weight + Kd × (weight - weight_prev)
```

---

> **生成时间**: 2026-06-24
> **源项目**: `2026_MSPM0G3507_CAR` → `BlueCar V3.0` → `BlueCar V4.0`
> **适用范围**: STM32F103ZET6 及任意 STM32F1/GD32/MM32/AT32 系列

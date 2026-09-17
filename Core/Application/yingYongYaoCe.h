/**
 * @file    yingYongYaoCe.h
 * @brief   VOFA 遥测数据接口。
 */
#ifndef __YING_YONG_YAO_CE_H
#define __YING_YONG_YAO_CE_H  /* 头文件保护宏，防止接口重复声明。 */

#include <stdint.h>

/**
 * @brief  VOFA CSV 遥测快照。
 * @note   控制任务负责填充数据，遥测发送任务只读取并格式化输出。
 */
typedef struct {
    float targetLeft;      /* 左轮目标转速，单位 RPM。 */
    float targetRight;     /* 右轮目标转速，单位 RPM。 */
    float rpmRight;        /* 右轮实际转速，单位 RPM。 */
    float rpmLeft;         /* 左轮实际转速，单位 RPM。 */
    float pwmLeft;         /* 左轮 PWM 输出百分比。 */
    float pwmRight;        /* 右轮 PWM 输出百分比。 */
    float yawOutput;       /* 手动模式为航向环输出；循迹模式为线路环转向差速，单位 RPM。 */
    float rpmDiff;         /* 左右轮实际转速差，单位 RPM。 */
    float yawAngle;        /* MPU6050 偏航角，单位度。 */
    int16_t bluetoothLv;   /* 蓝牙左摇杆纵向值。 */
    int16_t bluetoothRh;   /* 蓝牙右摇杆横向值。 */
    int16_t runFlag;       /* 运行状态，1=运行，0=停车。 */
    int16_t gear;          /* 当前速度档位。 */
    float rpmFilteredRight;/* 右轮滤波转速，单位 RPM。 */
    float rpmFilteredLeft; /* 左轮滤波转速，单位 RPM。 */
    int16_t fresh;         /* 新数据标志，1=需要发送。 */
} telemetryData_t;

/* 全局遥测快照，控制任务写、遥测任务读。 */
extern telemetryData_t telemetryData;

/**
 * @brief  初始化遥测数据结构。
 * @note   将所有字段清零，并清除待发送标志。
 */
void appTelemetryInit(void);

/**
 * @brief  发送一帧 VOFA CSV 遥测数据。
 * @note   无新数据、已关闭遥测或停车且转速接近 0 时不发送。
 */
void appTelemetrySend(void);

#endif

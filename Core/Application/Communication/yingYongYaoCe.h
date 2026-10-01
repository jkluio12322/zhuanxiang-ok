/**
 * @file    yingYongYaoCe.h
 * @brief   VOFA+ 遥测接口。
 */
#ifndef __YING_YONG_YAO_CE_H
#define __YING_YONG_YAO_CE_H  /* 头文件保护宏，防止遥测接口重复声明。 */

#include <stdint.h>

/**
 * @brief  VOFA+ 遥测快照。
 * @note   轮速和差速单位为 RPM，航向角单位为度，mode 为 workMode_t 整数。
 */
typedef struct {
    float targetLeftRpm;          /* 左轮目标速度，单位 RPM。 */
    float targetRightRpm;         /* 右轮目标速度，单位 RPM。 */
    float actualLeftRpm;          /* 左轮编码器速度，单位 RPM。 */
    float actualRightRpm;         /* 右轮编码器速度，单位 RPM。 */
    float filteredLeftRpm;        /* 左轮滤波后速度，单位 RPM。 */
    float filteredRightRpm;       /* 右轮滤波后速度，单位 RPM。 */
    float steerRpm;               /* 航向或循迹差速量，单位 RPM。 */
    float actualDiffRpm;          /* 左轮减右轮的实际速度差，单位 RPM。 */
    float yawAngleDeg;            /* MPU6050 航向角，单位度。 */
    float trackError;             /* 循迹加权误差，负值偏左，正值偏右。 */
    int16_t mode;                 /* 当前 workMode_t 值。 */
    int16_t runFlag;              /* 1 表示运行，0 表示停车。 */
    int16_t gear;                 /* 当前速度档位。 */
    int16_t turnPhase;            /* TrackTurnPhase_t 整数。 */
    float commandVoltageLeft;     /* 左轮最终命令电压，单位 V。 */
    float commandVoltageRight;    /* 右轮最终命令电压，单位 V。 */
    int16_t fresh;                /* 1 表示有尚未发送的新数据。 */
} telemetryData_t;

extern telemetryData_t telemetryData;

/**
 * @brief  清零遥测快照。
 */
void appTelemetryInit(void);

/**
 * @brief  将最新遥测快照格式化为 VOFA CSV 并发送。
 */
void appTelemetrySend(void);

#endif

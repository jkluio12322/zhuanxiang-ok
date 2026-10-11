/**
 * @file    app_telemetry.h
 * @brief   VOFA+ 遥测接口。
 */
#ifndef __APP_TELEMETRY_H
#define __APP_TELEMETRY_H  /* 头文件保护宏，防止遥测接口重复声明。 */

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
    volatile int16_t fresh;       /* 1 表示有尚未发送的新数据。
                                     跨上下文标志：50 ms 中断任务置 1，
                                     主循环读并清零，所以必须是 volatile。 */
} telemetryData_t;

extern telemetryData_t telemetryData;

/**
 * @brief  清零遥测快照。
 */
void AppTelemetry_Init(void);

/**
 * @brief  发送开机横幅和 VOFA CSV 表头。
 * @note   表头字段顺序必须和 AppTelemetry_Send() 里的格式串一致；
 *         这里是表头的唯一来源，不要在别处硬编码。
 */
void AppTelemetry_BootHeader(void);

/**
 * @brief  用本周期的控制结果填充遥测快照。
 * @param  targetLeft    左轮目标转速，单位 RPM。
 * @param  targetRight   右轮目标转速，单位 RPM。
 * @param  leftPwm       左轮实际输出百分比。
 * @param  rightPwm      右轮实际输出百分比。
 * @param  steerRpm      本周期差速量，单位 RPM。
 * @param  trackError    本周期误差（e_y）。
 * @param  turnPhase     循迹状态机的状态值。
 * @param  rpmDifference 左右实际速度差，单位 RPM。
 * @note   **中断上下文**（50 ms 控制任务调用）。只做赋值，不做格式化。
 *         内容要等主循环的 AppTelemetry_Send() 才真正发出去。
 */
void AppTelemetry_Publish(float targetLeft, float targetRight,
                          int16_t leftPwm, int16_t rightPwm,
                          float steerRpm, float trackError,
                          int16_t turnPhase, float rpmDifference);

/**
 * @brief  将最新遥测快照格式化为 VOFA CSV 并发送。
 * @note   在主循环里调用，不在中断里。
 */
void AppTelemetry_Send(void);

#endif

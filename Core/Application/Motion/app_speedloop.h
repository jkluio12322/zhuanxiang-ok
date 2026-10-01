/**
 * @file    app_speedloop.h
 * @brief   50 ms 控制任务：速度环、模式分发、电机输出。
 * @note    这是唯一直接驱动电机的模块。
 */
#ifndef __APP_SPEEDLOOP_H
#define __APP_SPEEDLOOP_H

#include <stdint.h>
#include "app_pid.h"

/* 速度环通道。只有左右两个环，没有平均环。 */
typedef enum {
    SPEED_CH_LEFT = 0,
    SPEED_CH_RIGHT
} SpeedChannel_t;

/* 供 show 命令打印的速度环信息。 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float actual;
    float out;
} SpeedPidInfo_t;

/**
 * @brief  初始化左右两个速度环。
 */
void AppSpeedLoop_Init(void);

/**
 * @brief  执行一次 50 ms 控制任务。
 * @note   在 TIM6 中断里调用。
 */
void AppSpeedLoop_Task50ms(void);

/**
 * @brief  清空速度环运行时状态，保留增益。
 */
void AppSpeedLoop_ResetRuntime(void);

/**
 * @brief  停车：清运行标志、清目标、复位速度环、关电机。
 */
void AppSpeedLoop_ResetMotion(void);

/**
 * @brief  复位整个模式：停车 + 复位循迹 + 复位测试模式。
 * @note   对应原来的 appControlResetModeState()，供开机页切换模式时使用。
 */
void AppSpeedLoop_ResetModeState(void);

/**
 * @brief  在线设置一个速度环增益。
 */
void AppSpeedLoop_SetGain(SpeedChannel_t channel, PidField_t field, float value);

/**
 * @brief  读取一个速度环的增益与运行状态。
 */
void AppSpeedLoop_GetInfo(SpeedChannel_t channel, SpeedPidInfo_t *out);

#endif /* __APP_SPEEDLOOP_H */

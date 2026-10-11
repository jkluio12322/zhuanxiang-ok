/**
 * @file    app_tracking.h
 * @brief   八路循迹：归一化质心 + 单个 PD 位置环 + 计时计圈。
 * @note    算法对齐参考实现 32_XJPIDJX：误差取归一化质心 e_y，单个位置式 PID
 *          输出差速，目标为居中（e_y = 0）。
 */
#ifndef __APP_TRACKING_H
#define __APP_TRACKING_H

#include <stdint.h>
#include "app_pid.h"

/* 循迹状态机状态。 */
typedef enum {
    TRK_ACQUIRE = 0,   /* 起步直行，直到线路居中或超时。 */
    TRK_FOLLOW,        /* 正常循迹。 */
    TRK_LOST,          /* 确认丢线，保持丢线前的舵量。 */
    TRK_STOPPED        /* 已停车。 */
} TrackState_t;

/* 只读循迹快照，字段直接读。 */
typedef struct {
    float        base_rpm;   /* 本周期使用的基础速度，即当前档位速度。 */
    float        steer_rpm;  /* 循迹环输出的差速量，单位 RPM。 */
    float        e_y;        /* 当前归一化横向偏差，范围约 ±7。 */
    TrackState_t state;      /* 当前状态机状态。 */
    uint8_t      pattern;    /* 八路黑线掩码，bit0..7 对应 IR1..8，压黑置 1。 */
                             /* 转发自驱动层，供显示模块画红外状态，
                                免得显示去直接读 ir_data。 */
} TrackOutput_t;

/**
 * @brief  获取只读循迹快照。
 */
const TrackOutput_t *AppTracking_Output(void);

/**
 * @brief  初始化循迹环。
 */
void AppTracking_Init(void);

/**
 * @brief  启动循迹，进入起步直行阶段。
 */
void AppTracking_Start(void);

/**
 * @brief  复位状态机、PID 和计时器。
 */
void AppTracking_ResetState(void);

/**
 * @brief  执行一次 20 ms 循迹任务。
 * @note   在 TIM6 中断里调用。
 */
void AppTracking_Task20ms(void);

/**
 * @brief  读取循迹环增益。
 * @note   只读。循迹环增益不再支持在线修改——在 app_config.h 里改后重新编译。
 */
float AppTracking_GetGain(PidField_t field);

/**
 * @brief  设置「差速 RPM 换算为 PWM 百分比」的增益，供 PWM 循迹测试用。
 */
void AppTracking_SetSteerRpmToPwmGain(float gain);

/**
 * @brief  读取上面的换算增益。
 */
float AppTracking_GetSteerRpmToPwmGain(void);

/**
 * @brief  PWM 循迹测试的一步：把一段八路图案当成传感器读数跑一遍循迹。
 * @param  pattern 要模拟的黑线掩码，bit0..7 对应 IR1..8，压黑置 1。
 * @return 1 表示本周期没有算差速（未运行，或已请求停车）；0 表示算过了。
 * @note   与实车路径共用同一个 sample、状态机、确认计数、PID 限幅和 lastSteer，
 *         所以丢线保持、全白停车、丢线超时的行为与实车一致；只是不读 GPIO、
 *         不做过弯判定与计圈。
 * @note   未运行时负责把状态机收回 TRK_STOPPED（与实车路径的 !running 分支对齐）。
 * @note   在 TIM6 的 20 ms 时隙里由 app_testmode 调用。
 */
uint8_t AppTracking_TestStep20ms(uint8_t pattern);

/**
 * @brief  写入本周期算出的差速输出，供 PWM 循迹测试发布用。
 */
void AppTracking_SetPwmSteer(float steerRpm);

/**
 * @brief  读取当前差速输出。
 */
float AppTracking_GetSteer(void);

#endif /* __APP_TRACKING_H */

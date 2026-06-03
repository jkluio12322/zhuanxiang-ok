/**
 * @file    encoder.h
 * @brief   JGB37-520 霍尔编码器数据结构与接口
 *
 *          编码器参数说明:
 *          - multiple: 倍频系数 (4 倍频 = 4)
 *          - reduction_ratio: 减速比 (JGB37-520 = 30)
 *          - ppr: 编码器线数 (每圈脉冲数 = 11)
 *          - r: 车轮半径 (单位: m)
 *
 *          转速 (RPM) = count / (multiple * ratio * ppr) * (1000/T) * 60
 *          线速度 (m/s) = RPM / 60 * PI * 2 * r
 */

#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H
#include "main.h"

/** 编码器物理参数 */
typedef struct {
    uint8_t  multiple;         /**< 倍频系数 */
    float    reduction_ratio;  /**< 减速比 */
    uint16_t ppr;              /**< 编码器线数 (脉冲/圈) */
    float    r;                /**< 车轮半径 (m) */
} Parameter;

/** 速度 (角速度 & 线速度) */
typedef struct {
    float angular;  /**< 角速度 (RPM) */
    float linear;   /**< 线速度 (m/s) */
} Velocity;

/** 位置 (圈数/距离/角度) */
typedef struct {
    double rotations;  /**< 累计圈数 */
    double distance;   /**< 累计距离 (m) */
    double angle;      /**< 累计角度 (°) */
} Position;

/** 编码器计数值 */
typedef struct {
    int16_t count_now;        /**< 当前采样周期脉冲数 */
    int32_t count_last;       /**< 上次采样脉冲数 */
    int64_t count_increment;  /**< 本周期脉冲增量 */
    int64_t count_total;      /**< 累计脉冲总数 */
} Counter;

/** 编码器对象 (组合参数/速度/位置/计数) */
typedef struct {
    Parameter param;
    Velocity velocity;
    Position position;
    Counter  counter;
} Encoder;

void Encoder_Init_TIM_All(void);
void initEncoder(Encoder *ecd, const Parameter param);
void updateEncoderLoopSimpleVersion(Encoder *ecd, uint16_t loop_period, TIM_TypeDef *TIMx);

#endif

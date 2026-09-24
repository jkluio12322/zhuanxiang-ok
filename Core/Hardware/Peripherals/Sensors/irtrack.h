/**
 * @file    irtrack.h
 * @brief   八路数字循迹传感器接口。
 */
#ifndef __IRTRACK_H
#define __IRTRACK_H

#include "main.h"

/* 循迹通道数量。 */
#define IRTRACK_CHANNELS  8

/**
 * @brief  八路循迹采样、位置和计算结果。
 */
typedef struct {
    uint8_t   raw[IRTRACK_CHANNELS];    /* 物理逻辑值：0=黑线，1=白线。 */
    int8_t    weight_sum;               /* 各通道权重之和。 */
    float     position;                 /* 归一化位置，范围约为 -7...+7。 */
    uint8_t   black_count;              /* 检测到黑线的通道数量。 */
    uint8_t   pattern;                  /* 黑线掩码，bit0 对应 IR1。 */
    int8_t    last_side;                /* 最后偏线方向：-1 左、0 未知、+1 右。 */
} IRTrack_Data_t;

/* 最新一次循迹采样结果。 */
extern IRTrack_Data_t ir_data;

/**
 * @brief  初始化循迹数据。
 */
void IRTrack_Init(void);

/**
 * @brief  采集八路红外输入。
 */
void IRTrack_Read(void);

/**
 * @brief  计算黑线图案、位置、通道数量和权重和。
 */
void IRTrack_Compute(void);

#endif

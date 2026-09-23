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
 * @brief  循迹模块工作模式。
 */
typedef enum {
    IR_MODE_DIGITAL = 0,  /* 数字量采样和查表。 */
    IR_MODE_ANALOG  = 1,  /* 预留模式，当前驱动不使用。 */
} IR_Mode_t;

/**
 * @brief  查表输出的线路状态。
 */
typedef enum {
    TRACK_NORMAL     = 0,  /* 普通线路。 */
    TRACK_HARD_LEFT  = 1,  /* 左侧强偏线。 */
    TRACK_HARD_RIGHT = 2,  /* 右侧强偏线。 */
    TRACK_FINISH     = 3,  /* 八路全黑终点图案。 */
    TRACK_LOST       = 4,  /* 八路全白丢线图案。 */
} IR_State_t;

/**
 * @brief  八路循迹采样、位置和查表结果。
 */
typedef struct {
    uint8_t   raw[IRTRACK_CHANNELS];    /* 物理逻辑值：0=黑线，1=白线。 */
    int8_t    weight_sum;               /* 各通道权重之和。 */
    float     position;                 /* 归一化位置，范围约为 -7...+7。 */
    uint8_t   left_percent;             /* 查表左轮百分比，范围 40~180。 */
    uint8_t   right_percent;            /* 查表右轮百分比，范围 40~180。 */
    uint8_t   black_count;              /* 检测到黑线的通道数量。 */
    IR_State_t state;                   /* 当前线路状态。 */
    uint8_t   pattern;                  /* 黑线掩码，bit0 对应 IR1。 */
    int8_t    last_side;                /* 最后偏线方向：-1 左、0 未知、+1 右。 */
} IRTrack_Data_t;

/* 最新一次循迹采样结果。 */
extern IRTrack_Data_t ir_data;

/**
 * @brief  初始化循迹数据。
 * @param  mode 传感器模式。
 */
void IRTrack_Init(IR_Mode_t mode);

/**
 * @brief  采集八路红外输入。
 */
void IRTrack_Read(void);

/**
 * @brief  计算黑线图案、位置和左右轮百分比。
 */
void IRTrack_Compute(void);

/**
 * @brief  返回当前图案对应的线路状态。
 * @return 查表状态值。
 */
IR_State_t IRTrack_DetectCrossroad(void);

/**
 * @brief  初始化循迹 GPIO。
 */
void IRTrack_GPIO_Init(void);

#endif

/**
 * @file    irtrack.h
 * @brief   8路红外循迹传感器驱动接口
 *
 *          传感器排列 (车头视角):
 *            IR8 IR7 IR6 IR5 IR4 IR3 IR2 IR1
 *            <- 右                        左 ->
 *
 *          权重计算: 检测到黑线=1, 白=0
 *            weight = sum(sensor[i] * w[i])
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

typedef enum {
    TRACK_NORMAL     = 0,
    TRACK_HARD_LEFT  = 1,   /* 左直角弯 */
    TRACK_HARD_RIGHT = 2,   /* 右直角弯 */
    TRACK_CROSS      = 3,   /* 十字路口 */
    TRACK_LOST       = 4,   /* 全部丢线 */
} IR_State_t;

typedef struct {
    uint8_t   raw[IRTRACK_CHANNELS];    /* 数字量: 0=白/1=黑 */
    int8_t    weight_sum;               /* 加权和 */
    uint8_t   black_count;              /* 检测到黑线数量 */
    IR_State_t state;                   /* 路口状态 */
} IRTrack_Data_t;

extern IRTrack_Data_t ir_data;

void IRTrack_Init(IR_Mode_t mode);
void IRTrack_Read(void);
void IRTrack_Compute(void);
IR_State_t IRTrack_DetectCrossroad(void);
void IRTrack_GPIO_Init(void);

#endif

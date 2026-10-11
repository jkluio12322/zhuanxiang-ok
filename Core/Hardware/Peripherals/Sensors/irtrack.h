/* 八路数字循迹传感器接口 */

#ifndef __IRTRACK_H
#define __IRTRACK_H

#include "main.h"

/* 通道数量 */
#define IRTRACK_CHANNELS  8

/* 八路采样与计算结果 */
typedef struct {
    uint8_t   raw[IRTRACK_CHANNELS];    /* 0=黑线，1=白线 */
    int8_t    weight_sum;               /* 权重和 */
    float     position;                 /* 归一化位置，约 -7..+7 */
    uint8_t   black_count;              /* 黑线通道数 */
    uint8_t   pattern;                  /* 黑线掩码，bit0=IR1 */
} IRTrack_Data_t;

extern IRTrack_Data_t ir_data;   /* 最新采样结果 */

void IRTrack_Init(void);     /* 初始化 */
void IRTrack_Read(void);     /* 采集八路输入 */
void IRTrack_Compute(void);  /* 计算图案/位置/通道数/权重和 */

#endif

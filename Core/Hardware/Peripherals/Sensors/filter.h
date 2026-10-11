/* PT1 / PT2 低通滤波器 + 一维卡尔曼滤波器接口 */

#ifndef __FILTER_H
#define __FILTER_H  /* 头文件保护 */
#include "stdint.h"


/* PT1 一阶低通滤波器 */
typedef struct {
    float tau;          /* 时间常数，越大越平缓 */
    float last_output;  /* 上次输出 */
    float dt;           /* 采样周期(秒) */
} PT1Filter_t;

void PT1Filter_Init(PT1Filter_t *filter, float tau, uint16_t Hz);           /* 按时间常数初始化 */
void PT1Filter_InitWithFreq(PT1Filter_t *filter, float fc, uint16_t Hz);    /* 按截止频率初始化 */
float PT1Filter_Apply(PT1Filter_t *filter, float input);                    /* 应用一次滤波 */


/* PT2 二阶低通滤波器 */
typedef struct {
    float cutoffHz;        /* 截止频率 */
    float sampleRateHz;    /* 采样频率 */
    float b0, b1, b2;      /* 系数 */
    float a1, a2;          /* 系数 */
    float x1, x2;          /* 输入历史 */
    float y1, y2;          /* 输出历史 */
    uint8_t initialized;   /* 初始化标志 */
} pt2Filter_t;

void pt2FilterInit(pt2Filter_t *filter, float cutoffHz, float sampleRateHz, float damping);   /* 初始化二阶滤波 */
float pt2FilterApply(pt2Filter_t *filter, float input);                                       /* 应用一次二阶滤波 */


/* 一维卡尔曼滤波器 */
typedef struct {
    float q; /* 过程噪声协方差 */
    float r; /* 测量噪声协方差 */
    float x; /* 状态估计值 */
    float p; /* 估计误差协方差 */
    float k; /* 卡尔曼增益 */
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *kf, float q,float r,float x,float p);   /* 初始化 */
float KalmanFilter_Update(KalmanFilter *kf, float measurement);              /* 一次预测 + 更新 */

#endif

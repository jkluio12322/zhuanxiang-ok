#ifndef __FILTER_H
#define __FILTER_H  /* 头文件保护宏，防止滤波器接口重复声明。 */
#include "stdint.h"


/////////////PT1低通滤波器//////////////////
typedef struct {
    float tau;          // 时间常数（控制滤波强度，tau越大滤波越平缓）
    float last_output;  // 上一次滤波输出
    float dt;           // 数据采样周期（单位：秒，如飞控常用1kHz采样则dt=0.001）
} PT1Filter_t;
/**
 * @brief  初始化一阶低通滤波器。
 * @param  filter 滤波器对象。
 * @param  tau 时间常数，数值越大滤波越平缓。
 * @param  Hz 采样频率，单位 Hz。
 */
void PT1Filter_Init(PT1Filter_t *filter, float tau, uint16_t Hz);

/**
 * @brief  按截止频率初始化一阶低通滤波器。
 * @param  filter 滤波器对象。
 * @param  fc 截止频率，单位 Hz。
 * @param  Hz 采样频率，单位 Hz。
 */
void PT1Filter_InitWithFreq(PT1Filter_t *filter, float fc, uint16_t Hz);

/**
 * @brief  应用一次一阶低通滤波。
 * @param  filter 滤波器对象。
 * @param  input 本次输入值。
 * @return 滤波后的输出值。
 */
float PT1Filter_Apply(PT1Filter_t *filter, float input);
/////////////PT1低通滤波器//////////////////


//////////////////////PT2滤波器(二阶低通滤波器)////////////////////////////
typedef struct {
    float cutoffHz;        // 截止频率
    float sampleRateHz;    // 采样频率
    float b0, b1, b2;      // 滤波器系数
    float a1, a2;          // 滤波器系数
    float x1, x2;          // 输入历史值
    float y1, y2;          // 输出历史值
    uint8_t initialized;   // 初始化标志
} pt2Filter_t;
/**
 * @brief  初始化二阶低通滤波器。
 * @param  filter 滤波器对象。
 * @param  cutoffHz 截止频率。
 * @param  sampleRateHz 采样频率。
 * @param  damping 阻尼系数，常用 0.707。
 */
void pt2FilterInit(pt2Filter_t *filter, float cutoffHz, float sampleRateHz, float damping);

/**
 * @brief  应用一次二阶低通滤波。
 * @param  filter 滤波器对象。
 * @param  input 本次输入值。
 * @return 滤波后的输出值。
 */
float pt2FilterApply(pt2Filter_t *filter, float input);
//////////////////////PT2滤波器(二阶低通滤波器)////////////////////////////


/////////////////////卡尔曼滤波器/////////////////////////
typedef struct {
    float q; // 过程噪声协方差
    float r; // 测量噪声协方差
    float x; // 状态估计值
    float p; // 估计误差协方差
    float k; // 卡尔曼增益
} KalmanFilter;
/**
 * @brief  初始化卡尔曼滤波器。
 * @param  kf 卡尔曼滤波器对象。
 * @param  q 过程噪声协方差。
 * @param  r 测量噪声协方差。
 * @param  x 状态估计初值。
 * @param  p 估计误差协方差初值。
 */
void KalmanFilter_Init(KalmanFilter *kf, float q,float r,float x,float p);

/**
 * @brief  执行一次卡尔曼滤波更新。
 * @param  kf 卡尔曼滤波器对象。
 * @param  measurement 最新测量值。
 * @return 更新后的状态估计值。
 */
float KalmanFilter_Update(KalmanFilter *kf, float measurement);

#endif

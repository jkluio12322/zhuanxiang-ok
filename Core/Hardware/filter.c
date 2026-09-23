/**
 * @file    filter.c
 * @brief   PT1、PT2 低通滤波器和一维卡尔曼滤波器。
 */

#include "filter.h"
#include "math.h"

#ifndef M_PI
/* 圆周率常量，用于按截止频率计算滤波器时间常数。 */
#define M_PI	3.1415926535f
#endif

/**
 * @brief  按时间常数初始化 PT1 一阶低通滤波器。
 * @param  filter PT1 滤波器对象。
 * @param  tau 时间常数，单位秒。
 * @param  Hz 采样频率，单位 Hz，必须大于 0。
 */
void PT1Filter_Init(PT1Filter_t *filter, float tau, uint16_t Hz) {
    filter->tau = tau;
    filter->dt = (float)1.0f / Hz;
    filter->last_output = 0.0f;  // 初始输出设为0
}

/**
 * @brief  按截止频率初始化 PT1 一阶低通滤波器。
 * @param  filter PT1 滤波器对象。
 * @param  fc 截止频率，单位 Hz，必须大于 0。
 * @param  Hz 采样频率，单位 Hz，必须大于 0。
 */
void PT1Filter_InitWithFreq(PT1Filter_t *filter, float fc, uint16_t Hz) {
    float tau = 1.0f / (2 * M_PI * fc);  // 转换公式
    PT1Filter_Init(filter, tau, Hz);
}

/**
 * @brief  应用一次 PT1 一阶低通滤波。
 * @param  filter PT1 滤波器对象。
 * @param  input 本次输入。
 * @return 滤波后的输出。
 */
float PT1Filter_Apply(PT1Filter_t *filter, float input) {
    // PT1核心公式：output = (input * dt + last_output * tau) / (tau + dt)
    filter->last_output = (input * filter->dt + filter->last_output * filter->tau) / (filter->tau + filter->dt);
    return filter->last_output;
}

/**
 * @brief  初始化二阶低通滤波器。
 * @param  filter PT2 滤波器对象。
 * @param  cutoffHz 截止频率，单位 Hz。
 * @param  sampleRateHz 采样频率，单位 Hz。
 * @param  damping 阻尼系数，常用 0.707，内部限制到 0.1~2.0。
 */
void pt2FilterInit(pt2Filter_t *filter, float cutoffHz, float sampleRateHz, float damping) {
    // 限制截止频率为非负值
    filter->cutoffHz = (cutoffHz > 0) ? cutoffHz : 0;
    filter->sampleRateHz = (sampleRateHz > 0) ? sampleRateHz : 0;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->initialized = 0;

    // 计算滤波器系数（仅当参数有效时）
    if (filter->cutoffHz > 0.0f && filter->sampleRateHz > 0.0f) {
        // 限制阻尼系数范围（0.1到2.0之间，避免极端值导致不稳定）
        if (damping < 0.1f) damping = 0.1f;
        if (damping > 2.0f) damping = 2.0f;

        const float fr = filter->sampleRateHz / filter->cutoffHz;  // 采样频率/截止频率
        const float ohm = tanf(M_PI / fr);                        // 预计算值
        const float c = 1.0f + 2.0f * damping * ohm + ohm * ohm;  // 分母，包含阻尼参数

        // 计算滤波器系数
        filter->b0 = ohm * ohm / c;
        filter->b1 = 2.0f * filter->b0;
        filter->b2 = filter->b0;
        filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
        filter->a2 = (1.0f - 2.0f * damping * ohm + ohm * ohm) / c;
    }
}

/**
 * @brief  应用一次 PT2 二阶低通滤波。
 * @param  filter PT2 滤波器对象。
 * @param  input 本次输入。
 * @return 滤波后的输出；首次调用时直接返回输入并用其初始化历史值。
 */
float pt2FilterApply(pt2Filter_t *filter, float input) {
    if (!filter->initialized) {
        // 首次使用，初始化历史值
        filter->y2 = filter->y1 = filter->x1 = filter->x2 = input;
        filter->initialized = 1;
        return input;
    }

    // 二阶滤波器公式: y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2
    const float y0 = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                   - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    // 更新历史值
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = y0;

    return y0;
}

/**
 * @brief  初始化一维卡尔曼滤波器。
 * @param  kf 卡尔曼滤波器对象。
 * @param  q 过程噪声协方差。
 * @param  r 测量噪声协方差。
 * @param  x 状态估计初值。
 * @param  p 估计误差协方差初值。
 */
void KalmanFilter_Init(KalmanFilter *kf, float q,float r,float x,float p) {
	kf->q = q;
	kf->r = r;
	kf->x = x;
	kf->p = p;
}
/**
 * @brief  执行一次一维卡尔曼预测和更新。
 * @param  kf 卡尔曼滤波器对象。
 * @param  measurement 最新测量值。
 * @return 更新后的状态估计值。
 */
float KalmanFilter_Update(KalmanFilter *kf, float measurement) {
    // 预测步骤
    kf->p = kf->p + kf->q;
    // 计算卡尔曼增益
    kf->k = kf->p / (kf->p + kf->r);
    // 更新估计值
    kf->x = kf->x + kf->k * (measurement - kf->x);
    // 更新估计误差协方差
    kf->p = (1 - kf->k) * kf->p;
    return kf->x;
}

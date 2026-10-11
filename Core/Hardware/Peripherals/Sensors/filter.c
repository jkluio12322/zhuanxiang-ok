/* PT1 / PT2 低通滤波器 + 一维卡尔曼滤波器 */

#include "filter.h"
#include "math.h"

#ifndef M_PI
/* 圆周率，用于按截止频率计算时间常数 */
#define M_PI	3.1415926535f
#endif

void PT1Filter_Init(PT1Filter_t *filter, float tau, uint16_t Hz) {   /* 按时间常数 tau 初始化 */
    filter->tau = tau;
    filter->dt = (float)1.0f / Hz;
    filter->last_output = 0.0f;  /* 初始输出 0 */
}

void PT1Filter_InitWithFreq(PT1Filter_t *filter, float fc, uint16_t Hz) {   /* 按截止频率 fc 初始化 */
    float tau = 1.0f / (2 * M_PI * fc);  /* tau=1/(2πfc) */
    PT1Filter_Init(filter, tau, Hz);
}

float PT1Filter_Apply(PT1Filter_t *filter, float input) {   /* 应用一次 PT1 滤波 */
    /* PT1: output=(input*dt+last_output*tau)/(tau+dt) */
    filter->last_output = (input * filter->dt + filter->last_output * filter->tau) / (filter->tau + filter->dt);
    return filter->last_output;
}

void pt2FilterInit(pt2Filter_t *filter, float cutoffHz, float sampleRateHz, float damping) {   /* 初始化 PT2，damping 常用 0.707，限 0.1~2.0 */
    filter->cutoffHz = (cutoffHz > 0) ? cutoffHz : 0;
    filter->sampleRateHz = (sampleRateHz > 0) ? sampleRateHz : 0;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->initialized = 0;

    if (filter->cutoffHz > 0.0f && filter->sampleRateHz > 0.0f) {
        /* 阻尼限 0.1~2.0 防不稳定 */
        if (damping < 0.1f) damping = 0.1f;
        if (damping > 2.0f) damping = 2.0f;

        const float fr = filter->sampleRateHz / filter->cutoffHz;  /* 采样/截止 */
        const float ohm = tanf(M_PI / fr);                        /* 预计算 */
        const float c = 1.0f + 2.0f * damping * ohm + ohm * ohm;  /* 分母 */

        filter->b0 = ohm * ohm / c;
        filter->b1 = 2.0f * filter->b0;
        filter->b2 = filter->b0;
        filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
        filter->a2 = (1.0f - 2.0f * damping * ohm + ohm * ohm) / c;
    }
}

float pt2FilterApply(pt2Filter_t *filter, float input) {   /* 应用一次 PT2 滤波；首次调用用输入初始化历史 */
    if (!filter->initialized) {
        /* 首次：用输入初始化历史 */
        filter->y2 = filter->y1 = filter->x1 = filter->x2 = input;
        filter->initialized = 1;
        return input;
    }

    /* y0=b0*x0+b1*x1+b2*x2-a1*y1-a2*y2 */
    const float y0 = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                   - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = y0;

    return y0;
}

void KalmanFilter_Init(KalmanFilter *kf, float q,float r,float x,float p) {   /* 初始化卡尔曼滤波器 */
	kf->q = q;
	kf->r = r;
	kf->x = x;
	kf->p = p;
}
float KalmanFilter_Update(KalmanFilter *kf, float measurement) {   /* 一次卡尔曼预测 + 更新 */
    kf->p = kf->p + kf->q;                          /* 预测 */
    kf->k = kf->p / (kf->p + kf->r);                /* 卡尔曼增益 */
    kf->x = kf->x + kf->k * (measurement - kf->x);  /* 更新估计 */
    kf->p = (1 - kf->k) * kf->p;                    /* 更新误差协方差 */
    return kf->x;
}

/* JGB37-520 霍尔编码器数据结构与接口。
   multiple=倍频(4)、reduction_ratio=减速比(56)、ppr=线数(11)、r=轮半径(m)。
   RPM=count/(multiple*ratio*ppr)*1000/T*60，线速度=RPM/60*PI*2*r。 */

#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H  /* 头文件保护 */
#include "main.h"

/* 编码器物理参数 */
typedef struct {
    uint8_t  multiple;         /* 倍频系数 */
    float    reduction_ratio;  /* 减速比 */
    uint16_t ppr;              /* 线数(脉冲/圈) */
    float    r;                /* 轮半径(m) */
} Parameter;

/* 速度 (角速度 & 线速度) */
typedef struct {
    float angular;  /* 角速度(RPM) */
    float linear;   /* 线速度(m/s) */
} Velocity;

/* 位置 (圈数/距离/角度) */
typedef struct {
    double rotations;  /* 累计圈数 */
    double distance;   /* 累计距离(m) */
    double angle;      /* 累计角度(°) */
} Position;

/* 编码器计数值 */
typedef struct {
    int16_t count_now;        /* 本周期脉冲数 */
    int32_t count_last;       /* 上次采样脉冲数 */
    int64_t count_increment;  /* 本周期增量 */
    int64_t count_total;      /* 累计脉冲总数 */
} Counter;

/* 编码器对象 (参数/速度/位置/计数) */
typedef struct {
    Parameter param;
    Velocity velocity;
    Position position;
    Counter  counter;
} Encoder;

void Encoder_Init_TIM_All(void);                                            /* 启动左右轮编码器定时器 */
void initEncoder(Encoder *ecd, const Parameter param);                       /* 初始化对象 */
void updateEncoderLoopSimpleVersion(Encoder *ecd, uint16_t loop_period, TIM_TypeDef *TIMx);   /* 更新速度/位置/累计 */

#endif

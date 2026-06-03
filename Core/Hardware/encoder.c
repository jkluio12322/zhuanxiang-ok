/**
 * @file    encoder.c
 * @brief   JGB37-520 霍尔编码器驱动 + 速度/位置解算
 * @note    使用 TIM2/TIM3 编码器模式 (4倍频)
 *          - TIM2: 左轮编码器 (PA0=CH1, PA1=CH2)
 *          - TIM3: 右轮编码器 (PA6=CH1, PA7=CH2)
 *
 *          转速计算公式:
 *            RPM = count_increment / (multiple * ratio * ppr) * 1000 / T * 60
 *
 *          轮子参数 (JGB37-520):
 *            multiple = 4 (4倍频), ppr = 11 (编码器线数)
 *            reduction_ratio = 30 (减速比), r = 0.065m (轮子半径)
 */

#include "encoder.h"

#define PI  3.14159265358979323846f

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/** 编码器定时器溢出计数器 (用于长距离累积, 当前未使用) */
volatile int32_t overflow_count_TIM2 = 0;
volatile int32_t overflow_count_TIM3 = 0;

/**
 * @brief  启动所有编码器定时器, 并使能溢出中断
 */
void Encoder_Init_TIM_All(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

/**
 * @brief  读取编码器当前计数值并清零
 * @param  TIMx 编码器定时器 (TIM2 或 TIM3)
 * @return 16位有符号计数值 (带方向)
 */
int16_t Encoder_Get(TIM_TypeDef *TIMx)
{
    int16_t Temp;
    Temp = TIMx->CNT;
    TIMx->CNT = 0;   /* 读后清零, 适用于周期采样 */
    return Temp;
}

/* TIM2 溢出中断: 计数器翻转次数 +1 (当前项目中未使用溢出累积) */
void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) {
        overflow_count_TIM2++;
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    }
}

/* TIM3 溢出中断: 计数器翻转次数 +1 (当前项目中未使用溢出累积) */
void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
        overflow_count_TIM3++;
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    }
}

/**
 * @brief  初始化编码器数据结构
 * @param  ecd   编码器对象指针
 * @param  param 编码器物理参数 (倍频/减速比/线数/轮半径)
 */
void initEncoder(Encoder *ecd, const Parameter param)
{
    ecd->param = param;
    ecd->velocity.angular = 0;      /* 角速度 (RPM) */
    ecd->velocity.linear = 0;       /* 线速度 (m/s) */
    ecd->position.rotations = 0;    /* 累计圈数 */
    ecd->position.distance = 0;     /* 累计距离 (m) */
    ecd->position.angle = 0;        /* 累计角度 (°) */
    ecd->counter.count_now = 0;     /* 当前采样周期脉冲数 */
    ecd->counter.count_last = 0;
    ecd->counter.count_increment = 0;
    ecd->counter.count_total = 0;   /* 总脉冲累积 */
}

/**
 * @brief  更新编码器: 读取脉冲并计算速度/位置 (简化版, 仅用当前值)
 * @param  ecd         编码器对象指针
 * @param  loop_period 采样周期 (ms)
 * @param  TIMx        编码器定时器 (TIM2=左轮, TIM3=右轮, TIM3 自动反转符号)
 *
 * @note   速度计算:
 *           RPM = count / (multiple * ratio * ppr) * (1000/T) * 60
 *           线速度 (m/s) = RPM / 60 * PI * 2 * r
 *         位置计算:
 *           圈数 = total_count / (multiple * ratio * ppr)
 *           角度 = 圈数 * 360
 *           距离 = 圈数 * PI * 2 * r  (单位: m)
 */
void updateEncoderLoopSimpleVersion(Encoder *ecd, uint16_t loop_period, TIM_TypeDef *TIMx)
{
    /* 读取当前周期脉冲数 (读后自动清零) */
    ecd->counter.count_now = Encoder_Get(TIMx);

    /* 右轮编码器物理方向相反, 取反 */
    if (TIMx == TIM3)
        ecd->counter.count_now = -ecd->counter.count_now;

    /* 本周期增量 = 当前值 (因为已清零, 上次值为0) */
    ecd->counter.count_increment = (int64_t)(int16_t)(ecd->counter.count_now - 0);
    ecd->counter.count_total += ecd->counter.count_increment;

    /* --- 位置解算 --- */
    ecd->position.rotations = (double)ecd->counter.count_total
        / ((double)ecd->param.multiple * ecd->param.reduction_ratio * ecd->param.ppr);
    ecd->position.angle = ecd->position.rotations * 360.0;
    ecd->position.distance = ecd->position.rotations / 60.0 * PI * 2.0 * ecd->param.r;

    /* --- 速度解算 (RPM & m/s) --- */
    ecd->velocity.angular = (float)(ecd->counter.count_increment
        / ((float)ecd->param.multiple * ecd->param.reduction_ratio * ecd->param.ppr)
        * 1000.0 / loop_period * 60);
    ecd->velocity.linear = (float)(ecd->velocity.angular / 60.0 * PI * 2.0 * ecd->param.r);
}

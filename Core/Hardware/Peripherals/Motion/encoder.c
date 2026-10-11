/* JGB37-520 霍尔编码器 + 速度/位置解算。TIM2=左轮、TIM3=右轮（以 app_motion.c 为准），4 倍频。
   RPM = count/(multiple*ratio*ppr)*1000/T*60；multiple=4, ppr=11, ratio=56, r=0.065m。 */

#include "encoder.h"

/* 圆周率，用于把圈数换算为线速度和距离 */
#define PI  3.14159265358979323846f

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

volatile int32_t overflow_count_TIM2 = 0;   /* 溢出计数，当前未用 */
volatile int32_t overflow_count_TIM3 = 0;

void Encoder_Init_TIM_All(void)   /* 启动 TIM2/TIM3 编码器；速度解算用周期差分，不依赖溢出中断 */
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

int16_t Encoder_Get(TIM_TypeDef *TIMx)   /* 有符号脉冲增量（内部按定时器分别保存上次 CNT） */
{
    static uint16_t last_TIM2 = 0;
    static uint16_t last_TIM3 = 0;
    static uint8_t first_TIM2 = 1;
    static uint8_t first_TIM3 = 1;

    uint16_t now = TIMx->CNT;
    int16_t delta;

    if (TIMx == TIM2) {
        delta = first_TIM2 ? 0 : (int16_t)(now - last_TIM2);
        last_TIM2 = now;
        first_TIM2 = 0;
    } else {
        delta = first_TIM3 ? 0 : (int16_t)(now - last_TIM3);
        last_TIM3 = now;
        first_TIM3 = 0;
    }
    return delta;
}
/* TIM2/TIM3 溢出 ISR 已移至 stm32f1xx_it.c (CubeMX 管理) */

void initEncoder(Encoder *ecd, const Parameter param)   /* 初始化编码器对象 */
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

/* 读脉冲并算速度/位置。RPM=count/(multiple*ratio*ppr)*1000/T*60，
   线速度=RPM/60*PI*2*r，圈数=total/(multiple*ratio*ppr)。 */
void updateEncoderLoopSimpleVersion(Encoder *ecd, uint16_t loop_period, TIM_TypeDef *TIMx)
{
    ecd->counter.count_now = Encoder_Get(TIMx);

    /* TIM3 一路正转计负，取反统一为"前进为正"（TIM3 绑右轮，见 app_motion.c） */
    if (TIMx == TIM3)
        ecd->counter.count_now = -ecd->counter.count_now;

    ecd->counter.count_increment = ecd->counter.count_now;
    ecd->counter.count_total += ecd->counter.count_increment;

    ecd->position.rotations = (double)ecd->counter.count_total
        / ((double)ecd->param.multiple * ecd->param.reduction_ratio * ecd->param.ppr);
    ecd->position.angle = ecd->position.rotations * 360.0;
    ecd->position.distance = ecd->position.rotations * PI * 2.0 * ecd->param.r;

    ecd->velocity.angular = (float)(ecd->counter.count_increment
        / ((float)ecd->param.multiple * ecd->param.reduction_ratio * ecd->param.ppr)
        * 1000.0 / loop_period * 60);
    ecd->velocity.linear = (float)(ecd->velocity.angular / 60.0 * PI * 2.0 * ecd->param.r);
}

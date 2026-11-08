/* JGB37-520 霍尔编码器 + 速度/位置解算。TIM2=左轮、TIM3=右轮（以 app_motion.c 为准），4 倍频。
   RPM = count/(multiple*ratio*ppr)*1000/T*60；multiple=4, ppr=11, ratio=56, r=0.065m。 */

#include "encoder.h"

/* 圆周率，用于把圈数换算为线速度和距离 */
#define PI  3.14159265358979323846f

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

volatile int32_t overflow_count_TIM2 = 0;   /* 溢出计数，当前未用 */
volatile int32_t overflow_count_TIM3 = 0;

void encoder_init(void)   /* 启动 TIM2/TIM3 编码器；速度解算用周期差分，不依赖溢出中断 */
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

int16_t encoder_get(TIM_TypeDef *TIMx)   /* 有符号脉冲增量（内部按定时器分别保存上次 CNT） */
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
/* TIM2/TIM3 溢出 ISR 已移至 isr.c (CubeMX 管理) */

void encoder_setup(Encoder *ecd, const Parameter param)   /* 初始化编码器对象 */
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

/* 读脉冲并算速度/位置。RPM=count/(multiple*ratio*ppr)*60000000/dt_us，
   线速度=RPM/60*PI*2*r，圈数=total/(multiple*ratio*ppr)。
   dt_us 由调度器用 DWT 实测给出，不再是标称周期 —— 中断延迟不会进到速度里。 */
void encoder_update(Encoder *ecd, uint32_t dt_us, TIM_TypeDef *TIMx)
{
    float counts_per_rev;
    float rotations;

    ecd->counter.count_now = encoder_get(TIMx);

    /* TIM3 一路正转计负，取反统一为"前进为正"（TIM3 绑右轮，见 app_motion.c） */
    if (TIMx == TIM3)
        ecd->counter.count_now = -ecd->counter.count_now;

    ecd->counter.count_increment = ecd->counter.count_now;
    ecd->counter.count_total += ecd->counter.count_increment;

    /* 全部用 float 算：Cortex-M3 没有 FPU，double 的除法/转换全靠软件模拟，
       放在 2 ms 采样中断里实测要 400 us 以上，会把 5 ms 控制拍挤到后面。
       counts_per_rev 每次现算（3 次乘法），比一次软浮点除法便宜得多。 */
    counts_per_rev = (float)ecd->param.multiple * ecd->param.reduction_ratio
                   * (float)ecd->param.ppr;
    if (counts_per_rev <= 0.0f) {
        counts_per_rev = 1.0f;
    }

    rotations = (float)ecd->counter.count_total / counts_per_rev;   /* int64→float 只有一次 */
    ecd->position.rotations = (double)rotations;
    ecd->position.angle = (double)(rotations * 360.0f);
    ecd->position.distance = (double)(rotations * (PI * 2.0f) * ecd->param.r);

    if (dt_us == 0U) {
        ecd->velocity.angular = 0.0f;
    } else {
        /* 用本拍增量（int16，转换便宜）而不是 int64 的 count_increment */
        ecd->velocity.angular = (float)ecd->counter.count_now * 60000000.0f
            / (counts_per_rev * (float)dt_us);
    }
    ecd->velocity.linear = ecd->velocity.angular * (1.0f / 60.0f) * (PI * 2.0f) * ecd->param.r;
}

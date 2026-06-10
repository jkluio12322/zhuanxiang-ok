/**
 * @file    pid.h
 * @brief   位置式 PID 控制器 (参考平衡车测试程序-V1.0)
 * @note    公式: Out = Kp*e(k) + Ki*∑e + Kd*[Actual(k-1)-Actual(k)] + Kf*Target
 */

#ifndef __BSP_PID_H
#define __BSP_PID_H

/**
 * @brief 位置式 PID 控制器结构体
 * @note  公式: Out = Kp*e(k) + Ki*Σe + Kd*[Actual(k-1)-Actual(k)] + Kf*Target
 *        Out > 0 时 +OutOffset, Out < 0 时 -OutOffset (死区补偿)
 *        最后经 OutputRateLimit 限变化率, 再经 OutMax/OutMin 限幅输出
 */
typedef struct {
    /* ── 输入/输出 ───────────────────────────────────── */
    float Target;       /* 设定目标 (SpeedPID: RPM, YawPID: °)       */
    float Actual;       /* 当前实际值, 调用 PID_Update 前由外部填入   */
    float Actual1;      /* 上一次的实际值, 用于 D 项微分计算          */
    float Out;          /* 最终输出 (SpeedPID: PWM占空比, YawPID: 差速RPM) */

    /* ── 调参增益 ───────────────────────────────────── */
    float Kp;           /* 比例增益: 每单位误差产生的输出             */
    float Ki;           /* 积分增益: 每周期累加到 I 项, 消除稳态静差   */
    float Kd;           /* 微分增益: 抑制实际值突变 (微分先行)        */
    float Kf;           /* 前馈增益: Target*Kf 直接叠加到输出         */

    /* ── 误差 (内部状态) ─────────────────────────────── */
    float Error0;       /* 当前误差: Target - Actual                  */
    float Error1;       /* 上一次误差 (当前代码未直接使用)            */

    /* ── 各分量输出 (调试/观察用) ─────────────────────── */
    float POut;         /* 比例项输出 */
    float IOut;         /* 积分项输出 (累加器) */
    float DOut;         /* 微分项输出 */

    /* ── 输出限制 ────────────────────────────────────── */
    float OutMax;       /* 输出上限 (SpeedPID=100对应PWM满占空比, YawPID=20对应最大差速RPM) */
    float OutMin;       /* 输出下限 (通常为 -OutMax, 支持非对称限幅)  */
    float OutOffset;    /* 死区补偿值: 补偿TB6612驱动芯片低占空比不转的死区 */
    float IntegralLimit;/* 积分限幅: I 项绝对值上限, 防止积分饱和导致超调/迟钝 */
    float Deadband;     /* 积分死区: |误差|≤Deadband 时停积积分, 防止编码器噪声引起稳态震荡 */
    float OutputRateLimit; /* 输出变化率限制: 单次 update 输出最大变化量, 防止急加速/急刹车 */
    float LastOutput;   /* 上一次输出值, 用于计算输出变化量 */
} PID_t;

void PID_Init(PID_t *p);
void PID_SetParam(PID_t *p, float kp, float ki, float kd);
void PID_SetTarget(PID_t *p, float target);
void PID_Update(PID_t *p);

#endif

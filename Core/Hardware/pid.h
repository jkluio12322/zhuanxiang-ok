/**
 * @file    pid.h
 * @brief   位置式和增量式 PID 控制器。
 * @note    位置式公式:
 *          Out = Kp*e(k) + Ki*Σe + Kd*[Actual(k-1)-Actual(k)] + Kf*Target
 *          增量式公式:
 *          ΔOut = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2)
 */

#ifndef __BSP_PID_H
#define __BSP_PID_H  /* 头文件保护宏，防止 PID 接口重复声明。 */

/**
 * @brief 位置式和增量式 PID 共用控制器结构体。
 * @note  位置式更新会按完整公式计算绝对输出；增量式更新只计算本周期增量，
 *        再累加到 Out。Deadband、OutputRateLimit 和 OutMax/OutMin 对两种
 *        更新方式都有效。
 */
typedef struct {
    /* ── 输入/输出 ───────────────────────────────────── */
    float Target;       /* 设定目标 (速度环: RPM, 航向环: 度)          */
    float Actual;       /* 当前实际值, 调用 PID_Update 前由外部填入   */
    float Actual1;      /* 上一次的实际值, 用于 D 项微分计算          */
    float Out;          /* 最终输出 (速度环: PWM 百分比, 航向环: 差速 RPM) */

    /* ── 调参增益 ───────────────────────────────────── */
    float Kp;           /* 比例增益: 每单位误差产生的输出             */
    float Ki;           /* 积分增益: 每周期累加到 I 项, 消除稳态静差   */
    float Kd;           /* 微分增益: 抑制实际值突变 (微分先行)        */
    float Kf;           /* 位置式可选前馈增益；当前闭环默认置 0       */

    /* ── 误差 (内部状态) ─────────────────────────────── */
    float Error0;       /* 当前误差: e0 = Target - Actual             */
    float Error1;       /* 上一周期误差: e1                           */
    float Error2;       /* 上上周期误差: e2，供增量式 D 项使用        */

    /* ── 各分量输出 (调试/观察用) ─────────────────────── */
    float POut;         /* 比例项输出 */
    float IOut;         /* 积分项输出 (累加器) */
    float DOut;         /* 微分项输出 */

    /* ── 输出限制 ────────────────────────────────────── */
    float OutMax;       /* 输出上限，由调用模块按 PWM 百分比或差速 RPM 设置 */
    float OutMin;       /* 输出下限 (通常为 -OutMax, 支持非对称限幅)  */
    float OutOffset;    /* 位置式兼容死区补偿；当前闭环默认置 0       */
    float IntegralLimit;/* 积分限幅: I 项绝对值上限, 防止积分饱和导致超调/迟钝 */
    float Deadband;     /* 积分死区: |误差|≤Deadband 时停积积分, 防止编码器噪声引起稳态震荡 */
    float OutputRateLimit; /* 输出变化率限制: 单次 update 输出最大变化量, 防止急加速/急刹车 */
    float LastIncrement;/* 增量式 PID 本周期实际输出增量                */
    float LastOutput;   /* 上一次输出值, 用于计算输出变化量             */
} PID_t;

/**
 * @brief  初始化 PID 控制器。
 * @param  p PID 对象指针。
 */
void PID_Init(PID_t *p);

/**
 * @brief  设置 PID 的 P、I、D 三个基础增益。
 * @param  p PID 对象指针。
 * @param  kp 比例增益。
 * @param  ki 积分增益。
 * @param  kd 微分增益。
 */
void PID_SetParam(PID_t *p, float kp, float ki, float kd);

/**
 * @brief  设置 PID 目标值。
 * @param  p PID 对象指针。
 * @param  target 新目标值。
 * @note   目标跨越零点时清除积分项，包括切换到目标 0 的情况。
 */
void PID_SetTarget(PID_t *p, float target);

/**
 * @brief  执行一次 PID 更新。
 * @param  p PID 对象指针。
 */
void PID_Update(PID_t *p);

/**
 * @brief  执行一次增量式 PID 更新。
 * @param  p PID 对象指针。
 * @note   本周期误差历史依次传递，计算结果累加到 p->Out。
 */
void PID_IncrementalUpdate(PID_t *p);

#endif

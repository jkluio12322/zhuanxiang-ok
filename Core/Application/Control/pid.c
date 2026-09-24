/**
 * @file    pid.c
 * @brief   位置式和增量式 PID 控制器。
 * @note    增量式速度环使用 PID_IncrementalUpdate，逐周期累加输出增量；
 *          直行航向环使用 PID_Update，按完整位置式公式计算绝对输出。
 */

#include "pid.h"
#include <math.h>

/**
 * @brief  初始化 PID 对象的所有状态和参数。
 * @param  p PID 对象指针。
 */
void PID_Init(PID_t *p)
{
    p->Target    = 0;
    p->Actual    = 0;
    p->Actual1   = 0;
    p->Out       = 0;
    p->Kp        = 0;
    p->Ki        = 0;
    p->Kd        = 0;
    p->Kf        = 0;
    p->Error0    = 0;
    p->Error1    = 0;
    p->Error2    = 0;
    p->POut      = 0;
    p->IOut      = 0;
    p->DOut      = 0;
    p->OutMax    = 0;
    p->OutMin    = 0;
    p->OutOffset = 0;
    p->IntegralLimit   = 0;
    p->Deadband        = 0;
    p->OutputRateLimit = 0;
    p->LastIncrement   = 0;
    p->LastOutput      = 0;
}

/**
 * @brief  设置 PID 比例、积分和微分增益。
 * @param  p PID 对象指针。
 * @param  kp 比例增益。
 * @param  ki 积分增益。
 * @param  kd 微分增益。
 */
void PID_SetParam(PID_t *p, float kp, float ki, float kd)
{
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
}

/**
 * @brief  设置 PID 的指定参数。
 */
void PID_SetField(PID_t *p, PID_Field_t field, float value)
{
    switch (field) {
    case PID_FIELD_KP:
        p->Kp = value;
        break;
    case PID_FIELD_KI:
        p->Ki = value;
        break;
    case PID_FIELD_KD:
        p->Kd = value;
        break;
    case PID_FIELD_KF:
        p->Kf = value;
        break;
    default:
        break;
    }
}

/**
 * @brief  清除 PID 的输出分量和误差历史。
 */
void PID_ClearHistory(PID_t *p)
{
    p->POut = 0.0f;
    p->IOut = 0.0f;
    p->DOut = 0.0f;
    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->Error2 = 0.0f;
    p->LastIncrement = 0.0f;
}

/**
 * @brief  复位 PID 运行状态，保留目标和调参增益。
 */
void PID_ResetRuntime(PID_t *p)
{
    p->Actual = 0.0f;
    p->Actual1 = 0.0f;
    p->Out = 0.0f;
    p->LastOutput = 0.0f;
    PID_ClearHistory(p);
}

/**
 * @brief  按 OutMin/OutMax 限制 PID 当前输出。
 */
void PID_ClampOutput(PID_t *p)
{
    if (p->Out > p->OutMax) {
        p->Out = p->OutMax;
    }
    if (p->Out < p->OutMin) {
        p->Out = p->OutMin;
    }
}

/**
 * @brief  设置 PID 目标值并处理方向反转。
 * @param  p PID 对象指针。
 * @param  target 新目标值。
 * @note   当目标值符号变化时清零积分项，避免反向调节延迟。
 */
void PID_SetTarget(PID_t *p, float target)
{
    /* 目标跨越零点时清零积分，包括切换到目标 0 的停车命令。 */
    float old_sign = (p->Target > 0) ? 1.0f : ((p->Target < 0) ? -1.0f : 0);
    float new_sign = (target > 0) ? 1.0f : ((target < 0) ? -1.0f : 0);
    p->Target = target;
    if (new_sign != old_sign) {
        p->IOut = 0;
    }
}

/**
 * @brief 位置式 PID 单次更新 (每控制周期调用一次)
 * @param p PID 对象指针。
 * @note  处理顺序: 误差传递 → P → I(含死区+限幅) → D(微分先行) →
 *                  合成 → 死区补偿 → 变化率限制 → 最终限幅
 *        位置式更新当前用于 100 ms 航向环；50 ms 速度环使用增量式更新。
 */
void PID_Update(PID_t *p)
{
    /*──────────────────────────────────────────────────────
     * 第1步: 误差传递
     *   Error0 = 本次误差, Error1 = 上一次误差
     *──────────────────────────────────────────────────────*/
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    /* 比例项直接响应当前误差。 */
    p->POut = p->Kp * p->Error0;

    /*──────────────────────────────────────────────────────
     * 第3步: 积分项 I (累积历史误差, 消除稳态静差)
     *
     *   [积分死区] Deadband
     *     当 |误差| ≤ Deadband 时停止积分累加
     *     设 Deadband=0 则死区功能关闭
     *
     *   [停止条件] Ki==0 时禁用积分；目标符号变化或切换到 0 时清积分。
     *     Target=0 意味着要停车, 避免残留积分延迟制动。
     *──────────────────────────────────────────────────────*/
    if (p->Ki != 0) {
        if (p->Deadband == 0 || fabsf(p->Error0) > p->Deadband) {
            p->IOut += p->Ki * p->Error0;   /* 误差在死区外: 正常累积 */
        }
        /* 误差在死区内: 不累积 (保持当前I值, 避免震荡) */
    } else {
        p->IOut = 0;   /* Ki=0 即不使用积分项 */
    }

    /*──────────────────────────────────────────────────────
     *   积分限幅 IntegralLimit — 防止 "积分饱和 (windup)"
     *
     *   场景: 电机堵转或目标突变时, 误差长时间存在,
     *   I 项会无限累积到极大值, 导致:
     *     - 障碍消除后输出瞬间冲到顶 → 严重超调
     *     - 反向时需要先 "消耗" 积攒的积分 → 响应迟钝
     *
     *   当前航向环的积分限幅由应用状态初始化代码设置。
     *──────────────────────────────────────────────────────*/
    if (p->IOut >  p->IntegralLimit) { p->IOut =  p->IntegralLimit; }
    if (p->IOut < -p->IntegralLimit) { p->IOut = -p->IntegralLimit; }

    /*──────────────────────────────────────────────────────
     * 第4步: 微分项 D (微分先行 — 对测量值微分, 而非误差)
     *
     *   D = -Kd * (Actual - Actual1)  即负的测量值变化量
     *
     *   为什么用 "微分先行" 而不对误差微分?
     *     - 对误差微分: 目标突变 → 误差突变 → D项冲击 (阶跃响应振荡)
     *     - 对测量值微分: 目标突变不影响D项, 只在实际转速变化时才起作用
     *     - 物理意义: 转速变化越快, D项越强地反向抑制, 提供阻尼
     *
     *──────────────────────────────────────────────────────*/
    p->DOut = -p->Kd * (p->Actual - p->Actual1);

    /*──────────────────────────────────────────────────────
     * 第5步: 前馈项 + 四项合成
     *
     *   Kf 为位置式可选前馈增益，用于提供目标值对应的稳态输出基线；
     *   当前航向环默认 Kf=0，增量式速度环不读取 Kf。
     *
     *   为什么需要前馈?
     *     纯反馈PID需要先产生误差才能有输出, 响应慢
     *     前馈直接把目标映射为预期输出 → 响应快, PI只需补偏差
     *
     *──────────────────────────────────────────────────────*/
    p->Out = p->POut + p->IOut + p->DOut + p->Kf * p->Target;

    /*──────────────────────────────────────────────────────
     * 第6步: 死区补偿 OutOffset
     *
     *   这是位置式 PID 的兼容补偿分支。当前航向环 OutOffset=0；
     *   增量式速度环不读取 OutOffset，其低速死区补偿由应用层单独完成。
     *──────────────────────────────────────────────────────*/
    if (p->Out > 0)       { p->Out += p->OutOffset; }
    else if (p->Out < 0)  { p->Out -= p->OutOffset; }

    /*──────────────────────────────────────────────────────
     * 第7步: 输出变化率限制 OutputRateLimit
     
     *
     *   限制单次 update 的输出最大变化量, 防止急加速/急刹车
     *
     *   该限制可减小电流冲击和机械冲击；设为 0 则跳过此限制。
     *──────────────────────────────────────────────────────*/
    if (p->OutputRateLimit > 0) {
        float delta = p->Out - p->LastOutput;
        if (delta >  p->OutputRateLimit) { p->Out = p->LastOutput + p->OutputRateLimit; }
        if (delta < -p->OutputRateLimit) { p->Out = p->LastOutput - p->OutputRateLimit; }
    }
    p->LastOutput = p->Out;

    /*──────────────────────────────────────────────────────
     * 第8步: 最终输出限幅 (最后一道防线)
     *
     *   OutMax/OutMin 由调用模块按输出物理意义分别设置。
     *──────────────────────────────────────────────────────*/
    PID_ClampOutput(p);

    /* 保存本次实际值, 供下周期 D 项计算 */
    p->Actual1 = p->Actual;
}

/**
 * @brief  执行一次增量式 PID 更新。
 * @param  p PID 对象指针。
 * @note   计算公式:
 *         ΔOut = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2)
 *         误差进入死区时本周期增量为零；OutputRateLimit 用于限制单周期
 *         增量；最终输出继续受 OutMax/OutMin 限制。增量式不使用 Kf、
 *         IntegralLimit 和 OutOffset。
 */
void PID_IncrementalUpdate(PID_t *p)
{
    float deltaOut;

    /* 依次保存 e2、e1，再计算本周期误差 e0。 */
    p->Error2 = p->Error1;
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    /* 保存各分量，便于串口 show 和 VOFA 调参时观察。 */
    p->POut = p->Kp * (p->Error0 - p->Error1);
    p->IOut = p->Ki * p->Error0;
    p->DOut = p->Kd * (p->Error0 - 2.0f * p->Error1 + p->Error2);
    deltaOut = p->POut + p->IOut + p->DOut;

    /* 误差在死区内时保持当前 PWM，不继续产生增量。 */
    if (p->Deadband > 0.0f
        && fabsf(p->Error0) <= p->Deadband) {
        deltaOut = 0.0f;
    }

    /* OutputRateLimit 在增量式中限制单周期最大 PWM 变化量。 */
    if (p->OutputRateLimit > 0.0f) {
        if (deltaOut > p->OutputRateLimit) {
            deltaOut = p->OutputRateLimit;
        }
        if (deltaOut < -p->OutputRateLimit) {
            deltaOut = -p->OutputRateLimit;
        }
    }

    p->LastIncrement = deltaOut;
    p->Out += deltaOut;

    /* 输出限幅后保存状态，防止状态量超出实际 PWM 边界。 */
    PID_ClampOutput(p);

    p->LastOutput = p->Out;
    p->Actual1 = p->Actual;
}

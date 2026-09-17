/**
 * @file    pid.c
 * @brief   位置式和增量式 PID 控制器。
 *
 *          位置式公式:
 *            Out = Kp*e(k) + Ki*Σe
 *                + Kd*[Actual(k-1)-Actual(k)] + Kf*Target
 *
 *          增量式公式:
 *            ΔOut = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2)
 *            Out(k) = Out(k-1) + ΔOut
 *
 *          处理流程 (每控制周期):
 *            误差传递 → P 项 → I 项(含死区+限幅) → D 项(微分先行)
 *            → 四项合成 → 死区补偿(OutOffset) → 变化率限制(OutputRateLimit)
 *            → 最终限幅(OutMax/Min)
 *
 *          限制参数共 5 个:
 *            IntegralLimit   — 积分限幅, 防止积分饱和 (windup)
 *            Deadband        — 积分死区, 防止编码器噪声引起的稳态震荡
 *            OutOffset       — 死区补偿, 克服 TB6612 电机驱动低占空比不转
 *            OutputRateLimit — 输出变化率限制, 防急加速/急刹车伤齿轮
 *            OutMax/OutMin   — 最终输出限幅, PWM 物理边界或差速上限
 *
 *          调参经验:
 *            Kp 不能大 — 电机响应快 (50ms RPM 可跳 200), Kp>0.5 立刻震荡
 *            Ki 要远小于 Kp — 积分累积快, Ki=0.05 几个周期就显著摆动
 *            Kd 微分先行 — 对测量值微分会而非误差, 目标突变不引起 D 项冲击
 *            Kf 先大致确定 — 稳态 PWM / 目标 RPM ≈ Kf, 当前0.65偏保守
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
 * @brief  设置 PID 目标值并处理方向反转。
 * @param  p PID 对象指针。
 * @param  target 新目标值。
 * @note   当目标值符号变化时清零积分项，避免反向调节延迟。
 */
void PID_SetTarget(PID_t *p, float target)
{
    /* 方向反转检测: 目标符号变化时清零积分, 防止反向响应迟钝 */
    float old_sign = (p->Target > 0) ? 1.0f : ((p->Target < 0) ? -1.0f : 0);
    float new_sign = (target > 0) ? 1.0f : ((target < 0) ? -1.0f : 0);
    p->Target = target;
    if (new_sign != 0 && new_sign != old_sign) {
        p->IOut = 0;
    }
}

/**
 * @brief 位置式 PID 单次更新 (每控制周期调用一次)
 * @note  处理顺序: 误差传递 → P → I(含死区+限幅) → D(微分先行) →
 *                  合成 → 死区补偿 → 变化率限制 → 最终限幅
 *        SpeedPID 每 50ms 调用, YawPID 每 100ms 调用
 */
void PID_Update(PID_t *p)
{
    /*──────────────────────────────────────────────────────
     * 第1步: 误差传递
     *   Error0 = 本次误差, Error1 = 上一次误差
     *──────────────────────────────────────────────────────*/
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    /*──────────────────────────────────────────────────────
     * 第2步: 比例项 P (当前误差的比例响应)
     *   P项直接响应误差, 误差大则输出大, 但单独P项永远有稳态静差
     *   Kp不能太大 — 电机响应快 (50ms内RPM可跳200),
     *   实测 Kp>0.5 就开始震荡, 故 SpeedPID 仅用 0.08
     *──────────────────────────────────────────────────────*/
    p->POut = p->Kp * p->Error0;

    /*──────────────────────────────────────────────────────
     * 第3步: 积分项 I (累积历史误差, 消除稳态静差)
     *
     *   [积分死区] Deadband
     *     当 |误差| ≤ Deadband 时, 停止积分累加
     *     原因: 编码器 RPM 有 ±3 的测量抖动, 若在目标附近
     *     仍持续积分会导致输出来回摆动, 无法收敛到稳态
     *     设 Deadband=0 则死区功能关闭
     *
     *   [停止条件] Ki==0 或 Target==0 时清零积分
     *     Target=0 意味着要停车, 立即清零积分避免刹车延迟
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
     *   SpeedPID IntegralLimit=30:
     *     Kf=0.65 已提供 ~65% 稳态输出基底,
     *     I 项只需补齐剩余 0~30%, 超过 30 说明前馈偏差过大或异常工况
     *
     *   YawPID IntegralLimit=40:
     *     航向角可能长时间存在小角度偏差 (陀螺仪漂移),
     *     需要更大的积分余量来持续纠偏
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
     *   例: SpeedPID.Kd=0.15, 若 50ms 内 RPM 跳变 100
     *       DOut = -0.15 * 100 = -15, 反向抵消 15 单位输出 → 抑制震荡
     *──────────────────────────────────────────────────────*/
    p->DOut = -p->Kd * (p->Actual - p->Actual1);

    /*──────────────────────────────────────────────────────
     * 第5步: 前馈项 + 四项合成
     *
     *   Kf (前馈增益) 的作用:
     *     Out = Kf * Target 作为稳态输出的 "基线"
     *     P/I/D 只需在此基础上做小幅修正
     *
     *   为什么需要前馈?
     *     纯反馈PID需要先产生误差才能有输出, 响应慢
     *     前馈直接把目标映射为预期输出 → 响应快, PI只需补偏差
     *
     *   Kf 的估算: 稳态所需PWM / 目标RPM
     *     例: 维持50RPM需要50%占空比 → Kf≈1.0
     *     当前 Kf=0.65 偏保守, 留余量给I项补充
     *──────────────────────────────────────────────────────*/
    p->Out = p->POut + p->IOut + p->DOut + p->Kf * p->Target;

    /*──────────────────────────────────────────────────────
     * 第6步: 死区补偿 OutOffset
     *
     *   TB6612 电机驱动芯片特性:
     *     PWM 占空比低于 10~15% 时, H桥输出电压不足以克服
     *     电机静摩擦力 (stiction), 电机实际不转
     *
     *   补偿方式: Out>0 → +Offset, Out<0 → -Offset
     *     确保PID算出的小输出也能让电机真正动起来
     *
     *   SpeedPID.OutOffset=15: JGB37-520带30:1减速箱+车体负载, 实测死区约15%
     *   YawPID.OutOffset=0:   差速RPM不需要死区补偿 (直接叠加到PWM)
     *──────────────────────────────────────────────────────*/
    if (p->Out > 0)       { p->Out += p->OutOffset; }
    else if (p->Out < 0)  { p->Out -= p->OutOffset; }

    /*──────────────────────────────────────────────────────
     * 第7步: 输出变化率限制 OutputRateLimit
     
     *
     *   限制单次 update 的输出最大变化量, 防止急加速/急刹车
     *
     *   VOFA 实测: 无限制时相邻周期 (50ms) 输出跳变 35~80 个单位
     *     → 电机电流冲击大、齿轮磨损、车体抖动
     *
     *   SpeedPID.OutputRateLimit=20:
     *     每 50ms 最多变 ±20%, 从 0 到 100 至少需要 5 个周期 = 250ms
     *     给电机和齿轮一个缓冲时间
     *
     *   YawPID.OutputRateLimit=10:
     *     外环 100ms 一次, 差速变化需更平缓, 防止突然甩头
     *
     *   设为 0 则跳过此限制
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
     *   SpeedPID: ±100 → PWM 占空比物理上限
     *   YawPID:   ±20  → 外环只做微调, 差速 RPM 不超过 20
     *──────────────────────────────────────────────────────*/
    if (p->Out > p->OutMax) { p->Out = p->OutMax; }
    if (p->Out < p->OutMin) { p->Out = p->OutMin; }

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
    if (p->Out > p->OutMax) {
        p->Out = p->OutMax;
    }
    if (p->Out < p->OutMin) {
        p->Out = p->OutMin;
    }

    p->LastOutput = p->Out;
    p->Actual1 = p->Actual;
}

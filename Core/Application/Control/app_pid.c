/**
 * @file    app_pid.c
 * @brief   统一的 PID 实现 — 增量式（速度环）与位置式（循迹环）合一。
 *
 * @note    取代原来的 pid.c：那里 PID_t 和 TrackPid_t 两套并行的
 *          init / set / reset / update，现在塌缩成一套。
 *
 * @note    两种模式的差别集中在 pidCompute()：
 * @code
 *   增量式 PAST  delta = Kp*(e0-e1) + Ki*e0 + Kd项；out += delta；累加器限幅
 *   位置式 PAST  integral += Ki*e0*dt；out = Kp*e0 + integral + Kd项
 * @endcode
 *
 * @note    调用方约定（两个使用者必须遵守，否则会互相污染）：
 *          - 速度环用 Pid_Update(target, actual)，增量式，微分作用于测量值；
 *          - 循迹环用 Pid_UpdatePosition(error)，位置式，微分作用于误差。
 *          同一个 Pid_t 不能两种模式混用。
 *
 * @note    执行上下文：速度环在 TIM6 中断里调用，循迹环也在中断里调用；
 *          命令行调参在主循环里调 Pid_SetGain()。增益是 float 写入，
 *          Cortex-M3 上是单条 STR，不会读到半个值，但可能会丢一次更新，
 *          对调参这种人工操作可以接受。
 */

#include "app_pid.h"
#include <string.h>

/**
 * @brief  把浮点值限制到闭区间内。
 * @param  value    待限制的值。
 * @param  minValue 下界。
 * @param  maxValue 上界。
 * @return 限制后的值。
 */
static float pidClamp(float value, float minValue, float maxValue)
{
    if (value > maxValue) return maxValue;
    if (value < minValue) return minValue;
    return value;
}

/**
 * @brief  目标符号变化时清掉误差历史与输出。
 * @param  p         PID 对象。
 * @param  newTarget 本次的目标值。
 * @note   只在 cfg.reset_on_sign_change 打开时生效（速度环打开、循迹环关闭）。
 *         目的是避免转弯前残留的正向输出被带到反转周期，造成内侧轮先冲一下。
 * @note   「反号」包含 0，所以 0 -> 正 与 0 -> 负 也会触发复位。
 */
static void pidCheckTargetSign(Pid_t *p, float newTarget)
{
    int oldSign;
    int newSign;

    if (p->cfg.reset_on_sign_change == 0U) {
        return;
    }

    oldSign = (p->target > 0.0f) ? 1 : ((p->target < 0.0f) ? -1 : 0);
    newSign = (newTarget > 0.0f) ? 1 : ((newTarget < 0.0f) ? -1 : 0);

    if (oldSign != newSign) {
        p->e0 = 0.0f;
        p->e1 = 0.0f;
        p->e2 = 0.0f;
        p->integral = 0.0f;
        p->out = 0.0f;
    }
}

/**
 * @brief  用配置初始化 PID，运行时状态清零。
 * @param  p   PID 对象。
 * @param  cfg 配置，按值拷贝进对象，调用方此后可以丢弃它。
 * @note   配置一旦写入就不再变化（除非显式调 Pid_SetGain/Pid_SetLimits），
 *         所以传进来的 cfg 可以是 const 的静态结构。
 */
void Pid_Init(Pid_t *p, const PidConfig_t *cfg)
{
    memset(p, 0, sizeof(*p));
    p->cfg = *cfg;
}

/**
 * @brief  清空运行时状态，保留增益与配置。
 * @param  p PID 对象。
 * @note   停机、切换模式、从丢线恢复时调用。
 *         不停车就复位会让输出突然归零，所以只在状态切换的边界上调。
 */
void Pid_Reset(Pid_t *p)
{
    p->target = 0.0f;
    p->actual = 0.0f;
    p->actual_prev = 0.0f;
    p->e0 = 0.0f;
    p->e1 = 0.0f;
    p->e2 = 0.0f;
    p->integral = 0.0f;
    p->out = 0.0f;
}

/**
 * @brief  在线设置一个增益。
 * @param  p     PID 对象。
 * @param  field 要设置的增益，见 PidField_t。
 * @param  value 新值。
 * @note   越界的 field 被静默忽略。
 */
void Pid_SetGain(Pid_t *p, PidField_t field, float value)
{
    switch (field) {
    case PID_FIELD_KP: p->cfg.kp = value; break;
    case PID_FIELD_KI: p->cfg.ki = value; break;
    case PID_FIELD_KD: p->cfg.kd = value; break;
    default: break;
    }
}

/**
 * @brief  读取一个增益。
 * @param  p     PID 对象。
 * @param  field 要读的增益。
 * @return 当前增益值；field 越界时返回 0.0f。
 */
float Pid_GetGain(const Pid_t *p, PidField_t field)
{
    switch (field) {
    case PID_FIELD_KP: return p->cfg.kp;
    case PID_FIELD_KI: return p->cfg.ki;
    case PID_FIELD_KD: return p->cfg.kd;
    default:           return 0.0f;
    }
}

/**
 * @brief  修改输出限幅。
 * @param  p       PID 对象。
 * @param  out_min 输出下限。
 * @param  out_max 输出上限。
 * @note   主要用途有两个：
 *         - 循迹模式把下限收到 0，禁止轮子反转（见 app_speedloop.c）；
 *         - PWM 循迹测试临时改限幅复用循迹环（见 app_tracking.c）。
 *         增量式下改限幅会同时影响累加器的钳位，这正是抗饱和所依赖的。
 */
void Pid_SetLimits(Pid_t *p, float out_min, float out_max)
{
    p->cfg.out_min = out_min;
    p->cfg.out_max = out_max;
}

/**
 * @brief  按模式计算一次原始输出。
 * @param  p PID 对象，要求 e0/e1/e2/actual/actual_prev 已由调用方更新。
 * @return 原始输出（尚未做零目标归零与静摩擦补偿）。
 *
 * @note   位置式：积分项累加后立刻按输出上下限钳位，微分作用于误差。
 * @note   增量式：先算本次增量再累加；**累加器本身必须钳位**——
 *         否则输出饱和期间累加器会继续增长，等误差反向时要爬很久才回来，
 *         这就是积分饱和。钳位之后，out_min = 0 才能让「禁止反转」真正生效。
 * @note   增量式的微分项有两种来源：默认对测量值微分（-Kd*(actual-actual_prev)），
 *         可避免目标突变时的微分冲击；cfg.derivative_on_measurement = 0 时
 *         退回对误差微分。
 */
static float pidCompute(Pid_t *p)
{
    float derivative;
    float output;

    if (p->cfg.mode == PID_MODE_POSITIONAL) {
        /* 位置式：积分项直接累加，微分作用于误差。 */
        p->integral += p->cfg.ki * p->e0 * p->cfg.dt;
        p->integral = pidClamp(p->integral,
                               p->cfg.out_min,
                               p->cfg.out_max);

        derivative = p->cfg.kd * (p->e0 - p->e1);

        output = p->cfg.kp * p->e0 + p->integral + derivative;
        return output;
    }

    /* 增量式：输出是增量的累加，累加器本身限幅即是抗饱和。 */
    if (p->cfg.derivative_on_measurement != 0U) {
        derivative = -p->cfg.kd * (p->actual - p->actual_prev);
    } else {
        derivative = p->cfg.kd * (p->e0 - 2.0f * p->e1 + p->e2);
    }

    p->out += p->cfg.kp * (p->e0 - p->e1)
              + p->cfg.ki * p->e0
              + derivative;

    /* 累加器限幅 —— 少了这一步输出饱和后累加器会继续增长，形成积分饱和。 */
    p->out = pidClamp(p->out, p->cfg.out_min, p->cfg.out_max);

    return p->out;
}

/**
 * @brief  零目标归零与静摩擦补偿，只作用在返回给电机的值上。
 * @param  p   PID 对象。
 * @param  raw pidCompute() 的原始输出。
 * @return 最终输出。
 * @note   两个附加项都**不写回累加器**——写回去在增量式下会逐周期累积成漂移。
 * @note   zero_target_hold：目标接近 0 时强制输出 0，避免停车后残留输出顶着电机。
 * @note   out_offset：输出非零时同向叠加一个常量，把输出顶出电机与减速箱的
 *         静摩擦死区。代价是低速停车略难，所以 offset 要略小于实际死区。
 */
static float pidFinalize(const Pid_t *p, float raw)
{
    float output = pidClamp(raw, p->cfg.out_min, p->cfg.out_max);

    if (p->cfg.zero_target_hold > 0.0f) {
        if ((p->target <= p->cfg.zero_target_hold)
            && (p->target >= -p->cfg.zero_target_hold)) {
            return 0.0f;
        }
    }

    if (p->cfg.out_offset > 0.0f) {
        if (output > 0.0f) {
            output += p->cfg.out_offset;
        } else if (output < 0.0f) {
            output -= p->cfg.out_offset;
        }
    }

    return pidClamp(output, p->cfg.out_min, p->cfg.out_max);
}

/**
 * @brief  按 目标/实测 更新一次 PID（速度环用）。
 * @param  p      PID 对象。
 * @param  target 目标值，单位与被控量一致（速度环为 RPM）。
 * @param  actual 实测值，单位同上。
 * @return 本次输出。
 * @note   顺序：先按目标符号判断要不要复位 -> 更新目标与误差历史 ->
 *         按模式算输出 -> 最后做归零与补偿。
 *         actual_prev 在算完之后才更新，保证微分看到的是「上一周期」的测量值。
 */
float Pid_Update(Pid_t *p, float target, float actual)
{
    pidCheckTargetSign(p, target);

    p->target = target;
    p->actual = actual;

    p->e2 = p->e1;
    p->e1 = p->e0;
    p->e0 = target - actual;

    p->out = pidCompute(p);
    p->actual_prev = actual;

    return pidFinalize(p, p->out);
}

/**
 * @brief  位置式的便捷入口，直接喂误差（循迹环用）。
 * @param  p     PID 对象。
 * @param  error 误差值，单位与被控量一致（循迹环为归一化位置，约 ±7）。
 * @return 本次输出。
 * @note   内部把 target 与 e0 都设成 error、actual 固定为 0，
 *         这样两种模式共用 pidCompute() 的误差历史。
 * @note   不调用 pidCheckTargetSign()：循迹误差正负交替是正常现象，
 *         在这里复位会把跨过中心的连续性打断。循迹环的 reset_on_sign_change
 *         也必须配成 0。
 */
float Pid_UpdatePosition(Pid_t *p, float error)
{
    float output;

    p->target = error;
    p->actual = 0.0f;

    p->e2 = p->e1;
    p->e1 = p->e0;
    p->e0 = error;

    output = pidCompute(p);
    p->out = output;

    return pidFinalize(p, output);
}

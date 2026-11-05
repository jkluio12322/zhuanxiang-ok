/* 统一 PID：增量式（速度环）与位置式（循迹环）合一，取代原 pid.c 里两套并行实现。
   两个更新函数都在 TIM6 中断里被调用；增益只在 config.h 改，无在线整定竞争。 */

#include "pid.h"
#include <string.h>

static float pidClamp(float value, float minValue, float maxValue);
static void  pidCheckTargetSign(Pid_t *p, float newTarget);
static float pidCompute(Pid_t *p, float incremental_integral_scale);
static float pidFinalize(const Pid_t *p, float raw);

/* 按 目标/实测 更新一次 PID（速度环用）。中断上下文。 */
float pid_update(Pid_t *p, float target, float actual)
{
    return pid_update_scaled(p, target, actual, 1.0f);
}

float pid_update_scaled(Pid_t *p, float target, float actual,
                        float incremental_integral_scale)
{
    pidCheckTargetSign(p, target);

    p->target = target;
    p->actual = actual;

    p->e2 = p->e1;
    p->e1 = p->e0;
    p->e0 = target - actual;

    p->out = pidCompute(p, incremental_integral_scale);
    p->actual_prev = actual;

    return pidFinalize(p, p->out);
}

/* 位置式便捷入口，直接喂误差（循迹环用）。不查目标反号——误差正负交替是正常的，
   复位会打断跨中心的连续性；循迹环 reset_on_sign_change 必须配 0。 */
float pid_update_position(Pid_t *p, float error)
{
    float output;

    p->target = error;
    p->actual = 0.0f;

    p->e2 = p->e1;
    p->e1 = p->e0;
    p->e0 = error;

    output = pidCompute(p, 1.0f);
    p->out = output;

    return pidFinalize(p, output);
}

/* 把值夹到闭区间。 */
static float pidClamp(float value, float minValue, float maxValue)
{
    if (value > maxValue) return maxValue;
    if (value < minValue) return minValue;
    return value;
}

/* 目标反号时清误差历史与输出（速度环开、循迹环关）。「反号」含 0，0->正 / 0->负 也触发。 */
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

/* 按模式算原始输出。增量式累加器必须钳位——否则输出饱和期间累加器继续涨，
   误差反向时要爬很久才回来（积分饱和）；微分默认作用于测量值，避免目标突变冲击。 */
static float pidCompute(Pid_t *p, float incremental_integral_scale)
{
    float derivative;
    float output;

    if (p->cfg.mode == PID_MODE_POSITIONAL) {
        p->integral += p->cfg.ki * p->e0 * p->cfg.dt;
        p->integral = pidClamp(p->integral,
                               p->cfg.out_min,
                               p->cfg.out_max);

        derivative = p->cfg.kd * (p->e0 - p->e1);

        output = p->cfg.kp * p->e0 + p->integral + derivative;
        return output;
    }

    if (p->cfg.derivative_on_measurement != 0U) {
        derivative = -p->cfg.kd * (p->actual - p->actual_prev);
    } else {
        derivative = p->cfg.kd * (p->e0 - 2.0f * p->e1 + p->e2);
    }

    p->out += p->cfg.kp * (p->e0 - p->e1)
              + p->cfg.ki * p->e0 * incremental_integral_scale
              + derivative;

    /* 累加器限幅——少了这步输出饱和后累加器会继续增长，形成积分饱和。 */
    p->out = pidClamp(p->out, p->cfg.out_min, p->cfg.out_max);

    return p->out;
}

/* 零目标归零 + 静摩擦补偿，只作用在返回值上，不写回累加器（写回会逐周期累积成漂移）。 */
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

/* 用配置初始化 PID，运行时状态清零。cfg 按值拷贝，调用方此后可丢弃。 */
void pid_init(Pid_t *p, const PidConfig_t *cfg)
{
    memset(p, 0, sizeof(*p));
    p->cfg = *cfg;
}

/* 清空运行时状态，保留增益。停机/切模式/丢线恢复时调。 */
void pid_reset(Pid_t *p)
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

/* 读一个增益；field 越界返回 0。只读，无在线改增益接口。 */
float pid_get_gain(const Pid_t *p, PidField_t field)
{
    switch (field) {
    case PID_FIELD_KP: return p->cfg.kp;
    case PID_FIELD_KI: return p->cfg.ki;
    case PID_FIELD_KD: return p->cfg.kd;
    default:           return 0.0f;
    }
}

uint8_t pid_set_gain(Pid_t *p, PidField_t field, float value)
{
    if ((p == 0) || !((value >= 0.0f) && (value <= 1000.0f))) {
        return 0U;
    }

    switch (field) {
    case PID_FIELD_KP:
        p->cfg.kp = value;
        return 1U;
    case PID_FIELD_KI:
        p->cfg.ki = value;
        return 1U;
    case PID_FIELD_KD:
        p->cfg.kd = value;
        return 1U;
    default:
        return 0U;
    }
}

/* 改输出限幅。增量式下同时影响累加器钳位（抗饱和所依赖）。 */
void pid_set_limits(Pid_t *p, float out_min, float out_max)
{
    p->cfg.out_min = out_min;
    p->cfg.out_max = out_max;
}

/**
 * @file    app_pid.c
 * @brief   统一 PID 的实现。
 */

#include "app_pid.h"
#include <string.h>

static float pidClamp(float value, float minValue, float maxValue)
{
    if (value > maxValue) return maxValue;
    if (value < minValue) return minValue;
    return value;
}

/* 目标反号时清掉误差历史和输出，避免转弯前留下的正 PWM 带到反转周期。 */
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

void Pid_Init(Pid_t *p, const PidConfig_t *cfg)
{
    memset(p, 0, sizeof(*p));
    p->cfg = *cfg;
}

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

void Pid_SetGain(Pid_t *p, PidField_t field, float value)
{
    switch (field) {
    case PID_FIELD_KP: p->cfg.kp = value; break;
    case PID_FIELD_KI: p->cfg.ki = value; break;
    case PID_FIELD_KD: p->cfg.kd = value; break;
    default: break;
    }
}

float Pid_GetGain(const Pid_t *p, PidField_t field)
{
    switch (field) {
    case PID_FIELD_KP: return p->cfg.kp;
    case PID_FIELD_KI: return p->cfg.ki;
    case PID_FIELD_KD: return p->cfg.kd;
    default:           return 0.0f;
    }
}

void Pid_SetLimits(Pid_t *p, float out_min, float out_max)
{
    p->cfg.out_min = out_min;
    p->cfg.out_max = out_max;
}

/* 计算一次原始输出，分别走增量式和位置式。 */
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

    return p->out;
}

/* 静摩擦补偿和零目标归零，只作用在返回给电机的值上，不污染累加器。 */
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

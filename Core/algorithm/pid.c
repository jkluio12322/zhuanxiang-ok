#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void pid_init(Pid *p, float kp, float ki, float out_min, float out_max)
{
    if (p == 0) {
        return;
    }
    p->kp = kp;
    p->ki = ki;
    p->out_min = out_min;
    p->out_max = out_max;
    pid_reset(p);
}

void pid_set_gains(Pid *p, float kp, float ki)
{
    if (p == 0) {
        return;
    }
    p->kp = kp;
    p->ki = ki;
}

/* 复位后输出从 0 起算：增量式的「积分」就是 out 本身，
   所以停车时清 out 就等于清积分，下次起步不会带着上次的积分冲出去。 */
void pid_reset(Pid *p)
{
    if (p == 0) {
        return;
    }
    p->prev_error = 0.0f;
    p->target = 0.0f;
    p->actual = 0.0f;
    p->out = 0.0f;
}

float pid_update(Pid *p, float target, float actual, float dt_s)
{
    float error;
    float du;

    if (p == 0) {
        return 0.0f;
    }
    if (dt_s <= 0.0f) {
        dt_s = 0.002f;      /* 首拍或计时异常：退回采样拍标称周期 */
    }

    error = target - actual;
    p->target = target;
    p->actual = actual;

    du = (p->kp * (error - p->prev_error)) + (p->ki * error * dt_s);
    p->prev_error = error;

    p->out = clampf(p->out + du, p->out_min, p->out_max);
    return p->out;
}

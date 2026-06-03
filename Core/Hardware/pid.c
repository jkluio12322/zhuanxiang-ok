/**
 * @file    pid.c
 * @brief   增量式 PID 控制器
 * @note    公式: output = Kp*e(k) + Ki*∑e + Kd*[e(k)-e(k-1)]
 *          包含输出限幅 (MAX_OUTPUT) 和积分限幅 (MAX_ERROR_INTEGRAL)
 */

#include "pid.h"

/**
 * @brief  限幅函数: 将输入限制在 ±MAX_OUTPUT_ABS 范围内
 * @param  input           输入值
 * @param  MAX_OUTPUT_ABS  绝对值上限
 * @return 限幅后的值
 */
static float limit(float input, const float MAX_OUTPUT_ABS)
{
    input = input > +MAX_OUTPUT_ABS ? +MAX_OUTPUT_ABS : input;
    input = input < -MAX_OUTPUT_ABS ? -MAX_OUTPUT_ABS : input;
    return input;
}

/**
 * @brief  初始化 PID 对象
 * @param  pid         PID 对象指针
 * @param  MAX_OUTPUT  输出限幅绝对值
 * @param  MAX_E_I     积分限幅绝对值
 */
void initPID(PID *pid, const float MAX_OUTPUT, const float MAX_E_I)
{
    pid->kp = 0;
    pid->ki = 0;
    pid->kd = 0;
    pid->error.now = 0;
    pid->error.last = 0;
    pid->error.integral = 0;
    pid->target = 0;
    pid->input = 0;
    pid->output = 0;
    pid->MAX_OUTPUT = MAX_OUTPUT;
    pid->MAX_ERROR_INTEGRAL = MAX_E_I;
}

/**
 * @brief  PID 计算迭代 (位置式)
 * @param  pid    PID 对象指针
 * @param  input  当前反馈值 (测量值)
 *
 * @note  每次调用更新一次 PID 输出:
 *          1. 计算当前误差 e(k) = target - input
 *          2. 累加积分项 (带积分限幅防饱和)
 *          3. output = Kp*e(k) + Ki*∑e + Kd*(e(k)-e(k-1))
 *          4. 输出限幅
 *          5. 保存本次误差供下次微分用
 */
void updatePID(PID *pid, float input)
{
    pid->input = input;
    pid->error.now = pid->target - pid->input;           /* 当前误差 */

    pid->error.integral += pid->error.now;                /* 积分累加 */
    pid->error.integral = limit(pid->error.integral, pid->MAX_ERROR_INTEGRAL); /* 积分限幅 */

    /* 位置式 PID 计算 */
    pid->output = pid->kp * pid->error.now
                + pid->ki * pid->error.integral
                + pid->kd * (pid->error.now - pid->error.last); /* 微分用差分代替 */

    pid->output = limit(pid->output, pid->MAX_OUTPUT);    /* 输出限幅 */
    pid->error.last = pid->error.now;                     /* 保存误差供下次使用 */
}

/**
 * @brief  在线修改 PID 参数
 * @param  pid PID 对象指针
 * @param  kp  比例系数
 * @param  ki  积分系数
 * @param  kd  微分系数
 */
void setPIDParam(PID *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief  设置 PID 目标值
 * @param  pid    PID 对象指针
 * @param  target 目标值
 */
void setPIDTarget(PID *pid, float target)
{
    pid->target = target;
}

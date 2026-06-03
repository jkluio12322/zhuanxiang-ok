/**
 * @file    pid.h
 * @brief   增量式 PID 控制器数据结构与接口
 */

#ifndef __BSP_PID_H
#define __BSP_PID_H

/** 误差结构体 */
typedef struct {
    float now;       /**< 当前误差 e(k) */
    float last;      /**< 上次误差 e(k-1) */
    float integral;  /**< 积分累加值 ∑e */
} Error;

/** PID 控制器对象 */
typedef struct {
    float kp, ki, kd;             /**< PID 系数 */
    float target, input, output;  /**< 目标值 / 当前值 / 输出值 */
    Error error;                  /**< 误差数据 */
    float MAX_OUTPUT;             /**< 输出限幅绝对值 */
    float MAX_ERROR_INTEGRAL;     /**< 积分限幅绝对值 (防积分饱和) */
} PID;

void initPID(PID *pid, float MAX_OUTPUT, float MAX_E_I);
void updatePID(PID *pid, float input);
void setPIDParam(PID *pid, float kp, float ki, float kd);
void setPIDTarget(PID *pid, float target);

#endif

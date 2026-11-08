/* 循迹决策：红外偏差 + 陀螺阻尼 + 车速解耦 → 左右轮目标转速。
 * 全白（丢线）按策略处理，默认当拍停车；可选按航向转固定角度找线。
 * 转角闭环也在这里：mission 的 SPIN 任务复用它，避免两份陀螺整定。 */
#ifndef __TRACK_H
#define __TRACK_H

#include <stdint.h>

/* 全白丢线策略 */
typedef enum {
    TRACK_LOST_STOP = 0,   /* 当拍（或确认拍后）停车，默认 */
    TRACK_LOST_HOLD = 1,   /* 保持上次转向量 TRACK_LOST_HOLD_MS 再停 */
    TRACK_LOST_ARC  = 2,   /* 按入弯前记录的航向转 CORNER_ANGLE_DEG 找线 */
    TRACK_LOST_TURN = 3    /* 按当前航向转 CORNER_ANGLE_DEG 找线 */
} TrackLostPolicy;

void track_init(void);

/* 每 5 ms 控制拍调用一次。dt_us 是控制拍实测间隔（DWT 测得），
   微分项按它归一，不再假设「这一拍一定是 5 ms」。 */
void track_task(uint32_t dt_us);

/* 重新起步/切模式时清微分状态与入弯上下文。 */
void track_reset(void);

float track_steer(void);
uint8_t track_line_ok(void);
float track_kv(void);

/* 运行期整定（串口命令用），立刻生效，下次控制拍就用新值。 */
void track_set_gains(float kp, float kd, float gyro_kp);
void track_get_gains(float *kp, float *kd, float *gyro_kp);
void track_set_diff_limit(float limit, float outer);
void track_set_lost_policy(TrackLostPolicy policy);
TrackLostPolicy track_lost_policy(void);

/* 陀螺闭环转角：只用 yaw，不看红外。
   track_turn_begin 设定目标航向后，每个控制拍调 track_turn_step，
   返回 1 = 已到位（连续 TRACK_TURN_SETTLE 拍在容差内，输出已归零）。 */
/* 转向量公式本体：u = clamp((KP*err + KD*deriv + GYRO_SIGN*GYRO_KP*wz) * kv, ±LIMIT)。
   实车循迹与串口采集的注入测试共用这一份，避免两条公式各自漂移。 */
float track_steer_eval(float err, float deriv, float wz, float kv);

/* 转角环参数：运行期可调（串口 V kp ki kd rpm tol db），扫参用，不改宏 */
void track_turn_set_gains(float kp, float ki, float kd, float rpm, float tol, float db);
void track_turn_get_gains(float *kp, float *ki, float *kd, float *rpm, float *tol, float *db);

void track_turn_begin(float target_yaw);
uint8_t track_turn_step(uint32_t dt_us);
float track_turn_err_deg(void);
float track_wrap180(float deg);

#endif /* __TRACK_H */

/* 全局运行状态。除本模块外任何代码都不许定义可写全局变量。 */
#ifndef __STATE_H
#define __STATE_H

#include <stdint.h>
#include "types.h"

typedef struct {
    /* 循迹输出，track_task 写，速度环读 */
    float target_left_rpm;
    float target_right_rpm;
    float steer;              /* -1..1，正=右转（整形前） */
    uint8_t line_ok;          /* 1=压线，0=丢线 */
    float gain_kv;            /* 车速解耦系数，1.0 = 标称车速 */
    float gyro_z_dps;         /* 车体 +Z 角速度，右转为负 */
    float yaw_deg;            /* 航向角（-180..180） */
    uint8_t line_lost_policy; /* 0=STOP 1=HOLD 2=ARC 3=TURN */
    uint16_t line_lost_ticks; /* 本次丢线已持续的控制拍数 */
    float track_err;          /* 转向环输入：normalized_position，-1..+1（线在右为正） */
    float track_deriv;        /* 转向环微分项：低通后的 d(e)/dt，单位 1/s */
    float track_u_raw;        /* 限幅前的转向量（对比 steer 看是否被 TRACK_STEER_LIMIT 切平） */

    /* 速度环输出，control_speed_task 写，显示与遥测读 */
    float pwm_left;
    float pwm_right;
    float actual_left_rpm;
    float actual_right_rpm;
    float cmd_left_rpm;       /* 斜坡后的实际命令（不是 track 的原始目标） */
    float cmd_right_rpm;
    float distance_m;         /* 左右轮平均里程，绝对值累加 */

    /* 计圈 */
    uint8_t lap_count;
    uint8_t lap_target;

    /* 运行状态 */
    AppMode mode;
    StopReason stop_reason;

    /* 战术包状态，mission 写，显示读 */
    uint8_t mission_id;       /* 0=IDLE 1=TRACK 2=SPIN */
    uint8_t mission_status;   /* 0=IDLE 1=RUNNING 2=DONE 3=TIMEOUT 4=ABORTED */
    float spin_err_deg;       /* 转角当前误差 */
    uint32_t spin_ms;         /* 转角已用时 */
    uint8_t spin_count;       /* 累计完成的转角段数 */
    uint32_t spin_stall_ms;   /* 本步「没有更靠近目标」已持续多久，到 MISSION_SPIN_STALL_MS 就认输 */

    /* 战术包状态，tactic 写，显示/串口读 */
    uint8_t tactic_id;        /* 0=循迹 1=步进转角 */
    uint8_t tactic_running;
    float tactic_step_deg;    /* 战术 2 当前步长 */
    uint8_t tactic_steps;     /* 战术 2 已完成步数 */

    /* 调度统计：全部由 sched 在中断里写入，主循环只读 */
    uint32_t sample_isr_latency_us;    /* TIM4 溢出 → 采样拍入口的延迟（µs，0..1999） */
    uint32_t sample_isr_latency_max_us;
    uint32_t control_isr_latency_us;   /* TIM7 溢出 → 控制拍入口的延迟（µs，0..4999） */
    uint32_t control_isr_latency_max_us;
    uint32_t sample_exec_us;           /* 最近一次 2 ms 采样拍（含速度环）的耗时（µs） */
    uint32_t sample_exec_max_us;
    uint32_t speed_exec_us;            /* 最近一次速度环耗时（µs），单独打点 */
    uint32_t speed_exec_max_us;
    uint32_t control_exec_us;          /* 最近一次 5 ms 控制拍的耗时（µs） */
    uint32_t control_exec_max_us;
    uint32_t control_overrun;          /* 控制拍超过 SCHED_OVERRUN_US 的次数 */
    uint32_t control_dt_us;            /* 最近一次控制拍的真实间隔（DWT 实测，µs） */
    uint32_t control_dt_dev_max_us;    /* 真实间隔相对标称 5 ms 的最大偏差（µs） */
    uint32_t win_retry;                /* 测速快照 seqlock 重读失败次数 */
    uint32_t loop_period_us;           /* 主循环相邻两圈的间隔（µs） */
    uint32_t loop_period_max_us;
    uint32_t loop_over_1ms;            /* 主循环一圈超过 1 ms 的次数 */
    uint32_t imu_latency_us;           /* IMU 标志置位 → imu_task 真正执行的延迟（µs） */
    uint32_t imu_latency_max_us;
    uint32_t tick_count;               /* 控制拍累计计数 */
} AppState;

AppState *state(void);
void state_init(void);
void state_start_run(void);
void state_stop(StopReason reason);
void state_set_policy(uint8_t policy);

#endif /* __STATE_H */

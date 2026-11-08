#include "state.h"
#include "config.h"

static AppState g_state;

AppState *state(void)
{
    return &g_state;
}

void state_init(void)
{
    g_state.target_left_rpm = 0.0f;
    g_state.target_right_rpm = 0.0f;
    g_state.steer = 0.0f;
    g_state.line_ok = 1U;
    g_state.gain_kv = 1.0f;
    g_state.gyro_z_dps = 0.0f;
    g_state.yaw_deg = 0.0f;
    g_state.line_lost_policy = (uint8_t)TRACK_LOSS_POLICY_DEFAULT;
    g_state.line_lost_ticks = 0U;

    g_state.pwm_left = 0.0f;
    g_state.pwm_right = 0.0f;
    g_state.actual_left_rpm = 0.0f;
    g_state.actual_right_rpm = 0.0f;
    g_state.cmd_left_rpm = 0.0f;
    g_state.cmd_right_rpm = 0.0f;
    g_state.distance_m = 0.0f;

    g_state.lap_count = 0U;
    g_state.lap_target = LAP_DEFAULT_TARGET;

    g_state.mode = APP_MODE_IDLE;
    g_state.stop_reason = STOP_NONE;

    g_state.mission_id = 0U;
    g_state.mission_status = 0U;
    g_state.spin_err_deg = 0.0f;
    g_state.spin_ms = 0U;
    g_state.spin_count = 0U;

    g_state.sample_isr_latency_us = 0U;
    g_state.sample_isr_latency_max_us = 0U;
    g_state.control_isr_latency_us = 0U;
    g_state.control_isr_latency_max_us = 0U;
    g_state.sample_exec_us = 0U;
    g_state.sample_exec_max_us = 0U;
    g_state.speed_exec_us = 0U;
    g_state.speed_exec_max_us = 0U;
    g_state.control_exec_us = 0U;
    g_state.control_exec_max_us = 0U;
    g_state.control_overrun = 0U;
    g_state.control_dt_us = (uint32_t)TASK_CONTROL_MS * 1000UL;
    g_state.control_dt_dev_max_us = 0U;
    g_state.win_retry = 0U;
    g_state.loop_period_us = 0U;
    g_state.loop_period_max_us = 0U;
    g_state.loop_over_1ms = 0U;
    g_state.imu_latency_us = 0U;
    g_state.imu_latency_max_us = 0U;
    g_state.tick_count = 0U;
}

void state_start_run(void)
{
    g_state.lap_count = 0U;
    g_state.stop_reason = STOP_NONE;
    g_state.mode = APP_MODE_TRACK;
    g_state.line_lost_ticks = 0U;
}

void state_stop(StopReason reason)
{
    g_state.mode = APP_MODE_IDLE;
    g_state.stop_reason = reason;
    g_state.target_left_rpm = 0.0f;
    g_state.target_right_rpm = 0.0f;
    g_state.steer = 0.0f;
}

/* 全白策略运行期切换（串口 p 命令）。写单字节，中断里读到的必然是完整值。 */
void state_set_policy(uint8_t policy)
{
    g_state.line_lost_policy = policy;
}

#include "console.h"
#include "config.h"
#include "serial.h"
#include "state.h"
#include "control.h"
#include "laps.h"
#include "imu.h"
#include "motion.h"
#include "track.h"
#include "mission.h"
#include "tactic.h"
#include "pidtrace.h"
#include "turntest.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CONSOLE_LINE_MAX  40U

static char s_line[CONSOLE_LINE_MAX];
static uint8_t s_len = 0U;
static uint8_t s_drop = 0U;

static void console_help(void)
{
    serial_write("s          start tracking (mission)\n");
    serial_write("x          stop / abort\n");
    serial_write("m L R      manual wheel rpm\n");
    serial_write("t N        lap target\n");
    serial_write("y          reset yaw\n");
    serial_write("w N        speed window ms (5-200)\n");
    serial_write("f N        speed filter Hz (0-100, 0=off)\n");
    serial_write("k KP KI    speed PI gains (incremental)\n");
    serial_write("G KP KD KG track gains (pos / diff / gyro)\n");
    serial_write("V KP KI KD RPM TOL DB  turn-loop gains (live)\n");
    serial_write("L LIM OUT  diff shaping (inner / outer)\n");
    serial_write("p N        line-lost policy 0=stop 1=hold 2=arc 3=turn\n");
    serial_write("r DEG      spin clockwise DEG (gyro only)\n");
    serial_write("B DEG LAPS turn self-test: LAPS laps of DEG steps + ir check\n");
    serial_write("T          next tactic (1=track 2=step-spin)\n");
    serial_write("A DEG      tactic2 step angle (45..360)\n");
    serial_write("N          tactic1 +1 lap, tactic2 +45 deg\n");
    serial_write("o          mission / distance status\n");
    serial_write("d          dump timing stats\n");
    serial_write("q          toggle 1Hz diag push\n");
    serial_write("P          toggle steering trace frames (binary, 50Hz)\n");
    serial_write("Z          steering trace off (clean text output)\n");
    serial_write("C          toggle trace injection sweep (bench only)\n");
    serial_write("?          this help\n");
}

/* 时间轴体检，全部是实测值（单位 us，除特别标注）：
     smp_lat = 采样拍中断响应延迟（TIM4 溢出到进函数），对应 2 ms 拍的抖动
     ctl_lat = 控制拍中断响应延迟（TIM7 溢出到进函数），对应 5 ms 拍的抖动
     spd     = 速度环耗时（当期 / 最大），它跑在采样拍里，直接吃掉 2 ms 预算
     dt/dev  = 控制拍实测间隔 / 与标称 5 ms 的最大偏差
     ctl/smp = 控制拍耗时 / 采样拍（含速度环）总耗时
     win     = 当前测速窗长度 ms，retry = 快照读到撕裂后回退的次数
     loop    = 主循环单圈耗时（当期 / 最大），o1ms = 超过 1 ms 的圈数
     imu     = IMU 标志置位 → imu_task 执行的排队延迟（当期 / 最大） */
static void console_dump_stats(void)
{
    const AppState *st = state();
    char buf[320];
    float kp = 0.0f;
    float ki = 0.0f;

    control_get_gains(&kp, &ki);

    (void)snprintf(buf, sizeof(buf),
                   "smp_lat=%lu/%lu ctl_lat=%lu/%lu\r\n"
                   "dt=%lu dev=%lu ctl=%lu/%lu ovr=%lu\r\n"
                   "smp=%lu/%lu spd=%lu/%lu\r\n"
                   "win=%ums retry=%lu tick=%lu loop=%lu/%lu\r\n"
                   "o1ms=%lu rxov=%lu txov=%lu imu=%lu/%lu\r\n"
                   "spd kp=%.3f ki=%.3f out=%.0f/%.0f lim=%.0f/%.0f\r\n",
                   (unsigned long)st->sample_isr_latency_us,
                   (unsigned long)st->sample_isr_latency_max_us,
                   (unsigned long)st->control_isr_latency_us,
                   (unsigned long)st->control_isr_latency_max_us,
                   (unsigned long)st->control_dt_us,
                   (unsigned long)st->control_dt_dev_max_us,
                   (unsigned long)st->control_exec_us,
                   (unsigned long)st->control_exec_max_us,
                   (unsigned long)st->control_overrun,
                   (unsigned long)st->sample_exec_us,
                   (unsigned long)st->sample_exec_max_us,
                   (unsigned long)st->speed_exec_us,
                   (unsigned long)st->speed_exec_max_us,
                   (unsigned)motion_get_measure_window_ms(),
                   (unsigned long)st->win_retry,
                   (unsigned long)st->tick_count,
                   (unsigned long)st->loop_period_us,
                   (unsigned long)st->loop_period_max_us,
                   (unsigned long)st->loop_over_1ms,
                   (unsigned long)serial_rx_overflow(),
                   (unsigned long)serial_tx_overflow(),
                   (unsigned long)st->imu_latency_us,
                   (unsigned long)st->imu_latency_max_us,
                   (double)kp,
                   (double)ki,
                   (double)st->pwm_left,
                   (double)st->pwm_right,
                   (double)SPEED_PID_OUT_MAX,
                   (double)-SPEED_PID_OUT_MAX);
    serial_write(buf);
}

static void console_dump_mission(void)
{
    const AppState *st = state();
    float kp;
    float kd;
    float kg;
    char buf[176];

    track_get_gains(&kp, &kd, &kg);
    (void)snprintf(buf, sizeof(buf),
                   "mode=%u stop=%u policy=%u line=%u lost=%u\r\n"
                   "yaw=%.2f gy=%.2f kv=%.2f dist=%.2fm\r\n"
                   "track kp=%.2f kd=%.2f kg=%.4f\r\n"
                   "mission=%u %s spin_err=%.2f spin_ms=%lu spins=%u stall=%lu\r\n"
                   "tactic=%u %s run=%u step=%.0f steps=%u\r\n",
                   (unsigned)st->mode, (unsigned)st->stop_reason,
                   (unsigned)st->line_lost_policy, (unsigned)st->line_ok,
                   (unsigned)st->line_lost_ticks,
                   (double)st->yaw_deg, (double)st->gyro_z_dps,
                   (double)st->gain_kv, (double)motion_distance_m(),
                   (double)kp, (double)kd, (double)kg,
                   (unsigned)st->mission_id,
                   mission_status_text((MissionStatus)st->mission_status),
                   (double)st->spin_err_deg,
                   (unsigned long)st->spin_ms,
                   (unsigned)st->spin_count,
                   (unsigned long)st->spin_stall_ms,
                   (unsigned)st->tactic_id,
                   tactic_name((TacticId)st->tactic_id),
                   (unsigned)st->tactic_running,
                   (double)st->tactic_step_deg,
                   (unsigned)st->tactic_steps);
    serial_write(buf);
}

/* ---- 自动诊断：每秒往串口收发区推一段中文状态，不用打命令 ---- */
static uint8_t s_diag_on = 1U;

static const char *mode_cn(AppMode m)
{
    switch (m) {
    case APP_MODE_TRACK:  return "循迹";
    case APP_MODE_MANUAL: return "手动";
    default:              return "待机";
    }
}

static const char *mission_cn(MissionStatus s)
{
    switch (s) {
    case MISSION_ST_RUNNING: return "运行";
    case MISSION_ST_DONE:    return "完成";
    case MISSION_ST_TIMEOUT: return "超时";
    case MISSION_ST_ABORTED: return "中止";
    default:                 return "空闲";
    }
}

static void console_push_diag(void)
{
    const AppState *st = state();
    static char buf[320];

    (void)snprintf(buf, sizeof(buf),
                   "[状态] 战术=%s 模式=%s 任务=%s 停车=%u 丢线策略=%u 有线=%u 圈=%u/%u 步=%u×%.0f° 里程=%d.%02dm\r\n"
                   "[时间] 采样延迟=%lu/%lu 控制延迟=%lu/%lu 拍间隔=%lu 偏差=%lu "
                   "控制耗时=%lu 超预算=%lu 测速窗=%ums 重试=%lu 主循环=%lu/%lu 溢出=%lu/%lu\r\n",
                   tactic_name((TacticId)st->tactic_id),
                   mode_cn(st->mode), mission_cn((MissionStatus)st->mission_status),
                   (unsigned)st->stop_reason, (unsigned)st->line_lost_policy,
                   (unsigned)st->line_ok,
                   (unsigned)laps_count(), (unsigned)st->lap_target,
                   (unsigned)st->tactic_steps, (double)st->tactic_step_deg,
                   (int)motion_distance_m(),
                   (int)((motion_distance_m() - (float)(int)motion_distance_m()) * 100.0f),
                   (unsigned long)st->sample_isr_latency_us,
                   (unsigned long)st->sample_isr_latency_max_us,
                   (unsigned long)st->control_isr_latency_us,
                   (unsigned long)st->control_isr_latency_max_us,
                   (unsigned long)st->control_dt_us,
                   (unsigned long)st->control_dt_dev_max_us,
                   (unsigned long)st->control_exec_us,
                   (unsigned long)st->control_overrun,
                   (unsigned)motion_get_measure_window_ms(),
                   (unsigned long)st->win_retry,
                   (unsigned long)st->loop_period_us,
                   (unsigned long)st->loop_period_max_us,
                   (unsigned long)serial_rx_overflow(),
                   (unsigned long)serial_tx_overflow());
    serial_write(buf);
}

static void console_exec(char *line)
{
    AppState *st = state();
    char help_line[40];

    if (strcmp(line, "s") == 0) {
        mission_start_track();
        st->lap_target = laps_target();
        serial_write("OK track mission\n");
    } else if (strcmp(line, "x") == 0) {
        turntest_abort();
        mission_abort();
        serial_write("OK stop\n");
    } else if (strcmp(line, "y") == 0) {
        imu_yaw_reset();
        serial_write("OK yaw reset\n");
    } else if (strcmp(line, "?") == 0) {
        console_help();
    } else if (strcmp(line, "q") == 0) {
        s_diag_on = (uint8_t)(s_diag_on == 0U);
        serial_write((s_diag_on != 0U) ? "OK 自动诊断 开\n" : "OK 自动诊断 关\n");
    } else if (strcmp(line, "P") == 0) {
        pidtrace_set_on((uint8_t)(pidtrace_on() == 0U));
        serial_write((pidtrace_on() != 0U) ? "OK trace on\n" : "OK trace off\n");
    } else if (strcmp(line, "Z") == 0) {
        pidtrace_set_on(0U);
        serial_write("OK trace off\n");
    } else if (strcmp(line, "C") == 0) {
        pidtrace_set_inject((uint8_t)(pidtrace_inject() == 0U));
        serial_write((pidtrace_inject() != 0U) ? "OK inject on\n" : "OK inject off\n");
    } else if (strcmp(line, "d") == 0) {
        console_dump_stats();
    } else if (strcmp(line, "o") == 0) {
        console_dump_mission();
    } else if (strncmp(line, "m ", 2U) == 0) {
        char *p = line + 2;
        float left = strtof(p, &p);
        float right = strtof(p, &p);
        control_set_manual(left, right);
        serial_write("OK manual\n");
    } else if (strncmp(line, "k ", 2U) == 0) {
        char *p = line + 2;
        float kp = strtof(p, &p);
        float ki = strtof(p, &p);
        control_set_gains(kp, ki);
        serial_write("OK speed gains\n");
    } else if (strncmp(line, "V ", 2U) == 0) {
        char *p = line + 2;
        float kp = strtof(p, &p);
        float ki = strtof(p, &p);
        float kd = strtof(p, &p);
        float rpm = strtof(p, &p);
        float tol = strtof(p, &p);
        float db = strtof(p, &p);

        track_turn_set_gains(kp, ki, kd, rpm, tol, db);
        (void)snprintf(help_line, sizeof(help_line), "OK turn kp=%.2f ki=%.2f kd=%.2f\r\n", (double)kp, (double)ki, (double)kd);
        serial_write(help_line);
    } else if (strncmp(line, "G ", 2U) == 0) {
        char *p = line + 2;
        float kp = strtof(p, &p);
        float kd = strtof(p, &p);
        float kg = strtof(p, &p);
        track_set_gains(kp, kd, kg);
        serial_write("OK track gains\n");
    } else if (strncmp(line, "L ", 2U) == 0) {
        char *p = line + 2;
        float limit = strtof(p, &p);
        float outer = strtof(p, &p);
        track_set_diff_limit(limit, outer);
        serial_write("OK diff shaping\n");
    } else if (strncmp(line, "p ", 2U) == 0) {
        uint8_t policy = (uint8_t)atoi(line + 2);
        track_set_lost_policy((TrackLostPolicy)policy);
        st->line_lost_policy = (uint8_t)track_lost_policy();
        serial_write("OK policy\n");
    } else if (strncmp(line, "r ", 2U) == 0) {
        float deg = strtof(line + 2, (char **)0);
        mission_start_spin(deg);
        if (mission_status() == MISSION_ST_ABORTED) {
            serial_write("ERR gyro not ready\n");
        } else {
            serial_write("OK spin\n");
        }
    } else if (strncmp(line, "B ", 2U) == 0) {
        char *p = line + 2;
        float deg = strtof(p, &p);
        uint16_t laps = (uint16_t)atoi(p);
        turntest_start(deg, laps);
        if (turntest_active() != 0U) {
            serial_write("OK selftest\n");
        } else {
            serial_write("ERR gyro not ready\n");
        }
    } else if (strncmp(line, "w ", 2U) == 0) {
        uint16_t window = (uint16_t)atoi(line + 2);
        if (motion_set_measure_window_ms(window) != 0U) {
            serial_write("OK window\n");
        } else {
            serial_write("ERR range 5-200\n");
        }
    } else if (strncmp(line, "f ", 2U) == 0) {
        uint16_t hz = (uint16_t)atoi(line + 2);
        if (motion_set_filter_hz(hz) != 0U) {
            serial_write("OK filter\n");
        } else {
            serial_write("ERR range 0-100\n");
        }
    } else if (strcmp(line, "T") == 0) {
        tactic_next();
        (void)snprintf(help_line, sizeof(help_line), "OK tactic %s\r\n",
                       tactic_name(tactic_id()));
        serial_write(help_line);
    } else if (strcmp(line, "N") == 0) {
        tactic_bump_primary();
        (void)snprintf(help_line, sizeof(help_line),
                       "OK target=%u step=%.0f\r\n",
                       (unsigned)st->lap_target, (double)tactic_step_deg());
        serial_write(help_line);
    } else if (strncmp(line, "A ", 2U) == 0) {
        tactic_set_step_deg(strtof(line + 2, (char **)0));
        (void)snprintf(help_line, sizeof(help_line), "OK step=%.0f\r\n",
                       (double)tactic_step_deg());
        serial_write(help_line);
    } else if (strncmp(line, "t ", 2U) == 0) {
        uint8_t target = (uint8_t)atoi(line + 2);
        laps_set_target(target);
        st->lap_target = laps_target();
        serial_write("OK target\n");
    } else if (line[0] != '\0') {
        serial_write("ERR unknown, '?' for help\n");
    }
}

void console_init(void)
{
    s_len = 0U;
    s_drop = 0U;
    console_help();
}

void console_task(void)
{
    uint8_t c;
    static uint32_t next_ms = 0U;

    /* 每秒自动往串口收发区推一段中文状态；q 命令可关掉（用 VOFA 抓波形时省带宽） */
    if ((DIAG_PUSH_MS != 0U) && (s_diag_on != 0U)) {
        uint32_t now = HAL_GetTick();
        if ((int32_t)(now - next_ms) >= 0) {
            next_ms = now + DIAG_PUSH_MS;
            console_push_diag();
        }
    }

    while (serial_getc(&c) != 0U) {
        if ((c == '\r') || (c == '\n')) {
            if ((s_drop == 0U) && (s_len > 0U)) {
                s_line[s_len] = '\0';
                console_exec(s_line);
            }
            s_len = 0U;
            s_drop = 0U;
        } else if (s_len < (CONSOLE_LINE_MAX - 1U)) {
            s_line[s_len] = (char)c;
            s_len++;
        } else {
            s_drop = 1U;   /* 超长行整行丢弃，避免解析到半截命令 */
        }
    }
}

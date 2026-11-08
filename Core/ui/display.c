/* OLED 主界面：1 行大字战术条 + 6 行小字正文。
 *
 * 为什么第 1 行用 8×16 大字：跑车时人离车 2~3 m，6×8 小字读不出来；
 * 「现在打哪个战术、步长多少、在不在跑」是唯一必须远处可读的信息，其余都是调参时才看。
 *
 * 事件横幅全部在这里自己推：本文件盯 state()->tactic_id 和 mission_status，
 * 一旦变化就自己开计时，不需要 mission/tactic/main 反过来调 UI（分层里 ui 是侧枝，不被下层依赖）。
 */

#include "display.h"
#include "config.h"
#include "state.h"
#include "irtrack.h"
#include "imu.h"
#include "oled.h"
#include "laps.h"
#include "control.h"
#include "motion.h"
#include "mission.h"
#include "tactic.h"
#include <stdio.h>

/* 8 路红外：压黑线画 '1'（灯亮），白线画 '0'（灯灭）。
 * ir_data.raw 的下标已按物理位置从左到右排好（0 = 最左），直接顺序输出。 */
static void format_ir(char *out)
{
    uint8_t i;

    out[0] = 'I';
    out[1] = 'R';
    out[2] = ' ';
    for (i = 0U; i < IRTRACK_CHANNELS; i++) {
        out[3U + i] = (ir_data.raw[i] == 0U) ? '1' : '0';
    }
    out[3U + IRTRACK_CHANNELS] = '\0';
}

/* -0.04 用 %+.1f 会印成 -0.0，看着像故障；小于半个显示单位的量统一归零。 */
static float nz(float v)
{
    if ((v < 0.05f) && (v > -0.05f)) {
        return 0.0f;
    }
    return v;
}

/* ---- 事件横幅：只保留「现在最该看的那条」 ---- */
static uint8_t s_prev_tactic = 0xFFU;     /* 0xFF = 还没渲染过 */
static uint8_t s_prev_status = 0xFFU;
static uint32_t s_tactic_until_ms = 0U;
static uint32_t s_done_until_ms = 0U;

void display_boot(void)
{
    oled_clear();
    oled_show_string(1, 1, " BlueCar ");
    oled_show_string(2, 1, " v11-08  ");
    oled_flush();
}

/* 第一行常态：战术号 + 战术名 + （战术2 的步长）+ 动作状态，定宽 16 字符不抖。
   战术 2 的步长是它唯一的可调参数，所以紧跟名字；单位见第 7 行的 'd'。 */
static void format_title(char *out, uint8_t size, uint8_t tactic, float step_deg,
                         const char *status)
{
    if (tactic == (uint8_t)TACTIC_STEP_SPIN) {
        (void)snprintf(out, size, "T2 STEP%3.0f %4s", (double)step_deg, status);
    } else {
        (void)snprintf(out, size, "T1 TRACK%8s", status);
    }
}

/* 第 8 行：优先级 丢线/陀螺 > 任务结束(2 s) > 战术切换(1.5 s) > 常态提示。
   丢线、陀螺未就绪直接由状态推出来，不需要谁点火；只有「刚切换到战术 2」这种瞬时事件才靠计时。 */
static uint8_t banner_pick(const AppState *st, const imu_data_t *imu, uint32_t now_ms,
                           char *out, uint8_t size, uint8_t *takeover_title)
{
    uint8_t ret = 0U;

    *takeover_title = 0U;

    if (st->stop_reason == STOP_LINE_LOST) {
        (void)snprintf(out, size, "!! LINE LOST");
        return 1U;
    }
    if ((imu == 0) || (imu->ok == 0U)) {
        (void)snprintf(out, size, "!! IMU CAL");
        return 1U;
    }
    if ((s_done_until_ms != 0U) && ((int32_t)(s_done_until_ms - now_ms) > 0)) {
        (void)snprintf(out, size, "== MISSION DONE");
        ret = 1U;
    } else if ((s_tactic_until_ms != 0U) && ((int32_t)(s_tactic_until_ms - now_ms) > 0)) {
        (void)snprintf(out, size, ">> TACTIC %u %s", (unsigned)st->tactic_id,
                       tactic_name((TacticId)st->tactic_id));
        *takeover_title = 1U;      /* 切战术时第一行整行换成确认条，1.5 s 后回常态 */
        ret = 1U;
    }

    if ((ret == 0U) && (*takeover_title == 0U)) {
        (void)snprintf(out, size, "RDY K1=GO K2=TAC");
    }
    return ret;
}

void display_task(void)
{
    const imu_data_t *imu = imu_get();
    const Pid *pl = control_pid_left();
    const Pid *pr = control_pid_right();
    const AppState *st = state();
    uint32_t now = HAL_GetTick();
    char title[20];
    char line[24];
    char irbuf[12];
    char banner[24];
    uint8_t takeover = 0U;

    /* 战术号变了 → 开 1.5 s 的切换提示；任务状态进 DONE/TIMEOUT → 开 2 s 的结束提示。
       都放这里做，避免任务层为了点灯反过来 include UI。 */
    if (s_prev_tactic != st->tactic_id) {
        if (s_prev_tactic != 0xFFU) {
            s_tactic_until_ms = now + UI_BANNER_MS_TACTIC;
        }
        s_prev_tactic = st->tactic_id;
    }
    if (s_prev_status != st->mission_status) {
        if ((st->mission_status == (uint8_t)MISSION_ST_DONE) ||
            (st->mission_status == (uint8_t)MISSION_ST_TIMEOUT)) {
            s_done_until_ms = now + UI_BANNER_MS_DONE;
        }
        s_prev_status = st->mission_status;
    }

    oled_clear();

    (void)banner_pick(st, imu, now, banner, sizeof(banner), &takeover);

    /* 1 行大字战术条（占物理 1-2 行）：T1 TRACK   RUN / T2 STEP 45  RUN */
    if (takeover != 0U) {
        (void)snprintf(title, sizeof(title), ">> TACTIC %u %s", (unsigned)st->tactic_id,
                       tactic_name((TacticId)st->tactic_id));
    } else {
        format_title(title, sizeof(title), st->tactic_id, st->tactic_step_deg,
                     mission_status_text((MissionStatus)st->mission_status));
    }
    oled_show_string(1, 0, title);

    /* 2 红外 + 圈数（战术 2 里圈数只作累计参考，不参与决策） */
    format_ir(irbuf);
    (void)snprintf(line, sizeof(line), "%s  L%2u/%2u", irbuf,
                   (unsigned)laps_count(), (unsigned)st->lap_target);
    oled_show_string_small(3, 0, line);

    /* 3 三轴角度：一位小数，符号强制；IMU 没就绪时给 --- 而不是假装 0 */
    if ((imu != 0) && (imu->ok != 0U)) {
        (void)snprintf(line, sizeof(line), "Y%+5.1f P%+5.1f R%+5.1f",
                       (double)nz(imu->yaw), (double)nz(imu->pitch_deg),
                       (double)nz(imu->roll_deg));
    } else {
        (void)snprintf(line, sizeof(line), "Y  --- P  --- R  ---");
    }
    oled_show_string_small(4, 0, line);

    /* 4 速度目标/实际：T = 目标 RPM（左/右），A = 实测 RPM（左/右） */
    (void)snprintf(line, sizeof(line), "T%4.0f/%3.0f A%4.0f/%3.0f",
                   (double)pl->target, (double)pr->target,
                   (double)pl->actual, (double)pr->actual);
    oled_show_string_small(5, 0, line);

    /* 5 占空比 + 里程 */
    (void)snprintf(line, sizeof(line), "PWM%4.0f%4.0f D%6.2fm",
                   (double)st->pwm_left, (double)st->pwm_right,
                   (double)motion_distance_m());
    oled_show_string_small(6, 0, line);

    /* 6 战术相关的量：战术 2 看步数与步长，战术 1 看圈数是否到位；右边固定给角速度 */
    if (st->tactic_id == (uint8_t)TACTIC_STEP_SPIN) {
        /* 战术 2 看三件事：第几步、每步多少度、这一步还剩多少度（E = 误差）。
           剩余角度是转角闭环唯一能说明「为什么还没停」的量。 */
        (void)snprintf(line, sizeof(line), "S%2u %3.0fd E%+5.1f",
                       (unsigned)st->tactic_steps, (double)st->tactic_step_deg,
                       (double)nz(st->spin_err_deg));
    } else {
        (void)snprintf(line, sizeof(line), "LAP %3s  gy%+6.1f",
                       (laps_count() >= st->lap_target) ? "DONE" : "RUN",
                       (double)nz(st->gyro_z_dps));
    }
    oled_show_string_small(7, 0, line);

    /* 7 事件横幅 */
    oled_show_string_small(8, 0, banner);

    oled_flush();
}

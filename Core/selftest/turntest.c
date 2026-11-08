/* 转角自检的实现，见 turntest.h 的设计说明。
 *
 * 时序（与战术包 2 完全一致，只是多了一个判定窗口）：
 *   SPIN   —— mission_start_spin(step) 起一步，等它自然结束
 *   SETTLE —— 从到位那一拍开始数 TACTIC_STEP_GAP_MS，这整段就是停车窗口
 *             每拍采一次八路红外，末次采到的图案就是「停止那一刻屏幕上的图案」
 *   然后再起下一步，直到步数用完，最后打一行汇总。
 *
 * 全部判定量都是物理量（角度、图案），没有任何「到点算失败」的时钟判据；
 * 唯一的放弃路径是 mission 自己报 TIMEOUT（误差不再缩小），那一步记失败，
 * 下一步照样给机会，不因为一次失败就整轮作废。
 */
#include "main.h"
#include "turntest.h"
#include "config.h"
#include "irtrack.h"
#include "mission.h"
#include "imu.h"
#include "tactic.h"
#include "serial.h"
#include "state.h"
#include <math.h>
#include <stdio.h>

/* 连续采样帧（与 pidtrace 的 0xAA55 帧并列，用第二字节区分）：
   AA 56 | len | 12×int16 | CRC16
   「转一下采一下」样本太少，改为全程 50 Hz 连续记录，事后在上位机按步切片分析。
   字段：t_ms(低16) yaw×10 cum×10 spin_err×10 actL actR tgtL tgtR mask black_count
         (phase<<8|idx) resid×10 norm_pos×10000 weight_sum
   末尾两字段是横向偏移：红外加权归一化位置（-1..+1）与权重和，
   这样每帧都能算「车相对黑线的横向偏移」，不必只看 8 位图案的通过/失败。 */
#define TT_STREAM_HDR1      0x56U
#define TT_STREAM_FIELDS    14U
#define TT_STREAM_PAYLOAD   (TT_STREAM_FIELDS * 2U)
#define TT_STREAM_FRAME     (TT_STREAM_PAYLOAD + 5U)

static uint32_t s_stream_tick = 0xFFFFFFFFU;

/* 中间两路（索引 3、4）压黑线 = 车头正对十字臂 */
#define TT_MASK_CENTER_BLACK   0x18U
/* 判定窗口内允许的最大采样数（5 ms 一拍 × 2 s 窗口 = 400，留一倍余量） */
#define TT_WIN_N_MAX           0xFFFFU
/* 角度对齐容差：累计角度离 90° 整数倍小于它就算正对臂 */
#define TT_ALIGN_TOL_DEG       1.0f
/* 事件队列深度：主循环一圈就能排空，16 条足够 */
#define TT_EV_RING             16U

typedef enum {
    TT_PHASE_IDLE = 0,
    TT_PHASE_SPIN,
    TT_PHASE_SETTLE,
    TT_PHASE_DONE
} TtPhase;

typedef enum {
    TT_EV_START = 0,
    TT_EV_STEP,
    TT_EV_SUMMARY,
    TT_EV_ABORT
} TtEvKind;

typedef struct {
    uint8_t  kind;
    uint8_t  ok;
    uint8_t  mask;
    uint8_t  expect_mask;
    uint8_t  ratio;
    uint8_t  fails;
    uint16_t idx;
    uint16_t total;
    uint16_t laps;
    float    cum;
    float    resid;
    float    step;
} TtEvent;

static TtEvent s_ev[TT_EV_RING];
static volatile uint8_t s_ev_head;    /* 中断侧写 */
static volatile uint8_t s_ev_tail;    /* 主循环侧读 */

static TtPhase  s_phase = TT_PHASE_IDLE;
static float    s_step = TACTIC_STEP_DEG_DEFAULT;
static float    s_cum;
static float    s_resid;
static float    s_max_abs_resid;
static uint16_t s_total;
static uint16_t s_idx;
static uint16_t s_laps;
static uint16_t s_fail;
static uint32_t s_gap_us;
static uint16_t s_win_n;
static uint16_t s_win_ok;
static uint8_t  s_mask;
static uint8_t  s_expect_black;
static uint8_t  s_step_failed;

static void tt_push(const TtEvent *ev)
{
    s_ev[s_ev_head] = *ev;
    __DMB();
    s_ev_head = (uint8_t)((s_ev_head + 1U) % TT_EV_RING);
    if (s_ev_head == s_ev_tail) {
        s_ev_tail = (uint8_t)((s_ev_tail + 1U) % TT_EV_RING);   /* 满了丢最旧的一条 */
    }
}

/* 当前这一拍八路压出来的掩码（'1' = 压黑线），不需要累计窗口 */
static uint8_t tt_current_mask(void)
{
    uint8_t i;
    uint8_t mask = 0U;

    for (i = 0U; i < IRTRACK_CHANNELS; i++) {
        if (ir_data.raw[i] == 0U) {
            mask = (uint8_t)(mask | (uint8_t)(1U << i));
        }
    }
    return mask;
}

static void tt_push_simple(TtEvKind kind, uint16_t idx, uint16_t total)
{
    TtEvent ev;

    ev.kind = (uint8_t)kind;
    ev.ok = 0U;
    ev.mask = tt_current_mask();
    ev.expect_mask = 0U;
    ev.ratio = 0U;
    ev.fails = (uint8_t)((s_fail > 255U) ? 255U : s_fail);
    ev.idx = idx;
    ev.total = total;
    ev.laps = s_laps;
    ev.cum = s_cum;
    ev.resid = s_resid;
    ev.step = s_step;
    tt_push(&ev);
}

/* 累计角度是不是 90° 的整数倍（容差 1°）。是 → 车头对臂 → 期望中间两路黑。 */
static uint8_t tt_expect_black(float cum)
{
    float r = fmodf(cum + 45.0f, 90.0f) - 45.0f;

    return (uint8_t)((fabsf(r) < TT_ALIGN_TOL_DEG) ? 1U : 0U);
}

static uint8_t tt_expect_mask(void)
{
    return (uint8_t)((s_expect_black != 0U) ? TT_MASK_CENTER_BLACK : 0U);
}

/* 采一拍：把八路压线状态压成掩码，'1' 的语义与 OLED 一致（raw==0 表示压黑线） */
static void tt_sample(void)
{
    uint8_t i;
    uint8_t mask = 0U;

    for (i = 0U; i < IRTRACK_CHANNELS; i++) {
        if (ir_data.raw[i] == 0U) {
            mask = (uint8_t)(mask | (uint8_t)(1U << i));
        }
    }
    s_mask = mask;
    if (s_win_n < TT_WIN_N_MAX) {
        s_win_n++;
    }
    if (mask == tt_expect_mask()) {
        if (s_win_ok < TT_WIN_N_MAX) {
            s_win_ok++;
        }
    }
}

uint8_t turntest_active(void)
{
    return (uint8_t)(((s_phase == TT_PHASE_SPIN) || (s_phase == TT_PHASE_SETTLE)) ? 1U : 0U);
}

void turntest_abort(void)
{
    if (turntest_active() == 0U) {
        return;
    }
    s_phase = TT_PHASE_DONE;
    mission_abort();
    tt_push_simple(TT_EV_ABORT, (uint16_t)(s_idx + 1U), s_total);
}

void turntest_start(float step_deg, uint16_t laps)
{
    float span;
    uint16_t total;

    if (step_deg < 5.0f) {
        step_deg = TACTIC_STEP_DEG_DEFAULT;
    }
    if (step_deg > 360.0f) {
        step_deg = 360.0f;
    }
    if (laps == 0U) {
        laps = 1U;
    }

    span = 360.0f * (float)laps;
    total = (uint16_t)((span / step_deg) + 0.5f);
    if (total == 0U) {
        total = 1U;
    }

    /* 自检和战术包抢同一条 mission 链：先把战术停掉，避免两边互相起转 */
    tactic_stop();

    s_step = step_deg;
    s_laps = laps;
    s_total = total;
    s_idx = 0U;
    s_cum = 0.0f;
    s_resid = 0.0f;
    s_max_abs_resid = 0.0f;
    s_fail = 0U;
    s_gap_us = 0U;
    s_win_n = 0U;
    s_win_ok = 0U;
    s_mask = 0U;
    s_step_failed = 0U;
    s_expect_black = 1U;    /* 起点车头对着臂 */

    tt_push_simple(TT_EV_START, 0U, s_total);

    mission_start_spin(s_step);
    if (mission_status() == MISSION_ST_ABORTED) {
        s_phase = TT_PHASE_IDLE;
        tt_push_simple(TT_EV_ABORT, 0U, s_total);
        return;
    }
    s_phase = TT_PHASE_SPIN;
}

void turntest_tick(uint32_t dt_us)
{
    MissionStatus ms;

    if ((s_phase != TT_PHASE_SPIN) && (s_phase != TT_PHASE_SETTLE)) {
        return;
    }

    if (s_phase == TT_PHASE_SPIN) {
        ms = mission_status();
        if (ms == MISSION_ST_RUNNING) {
            return;
        }
        if (ms == MISSION_ST_ABORTED) {
            s_phase = TT_PHASE_DONE;
            tt_push_simple(TT_EV_ABORT, (uint16_t)(s_idx + 1U), s_total);
            return;
        }
        /* DONE 或 TIMEOUT 都进入停车窗口：TIMEOUT 说明这步没转到位，记失败但仍记录它停在哪个图案上 */
        s_step_failed = (uint8_t)((ms == MISSION_ST_TIMEOUT) ? 1U : 0U);
        s_cum += s_step;
        s_expect_black = tt_expect_black(s_cum);
        s_gap_us = 0U;
        s_win_n = 0U;
        s_win_ok = 0U;
        s_mask = 0U;
        s_phase = TT_PHASE_SETTLE;
        return;                      /* 从下一拍开始采，本拍的 ir 还是上一段运动末期的 */
    }

    /* TT_PHASE_SETTLE：停车窗口内每拍采一次 */
    tt_sample();
    s_gap_us += dt_us;
    if (s_gap_us < ((uint32_t)TACTIC_STEP_GAP_MS * 1000UL)) {
        return;
    }

    {
        TtEvent ev;
        uint8_t expect = tt_expect_mask();
        uint8_t ok = (uint8_t)(((s_mask == expect) && (s_step_failed == 0U)) ? 1U : 0U);
        uint8_t ratio = (uint8_t)((s_win_n == 0U) ? 0U : ((uint32_t)s_win_ok * 100UL / s_win_n));
        float abs_resid;

        /* 残差取停车窗口末端的值：转角环判到位那一刻车还在惯性地转，
           停下来之后误差才是「这一步真的转了多少」——测精度只能用它。 */
        s_resid = mission_spin_err_deg();
        abs_resid = fabsf(s_resid);

        s_idx++;
        if (abs_resid > s_max_abs_resid) {
            s_max_abs_resid = abs_resid;
        }
        if (ok == 0U) {
            s_fail++;
        }

        ev.kind = (uint8_t)TT_EV_STEP;
        ev.ok = ok;
        ev.mask = s_mask;
        ev.expect_mask = expect;
        ev.ratio = ratio;
        ev.fails = (uint8_t)((s_fail > 255U) ? 255U : s_fail);
        ev.idx = s_idx;
        ev.total = s_total;
        ev.laps = s_laps;
        ev.cum = s_cum;
        ev.resid = s_resid;
        ev.step = s_step;
        tt_push(&ev);
    }

    if (s_idx >= s_total) {
        tt_push_simple(TT_EV_SUMMARY, s_idx, s_total);
        s_phase = TT_PHASE_DONE;
        return;
    }

    mission_start_spin(s_step);
    if (mission_status() == MISSION_ST_ABORTED) {
        s_phase = TT_PHASE_DONE;
        tt_push_simple(TT_EV_ABORT, (uint16_t)(s_idx + 1U), s_total);
        return;
    }
    s_step_failed = 0U;
    s_gap_us = 0U;
    s_phase = TT_PHASE_SPIN;
}

static uint16_t tt_crc16(const uint8_t *p, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint8_t i;

    while (len != 0U) {
        crc ^= (uint16_t)((uint16_t)(*p) << 8);
        p++;
        len--;
        for (i = 0U; i < 8U; i++) {
            crc = (uint16_t)(((crc & 0x8000U) != 0U) ? ((uint16_t)(crc << 1) ^ 0x1021U)
                                                     : (uint16_t)(crc << 1));
        }
    }
    return crc;
}

static void tt_put_i16(uint8_t *p, int16_t v)
{
    uint16_t u = (uint16_t)v;

    p[0] = (uint8_t)(u & 0xFFU);
    p[1] = (uint8_t)((u >> 8) & 0xFFU);
}

static int16_t tt_i16(float v, float scale)
{
    float x = v * scale;

    if (x != x) {
        return 0;
    }
    x += (x > 0.0f) ? 0.5f : -0.5f;
    if (x > 32767.0f) {
        x = 32767.0f;
    } else if (x < -32767.0f) {
        x = -32767.0f;
    }
    return (int16_t)x;
}

/* 主循环调用：自检运行期间每 TASK_TRACE_MS 发一帧连续样本 */
static void tt_stream(void)
{
    const AppState *st = state();
    uint8_t f[TT_STREAM_FRAME];
    uint32_t tick = st->tick_count;
    uint16_t crc;
    uint8_t mask;
    uint8_t i;
    uint8_t black = 0U;

    if (tick == s_stream_tick) {
        return;
    }
    if ((tick % (TASK_TRACE_MS / TASK_CONTROL_MS)) != 0U) {
        return;
    }
    s_stream_tick = tick;
    if (serial_tx_free() < TT_STREAM_FRAME) {
        return;                      /* 宁丢一帧，不发半帧 */
    }

    mask = 0U;
    for (i = 0U; i < IRTRACK_CHANNELS; i++) {
        if (ir_data.raw[i] == 0U) {
            mask = (uint8_t)(mask | (uint8_t)(1U << i));
            black++;
        }
    }

    f[0] = 0xAAU;
    f[1] = TT_STREAM_HDR1;
    f[2] = (uint8_t)TT_STREAM_PAYLOAD;
    tt_put_i16(&f[3],  (int16_t)(HAL_GetTick() & 0xFFFFU));
    tt_put_i16(&f[5],  tt_i16(imu_get()->yaw, 10.0f));   /* 不能取 st->yaw_deg：那是 track_task 写的，转角期间不更新 */
    tt_put_i16(&f[7],  tt_i16(s_cum, 10.0f));
    tt_put_i16(&f[9],  tt_i16(mission_spin_err_deg(), 10.0f));
    tt_put_i16(&f[11], tt_i16(st->actual_left_rpm, 10.0f));
    tt_put_i16(&f[13], tt_i16(st->actual_right_rpm, 10.0f));
    tt_put_i16(&f[15], tt_i16(st->target_left_rpm, 10.0f));
    tt_put_i16(&f[17], tt_i16(st->target_right_rpm, 10.0f));
    tt_put_i16(&f[19], (int16_t)mask);
    tt_put_i16(&f[21], (int16_t)black);
    tt_put_i16(&f[23], (int16_t)(((uint16_t)s_phase << 8) | (uint16_t)(s_idx & 0xFFU)));
    tt_put_i16(&f[25], tt_i16(s_resid, 10.0f));
    tt_put_i16(&f[27], tt_i16(ir_data.normalized_position, 10000.0f));
    tt_put_i16(&f[29], (int16_t)ir_data.weight_sum);

    crc = tt_crc16(&f[2], (uint16_t)(TT_STREAM_PAYLOAD + 1U));
    f[31] = (uint8_t)(crc & 0xFFU);
    f[32] = (uint8_t)((crc >> 8) & 0xFFU);
    serial_write_bytes(f, TT_STREAM_FRAME);
}

/* 图案掩码 → "00011000"，索引 0 = 物理最左，'1' = 压黑线 */
static char *tt_pat_str(char *out, uint8_t mask)
{
    uint8_t i;

    for (i = 0U; i < IRTRACK_CHANNELS; i++) {
        out[i] = ((mask & (uint8_t)(1U << i)) != 0U) ? '1' : '0';
    }
    out[IRTRACK_CHANNELS] = '\0';
    return out;
}

void turntest_task(void)
{
    char buf[144];
    char pat[IRTRACK_CHANNELS + 1];
    char exp_pat[IRTRACK_CHANNELS + 1];

    if (turntest_active() != 0U) {
        tt_stream();
    }

    while (s_ev_tail != s_ev_head) {
        TtEvent ev = s_ev[s_ev_tail];

        __DMB();
        s_ev_tail = (uint8_t)((s_ev_tail + 1U) % TT_EV_RING);

        switch ((TtEvKind)ev.kind) {
        case TT_EV_START:
            (void)snprintf(buf, sizeof(buf),
                           "[自检] 开始 步长%.1f° 圈数%u 共%u步 停车判定%ums 起始图案%s\r\n",
                           (double)ev.step, (unsigned)ev.laps, (unsigned)ev.total,
                           (unsigned)TACTIC_STEP_GAP_MS, tt_pat_str(pat, ev.mask));
            serial_write(buf);
            break;

        case TT_EV_STEP:
            tt_pat_str(pat, ev.mask);
            tt_pat_str(exp_pat, ev.expect_mask);
            (void)snprintf(buf, sizeof(buf),
                           "[自检] %02u/%02u 累%6.1f° 残%+5.1f° 图案%s 期望%s 匹配%3u%% %s\r\n",
                           (unsigned)ev.idx, (unsigned)ev.total, (double)ev.cum,
                           (double)ev.resid, pat, exp_pat, (unsigned)ev.ratio,
                           (ev.ok != 0U) ? "OK" : "FAIL");
            serial_write(buf);
            break;

        case TT_EV_SUMMARY:
            (void)snprintf(buf, sizeof(buf),
                           "[自检] 汇总 %u步 通过%u 失败%u 最大|残|%.1f° 步长%.1f° 圈数%u 结束图案%s\r\n",
                           (unsigned)ev.total,
                           (unsigned)(ev.total - ev.fails), (unsigned)ev.fails,
                           (double)s_max_abs_resid, (double)ev.step, (unsigned)ev.laps,
                           tt_pat_str(pat, ev.mask));
            serial_write(buf);
            break;

        case TT_EV_ABORT:
        default:
            (void)snprintf(buf, sizeof(buf),
                           "[自检] 中止 步%u/%u（陀螺未就绪或已被停止）\r\n",
                           (unsigned)ev.idx, (unsigned)ev.total);
            serial_write(buf);
            break;
        }
    }
}
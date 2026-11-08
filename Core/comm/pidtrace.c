/* 转向环数据采集（pidtrace）。
 *
 * 为什么用二进制而不是 printf：5 ms 的控制拍要观测 e / de/dt / 陀螺 / 输出 5 个量，
 * 文本一行 60+ 字节且要 snprintf（本工程禁止在中断里 snprintf，主循环里也慢）。
 * 定长 29 字节一帧，50 Hz 只要 1.45 kB/s，115200 下一半带宽都用不满。
 *
 * 帧格式（小端，共 29 字节）：
 *   [0]=0xAA [1]=0x55 [2]=32（payload 字节数）
 *   [3..34]=payload，16 × int16
 *   [35..36]=CRC16-CCITT，覆盖 [2..34]（长度 + payload），多项式 0x1021，初值 0xFFFF
 *
 * payload 顺序（每个 int16 的缩放写在后面）：
 *   0  seq        帧序号（0..65535 回绕）
 *   1  err        ×10000  转向环输入：normalized_position，-1..+1
 *   2  deriv      ×100    低通后的 d(e)/dt，单位 1/s（×1 时 0.5 会被截成 0，实测全部读 0）
 *   3  wz         ×10     陀螺 +Z 角速度，dps（右转为负）
 *   4  steer      ×10000  限幅后的转向量，±TRACK_STEER_LIMIT
 *   5  kv         ×1000   车速解耦系数（0.60..1.80）
 *   6  tgtL       ×10     左轮目标 RPM
 *   7  tgtR       ×10     右轮目标 RPM
 *   8  actL       ×10     左轮实测 RPM（50 ms 窗）
 *   9  actR       ×10     右轮实测 RPM
 *  10  dt_us      ×1      控制拍实测间隔（DWT）
 *  11  flags              bit0 line_ok | bit1-2 丢线策略 | bit3-5 停车原因 |
 *                         bit6-7 模式 | bit8 注入测试 | bit9 限幅前已越界
 *                         | bit10 err 取自传感器（控制环未运行）
 *  12  cmdL       ×10     左轮斜坡后的命令 RPM（斜坡限速，和 tgt 不一定相等）
 *  13  cmdR       ×10     右轮斜坡后的命令 RPM
 *  14  pwmL       ×100    左轮占空比 %
 *  15  pwmR       ×100    右轮占空比 %
 *  （12..15 是 2026-11-08 加的：查「目标 0 但轮子还转」必须能看见 PWM 有没有给）
 *
 * 注入测试（串口 'C'）：合成一条 -1 → +1 → -1 的三角波当偏差，连同它的一阶导数
 * 送进 track_steer_eval()（与实车同一条公式），用来在台架上验证「增益 × 输入 → 输出」
 * 这条链路和限幅，不需要车在线上跑。注入期间真实控制输出不变，帧里 bit8 置 1。
 */
#include "pidtrace.h"
#include "config.h"
#include "state.h"
#include "serial.h"
#include "track.h"
#include "irtrack.h"
#include <math.h>

#define TRACE_HDR0     0xAAU
#define TRACE_HDR1     0x55U
#define TRACE_FIELDS   16U
#define TRACE_PAYLOAD  (TRACE_FIELDS * 2U)
#define TRACE_FRAME    (TRACE_PAYLOAD + 5U)   /* 帧头 2 + 长度 1 + payload + CRC 2 */

/* 注入三角波：一个周期 400 帧 = 8 s，-1 → +1 用 4 s，斜率 0.5 /s */
#define INJECT_PERIOD  400.0f
#define INJECT_HALF    200.0f
#define INJECT_SLOPE   0.5f

static uint8_t s_on = PIDTRACE_DEFAULT_ON;
static uint8_t s_inject = 0U;
static uint16_t s_seq = 0U;
static uint32_t s_last_tick = 0xFFFFFFFFU;
static float s_phase = 0.0f;
static uint32_t s_frames = 0U;
static uint32_t s_skipped = 0U;

static uint16_t crc16_ccitt(const uint8_t *p, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint8_t i;

    while (len != 0U) {
        crc ^= (uint16_t)((uint16_t)(*p) << 8);
        p++;
        len--;
        for (i = 0U; i < 8U; i++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* 定点打包：截断会把 0.9999 变成 0.9998 这种一档误差，四舍五入让主机的还原值
   与固件里的浮点值在 1/缩放 的半个单位内一致。 */
static int16_t to_i16(float v, float scale)
{
    float x = v * scale;
    int16_t r;

    if (x != x) {            /* NaN → 0，别让 NaN 污染整条曲线 */
        return 0;
    }
    if (x > 0.0f) {
        x += 0.5f;
    } else {
        x -= 0.5f;
    }
    if (x > 32767.0f) {
        x = 32767.0f;
    } else if (x < -32767.0f) {
        x = -32767.0f;
    }
    r = (int16_t)x;
    return r;
}

static void put_i16(uint8_t *p, int16_t v)
{
    uint16_t u = (uint16_t)v;

    p[0] = (uint8_t)(u & 0xFFU);
    p[1] = (uint8_t)((u >> 8) & 0xFFU);
}

static void trace_send(void)
{
    const AppState *st = state();
    uint8_t frame[TRACE_FRAME];
    float err;
    float deriv;
    float wz;
    float u_raw;
    float u;
    uint16_t flags = 0U;
    uint16_t crc;

    if (s_inject != 0U) {
        s_phase += 1.0f;
        if (s_phase >= INJECT_PERIOD) {
            s_phase -= INJECT_PERIOD;
        }
        if (s_phase < INJECT_HALF) {
            err = -1.0f + (s_phase / INJECT_HALF) * 2.0f;
            deriv = INJECT_SLOPE;
        } else {
            err = 1.0f - ((s_phase - INJECT_HALF) / INJECT_HALF) * 2.0f;
            deriv = -INJECT_SLOPE;
        }
        wz = 0.0f;
        u_raw = track_steer_eval(err, deriv, wz, 1.0f);
        u = u_raw;
        flags |= (uint16_t)(1U << 8);
    } else if (st->mode == APP_MODE_TRACK) {
        err = st->track_err;
        deriv = st->track_deriv;
        wz = st->gyro_z_dps;
        u_raw = st->track_u_raw;
        u = st->steer;
    } else {
        /* 转向环没在跑（IDLE / MANUAL）：报传感器当前看到的偏差，
           台架上也能确认「红外 → 归一化」这一段是通的；bit10 标记来源不是控制环 */
        err = ir_data.normalized_position;
        deriv = 0.0f;
        wz = st->gyro_z_dps;
        u_raw = 0.0f;
        u = 0.0f;
        flags |= (uint16_t)(1U << 10);
    }

    flags |= (uint16_t)((st->line_ok != 0U) ? 1U : 0U);
    flags |= (uint16_t)(((uint16_t)st->line_lost_policy & 3U) << 1);
    flags |= (uint16_t)(((uint16_t)st->stop_reason & 7U) << 3);
    flags |= (uint16_t)(((uint16_t)st->mode & 3U) << 6);
    if (fabsf(u_raw) > (TRACK_STEER_LIMIT + 0.0001f)) {
        flags |= (uint16_t)(1U << 9);
    }

    /* 整帧一次入队，宁可丢这一帧也不发半帧：半帧会让主机花更长的时间重新对齐 */
    if (serial_tx_free() < TRACE_FRAME) {
        s_skipped++;
        return;
    }

    frame[0] = TRACE_HDR0;
    frame[1] = TRACE_HDR1;
    frame[2] = (uint8_t)TRACE_PAYLOAD;
    put_i16(&frame[3], (int16_t)s_seq);
    put_i16(&frame[5], to_i16(err, 10000.0f));
    put_i16(&frame[7], to_i16(deriv, 100.0f));
    put_i16(&frame[9], to_i16(wz, 10.0f));
    put_i16(&frame[11], to_i16(u, 10000.0f));
    put_i16(&frame[13], to_i16(st->gain_kv, 1000.0f));
    put_i16(&frame[15], to_i16(st->target_left_rpm, 10.0f));
    put_i16(&frame[17], to_i16(st->target_right_rpm, 10.0f));
    put_i16(&frame[19], to_i16(st->actual_left_rpm, 10.0f));
    put_i16(&frame[21], to_i16(st->actual_right_rpm, 10.0f));
    put_i16(&frame[23], (int16_t)((st->control_dt_us > 32767UL) ? 32767UL : st->control_dt_us));
    put_i16(&frame[25], (int16_t)flags);
    put_i16(&frame[27], to_i16(st->cmd_left_rpm, 10.0f));
    put_i16(&frame[29], to_i16(st->cmd_right_rpm, 10.0f));
    put_i16(&frame[31], to_i16(st->pwm_left, 100.0f));
    put_i16(&frame[33], to_i16(st->pwm_right, 100.0f));

    crc = crc16_ccitt(&frame[2], (uint16_t)(TRACE_PAYLOAD + 1U));
    frame[35] = (uint8_t)(crc & 0xFFU);
    frame[36] = (uint8_t)((crc >> 8) & 0xFFU);

    serial_write_bytes(frame, TRACE_FRAME);
    s_seq++;
    s_frames++;
}

void pidtrace_init(void)
{
    s_on = PIDTRACE_DEFAULT_ON;
    s_inject = 0U;
    s_seq = 0U;
    s_last_tick = 0xFFFFFFFFU;
    s_phase = 0.0f;
    s_frames = 0U;
    s_skipped = 0U;
}

void pidtrace_task(void)
{
    uint32_t tick;

    if (s_on == 0U) {
        return;
    }
    tick = state()->tick_count;
    if (tick == s_last_tick) {
        return;
    }
#if ((TASK_TRACE_MS % TASK_CONTROL_MS) != 0U)
#error "TASK_TRACE_MS 必须是 TASK_CONTROL_MS 的整数倍"
#endif
    if ((tick % (uint32_t)(TASK_TRACE_MS / TASK_CONTROL_MS)) != 0U) {
        return;
    }
    s_last_tick = tick;
    trace_send();
}

uint8_t pidtrace_on(void)
{
    return s_on;
}

void pidtrace_set_on(uint8_t on)
{
    s_on = (uint8_t)((on != 0U) ? 1U : 0U);
    if (s_on == 0U) {
        s_inject = 0U;
    }
}

uint8_t pidtrace_inject(void)
{
    return s_inject;
}

void pidtrace_set_inject(uint8_t on)
{
    s_inject = (uint8_t)((on != 0U) ? 1U : 0U);
    s_phase = 0.0f;
    if (s_inject != 0U) {
        s_on = 1U;
    }
}

uint32_t pidtrace_frames(void)
{
    return s_frames;
}

uint32_t pidtrace_skipped(void)
{
    return s_skipped;
}
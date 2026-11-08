#include "track.h"
#include "config.h"
#include "state.h"
#include "irtrack.h"
#include "laps.h"
#include "imu.h"
#include "control.h"
#include <math.h>

static float s_prev_error = 0.0f;
static float s_d = 0.0f;          /* 低通后的偏差微分，单位 1/s */
static float s_steer = 0.0f;
static uint8_t s_line_ok = 1U;
static uint32_t s_lost_ticks = 0U;

/* 运行期可调的三组增益与差速整形参数，上电取 config.h 的宏 */
static float s_kp = TRACK_KP;
static float s_kd = TRACK_KD;
static float s_kgyro = TRACK_GYRO_KP;
static float s_diff_limit = TRACK_DIFF_LIMIT;
static float s_diff_outer = TRACK_DIFF_OUTER;
static TrackLostPolicy s_policy = (TrackLostPolicy)TRACK_LOSS_POLICY_DEFAULT;

/* 入弯上下文：供 ARC 策略在丢线后知道「刚才往哪边转、从哪个航向开始」 */
static uint8_t s_arc_ctx_valid = 0U;
static float s_arc_yaw0 = 0.0f;
static float s_arc_dir = 1.0f;    /* +1 = 右转（顺时针），-1 = 左转 */
static uint32_t s_arc_ctx_ticks = 0U;
static uint8_t s_arc_running = 0U;
static float s_arc_best_err = 0.0f;   /* 本次找线到达过的最小 |误差| */
static uint32_t s_arc_stall_ms = 0U;  /* 距上次「更靠近目标」已过去多久 */

/* 入弯降速状态 */
static uint8_t s_curve_confirm = 0U;
static uint8_t s_curve_active = 0U;
static uint32_t s_curve_hold = 0U;

/* 转角闭环状态 */
/* 转角环参数做成运行期变量：串口 V 命令可以改，扫参时不用每次重编译重烧录。 */
static float s_turn_kp = TRACK_TURN_KP;
static float s_turn_ki = TRACK_TURN_KI;
static float s_turn_kd = TRACK_TURN_KD;
static float s_turn_rpm = TRACK_TURN_RPM;
static float s_turn_tol = TRACK_TURN_TOL_DEG;
static float s_turn_db = TRACK_TURN_DEADBAND_RPM;
static float s_turn_target_yaw = 0.0f;
static uint8_t s_turn_settle = 0U;
static float s_turn_i = 0.0f;   /* 角度误差积分（度·秒），用来顶开端部静摩擦 */

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

float track_steer_eval(float err, float deriv, float wz, float kv)
{
    float u;

    u = ((s_kp * err) + (s_kd * deriv) + (TRACK_GYRO_SIGN * s_kgyro * wz)) * kv;
    return clampf(u, -TRACK_STEER_LIMIT, TRACK_STEER_LIMIT);
}

float track_wrap180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

void track_init(void)
{
    track_reset();
    laps_init(LAP_DEFAULT_TARGET);
}

void track_reset(void)
{
    s_prev_error = 0.0f;
    s_d = 0.0f;
    s_steer = 0.0f;
    s_line_ok = 1U;
    s_lost_ticks = 0U;
    s_arc_ctx_valid = 0U;
    s_arc_ctx_ticks = 0U;
    s_arc_running = 0U;
    s_arc_best_err = 0.0f;
    s_arc_stall_ms = 0U;
    s_curve_confirm = 0U;
    s_curve_active = 0U;
    s_curve_hold = 0U;
    s_turn_settle = 0U;
}

void track_set_gains(float kp, float kd, float gyro_kp)
{
    s_kp = kp;
    s_kd = kd;
    s_kgyro = gyro_kp;
}

void track_get_gains(float *kp, float *kd, float *gyro_kp)
{
    if (kp != 0)      *kp = s_kp;
    if (kd != 0)      *kd = s_kd;
    if (gyro_kp != 0) *gyro_kp = s_kgyro;
}

void track_set_diff_limit(float limit, float outer)
{
    s_diff_limit = clampf(limit, 0.05f, 1.0f);
    s_diff_outer = clampf(outer, 0.0f, 1.0f);
}

void track_set_lost_policy(TrackLostPolicy policy)
{
    if ((uint32_t)policy > (uint32_t)TRACK_LOST_TURN) {
        policy = TRACK_LOST_STOP;
    }
    s_policy = policy;
    state_set_policy((uint8_t)policy);
}

TrackLostPolicy track_lost_policy(void)
{
    return s_policy;
}

/* 非对称差速整形：内轮按 |u| 减、外轮只按 s_diff_outer 加。
   u > 0 = 右转 = 左轮是外轮。 */
static void apply_steer(AppState *st, float steer, float base)
{
    float u = clampf(steer, -s_diff_limit, s_diff_limit);
    float left;
    float right;

    if (u >= 0.0f) {
        left = base * (1.0f + (s_diff_outer * u));
        right = base * (1.0f - u);
    } else {
        left = base * (1.0f + u);
        right = base * (1.0f + (s_diff_outer * (-u)));
    }

    st->steer = steer;
    control_apply_targets(left, right);
}

/* 全白：按策略决定是转向找线还是停车。
   返回前一律把目标写好；停车走 state_stop（模式切 IDLE，速度环下一拍自行归零）。 */
static void track_handle_lost(AppState *st, uint32_t dt_us)
{
    const imu_data_t *imu = imu_get();
    float yaw_now = (imu != 0) ? imu->yaw : 0.0f;
    uint8_t turning = 0U;

    /* 确认窗口：线缝、反光、传感器短暂遮挡都会出现单拍全白，
       窗口内沿用上次转向量，不停车也不改策略。 */
    if (s_lost_ticks <= (uint32_t)TRACK_LINE_LOST_CONFIRM) {
        apply_steer(st, s_steer, TRACK_BASE_RPM);
        return;
    }

    switch (s_policy) {
    case TRACK_LOST_HOLD:
        if (s_lost_ticks <= ((uint32_t)TRACK_LINE_LOST_CONFIRM + TICKS_OF_MS(TRACK_LOST_HOLD_MS))) {
            apply_steer(st, s_steer, TRACK_BASE_RPM);
            return;
        }
        break;

    case TRACK_LOST_ARC:
    case TRACK_LOST_TURN:
        if ((imu == 0) || (imu->ok == 0U)) {
            break;                    /* 没陀螺就没法转角，直接停车 */
        }
        if (s_arc_running == 0U) {
            float dir;

            if (s_policy == TRACK_LOST_ARC) {
                if ((s_arc_ctx_valid == 0U)
                    || (s_arc_ctx_ticks > TICKS_OF_MS(TRACK_ARC_CONTEXT_MS))) {
                    break;            /* 没有可信的入弯上下文，不猜 */
                }
                dir = s_arc_dir;
            } else {
                s_arc_yaw0 = yaw_now;
                dir = (s_steer >= 0.0f) ? 1.0f : -1.0f;
            }
            /* 顺时针（dir=+1）时 yaw 减小，故目标 = 基准 - 角度 */
            track_turn_begin(track_wrap180(s_arc_yaw0 - (dir * TRACK_CORNER_ANGLE_DEG)));
            s_arc_running = 1U;
            s_arc_best_err = 999.0f;
            s_arc_stall_ms = 0U;
        }
        turning = (uint8_t)(track_turn_step(dt_us) == 0U);
        if (turning != 0U) {
            /* 两个放弃条件，都是物理量：
               ① 已经扫过的角度超过 TRACK_ARC_MAX_DEG（扫太远说明不是弯，继续转只会绕圈）；
               ② 连续 TRACK_ARC_STALL_MS 没有更靠近目标（堵转、陀螺没数据）。
               慢转不在判据里：转得慢就转得久，不该被钟判死。 */
            float progress = fabsf(track_wrap180(yaw_now - s_arc_yaw0));
            float err_abs = fabsf(track_turn_err_deg());

            if (err_abs < (s_arc_best_err - TRACK_ARC_PROGRESS_DEG)) {
                s_arc_best_err = err_abs;
                s_arc_stall_ms = 0U;
            } else if (s_arc_stall_ms < 0xFFFFFFFFU) {
                s_arc_stall_ms += (dt_us / 1000U);
            }

            if ((progress > TRACK_ARC_MAX_DEG) || (s_arc_stall_ms >= TRACK_ARC_STALL_MS)) {
                turning = 0U;
            } else {
                return;
            }
        }
        break;

    default:
        break;
    }

    control_apply_targets(0.0f, 0.0f);
    state_stop(STOP_LINE_LOST);
}

/* 转向量符号约定：normalized_position > 0 表示线在车身右侧，车要右转。
   若实车方向相反，把 TRACK_KP / TRACK_KD / TRACK_GYRO_SIGN 一起取负。 */
void track_task(uint32_t dt_us)
{
    AppState *st = state();
    const imu_data_t *imu = imu_get();
    float dt_s;
    float error;
    float raw_d;
    float alpha;
    float wz;
    float v;
    float kv;
    float u;
    float base;
    uint8_t online;

    if (st->mode != APP_MODE_TRACK) {
        return;
    }

    dt_s = (float)dt_us * 0.000001f;
    if (dt_s < 0.0005f) {
        dt_s = 0.0005f;
    }

    wz = ((imu != 0) && (imu->ok != 0U)) ? imu->gyro_dps[2] : 0.0f;
    st->gyro_z_dps = wz;
    if (imu != 0) {
        st->yaw_deg = imu->yaw;
    } else if (st->yaw_deg != st->yaw_deg) {
        st->yaw_deg = 0.0f;
    }

    if (s_arc_ctx_ticks < 0xFFFFFFFFU) {
        s_arc_ctx_ticks++;
    }

    online = (uint8_t)(ir_data.black_count != 0U);

    if (online == 0U) {
        s_line_ok = 0U;
        st->line_ok = 0U;
        if (s_lost_ticks < 0xFFFFFFFFU) {
            s_lost_ticks++;
        }
        st->line_lost_ticks = (uint16_t)((s_lost_ticks > 0xFFFFU) ? 0xFFFFU : s_lost_ticks);
        track_handle_lost(st, dt_us);
        return;
    }

    /* 见线：立刻回到闭环，全白状态一律清掉 */
    s_line_ok = 1U;
    st->line_ok = 1U;
    s_lost_ticks = 0U;
    st->line_lost_ticks = 0U;
    s_arc_running = 0U;

    error = ir_data.normalized_position;

    /* 微分先低通：8 路红外归一化后只有 8 个离散档，直接求导全是 ±5 的尖峰，
       KD 会被这些尖峰主导，等于随机抖动。8 Hz 一阶低通把尖峰摊平，KD 才有阻尼含义。 */
    raw_d = (error - s_prev_error) / dt_s;
    alpha = 6.2831853f * TRACK_D_FILTER_HZ * dt_s;
    if (alpha >= 1.0f) {
        alpha = 1.0f;
    } else if (alpha < 0.0f) {
        alpha = 0.0f;
    }
    s_d += alpha * (raw_d - s_d);
    s_prev_error = error;

    /* 车速解耦：转弯所需的差速角速度 ∝ v，固定增益在高速上必然过量。
       kv = BASE/|v|，标称车速下正好 1.0；限幅防止停车瞬间把增益放到无穷。 */
    v = 0.5f * (st->actual_left_rpm + st->actual_right_rpm);
    if (v < 0.0f) {
        v = -v;
    }
    if (v < (0.2f * TRACK_BASE_RPM)) {
        kv = TRACK_GAIN_SCALE_MAX;
    } else {
        kv = TRACK_BASE_RPM / v;
    }
    kv = clampf(kv, TRACK_GAIN_SCALE_MIN, TRACK_GAIN_SCALE_MAX);
    st->gain_kv = kv;

    /* 中间量留给串口采集（pidtrace）读：公式只此一份，采集不重算 */
    st->track_err = error;
    st->track_deriv = s_d;
    st->track_u_raw = ((s_kp * error) + (s_kd * s_d) + (TRACK_GYRO_SIGN * s_kgyro * wz)) * kv;

    u = track_steer_eval(error, s_d, wz, kv);
    s_steer = u;

    /* 入弯降速：连续 TRACK_CURVE_CONFIRM 拍大偏差才算真进弯，
       进入的那一拍记录航向基准（之后不再刷新，长弯里 yaw 一直在变）。 */
    if (fabsf(error) > TRACK_CURVE_ENTER_E) {
        if (s_curve_confirm < TRACK_CURVE_CONFIRM) {
            s_curve_confirm++;
            if (s_curve_confirm >= TRACK_CURVE_CONFIRM) {
                s_arc_ctx_valid = 1U;
                s_arc_ctx_ticks = 0U;
                s_arc_dir = (error > 0.0f) ? 1.0f : -1.0f;
                if (imu != 0) {
                    s_arc_yaw0 = imu->yaw;
                }
                s_curve_active = 1U;
            }
        }
        s_curve_hold = TICKS_OF_MS(TRACK_CURVE_HOLD_MS);
    } else {
        s_curve_confirm = 0U;
    }
    if (s_curve_hold > 0U) {
        s_curve_hold--;
    } else {
        s_curve_active = 0U;
    }

    base = TRACK_BASE_RPM * ((s_curve_active != 0U) ? TRACK_CURVE_SCALE : 1.0f);
    apply_steer(st, u, base);

    /* 计圈：用同一个偏差量判断进弯 */
    laps_update((uint8_t)((error > LAP_CORNER_ERROR) || (error < -LAP_CORNER_ERROR)));
}

float track_steer(void)
{
    return s_steer;
}

uint8_t track_line_ok(void)
{
    return s_line_ok;
}

float track_kv(void)
{
    return state()->gain_kv;
}

void track_turn_set_gains(float kp, float ki, float kd, float rpm, float tol, float db)
{
    s_turn_kp = kp;
    s_turn_ki = ki;
    s_turn_kd = kd;
    s_turn_rpm = rpm;
    s_turn_tol = tol;
    s_turn_db = db;
}

void track_turn_get_gains(float *kp, float *ki, float *kd, float *rpm, float *tol, float *db)
{
    if (kp != 0) { *kp = s_turn_kp; }
    if (ki != 0) { *ki = s_turn_ki; }
    if (kd != 0) { *kd = s_turn_kd; }
    if (rpm != 0) { *rpm = s_turn_rpm; }
    if (tol != 0) { *tol = s_turn_tol; }
    if (db != 0) { *db = s_turn_db; }
}

void track_turn_begin(float target_yaw)
{
    s_turn_target_yaw = track_wrap180(target_yaw);
    s_turn_settle = 0U;
    s_turn_i = 0.0f;
}

/* 顺时针为正：err = yaw - target，顺时针时 err > 0 → u > 0 → 左轮正、右轮负。
   左轮向前、右轮向后 = 车体顺时针旋转，符号自洽。 */
uint8_t track_turn_step(uint32_t dt_us)
{
    const imu_data_t *imu = imu_get();
    float err;
    float u;

    if ((imu == 0) || (imu->ok == 0U)) {
        control_apply_targets(0.0f, 0.0f);
        return 1U;
    }

    err = track_wrap180(imu->yaw - s_turn_target_yaw);

    /* 积分项不是为了精度，是为了顶开端部静摩擦：KD/KP 在 err≈2° 时只算出
       1.6 rpm 的差速，车轮根本转不动，于是每次都停在容差边界上（实测每步
       欠转 2.0°，8 步累计 16°）。积分把这点误差乘时间累起来，直到车轮动。
       限幅在三倍容差内，避免积分本身变成过冲源。 */
    if (dt_us == 0U) {
        dt_us = TASK_CONTROL_MS * 1000U;
    }
#if (TRACK_TURN_I_ENABLE != 0U)
    if (s_turn_ki > 0.0001f) {
        s_turn_i += err * ((float)dt_us * 0.000001f);
        {
            const float i_max = TRACK_TURN_I_LIMIT_RPM / s_turn_ki;
            s_turn_i = clampf(s_turn_i, -i_max, i_max);
        }
    } else {
        s_turn_i = 0.0f;
    }
#else
    /* KI=0：不要积分。末段靠 KP 抬高把命令顶在死区之上（10x1.5=15 rpm），
       比「积分顶到门槛再咯噔一下」干净；单步残余由起点绝对累计角在下一步补。 */
    s_turn_i = 0.0f;
#endif

    /* 只用一条 PD+KI 曲线收尾：加过「大步距恒速前馈」，但交接瞬间陀螺阻尼
       (KD*wz 在 300 dps 时有 90 rpm) 会压过 KP*err，命令瞬间反向，车身抖一下
       再慢慢回来，比慢转更难看出停稳。宁慢勿抖。 */
    u = (s_turn_kp * err) + (s_turn_ki * s_turn_i) +
        (s_turn_kd * imu->gyro_dps[2]);
    u = clampf(u, -s_turn_rpm, s_turn_rpm);

    /* 末段死区：小于电机能稳住的差速就干脆不动，等积分把它顶过门槛再来一下。
       留着这点命令只会让右轮（比左轮省力）单独爬，车身被慢慢带偏。 */
    if (fabsf(u) < s_turn_db) {
        u = 0.0f;
    }
    control_apply_targets(u, -u);

    if (fabsf(err) < s_turn_tol) {
        if (s_turn_settle < 255U) {
            s_turn_settle++;
        }
        if (s_turn_settle >= TRACK_TURN_SETTLE) {
            control_apply_targets(0.0f, 0.0f);
            return 1U;
        }
    } else {
        s_turn_settle = 0U;
    }
    return 0U;
}

float track_turn_err_deg(void)
{
    const imu_data_t *imu = imu_get();

    if (imu == 0) {
        return 0.0f;
    }
    return track_wrap180(imu->yaw - s_turn_target_yaw);
}

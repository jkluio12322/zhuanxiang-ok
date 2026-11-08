#include "control.h"
#include "config.h"
#include "state.h"
#include "motion.h"
#include "sched.h"
#include "main.h"
#include <math.h>

static Pid s_pid_left;
static Pid s_pid_right;

/* 目标斜坡状态：命令从 0 按 SPEED_RAMP_RPM_PER_S 爬升，
   起步瞬间不会把 100 RPM 的误差整个丢给 PI（那一下等于满占空比冲击）。 */
static float s_cmd_left = 0.0f;
static float s_cmd_right = 0.0f;
static uint8_t s_stop_n_left = 0U;    /* 停车锁存确认拍数 */
static uint8_t s_stop_n_right = 0U;

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

static float limit_target(float target)
{
    return clampf(target, -SPEED_TARGET_LIMIT_RPM, SPEED_TARGET_LIMIT_RPM);
}

static float ramp_step(float cmd, float target, float step)
{
    float d = target - cmd;

    if (d > step) {
        d = step;
    } else if (d < -step) {
        d = -step;
    }
    return cmd + d;
}

void control_init(void)
{
    pid_init(&s_pid_left, SPEED_PID_KP, SPEED_PID_KI,
             -SPEED_PID_OUT_MAX, SPEED_PID_OUT_MAX);
    pid_init(&s_pid_right, SPEED_PID_KP, SPEED_PID_KI,
             -SPEED_PID_OUT_MAX, SPEED_PID_OUT_MAX);
    s_cmd_left = 0.0f;
    s_cmd_right = 0.0f;
}

void control_hard_stop(void)
{
    AppState *st = state();

    st->target_left_rpm = 0.0f;
    st->target_right_rpm = 0.0f;
    s_cmd_left = 0.0f;
    s_cmd_right = 0.0f;
    pid_reset(&s_pid_left);
    pid_reset(&s_pid_right);
    st->pwm_left = 0.0f;
    st->pwm_right = 0.0f;
    motion_stop();
}

void control_get_gains(float *kp, float *ki)
{
    if (kp != 0) {
        *kp = s_pid_left.kp;
    }
    if (ki != 0) {
        *ki = s_pid_left.ki;
    }
}

void control_set_gains(float kp, float ki)
{
    pid_set_gains(&s_pid_left, kp, ki);
    pid_set_gains(&s_pid_right, kp, ki);
}

void control_apply_targets(float left_rpm, float right_rpm)
{
    AppState *st = state();

    /* 12 个周期：两个 float 必须在同一段临界区里落地，
       否则 2 ms 采样拍可能读到「左新右旧」的一对目标，差速瞬间偏一格。 */
    __disable_irq();
    st->target_left_rpm = left_rpm;
    st->target_right_rpm = right_rpm;
    __enable_irq();
}

void control_set_manual(float left_rpm, float right_rpm)
{
    AppState *st = state();

    __disable_irq();
    st->mode = APP_MODE_MANUAL;
    st->stop_reason = STOP_NONE;
    st->target_left_rpm = left_rpm;
    st->target_right_rpm = right_rpm;
    __enable_irq();
}

/* 速度环本体。每 2 ms 一拍：
   ① 取自洽快照（seqlock，读不到就退回上一份）
   ② 目标斜坡 + 单轮防爬行
   ③ 增量 PI 累加到占空比
   ④ 死区前馈（可选）
   ⑤ 输出到电机，并用 DWT 记录本拍耗时 */
void control_speed_task(uint32_t dt_us)
{
    AppState *st = state();
    MotionView_t mv;
    float dt_s;
    float target_left;
    float target_right;
    float step;
    float pwm_left;
    float pwm_right;
    uint32_t t0;
    uint32_t elapsed;

    t0 = DWT->CYCCNT;

    dt_s = (float)dt_us / 1000000.0f;
    if (dt_s <= 0.0f) {
        dt_s = (float)TASK_SAMPLE_MS / 1000.0f;   /* 首拍兜底 */
    }

    (void)motion_snapshot(&mv);
    st->actual_left_rpm = mv.left_rpm;
    st->actual_right_rpm = mv.right_rpm;

    if (st->mode == APP_MODE_IDLE) {
        s_stop_n_left = 0U;
        s_stop_n_right = 0U;
        pid_reset(&s_pid_left);
        pid_reset(&s_pid_right);
        s_cmd_left = 0.0f;
        s_cmd_right = 0.0f;
        st->cmd_left_rpm = 0.0f;
        st->cmd_right_rpm = 0.0f;
        motion_stop();
        st->pwm_left = 0.0f;
        st->pwm_right = 0.0f;
        return;
    }

    target_left = limit_target(st->target_left_rpm);
    target_right = limit_target(st->target_right_rpm);

    /* 低于电机能稳住的最小转速就直接当 0：实测 1 rpm 的命令会让轮子跑成 3.2 rpm，
       且右轮比左轮快 1.01~1.17 倍，两轮一停一转就把车身慢慢带歪。
       注意死区作用在「目标」上，不能作用在斜坡后的命令上——斜坡每拍才走 1 rpm，
       在命令上判死区会让它永远爬不出 0。 */
    if (fabsf(target_left) < SPEED_MIN_TARGET_RPM) {
        target_left = 0.0f;
    }
    if (fabsf(target_right) < SPEED_MIN_TARGET_RPM) {
        target_right = 0.0f;
    }

    step = SPEED_RAMP_RPM_PER_S * dt_s;
    if (step < 1.0f) {
        step = 1.0f;         /* 每拍至少走 1 RPM，否则极短 dt 会把斜坡拖死 */
    }
    s_cmd_left = ramp_step(s_cmd_left, target_left, step);
    s_cmd_right = ramp_step(s_cmd_right, target_right, step);
    st->cmd_left_rpm = s_cmd_left;
    st->cmd_right_rpm = s_cmd_right;


    /* 命令回到 0 且轮子确实停了，就把积分状态清掉：
       增量式的「状态即输出」，不清会在停车间隙一点点累起来，起步第一拍就窜。 */
    if ((s_cmd_left == 0.0f) && (fabsf(mv.left_rpm) < 2.0f)) {
        pid_reset(&s_pid_left);
    }
    if ((s_cmd_right == 0.0f) && (fabsf(mv.right_rpm) < 2.0f)) {
        pid_reset(&s_pid_right);
    }

    /* 停稳锁存：目标为 0 且实际已降到电机稳不住的转速以下，就锁定为「停住」。
       锁定期间不再跑 PI、直接断 PWM 并上刹车（short brake 把绕组短接），
       轮子不会在 0~4 rpm 之间来回蠕动；目标一旦非 0 立刻解锁。 */
    if ((s_cmd_left == 0.0f) && (fabsf(mv.left_rpm) < SPEED_STOP_RPM)) {
        if (s_stop_n_left < 3U) {
            s_stop_n_left++;
        }
    } else {
        s_stop_n_left = 0U;
    }
    if ((s_cmd_right == 0.0f) && (fabsf(mv.right_rpm) < SPEED_STOP_RPM)) {
        if (s_stop_n_right < 3U) {
            s_stop_n_right++;
        }
    } else {
        s_stop_n_right = 0U;
    }

    if ((s_stop_n_left >= 3U) && (s_stop_n_right >= 3U)) {
        pid_reset(&s_pid_left);
        pid_reset(&s_pid_right);
        st->pwm_left = 0.0f;
        st->pwm_right = 0.0f;
        motion_stop();      /* 走 MOTOR_BRAKE_ON_STOP：两根 IN 同高 = 短接制动 */
        return;
    }

    pwm_left = pid_update(&s_pid_left, s_cmd_left, mv.left_rpm, dt_s);
    pwm_right = pid_update(&s_pid_right, s_cmd_right, mv.right_rpm, dt_s);

    /* 死区前馈：电机在低占空比下转不动，PI 要花很久才积够。
       填了实测值就把它直接加在输出上（不进积分），起转立刻有响应。 */
    if (SPEED_DEADZONE_PCT > 0.0f) {
        if (s_cmd_left > SPEED_MIN_TARGET_RPM) {
            pwm_left += SPEED_DEADZONE_PCT;
        } else if (s_cmd_left < -SPEED_MIN_TARGET_RPM) {
            pwm_left -= SPEED_DEADZONE_PCT;
        }
        if (s_cmd_right > SPEED_MIN_TARGET_RPM) {
            pwm_right += SPEED_DEADZONE_PCT;
        } else if (s_cmd_right < -SPEED_MIN_TARGET_RPM) {
            pwm_right -= SPEED_DEADZONE_PCT;
        }
        pwm_left = clampf(pwm_left, -SPEED_PID_OUT_MAX, SPEED_PID_OUT_MAX);
        pwm_right = clampf(pwm_right, -SPEED_PID_OUT_MAX, SPEED_PID_OUT_MAX);
    }

    st->pwm_left = pwm_left;
    st->pwm_right = pwm_right;
    motion_apply(pwm_left, pwm_right);

    elapsed = (uint32_t)(DWT->CYCCNT - t0) / (SystemCoreClock / 1000000UL);
    st->speed_exec_us = elapsed;
    if (elapsed > st->speed_exec_max_us) {
        st->speed_exec_max_us = elapsed;
    }
}

/* 兼容入口：速度环留在 5 ms 控制拍时由 sched 调用。 */
void control_task(uint32_t dt_us)
{
    control_speed_task(dt_us);
}

const Pid *control_pid_left(void)
{
    return &s_pid_left;
}

const Pid *control_pid_right(void)
{
    return &s_pid_right;
}

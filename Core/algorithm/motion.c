/* 底盘运动：编码器测速、时间加权测速窗、电机输出。本模块是应用层里唯一直接调电机驱动的地方，
   把左右轮 PWM 百分比翻译成 TB6612 的方向 + PWM。
   硬件约定按实车接线写死，不要照文档「理顺」：encoderRight 取 TIM3、encoderLeft 取 TIM2（与
   README 引脚表相反）；motor_a_set 驱动物理右轮、B 驱动物理左轮（B 轮正反转对调在 board.h
   宏里，不在 motor.c）。改动前先架空车轮验证轮向与编码器符号。 */

#include "motion.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "main.h"
#include "state.h"
#include <math.h>

/* 编码器对象只在本文件里定义，外部通过 motion_view() 读滤波后的速度。 */
static Encoder encoderLeft;
static Encoder encoderRight;

static MotionView_t motionView;      /* 采样拍内部工作副本，只有 TIM4 中断写 */
static MotionView_t s_pub;           /* 已发布快照，seqlock 保护，任何上下文可读 */
static MotionView_t s_last_good;     /* 上一份确认无撕裂的快照，读失败时兜底 */
static volatile uint32_t s_seq = 0U; /* 偶数=稳定，奇数=正在发布 */

/* 运行期可改的四路极性开关（现场翻转）。
   上电初值取自 config.h，台架量出正确组合后再把宏改成同样值固化。 */
static int8_t signMotorLeft  = MOTOR_LEFT_SIGN;
static int8_t signMotorRight = MOTOR_RIGHT_SIGN;
static int8_t signEncoderLeft  = ENCODER_LEFT_SIGN;
static int8_t signEncoderRight = ENCODER_RIGHT_SIGN;

/* 跨界样本的定点倍数：窗口里的脉冲和按 1/8 个脉冲为单位累加。
   直接整数截断，每个窗口会系统性少算 0~1 个脉冲（55 RPM / 45 脉冲时约 2%）；
   放大 8 倍后量化误差 <= 1/8 脉冲（约 0.3%），全程仍是整数运算。 */
#define SPEED_SUBCOUNT  (8)

/* 测速样本：只存「本次与上次采样之间的真实间隔」，不存绝对时间戳。
   窗口靠累加 seg_us 到 50 ms 为止，边界样本按时间比例插值 —— 这是把
   ±1 count 量化误差摊到窗口边界上的关键，稳态误差从约 0.9% 降到约 0.45%。 */
typedef struct {
    int16_t  delta_left;
    int16_t  delta_right;
    uint32_t seg_us;
} SpeedSample;

static SpeedSample s_speedWin[SPEED_SAMPLE_MAX];
static uint8_t s_winHead;      /* 下一个写入位置 */
static uint8_t s_winCount;     /* 有效样本数 */
static uint32_t s_windowUs = SPEED_WINDOW_US;

/* 速度一阶低通状态。alpha 按实测 dt 归一，换采样周期不用重算系数。 */
static float s_filterLeft = 0.0f;
static float s_filterRight = 0.0f;
static float s_filterHz = SPEED_FILTER_HZ;
static uint8_t s_filterInit = 0U;

/* 累计里程（m）：按滤波后的轮速对时间积分，左右取平均、取绝对值。
   倒车时里程继续增加，这样"走过多少路"与方向无关。 */
static float s_distance_m = 0.0f;

/* 更新编码器位置，并用时间加权滑动窗降低采样量化误差。
   now_us 目前只用于对外契约一致性（窗口按 seg_us 累加），保留以便将来按绝对时间裁剪。 */
void motion_update(uint32_t now_us, uint32_t dt_us)
{
    int16_t left_increment;
    int16_t right_increment;
    float counts_per_revolution;
    uint32_t acc;
    int32_t sum_left;      /* 单位 1/SPEED_SUBCOUNT 个脉冲 */
    int32_t sum_right;
    uint32_t abs_counts;   /* 窗内脉冲绝对数之和，用来判断是不是该扩窗 */
    uint8_t k;
    uint8_t idx;

    (void)now_us;

    if (dt_us == 0U) {
        dt_us = 1U;
    }

    counts_per_revolution =
        (float)ENCODER_MULTIPLE * ENCODER_REDUCTION_RATIO * ENCODER_PPR;

    encoder_update(&encoderRight, dt_us, TIM3);
    encoder_update(&encoderLeft, dt_us, TIM2);

    motionView.left_rpm_raw = (float)signEncoderLeft * encoderLeft.velocity.angular;
    motionView.right_rpm_raw = (float)signEncoderRight * encoderRight.velocity.angular;
    motionView.left_angle_deg = (float)signEncoderLeft * (float)encoderLeft.position.angle;
    motionView.right_angle_deg = (float)signEncoderRight * (float)encoderRight.position.angle;

    left_increment = (int16_t)(signEncoderLeft * encoderLeft.counter.count_increment);
    right_increment = (int16_t)(signEncoderRight * encoderRight.counter.count_increment);
    motionView.left_encoder_delta = left_increment;
    motionView.right_encoder_delta = right_increment;

    s_speedWin[s_winHead].delta_left = left_increment;
    s_speedWin[s_winHead].delta_right = right_increment;
    s_speedWin[s_winHead].seg_us = dt_us;
    s_winHead++;
    if (s_winHead >= SPEED_SAMPLE_MAX) {
        s_winHead = 0U;
    }
    if (s_winCount < SPEED_SAMPLE_MAX) {
        s_winCount++;
    }

    /* 从最新样本往回累加，两条停止条件：
       ① 时间够了基础窗、且窗内脉冲数达到下限 → 正常停；
       ② 脉冲太少（低速），继续往更早的样本扩窗，直到计数够或撞上硬顶。
       跨过基础窗边界的那一笔按时间比例截断 —— 这是把 ±1 count 的量化误差
       摊到窗口边界上的关键，不截断就会周期性多算/少算一个脉冲。 */
    acc = 0U;
    sum_left = 0;
    sum_right = 0;
    abs_counts = 0U;
    idx = (s_winHead == 0U) ? (uint8_t)(SPEED_SAMPLE_MAX - 1U) : (uint8_t)(s_winHead - 1U);

    /* 累加全用整数：低速时这一圈要把 100 个历史样本全走一遍，浮点在这里
       每次都要走软件模拟（M3 无 FPU），实测能吃掉 500 us 的中断时间。
       跨界样本按整数比例截断，误差 < 1 个脉冲，够用。 */
    for (k = 0U; k < s_winCount; k++) {
        const SpeedSample *smp = &s_speedWin[idx];
        int32_t d_left = (int32_t)smp->delta_left;
        int32_t d_right = (int32_t)smp->delta_right;
        uint32_t seg_us = (smp->seg_us == 0U) ? 1U : smp->seg_us;
        uint8_t want_more;

        if ((acc < s_windowUs) && ((acc + seg_us) > s_windowUs)) {
            /* 跨界样本只取前 keep_us。先放大 SPEED_SUBCOUNT 倍再做整数除法：
               直接截断会把小数部分丢掉，每个窗口系统性少算最多 1 个脉冲（55 RPM 下 ~2%），
               放大 8 倍后量化误差降到 1/8 脉冲（<0.3%），代价只是一次乘法。
               溢出核算：|d| <= 2000、keep_us <= seg_us <= 5000 → 2000*5000*8 = 8e7 < 2^31。 */
            uint32_t keep_us = s_windowUs - acc;              /* 这一段里只要前 keep_us */
            sum_left += (d_left * (int32_t)keep_us * (int32_t)SPEED_SUBCOUNT) / (int32_t)seg_us;
            sum_right += (d_right * (int32_t)keep_us * (int32_t)SPEED_SUBCOUNT) / (int32_t)seg_us;
            abs_counts += (uint32_t)(((d_left < 0) ? -d_left : d_left) * (int32_t)keep_us)
                        / seg_us;
            abs_counts += (uint32_t)(((d_right < 0) ? -d_right : d_right) * (int32_t)keep_us)
                        / seg_us;
            acc = s_windowUs;
        } else {
            sum_left += d_left * (int32_t)SPEED_SUBCOUNT;
            sum_right += d_right * (int32_t)SPEED_SUBCOUNT;
            abs_counts += (uint32_t)((d_left < 0) ? -d_left : d_left);
            abs_counts += (uint32_t)((d_right < 0) ? -d_right : d_right);
            acc += seg_us;
        }

        idx = (idx == 0U) ? (uint8_t)(SPEED_SAMPLE_MAX - 1U) : (uint8_t)(idx - 1U);

        /* 基础窗已经填满、而且一个脉冲都没有：RPM 必然是 0，继续按低速扩窗
           只是白跑一百次循环。静止时这一句把采样中断从 134 us 拉回十几 us。 */
        if ((acc >= s_windowUs) && (abs_counts == 0U)) {
            break;
        }

        want_more = (uint8_t)(((acc < s_windowUs) || (abs_counts < SPEED_MIN_COUNTS))
                              && (acc < SPEED_WINDOW_MAX_US));
        if (want_more == 0U) {
            break;
        }
    }

    motionView.window_us = acc;   /* 实际用到的窗口长度，低速时会大于基础窗 */

    /* 样本还没攒够一个最短窗就先不更新，避免冷启动瞬间给出离谱的高转速 */
    if (acc >= SPEED_WINDOW_MIN_US) {
        /* sum 的单位是 1/SPEED_SUBCOUNT 个脉冲，所以常数要除回去（60e6 / 8 = 7.5e6） */
        float raw_left = (float)sum_left * (60000000.0f / (float)SPEED_SUBCOUNT)
                       / (counts_per_revolution * (float)acc);
        float raw_right = (float)sum_right * (60000000.0f / (float)SPEED_SUBCOUNT)
                        / (counts_per_revolution * (float)acc);
        float alpha = ((float)dt_us * 6.2831853f * s_filterHz) / 1000000.0f;

        if ((alpha <= 0.0f) || (alpha >= 1.0f)) {
            /* 0 = 不滤波；>=1 说明截止频率高到一阶低通已无意义，直接跟随 */
            s_filterLeft = raw_left;
            s_filterRight = raw_right;
            s_filterInit = 1U;
        } else {
            if (s_filterInit == 0U) {
                s_filterLeft = raw_left;
                s_filterRight = raw_right;
                s_filterInit = 1U;
            }
            s_filterLeft += alpha * (raw_left - s_filterLeft);
            s_filterRight += alpha * (raw_right - s_filterRight);
        }

        motionView.left_rpm = s_filterLeft;
        motionView.right_rpm = s_filterRight;
    }

    /* 里程积分：v[m/s] = rpm * 2πR / 60，用本拍实测 dt 积分。
       不假设 2.000 ms，主循环卡顿或中断被抢占时不会系统性少算。 */
    {
        float v_avg = 0.5f * (fabsf(motionView.left_rpm) + fabsf(motionView.right_rpm));
        float dt_s = (float)dt_us * 0.000001f;
        s_distance_m += v_avg * (6.2831853f * ENCODER_WHEEL_RADIUS_M / 60.0f) * dt_s;
        motionView.distance_m = s_distance_m;
    }

    /* 发布：奇数窗口内拷贝，偶数表示稳定。控制拍（低一级优先级）随时可能被这里
       打断半途读走，靠它自己重读序列号判断是否读到半截数据。 */
    s_seq++;
    __DMB();
    s_pub = motionView;
    __DMB();
    s_seq++;
}

uint8_t motion_set_measure_window_ms(uint16_t window_ms)
{
    uint32_t us = (uint32_t)window_ms * 1000UL;

    if ((us < SPEED_WINDOW_MIN_US) || (us > SPEED_WINDOW_MAX_US)) {
        return 0U;
    }
    s_windowUs = us;
    motionView.window_us = us;
    s_winHead = 0U;
    s_winCount = 0U;
    s_filterInit = 0U;   /* 窗口一变，旧滤波状态就没意义了 */
    return 1U;
}

uint16_t motion_get_measure_window_ms(void)
{
    return (uint16_t)(s_windowUs / 1000UL);
}

/* 改低通截止频率。0 表示关掉滤波，直接输出窗口原始值。 */
uint8_t motion_set_filter_hz(uint16_t hz)
{
    if (hz > 100U) {
        return 0U;
    }
    s_filterHz = (float)hz;
    s_filterInit = 0U;
    return 1U;
}

float motion_distance_m(void)
{
    return s_distance_m;
}

uint16_t motion_get_filter_hz(void)
{
    return (uint16_t)(s_filterHz + 0.5f);
}

/* 限幅后把左右轮 PWM 输出到电机。限幅 ±MOTOR_OUTPUT（±100）。禁止反转应通过 PID 的 out_min
   实现，不要在这里传负数。最后写入视图，供遥测取本周期实际输出。 */
void motion_apply(float left_pwm, float right_pwm)
{
    if (left_pwm > (float)MOTOR_OUTPUT_MAX_PERCENT) {
        left_pwm = (float)MOTOR_OUTPUT_MAX_PERCENT;
    }
    if (left_pwm < (float)MOTOR_OUTPUT_MIN_PERCENT) {
        left_pwm = (float)MOTOR_OUTPUT_MIN_PERCENT;
    }
    if (right_pwm > (float)MOTOR_OUTPUT_MAX_PERCENT) {
        right_pwm = (float)MOTOR_OUTPUT_MAX_PERCENT;
    }
    if (right_pwm < (float)MOTOR_OUTPUT_MIN_PERCENT) {
        right_pwm = (float)MOTOR_OUTPUT_MIN_PERCENT;
    }

    /* 交叉映射：motor_a_set 驱动物理右轮，motor_b_set 驱动物理左轮。
       MOTOR_x_SIGN 统一「命令为正 = 车往前」，电机线接反时置 -1。 */
    motor_a_set(right_pwm * (float)signMotorRight);
    motor_b_set(left_pwm * (float)signMotorLeft);

    motionView.left_pwm = left_pwm;
    motionView.right_pwm = right_pwm;
    /* 不在这里发布：发布只由采样拍做，否则控制拍与采样拍会同时动序列号。 */
}

/* 取只读运动视图。 */
const MotionView_t *motion_view(void)
{
    return &s_pub;
}

/* 取一份自洽快照。读者可能在拷贝途中被采样拍打断，所以读前读后各取一次序列号：
   两次相等且为偶数，说明这中间没有发布动作，拷到的字段互相自洽。 */
uint8_t motion_snapshot(MotionView_t *out)
{
    uint32_t s1;
    uint32_t s2;
    uint8_t tries;

    if (out == 0) {
        return 0U;
    }

    for (tries = 0U; tries < 3U; tries++) {
        s1 = s_seq;
        if ((s1 & 1U) != 0U) {
            continue;               /* 正处在发布中途，立刻重试 */
        }
        __DMB();
        *out = s_pub;
        __DMB();
        s2 = s_seq;
        if (s1 == s2) {
            s_last_good = *out;
            return 1U;
        }
    }

    *out = s_last_good;
    state()->win_retry++;
    return 0U;
}

/* 初始化编码器对象与速度窗口。须在 encoder_init() 之后、中断放行之前调。 */
void motion_init(void)
{
    const Parameter wheelParameter = {
        .multiple = ENCODER_MULTIPLE,
        .reduction_ratio = ENCODER_REDUCTION_RATIO,
        .ppr = ENCODER_PPR,
        .r = ENCODER_WHEEL_RADIUS_M
    };

    encoder_setup(&encoderLeft, wheelParameter);
    encoder_setup(&encoderRight, wheelParameter);

    for (uint8_t i = 0U; i < SPEED_SAMPLE_MAX; i++) {
        s_speedWin[i].delta_left = 0;
        s_speedWin[i].delta_right = 0;
        s_speedWin[i].seg_us = 0U;
    }
    s_winHead = 0U;
    s_winCount = 0U;
    s_windowUs = SPEED_WINDOW_US;

    motionView.left_rpm = 0.0f;
    motionView.right_rpm = 0.0f;
    motionView.left_rpm_raw = 0.0f;
    motionView.right_rpm_raw = 0.0f;
    motionView.left_angle_deg = 0.0f;
    motionView.right_angle_deg = 0.0f;
    motionView.left_encoder_delta = 0;
    motionView.right_encoder_delta = 0;
    motionView.left_pwm = 0.0f;
    motionView.right_pwm = 0.0f;
    motionView.distance_m = 0.0f;
    motionView.window_us = s_windowUs;
    s_distance_m = 0.0f;

    s_pub = motionView;
    s_last_good = motionView;
    s_seq = 0U;

    s_filterLeft = 0.0f;
    s_filterRight = 0.0f;
    s_filterHz = SPEED_FILTER_HZ;
    s_filterInit = 0U;
}

void motion_reset_position(void)
{
    encoderLeft.counter.count_total = 0;
    encoderLeft.position.rotations = 0.0;
    encoderLeft.position.distance = 0.0;
    encoderLeft.position.angle = 0.0;

    encoderRight.counter.count_total = 0;
    encoderRight.position.rotations = 0.0;
    encoderRight.position.distance = 0.0;
    encoderRight.position.angle = 0.0;

    motionView.distance_m = 0.0f;
    s_distance_m = 0.0f;

    motionView.left_angle_deg = 0.0f;
    motionView.right_angle_deg = 0.0f;
}

/* 立即停车：两路输出归零，直接调驱动层而非走 Apply（无条件停止）。不复位编码器/滤波窗口，
   让重启时速度反馈保持连续，避免复位瞬间反馈跳变引起速度环冲击。 */
void motion_stop(void)
{
    motor_a_set(0);
    motor_b_set(0);

    motionView.left_pwm = 0.0f;
    motionView.right_pwm = 0.0f;
}

/* 现场翻转极性：各翻一次。写 int8 在 Cortex-M3 上是原子的，
   控制拍（TIM7 中断）读到的不是半截值。翻转后闭环要重新建立，调用方负责复位速度环。 */
void motion_flip_motors(void)
{
    signMotorLeft  = (int8_t)(-signMotorLeft);
    signMotorRight = (int8_t)(-signMotorRight);
}

void motion_flip_encoders(void)
{
    signEncoderLeft  = (int8_t)(-signEncoderLeft);
    signEncoderRight = (int8_t)(-signEncoderRight);
    motion_reset_position();
}

/* 打包读出四路开关，供命令行回显。 */
void motion_get_signs(int8_t *motorLeft, int8_t *motorRight,
                      int8_t *encoderLeft, int8_t *encoderRight)
{
    if (motorLeft != 0)   *motorLeft = signMotorLeft;
    if (motorRight != 0)  *motorRight = signMotorRight;
    if (encoderLeft != 0) *encoderLeft = signEncoderLeft;
    if (encoderRight != 0)*encoderRight = signEncoderRight;
}

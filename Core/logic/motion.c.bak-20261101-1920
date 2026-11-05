/* 底盘运动：编码器测速、中值滤波、电机输出。本模块是应用层里唯一直接调电机驱动的地方，
   把左右轮 PWM 百分比翻译成 TB6612 的方向 + PWM。
   硬件约定按实车接线写死，不要照文档「理顺」：encoderRight 取 TIM3、encoderLeft 取 TIM2（与
   README 引脚表相反）；motor_a_set 驱动物理右轮、B 驱动物理左轮（B 轮正反转对调在 board.h
   宏里，不在 motor.c）。改动前先架空车轮验证轮向与编码器符号。 */

#include "motion.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"

/* 编码器对象只在本文件里定义，外部通过 motion_view() 读滤波后的速度。 */
static Encoder encoderLeft;
static Encoder encoderRight;

static MotionView_t motionView;

/* 中值滤波的采样窗口。 */
static float   sampleLeft[ENCODER_FILTER_WINDOW];
static float   sampleRight[ENCODER_FILTER_WINDOW];
static uint8_t sampleIndex = 0U;

static float median3(float a, float b, float c);

/* 读一次编码器并做中值滤波。50 ms 控制任务里调，全工程只有这里做速度滤波；须跟固定周期跑。 */
void motion_update(void)
{
    /* encoderRight 取 TIM3、encoderLeft 取 TIM2，与 README 引脚表相反，以代码为准。 */
    encoder_update(&encoderRight, CONTROL_PERIOD_MS, TIM3);
    encoder_update(&encoderLeft, CONTROL_PERIOD_MS, TIM2);

    /* 原始值直接暴露给遥测，用于对照滤波前后的差别。 */
    motionView.left_rpm_raw = encoderLeft.velocity.angular;
    motionView.right_rpm_raw = encoderRight.velocity.angular;

    sampleLeft[sampleIndex] = motionView.left_rpm_raw;
    sampleRight[sampleIndex] = motionView.right_rpm_raw;

    motionView.left_rpm = median3(sampleLeft[0], sampleLeft[1], sampleLeft[2]);
    motionView.right_rpm = median3(sampleRight[0], sampleRight[1], sampleRight[2]);

    sampleIndex++;
    if (sampleIndex >= ENCODER_FILTER_WINDOW) {
        sampleIndex = 0U;
    }
}

/* 限幅后把左右轮 PWM 输出到电机。限幅 ±MOTOR_OUTPUT（±100）。禁止反转应通过 PID 的 out_min
   实现，不要在这里传负数。最后写入视图，供遥测取本周期实际输出。 */
void motion_apply(int16_t left_pwm, int16_t right_pwm)
{
    if (left_pwm > MOTOR_OUTPUT_MAX_PERCENT) {
        left_pwm = MOTOR_OUTPUT_MAX_PERCENT;
    }
    if (left_pwm < MOTOR_OUTPUT_MIN_PERCENT) {
        left_pwm = MOTOR_OUTPUT_MIN_PERCENT;
    }
    if (right_pwm > MOTOR_OUTPUT_MAX_PERCENT) {
        right_pwm = MOTOR_OUTPUT_MAX_PERCENT;
    }
    if (right_pwm < MOTOR_OUTPUT_MIN_PERCENT) {
        right_pwm = MOTOR_OUTPUT_MIN_PERCENT;
    }

    /* 交叉映射：motor_a_set 驱动物理右轮，motor_b_set 驱动物理左轮。 */
    motor_a_set(right_pwm);
    motor_b_set(left_pwm);

    motionView.left_pwm = left_pwm;
    motionView.right_pwm = right_pwm;
}

/* 取三数中值，剔除单点尖峰（换向瞬间、堵转抖动），不是平滑。固定接受 3 参，
   窗口长度须恒为 3——ENCODER_FILTER_WINDOW 改成别的值会让窗口名不副实。 */
static float median3(float a, float b, float c)
{
    if (((a >= b) && (a <= c)) || ((a >= c) && (a <= b))) return a;
    if (((b >= a) && (b <= c)) || ((b >= c) && (b <= a))) return b;
    return c;
}

/* 取只读运动视图。 */
const MotionView_t *motion_view(void)
{
    return &motionView;
}

/* 初始化两个编码器对象与滤波窗口。须在 encoder_init() 之后、中断放行之前调。
   上电后前 2 个周期中值含 0，滤波速度偏低，约 100 ms 后进稳态，速度环反馈偏小通常无影响。 */
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

    for (uint8_t i = 0U; i < ENCODER_FILTER_WINDOW; i++) {
        sampleLeft[i] = 0.0f;
        sampleRight[i] = 0.0f;
    }
    sampleIndex = 0U;

    motionView.left_rpm = 0.0f;
    motionView.right_rpm = 0.0f;
    motionView.left_rpm_raw = 0.0f;
    motionView.right_rpm_raw = 0.0f;
    motionView.left_pwm = 0;
    motionView.right_pwm = 0;
}

/* 立即停车：两路输出归零，直接调驱动层而非走 Apply（无条件停止）。不复位编码器/滤波窗口，
   让重启时速度反馈保持连续，避免复位瞬间反馈跳变引起速度环冲击。 */
void motion_stop(void)
{
    motor_a_set(0);
    motor_b_set(0);

    motionView.left_pwm = 0;
    motionView.right_pwm = 0;
}

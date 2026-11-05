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

/* 运行期可改的四路极性开关（命令行 mot / enc 现场翻转）。
   上电初值取自 config.h，台架量出正确组合后再把宏改成同样值固化。 */
static int8_t signMotorLeft  = MOTOR_LEFT_SIGN;
static int8_t signMotorRight = MOTOR_RIGHT_SIGN;
static int8_t signEncoderLeft  = ENCODER_LEFT_SIGN;
static int8_t signEncoderRight = ENCODER_RIGHT_SIGN;

/* 5 ms 编码器增量滑窗，合成约 50 ms 速度测量。 */
static int16_t pulseWindowLeft[SPEED_MEASURE_WINDOW_MAX_MS];
static int16_t pulseWindowRight[SPEED_MEASURE_WINDOW_MAX_MS];
static uint16_t pulseWindowElapsed[SPEED_MEASURE_WINDOW_MAX_MS];
static uint8_t pulseWindowHead;
static uint8_t pulseWindowCount;
static uint8_t measureWindowMs = SPEED_MEASURE_WINDOW_MS;

/* 更新编码器位置，并用 50 ms 滑动计数窗降低 5 ms 速度量化。 */
void motion_update(uint16_t elapsed_ms)
{
    uint8_t index;
    uint8_t sample_count;
    int16_t left_increment;
    int16_t right_increment;
    int32_t left_sum = 0;
    int32_t right_sum = 0;
    uint32_t elapsed_sum = 0U;
    float counts_per_revolution =
        (float)ENCODER_MULTIPLE * ENCODER_REDUCTION_RATIO * ENCODER_PPR;

    if (elapsed_ms == 0U) {
        elapsed_ms = 1U;
    }

    encoder_update(&encoderRight, elapsed_ms, TIM3);
    encoder_update(&encoderLeft, elapsed_ms, TIM2);
    motionView.left_rpm_raw = (float)signEncoderLeft * encoderLeft.velocity.angular;
    motionView.right_rpm_raw = (float)signEncoderRight * encoderRight.velocity.angular;
    motionView.left_angle_deg = (float)signEncoderLeft * (float)encoderLeft.position.angle;
    motionView.right_angle_deg = (float)signEncoderRight * (float)encoderRight.position.angle;

    left_increment = (int16_t)(signEncoderLeft * encoderLeft.counter.count_increment);
    right_increment = (int16_t)(signEncoderRight * encoderRight.counter.count_increment);
    motionView.left_encoder_delta = left_increment;
    motionView.right_encoder_delta = right_increment;

    pulseWindowLeft[pulseWindowHead] = left_increment;
    pulseWindowRight[pulseWindowHead] = right_increment;
    pulseWindowElapsed[pulseWindowHead] = elapsed_ms;
    pulseWindowHead++;
    if (pulseWindowHead >= SPEED_MEASURE_WINDOW_MAX_MS) {
        pulseWindowHead = 0U;
    }
    if (pulseWindowCount < SPEED_MEASURE_WINDOW_MAX_MS) {
        pulseWindowCount++;
    }

    sample_count = pulseWindowCount;
    index = pulseWindowHead;
    while ((sample_count > 0U) && (elapsed_sum < measureWindowMs)) {
        if (index == 0U) {
            index = SPEED_MEASURE_WINDOW_MAX_MS;
        }
        index--;
        left_sum += pulseWindowLeft[index];
        right_sum += pulseWindowRight[index];
        elapsed_sum += pulseWindowElapsed[index];
        sample_count--;
    }

    if (elapsed_sum == 0U) {
        motionView.left_rpm = 0.0f;
        motionView.right_rpm = 0.0f;
    } else {
        motionView.left_rpm = (float)left_sum * 60000.0f
            / (counts_per_revolution * (float)elapsed_sum);
        motionView.right_rpm = (float)right_sum * 60000.0f
            / (counts_per_revolution * (float)elapsed_sum);
    }
}

uint8_t motion_set_measure_window_ms(uint8_t window_ms)
{
    if ((window_ms < SPEED_MEASURE_WINDOW_MIN_MS)
        || (window_ms > SPEED_MEASURE_WINDOW_MAX_CONFIG_MS)) {
        return 0U;
    }
    measureWindowMs = window_ms;
    pulseWindowHead = 0U;
    pulseWindowCount = 0U;
    return 1U;
}

uint8_t motion_get_measure_window_ms(void)
{
    return measureWindowMs;
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
}

/* 取只读运动视图。 */
const MotionView_t *motion_view(void)
{
    return &motionView;
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

    for (uint8_t i = 0U; i < SPEED_MEASURE_WINDOW_MAX_MS; i++) {
        pulseWindowLeft[i] = 0;
        pulseWindowRight[i] = 0;
        pulseWindowElapsed[i] = 0U;
    }
    pulseWindowHead = 0U;
    pulseWindowCount = 0U;
    measureWindowMs = SPEED_MEASURE_WINDOW_MS;

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

/* 现场翻转极性：命令行 mot / enc 各翻一次。写 int8 在 Cortex-M3 上是原子的，
   控制任务（TIM6 中断）读到的不是半截值。翻转后闭环要重新建立，调用方负责复位速度环。 */
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

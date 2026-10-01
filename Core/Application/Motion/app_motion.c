/**
 * @file    app_motion.c
 * @brief   底盘运动的实现。
 */

#include "app_motion.h"
#include "app_config.h"
#include "encoder.h"
#include "motor.h"

/* 编码器对象只在本文件里定义。 */
static Encoder encoderLeft;
static Encoder encoderRight;

static MotionView_t motionView;

/* 中值滤波的采样窗口。 */
static float   sampleLeft[ENCODER_FILTER_WINDOW];
static float   sampleRight[ENCODER_FILTER_WINDOW];
static uint8_t sampleIndex = 0U;

/* 取三个数的中值，用于剔除测速的单点尖峰。 */
static float median3(float a, float b, float c)
{
    if (((a >= b) && (a <= c)) || ((a >= c) && (a <= b))) return a;
    if (((b >= a) && (b <= c)) || ((b >= c) && (b <= a))) return b;
    return c;
}

const MotionView_t *AppMotion(void)
{
    return &motionView;
}

void AppMotion_Init(void)
{
    const Parameter wheelParameter = {
        .multiple = ENCODER_MULTIPLE,
        .reduction_ratio = ENCODER_REDUCTION_RATIO,
        .ppr = ENCODER_PPR,
        .r = ENCODER_WHEEL_RADIUS_M
    };

    initEncoder(&encoderLeft, wheelParameter);
    initEncoder(&encoderRight, wheelParameter);

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

void AppMotion_Update(void)
{
    /* 编码器号与定时器的对应关系照抄原 yingYongKongZhi.c:353-354，
       encoderRight 取 TIM3、encoderLeft 取 TIM2，与 README 的引脚表相反。 */
    updateEncoderLoopSimpleVersion(&encoderRight, APP_CONTROL_PERIOD_MS, TIM3);
    updateEncoderLoopSimpleVersion(&encoderLeft, APP_CONTROL_PERIOD_MS, TIM2);

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

void AppMotion_Apply(int16_t left_pwm, int16_t right_pwm)
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

    /* 交叉映射：Motor_SetSpeedA 驱动物理右轮，Motor_SetSpeedB 驱动物理左轮。
       照抄原 yingYongKongZhi.c:443-444，不要"理顺"。 */
    Motor_SetSpeedA(right_pwm);
    Motor_SetSpeedB(left_pwm);

    motionView.left_pwm = left_pwm;
    motionView.right_pwm = right_pwm;
}

void AppMotion_Stop(void)
{
    Motor_SetSpeedA(0);
    Motor_SetSpeedB(0);

    motionView.left_pwm = 0;
    motionView.right_pwm = 0;
}

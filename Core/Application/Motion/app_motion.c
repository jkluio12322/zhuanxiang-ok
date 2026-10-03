/**
 * @file    app_motion.c
 * @brief   底盘运动 — 编码器测速、中值滤波、电机输出。
 *
 * @note    本模块是**应用层里唯一直接调用电机驱动的地方**，负责把
 *          「左右轮 PWM 百分比」翻译成 TB6612 需要的方向 + PWM。
 *          驱动层的交叉映射与极性约定只在这里出现一次，别处一律用
 *          left / right 语义。
 *
 * @warning 两处硬件约定是按实车接线写死的，**不要照文档"理顺"**：
 * @code
 *   1) 编码器与定时器：encoderRight 取 TIM3、encoderLeft 取 TIM2
 *      ——与 README §2.1 早期的引脚表相反，以代码为准。
 *   2) 电机通道：Motor_SetSpeedA 驱动物理右轮、Motor_SetSpeedB 驱动物理左轮
 *      ——B 轮的 forward/reverse 对调发生在 board.h 的宏定义里，
 *        不是在 motor.c 的函数体里。
 * @endcode
 *          改动前先架空车轮验证轮向与编码器符号。
 *
 * @note    中值滤波：只用中值，**不用一阶低通 EMA**。原来的
 *          中值-3 + EMA(0.75/0.25) 群延迟约 200 ms（等于 4 个控制周期），
 *          是起步 judder 的成因之一；原始信号本身只有 1~2% 量化噪声
 *          （2464 计数/圈，50 ms 窗口下 30 RPM 有 61 个计数）。
 *          现在延迟降到约 50 ms（1 个周期）。
 */

#include "app_motion.h"
#include "app_config.h"
#include "encoder.h"
#include "motor.h"

/* 编码器对象只在本文件里定义，外部通过 AppMotion() 读滤波后的速度。 */
static Encoder encoderLeft;
static Encoder encoderRight;

static MotionView_t motionView;

/* 中值滤波的采样窗口。 */
static float   sampleLeft[ENCODER_FILTER_WINDOW];
static float   sampleRight[ENCODER_FILTER_WINDOW];
static uint8_t sampleIndex = 0U;

/**
 * @brief  取三个数的中值。
 * @param  a,b,c 三个输入值。
 * @return 三者中按大小排在中间的那个。
 * @note   与输入顺序无关，所以调用方不需要保证数组按时间排序。
 *         中值的作用是剔除单点尖峰（换向瞬间、堵转抖动），而不是平滑。
 * @warning 本函数固定接受 3 个参数，而窗口数组长度取的是 ENCODER_FILTER_WINDOW。
 *          两者当前都是 3 才成立；若把 ENCODER_FILTER_WINDOW 改成别的值，
 *          数组大小会变但这里仍然只取前三个元素，窗口长度就名不副实了。
 */
static float median3(float a, float b, float c)
{
    if (((a >= b) && (a <= c)) || ((a >= c) && (a <= b))) return a;
    if (((b >= a) && (b <= c)) || ((b >= c) && (b <= a))) return b;
    return c;
}

/**
 * @brief  获取只读运动视图。
 * @return 指向内部视图的指针，调用方只读。
 */
const MotionView_t *AppMotion(void)
{
    return &motionView;
}

/**
 * @brief  初始化两个编码器对象与滤波窗口。
 * @note   必须在 Encoder_Init_TIM_All() 之后调用（定时器要先启动），
 *         且在中断放行之前。
 * @note   滤波窗口清零；上电后前 2 个周期中值里含 0，滤波速度会偏低，
 *         约 100 ms 后进入稳态。这段时间速度环的反馈偏小，通常无影响。
 */
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

/**
 * @brief  读一次编码器并做中值滤波。
 * @note   在 50 ms 控制任务里调用；**全工程只有这里做速度滤波**。
 * @note   转速换算在驱动层完成，本函数只负责组织顺序与滤波。
 *         采样的固定周期由 APP_CONTROL_PERIOD_MS 决定，所以本函数必须
 *         跟着 50 ms 任务跑——挪到抖动的主循环里会让转速整体算错。
 */
void AppMotion_Update(void)
{
    /* 编码器号与定时器的对应关系照抄原 yingYongKongZhi.c:353-354，
       encoderRight 取 TIM3、encoderLeft 取 TIM2，与 README 的引脚表相反。 */
    updateEncoderLoopSimpleVersion(&encoderRight, APP_CONTROL_PERIOD_MS, TIM3);
    updateEncoderLoopSimpleVersion(&encoderLeft, APP_CONTROL_PERIOD_MS, TIM2);

    /* 原始值直接暴露给遥测，用于对照滤波前后的差别。 */
    motionView.left_rpm_raw = encoderLeft.velocity.angular;
    motionView.right_rpm_raw = encoderRight.velocity.angular;

    /* 写入环形窗口的当前位置。 */
    sampleLeft[sampleIndex] = motionView.left_rpm_raw;
    sampleRight[sampleIndex] = motionView.right_rpm_raw;

    motionView.left_rpm = median3(sampleLeft[0], sampleLeft[1], sampleLeft[2]);
    motionView.right_rpm = median3(sampleRight[0], sampleRight[1], sampleRight[2]);

    sampleIndex++;
    if (sampleIndex >= ENCODER_FILTER_WINDOW) {
        sampleIndex = 0U;
    }
}

/**
 * @brief  限幅后把左右轮 PWM 输出到电机。
 * @param  left_pwm  左轮 PWM 百分比，负值表示反转。
 * @param  right_pwm 右轮 PWM 百分比，负值表示反转。
 * @note   限幅范围是 MOTOR_OUTPUT_MIN_PERCENT ~ MOTOR_OUTPUT_MAX_PERCENT（±100）。
 *         调用方若需要「禁止反转」，应通过 PID 的 out_min 实现，
 *         而不是在这里传负数——见 app_speedloop.c 的说明。
 * @note   最后写入视图，供遥测取「本周期实际输出」。
 */
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

/**
 * @brief  立即停车：两路输出归零。
 * @note   直接调驱动层而不是走 AppMotion_Apply()，因为这里要的是
 *         无条件停止，不能被任何限幅或模式逻辑影响。
 * @note   停车**不**复位编码器和滤波窗口——重新启动时让速度反馈保持连续，
 *         避免复位瞬间反馈跳变引起速度环冲击。
 */
void AppMotion_Stop(void)
{
    Motor_SetSpeedA(0);
    Motor_SetSpeedB(0);

    motionView.left_pwm = 0;
    motionView.right_pwm = 0;
}

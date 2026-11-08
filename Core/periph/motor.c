/* TB6612 双路电机驱动：TIM1 CH1/CH2 输出 16kHz PWM。
   A 轮=TIM1_CH2(PE11) 方向 PF2/PF1，B 轮=TIM1_CH1(PE9) 方向 PF3/PF4；预分频 0、ARR 4499。 */

#include "motor.h"
#include "board.h"

extern TIM_HandleTypeDef htim1;

/* 单通道方向与 PWM 配置 */
typedef struct {
    GPIO_TypeDef *forwardPort;
    uint16_t forwardPin;
    GPIO_TypeDef *reversePort;
    uint16_t reversePin;
    uint32_t timChannel;
} Motor_ChannelConfig_t;

/* A 通道 = 物理右轮，PWM 用 TIM1_CH2 */
static const Motor_ChannelConfig_t motorChannelA = {
    MOTOR_A_FORWARD_GPIO_PORT,
    MOTOR_A_FORWARD_PIN,
    MOTOR_A_REVERSE_GPIO_PORT,
    MOTOR_A_REVERSE_PIN,
    TIM_CHANNEL_2
};

/* B 通道 = 物理左轮，PWM 用 TIM1_CH1 */
static const Motor_ChannelConfig_t motorChannelB = {
    MOTOR_B_FORWARD_GPIO_PORT,
    MOTOR_B_FORWARD_PIN,
    MOTOR_B_REVERSE_GPIO_PORT,
    MOTOR_B_REVERSE_PIN,
    TIM_CHANNEL_1
};

static inline uint32_t motor_ccr(float abs_speed)   /* 小数百分比 → CCR，保留 TIM1 的细分辨率 */
{
    uint32_t ccr = (uint32_t)(abs_speed * (float)MOTOR_PWM_CCR_PER_PERCENT + 0.5f);
    if (ccr > MOTOR_PWM_CCR_MAX) ccr = MOTOR_PWM_CCR_MAX;
    return ccr;
}

/* 按通道输出方向与 PWM；speed 正转负反转，0 停止 */
static void motorWriteChannel(const Motor_ChannelConfig_t *channel,
                              float speed)
{
    if (speed == 0.0f) {
        /* 停：占空比清零，两根方向线同电平。同低 = coast（高阻滑行），
           同高 = TB6612 的 short brake（绕组短接，立刻制动）。
           两轮摩擦不等，滑行时右轮比左轮多转一会儿会把车身带歪，所以默认用制动。 */
#if (MOTOR_BRAKE_ON_STOP != 0U)
        HAL_GPIO_WritePin(channel->forwardPort,
                          channel->forwardPin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(channel->reversePort,
                          channel->reversePin, GPIO_PIN_SET);
#else
        HAL_GPIO_WritePin(channel->forwardPort,
                          channel->forwardPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(channel->reversePort,
                          channel->reversePin, GPIO_PIN_RESET);
#endif
        __HAL_TIM_SET_COMPARE(&htim1, channel->timChannel, 0U);
        return;
    }

    if (speed > 0) {
        HAL_GPIO_WritePin(channel->forwardPort,
                          channel->forwardPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(channel->reversePort,
                          channel->reversePin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, channel->timChannel, motor_ccr(speed));
    } else {
        HAL_GPIO_WritePin(channel->forwardPort,
                          channel->forwardPin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(channel->reversePort,
                          channel->reversePin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, channel->timChannel, motor_ccr(-speed));
    }
}

void motor_init(void)   /* 启动 A/B 两轮 PWM 通道 */
{
    /* 先写 0 再把 PWM 通道拉起来：方向脚在上电时保持确定电平（coast），
       避免 GPIO 初始化与 TIM 启动之间那一瞬的浮空/残留高电平。 */
    motorWriteChannel(&motorChannelA, 0);
    motorWriteChannel(&motorChannelB, 0);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* 电机B PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  /* 电机A PWM */
}

void motor_a_set(float speed_percent)   /* 设置 A 轮 PWM 与方向 */
{
    motorWriteChannel(&motorChannelA, speed_percent);
}

void motor_b_set(float speed_percent)   /* 设置 B 轮 PWM 与方向 */
{
    /* B 轮接线反了：PF3↔PF4 对调 */
    motorWriteChannel(&motorChannelB, speed_percent);
}

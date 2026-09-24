/**
 * @file    motor.c
 * @brief   TB6612 双路直流电机驱动模块
 * @note    使用 TIM1 CH1/CH2 输出 16 kHz PWM，PF1-PF4 控制方向：
 *          - A 轮：TIM1_CH2 (PE11)，方向引脚 PF2/PF1
 *          - B 轮：TIM1_CH1 (PE9)，方向引脚 PF3/PF4
 *          - TIM1 预分频为 0，自动重装载值为 4499
 */

#include "motor.h"
#include "yingYongPeiZhi.h"

extern TIM_HandleTypeDef htim1;

/**
 * @brief  单个电机通道的方向和 PWM 配置。
 */
typedef struct {
    GPIO_TypeDef *forwardPort;
    uint16_t forwardPin;
    GPIO_TypeDef *reversePort;
    uint16_t reversePin;
    uint32_t timChannel;
} Motor_ChannelConfig_t;

/**
 * @brief  A 通道配置，对应物理右轮。
 * @note  PWM 使用 TIM1_CH2，方向引脚由 MOTOR_A_* 宏定义。
 */
static const Motor_ChannelConfig_t motorChannelA = {
    MOTOR_A_FORWARD_GPIO_PORT,
    MOTOR_A_FORWARD_PIN,
    MOTOR_A_REVERSE_GPIO_PORT,
    MOTOR_A_REVERSE_PIN,
    TIM_CHANNEL_2
};

/**
 * @brief  B 通道配置，对应物理左轮。
 * @note  PWM 使用 TIM1_CH1，方向引脚由 MOTOR_B_* 宏定义。
 */
static const Motor_ChannelConfig_t motorChannelB = {
    MOTOR_B_FORWARD_GPIO_PORT,
    MOTOR_B_FORWARD_PIN,
    MOTOR_B_REVERSE_GPIO_PORT,
    MOTOR_B_REVERSE_PIN,
    TIM_CHANNEL_1
};

/**
 * @brief  将 PWM 百分比转换为 TIM1 比较寄存器值。
 * @param  abs_speed PWM 百分比绝对值。
 * @return 对应 CCR 值，最大值限制为 MOTOR_PWM_CCR_MAX。
 */
static inline uint32_t motor_ccr(int16_t abs_speed)
{
    uint32_t ccr = (uint32_t)abs_speed * MOTOR_PWM_CCR_PER_PERCENT;
    if (ccr > MOTOR_PWM_CCR_MAX) ccr = MOTOR_PWM_CCR_MAX;
    return ccr;
}

/**
 * @brief  按指定通道配置输出方向和 PWM。
 * @param  channel 电机通道配置。
 * @param  speed 正数正转，负数反转，0 停止输出。
 * @note   A/B 共用此实现，但 A 仍使用 TIM1_CH2、B 仍使用 TIM1_CH1；
 *         物理左右轮映射由应用层调用决定。
 */
static void motorWriteChannel(const Motor_ChannelConfig_t *channel,
                              int16_t speed)
{
    if (speed >= 0) {
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

/**
 * @brief  启动 A、B 两轮 PWM 通道。
 * @note   TIM1_CH1 对应 B 轮，TIM1_CH2 对应 A 轮。
 */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* 电机B PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  /* 电机A PWM */
}

/**
 * @brief  设置 A 轮 PWM 和方向。
 * @param  Speed 正数正转，负数反转，0 停止输出。
 * @note   方向引脚状态根据 Speed 符号切换，PWM 使用绝对值。
 */
void Motor_SetSpeedA(int16_t Speed)
{
    motorWriteChannel(&motorChannelA, Speed);
}

/**
 * @brief  设置 B 轮 PWM 和方向。
 * @param  Speed 正数正转，负数反转，0 停止输出。
 * @note   B 轮方向接线与 A 轮相反，因此方向 IO 逻辑按实际硬件对调。
 */
void Motor_SetSpeedB(int16_t Speed)
{
    /* B轮接线反了: PF3↔PF4 对调 */
    motorWriteChannel(&motorChannelB, Speed);
}

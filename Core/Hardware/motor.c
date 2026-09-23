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
    if (Speed >= 0) {
        HAL_GPIO_WritePin(MOTOR_A_FORWARD_GPIO_PORT,
                          MOTOR_A_FORWARD_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_REVERSE_GPIO_PORT,
                          MOTOR_A_REVERSE_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_ccr(Speed));
    } else {
        HAL_GPIO_WritePin(MOTOR_A_FORWARD_GPIO_PORT,
                          MOTOR_A_FORWARD_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_A_REVERSE_GPIO_PORT,
                          MOTOR_A_REVERSE_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_ccr(-Speed));
    }
}

/**
 * @brief  设置 B 轮 PWM 和方向。
 * @param  Speed 正数正转，负数反转，0 停止输出。
 * @note   B 轮方向接线与 A 轮相反，因此方向 IO 逻辑按实际硬件对调。
 */
void Motor_SetSpeedB(int16_t Speed)
{
    /* B轮接线反了: PF3↔PF4 对调 */
    if (Speed >= 0) {
        HAL_GPIO_WritePin(MOTOR_B_FORWARD_GPIO_PORT,
                          MOTOR_B_FORWARD_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_B_REVERSE_GPIO_PORT,
                          MOTOR_B_REVERSE_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_ccr(Speed));
    } else {
        HAL_GPIO_WritePin(MOTOR_B_FORWARD_GPIO_PORT,
                          MOTOR_B_FORWARD_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_B_REVERSE_GPIO_PORT,
                          MOTOR_B_REVERSE_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_ccr(-Speed));
    }
}

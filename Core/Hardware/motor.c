/**
 * @file    motor.c
 * @brief   TB6612 双路直流电机驱动模块
 * @note    使用 TIM1 CH1/CH2 输出 PWM, PF1~PF4 控制方向
 *          - 电机A: TIM1_CH2 (PE11) + PF1(方向1) + PF2(方向2)
 *          - 电机B: TIM1_CH1 (PE9)  + PF3(方向1) + PF4(方向2)
 *          PWM 频率 = 72MHz / 72 / 4096 ≈ 244Hz, ARR=4095
 */

#include "motor.h"

extern TIM_HandleTypeDef htim1;

/* Speed 0~100 线性映射到 CCR 0~4096 (ARR+1=100%占空比) */
static inline uint32_t motor_ccr(int16_t abs_speed)
{
    uint32_t ccr = (uint32_t)abs_speed * 41;
    if (ccr > 4096) ccr = 4096;
    return ccr;
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* 电机B PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  /* 电机A PWM */
}

void Motor_SetSpeedA(int16_t Speed)
{
    if (Speed >= 0) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_ccr(Speed));
    } else {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_ccr(-Speed));
    }
}

void Motor_SetSpeedB(int16_t Speed)
{
    /* B轮接线反了: PF3↔PF4 对调 */
    if (Speed >= 0) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_ccr(Speed));
    } else {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_ccr(-Speed));
    }
}

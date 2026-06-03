/**
 * @file    motor.c
 * @brief   TB6612 双路直流电机驱动模块
 * @note    使用 TIM1 CH1/CH2 输出 PWM, PF1~PF4 控制方向
 *          - 电机A: TIM1_CH2 (PE11) + PF1(方向1) + PF2(方向2)
 *          - 电机B: TIM1_CH1 (PE9)  + PF3(方向1) + PF4(方向2)
 *          PWM 频率 = 72MHz / 72 / 4096 ≈ 244Hz
 */

#include "motor.h"

extern TIM_HandleTypeDef htim1;

/**
 * @brief  初始化电机驱动: 启动 TIM1 两路 PWM 输出
 */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* 电机B PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  /* 电机A PWM */
}

/**
 * @brief  设置电机 A (左轮) 速度和方向
 * @param  Speed 有符号速度值, 正值前进/负值后退, 范围 ±(ARR-1)
 */
void Motor_SetSpeedA(int16_t Speed)
{
    if (Speed >= 0) {
        /* 正转: PF2=0(低), PF1=1(高) */
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)Speed * 40);
    } else {
        /* 反转: PF2=1(高), PF1=0(低) */
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(-Speed) * 40);
    }
}

/**
 * @brief  设置电机 B (右轮) 速度和方向 (右电机物理安装方向与左电机相反, 方向引脚对调)
 * @param  Speed 有符号速度值, 正值前进/负值后退, 范围 0~100
 */
void Motor_SetSpeedB(int16_t Speed)
{
    if (Speed >= 0) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)Speed * 40);
    } else {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(-Speed) * 40);
    }
}

/**
 * @file    buzzer.c
 * @brief   有源蜂鸣器控制模块
 * @note    使用 PG6, 高电平关闭/低电平开启 (低电平有效)
 */

#include "buzzer.h"

/**
 * @brief  初始化蜂鸣器: 默认关闭 (高电平)
 */
void Buzzer_Init(void)
{
    Buzzer_OFF();
}

/**
 * @brief  蜂鸣器开启
 */
void Buzzer_ON(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  蜂鸣器关闭
 */
void Buzzer_OFF(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

/**
 * @brief  蜂鸣器状态翻转
 */
void Buzzer_Turn(void)
{
    HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
}

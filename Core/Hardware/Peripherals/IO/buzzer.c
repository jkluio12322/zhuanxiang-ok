/* 有源蜂鸣器：PG6，低电平开、高电平关 */

#include "buzzer.h"

void Buzzer_Init(void)   /* 默认关闭 */
{
    Buzzer_OFF();
}

void Buzzer_ON(void)   /* 开 */
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

void Buzzer_OFF(void)   /* 关 */
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

void Buzzer_Turn(void)   /* 翻转 */
{
    HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
}

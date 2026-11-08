/* 有源蜂鸣器：PG6，低电平开、高电平关 */

#include "buzzer.h"

void buzzer_init(void)   /* 默认关闭 */
{
    buzzer_off();
}

void buzzer_on(void)   /* 开 */
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

void buzzer_off(void)   /* 关 */
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

void buzzer_turn(void)   /* 翻转 */
{
    HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
}

/**
 * @file    irtrack.c
 * @brief   8路红外循迹传感器驱动 (数字量模式)
 * @note    GPIO: PE0~PE6 + PD12, 上拉输入
 *          黑线=GND → 读回 0, 逻辑取反后 raw[i]=1
 */

#include "irtrack.h"
#include <string.h>

IRTrack_Data_t ir_data = {0};

/* 权重表: 等差数列 (8 路等间距) */
static const int8_t ir_weight[IRTRACK_CHANNELS] = {
    -7, -5, -3, -1,  1,  3,  5,  7
};

/* GPIO 端口/引脚映射 */
static const struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} ir_pins[IRTRACK_CHANNELS] = {
    {GPIOE, GPIO_PIN_0},   /* IR1 最左 */
    {GPIOE, GPIO_PIN_1},   /* IR2 */
    {GPIOE, GPIO_PIN_2},   /* IR3 */
    {GPIOE, GPIO_PIN_3},   /* IR4 */
    {GPIOE, GPIO_PIN_4},   /* IR5 */
    {GPIOE, GPIO_PIN_5},   /* IR6 */
    {GPIOE, GPIO_PIN_6},   /* IR7 */
    {GPIOD, GPIO_PIN_12},  /* IR8 最右 */
};

void IRTrack_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
          | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOE, &g);

    g.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOD, &g);
}

void IRTrack_Init(IR_Mode_t mode)
{
    IRTrack_GPIO_Init();
    memset(&ir_data, 0, sizeof(ir_data));
    (void)mode;
}

void IRTrack_Read(void)
{
    for (uint8_t i = 0; i < IRTRACK_CHANNELS; i++) {
        /* 上拉输入: 黑线(GND) → 读回 0; 取反后 黑=1, 白=0 */
        ir_data.raw[i] = (HAL_GPIO_ReadPin(ir_pins[i].port, ir_pins[i].pin) == GPIO_PIN_RESET) ? 1 : 0;
    }
}

void IRTrack_Compute(void)
{
    int8_t  sum = 0;
    uint8_t cnt = 0;

    for (uint8_t i = 0; i < IRTRACK_CHANNELS; i++) {
        if (ir_data.raw[i]) {
            sum += ir_weight[i];
            cnt++;
        }
    }

    ir_data.weight_sum  = sum;
    ir_data.black_count = cnt;
}

IR_State_t IRTrack_DetectCrossroad(void)
{
    uint8_t cnt = ir_data.black_count;

    if (cnt == 0) {
        ir_data.state = TRACK_LOST;
    }
    /* 全部 8 路都黑 → 十字路口 */
    else if (cnt >= 7) {
        ir_data.state = TRACK_CROSS;
    }
    /* 连续多路检测到 → 可能是直角弯 */
    else if (cnt >= 5) {
        if (ir_data.raw[0] && ir_data.raw[1] && ir_data.raw[2] && ir_data.raw[3])
            ir_data.state = TRACK_HARD_LEFT;
        else if (ir_data.raw[4] && ir_data.raw[5] && ir_data.raw[6] && ir_data.raw[7])
            ir_data.state = TRACK_HARD_RIGHT;
        else
            ir_data.state = TRACK_CROSS;
    }
    else {
        ir_data.state = TRACK_NORMAL;
    }

    return ir_data.state;
}

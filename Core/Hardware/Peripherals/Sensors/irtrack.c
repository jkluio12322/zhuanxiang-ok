/**
 * @file    irtrack.c
 * @brief   八路数字循迹传感器采样、权重和位置计算。
 */

#include "irtrack.h"
#include <string.h>

IRTrack_Data_t ir_data = {0};

/*
 * 传感器中心间距：
 *   IR1 --20mm-- IR2 --20mm-- IR3 --15mm-- IR4
 *   IR4 --15mm-- IR5 --15mm-- IR6 --20mm-- IR7
 *   IR7 --20mm-- IR8
 *
 * 位置值保持 -7...+7 的归一化范围，仅用于位置诊断、居中确认和边缘判断。
 * 左右轮控制目标由 yingYongXunJi.c 中的加权误差 PD 统一生成。
 * 正位置表示线路偏右，负位置表示线路偏左。
 */

/* 8 路循迹通道权重，负值偏左，正值偏右。 */
#define IR_WEIGHT_CH1 (-25)
#define IR_WEIGHT_CH2 (-17)
#define IR_WEIGHT_CH3 (-9)
#define IR_WEIGHT_CH4 (-3)
#define IR_WEIGHT_CH5 3
#define IR_WEIGHT_CH6 9
#define IR_WEIGHT_CH7 17
#define IR_WEIGHT_CH8 25

static const int8_t ir_weight[IRTRACK_CHANNELS] = {
    IR_WEIGHT_CH1,
    IR_WEIGHT_CH2,
    IR_WEIGHT_CH3,
    IR_WEIGHT_CH4,
    IR_WEIGHT_CH5,
    IR_WEIGHT_CH6,
    IR_WEIGHT_CH7,
    IR_WEIGHT_CH8
};

static const struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} ir_pins[IRTRACK_CHANNELS] = {
    {IR1_GPIO_Port, IR1_Pin},
    {IR2_GPIO_Port, IR2_Pin},
    {IR3_GPIO_Port, IR3_Pin},
    {IR4_GPIO_Port, IR4_Pin},
    {IR5_GPIO_Port, IR5_Pin},
    {IR6_GPIO_Port, IR6_Pin},
    {IR7_GPIO_Port, IR7_Pin},
    {IR8_GPIO_Port, IR8_Pin},
};

/**
 * @brief  初始化循迹数据并设置初始白线状态。
 */
void IRTrack_Init(void)
{
    memset(&ir_data, 0, sizeof(ir_data));

    /* 传感器将黑线输出为高电平，因此软件 raw=0 表示黑线、raw=1 表示白线。 */
    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] = 1U;
    }
}

/**
 * @brief  采集八路传感器输入并规范化为 raw 数组。
 * @note   raw=0 表示黑线，raw=1 表示白线；调用后需执行 IRTrack_Compute。
 */
void IRTrack_Read(void)
{
    for (uint8_t i = 0; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] =
            (HAL_GPIO_ReadPin(ir_pins[i].port, ir_pins[i].pin)
             == GPIO_PIN_RESET) ? 1U : 0U;
    }
}

/**
 * @brief  计算黑线图案、通道数量、加权位置和权重和。
 * @note   pattern 的 bit0-bit7 对应 IR1-IR8，黑线通道置 1。
 */
void IRTrack_Compute(void)
{
    int16_t sum = 0;
    int16_t positionTimes100 = 0;
    uint8_t count = 0;
    uint8_t pattern = 0;

    for (uint8_t i = 0; i < IRTRACK_CHANNELS; i++) {
        /* 根据高电平表示黑线的物理输入生成逻辑黑线掩码。 */
        if (ir_data.raw[i] == 0U) {
            sum = (int16_t)(sum + ir_weight[i]);
            count++;
            pattern |= (uint8_t)(1U << i);
        }
    }

    ir_data.pattern = pattern;
    ir_data.weight_sum = (int8_t)sum;
    ir_data.black_count = count;

    if (count > 0U) {
        positionTimes100 = (int16_t)((sum * 28) / (int16_t)count);
    }

    ir_data.position = (float)positionTimes100 / 100.0f;
}

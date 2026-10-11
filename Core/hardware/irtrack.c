/* 八路数字循迹传感器采样、权重和位置计算 */

#include "irtrack.h"
#include <string.h>

IRTrack_Data_t ir_data = {0};

/*
 * 传感器间距：IR1-20-IR2-20-IR3-15-IR4-15-IR5-15-IR6-20-IR7-20-IR8。
 * 位置归一化到 -7...+7，仅用于位置诊断/居中确认/边缘判断；轮控目标由 app_tracking.c 生成。
 * 正位置=线偏右，负=线偏左。
 */

/* 8 路通道权重，负偏左、正偏右 */
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

void ir_init(void)   /* 初始化数据，置白线状态 */
{
    memset(&ir_data, 0, sizeof(ir_data));

    /* 黑线输出高电平，软件 raw=0 黑线、raw=1 白线 */
    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] = 1U;
    }
}

void ir_read(void)   /* 采集八路输入，raw=0 黑线、raw=1 白线 */
{
    for (uint8_t i = 0; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] =
            (HAL_GPIO_ReadPin(ir_pins[i].port, ir_pins[i].pin)
             == GPIO_PIN_RESET) ? 1U : 0U;
    }
}

void ir_compute(void)   /* 计算黑线图案、通道数、加权位置与权重和 */
{
    int16_t sum = 0;
    int16_t positionTimes100 = 0;
    uint8_t count = 0;
    uint8_t pattern = 0;

    for (uint8_t i = 0; i < IRTRACK_CHANNELS; i++) {
        /* 高电平=黑线的物理输入 → 逻辑黑线掩码 */
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

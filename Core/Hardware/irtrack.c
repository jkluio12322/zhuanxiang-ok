/**
 * @file    irtrack.c
 * @brief   八路数字循迹传感器采样、权重和查表驱动。
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
 * 位置表保持 -7...+7 的归一化范围。直接查表把位置转换为左右轮百分比。
 * 中心单路和中路双路图案保留位置用于诊断和急弯判断，但左右输出均使用
 * 100%，从而抑制直线行驶时的抖动：
 *
 *   |position|    slow wheel    fast wheel
 *      0.00          100%          100%
 *      0.84          100%          100%
 *      2.52           65%          135%
 *      4.76           50%          150%
 *      7.00           40%          180%
 *
 * 正位置表示线路偏右，因此理论左轮更快、右轮更慢；负位置相反。
 */

typedef struct {
    int16_t position_x100;
    uint8_t left_percent;
    uint8_t right_percent;
    uint8_t state;
} IRTrack_LutEntry_t;

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

static const IRTrack_LutEntry_t ir_track_lut[256] = {
    {    0, 100, 100, TRACK_LOST},   /* 0x00 */
    { -700,   40, 180, TRACK_NORMAL}, /* 0x01 */
    { -476,   50, 150, TRACK_NORMAL}, /* 0x02 */
    { -588,   45, 165, TRACK_NORMAL}, /* 0x03 */
    { -252,   65, 135, TRACK_NORMAL}, /* 0x04 */
    { -476,   50, 150, TRACK_NORMAL}, /* 0x05 */
    { -364,   58, 143, TRACK_NORMAL}, /* 0x06 */
    { -476,   50, 150, TRACK_NORMAL}, /* 0x07 */
    {  -84,  100, 100, TRACK_NORMAL}, /* 0x08 */
    { -392,   56, 144, TRACK_NORMAL}, /* 0x09 */
    { -280,   63, 137, TRACK_NORMAL}, /* 0x0A */
    { -420,   54, 146, TRACK_NORMAL}, /* 0x0B */
    { -168,   73, 128, TRACK_NORMAL}, /* 0x0C */
    { -345,   59, 141, TRACK_NORMAL}, /* 0x0D */
    { -271,   64, 136, TRACK_NORMAL}, /* 0x0E */
    { -378,   57, 143, TRACK_NORMAL}, /* 0x0F */
    {   84, 100, 100, TRACK_NORMAL}, /* 0x10 */
    { -308,   61, 139, TRACK_NORMAL}, /* 0x11 */
    { -196,   70, 130, TRACK_NORMAL}, /* 0x12 */
    { -364,   58, 143, TRACK_NORMAL}, /* 0x13 */
    {  -84,   80, 120, TRACK_NORMAL}, /* 0x14 */
    { -289,   63, 137, TRACK_NORMAL}, /* 0x15 */
    { -215,   68, 132, TRACK_NORMAL}, /* 0x16 */
    { -336,   59, 141, TRACK_NORMAL}, /* 0x17 */
    {    0, 100, 100, TRACK_NORMAL}, /* 0x18 */
    { -233,   67, 133, TRACK_NORMAL}, /* 0x19 */
    { -159,   73, 127, TRACK_NORMAL}, /* 0x1A */
    { -294,   62, 138, TRACK_NORMAL}, /* 0x1B */
    {  -84,   80, 120, TRACK_NORMAL}, /* 0x1C */
    { -238,   66, 134, TRACK_NORMAL}, /* 0x1D */
    { -182,   71, 129, TRACK_NORMAL}, /* 0x1E */
    { -286,   63, 137, TRACK_NORMAL}, /* 0x1F */
    {  252, 135,   65, TRACK_NORMAL}, /* 0x20 */
    { -224,   68, 133, TRACK_NORMAL}, /* 0x21 */
    { -112,   78, 123, TRACK_NORMAL}, /* 0x22 */
    { -308,   61, 139, TRACK_NORMAL}, /* 0x23 */
    {    0, 100, 100, TRACK_NORMAL}, /* 0x24 */
    { -233,   67, 133, TRACK_NORMAL}, /* 0x25 */
    { -159,   73, 127, TRACK_NORMAL}, /* 0x26 */
    { -294,   62, 138, TRACK_NORMAL}, /* 0x27 */
    {   84, 120,   80, TRACK_NORMAL}, /* 0x28 */
    { -177,   72, 128, TRACK_NORMAL}, /* 0x29 */
    { -103,   78, 122, TRACK_NORMAL}, /* 0x2A */
    { -252,   65, 135, TRACK_NORMAL}, /* 0x2B */
    {  -28,   93, 107, TRACK_NORMAL}, /* 0x2C */
    { -196,   70, 130, TRACK_NORMAL}, /* 0x2D */
    { -140,   75, 125, TRACK_NORMAL}, /* 0x2E */
    { -252,   65, 135, TRACK_NORMAL}, /* 0x2F */
    {  168, 128,   73, TRACK_NORMAL}, /* 0x30 */
    { -121,   77, 123, TRACK_NORMAL}, /* 0x31 */
    {  -47,   89, 111, TRACK_NORMAL}, /* 0x32 */
    { -210,   69, 131, TRACK_NORMAL}, /* 0x33 */
    {   28, 107,   93, TRACK_NORMAL}, /* 0x34 */
    { -154,   74, 126, TRACK_NORMAL}, /* 0x35 */
    {  -98,   79, 121, TRACK_NORMAL}, /* 0x36 */
    { -218,   68, 132, TRACK_NORMAL}, /* 0x37 */
    {   84, 120,   80, TRACK_NORMAL}, /* 0x38 */
    { -112,   78, 123, TRACK_NORMAL}, /* 0x39 */
    {  -56,   87, 113, TRACK_NORMAL}, /* 0x3A */
    { -185,   71, 129, TRACK_NORMAL}, /* 0x3B */
    {    0, 100, 100, TRACK_NORMAL}, /* 0x3C */
    { -140,   75, 125, TRACK_NORMAL}, /* 0x3D */
    {  -95,   79, 121, TRACK_NORMAL}, /* 0x3E */
    { -196,   70, 130, TRACK_NORMAL}, /* 0x3F */
    {  476, 150,   50, TRACK_NORMAL}, /* 0x40 */
    { -112,   78, 123, TRACK_NORMAL}, /* 0x41 */
    {    0, 100, 100, TRACK_NORMAL}, /* 0x42 */
    { -233,   67, 133, TRACK_NORMAL}, /* 0x43 */
    {  112, 123,   78, TRACK_NORMAL}, /* 0x44 */
    { -159,   73, 127, TRACK_NORMAL}, /* 0x45 */
    {  -84,   80, 120, TRACK_NORMAL}, /* 0x46 */
    { -238,   66, 134, TRACK_NORMAL}, /* 0x47 */
    {  196, 130,   70, TRACK_NORMAL}, /* 0x48 */
    { -103,   78, 122, TRACK_NORMAL}, /* 0x49 */
    {  -28,   93, 107, TRACK_NORMAL}, /* 0x4A */
    { -196,   70, 130, TRACK_NORMAL}, /* 0x4B */
    {   47, 111,   89, TRACK_NORMAL}, /* 0x4C */
    { -140,   75, 125, TRACK_NORMAL}, /* 0x4D */
    {  -84,   80, 120, TRACK_NORMAL}, /* 0x4E */
    { -207,   69, 131, TRACK_NORMAL}, /* 0x4F */
    {  280, 137,   63, TRACK_NORMAL}, /* 0x50 */
    {  -47,   89, 111, TRACK_NORMAL}, /* 0x51 */
    {   28, 107,   93, TRACK_NORMAL}, /* 0x52 */
    { -154,   74, 126, TRACK_NORMAL}, /* 0x53 */
    {  103, 122,   78, TRACK_NORMAL}, /* 0x54 */
    {  -98,   79, 121, TRACK_NORMAL}, /* 0x55 */
    {  -42,   90, 110, TRACK_NORMAL}, /* 0x56 */
    { -174,   72, 128, TRACK_NORMAL}, /* 0x57 */
    {  159, 127,   73, TRACK_NORMAL}, /* 0x58 */
    {  -56,   87, 113, TRACK_NORMAL}, /* 0x59 */
    {    0, 100, 100, TRACK_NORMAL}, /* 0x5A */
    { -140,   75, 125, TRACK_NORMAL}, /* 0x5B */
    {   56, 113,   87, TRACK_NORMAL}, /* 0x5C */
    {  -95,   79, 121, TRACK_NORMAL}, /* 0x5D */
    {  -50,   88, 112, TRACK_NORMAL}, /* 0x5E */
    { -159,   73, 127, TRACK_NORMAL}, /* 0x5F */
    {  364, 143,   58, TRACK_NORMAL}, /* 0x60 */
    {    9, 102,   98, TRACK_NORMAL}, /* 0x61 */
    {   84, 120,   80, TRACK_NORMAL}, /* 0x62 */
    { -112,   78, 123, TRACK_NORMAL}, /* 0x63 */
    {  159, 127,   73, TRACK_NORMAL}, /* 0x64 */
    {  -56,   87, 113, TRACK_NORMAL}, /* 0x65 */
    {    0, 100, 100, TRACK_NORMAL}, /* 0x66 */
    { -140,   75, 125, TRACK_NORMAL}, /* 0x67 */
    {  215, 132,   68, TRACK_NORMAL}, /* 0x68 */
    {  -14,   97, 103, TRACK_NORMAL}, /* 0x69 */
    {   42, 110,   90, TRACK_NORMAL}, /* 0x6A */
    { -106,   78, 122, TRACK_NORMAL}, /* 0x6B */
    {   98, 121,   79, TRACK_NORMAL}, /* 0x6C */
    {  -62,   85, 115, TRACK_NORMAL}, /* 0x6D */
    {  -17,   96, 104, TRACK_NORMAL}, /* 0x6E */
    { -131,   76, 124, TRACK_NORMAL}, /* 0x6F */
    {  271, 136,   64, TRACK_NORMAL}, /* 0x70 */
    {   28, 107,   93, TRACK_NORMAL}, /* 0x71 */
    {   84, 120,   80, TRACK_NORMAL}, /* 0x72 */
    {  -73,   83, 117, TRACK_NORMAL}, /* 0x73 */
    {  140, 125,   75, TRACK_NORMAL}, /* 0x74 */
    {  -28,   93, 107, TRACK_NORMAL}, /* 0x75 */
    {   17, 104,   96, TRACK_NORMAL}, /* 0x76 */
    { -103,   78, 122, TRACK_NORMAL}, /* 0x77 */
    {  182, 129,   71, TRACK_NORMAL}, /* 0x78 */
    {    6, 101,   99, TRACK_NORMAL}, /* 0x79 */
    {   50, 112,   88, TRACK_NORMAL}, /* 0x7A */
    {  -75,   82, 118, TRACK_NORMAL}, /* 0x7B */
    {   95, 121,   79, TRACK_NORMAL}, /* 0x7C */
    {  -37,   91, 109, TRACK_NORMAL}, /* 0x7D */
    {    0, 100, 100, TRACK_NORMAL}, /* 0x7E */
    { -100,   79, 121, TRACK_NORMAL}, /* 0x7F */
    {  700, 180,   40, TRACK_NORMAL}, /* 0x80 */
    {    0, 100, 100, TRACK_NORMAL}, /* 0x81 */
    {  112, 123,   78, TRACK_NORMAL}, /* 0x82 */
    { -159,   73, 127, TRACK_NORMAL}, /* 0x83 */
    {  224, 133,   68, TRACK_NORMAL}, /* 0x84 */
    {  -84,   80, 120, TRACK_NORMAL}, /* 0x85 */
    {   -9,   98, 102, TRACK_NORMAL}, /* 0x86 */
    { -182,   71, 129, TRACK_NORMAL}, /* 0x87 */
    {  308, 139,   61, TRACK_NORMAL}, /* 0x88 */
    {  -28,   93, 107, TRACK_NORMAL}, /* 0x89 */
    {   47, 111,   89, TRACK_NORMAL}, /* 0x8A */
    { -140,   75, 125, TRACK_NORMAL}, /* 0x8B */
    {  121, 123,   77, TRACK_NORMAL}, /* 0x8C */
    {  -84,   80, 120, TRACK_NORMAL}, /* 0x8D */
    {  -28,   93, 107, TRACK_NORMAL}, /* 0x8E */
    { -162,   73, 127, TRACK_NORMAL}, /* 0x8F */
    {  392, 144,   56, TRACK_NORMAL}, /* 0x90 */
    {   28, 107,   93, TRACK_NORMAL}, /* 0x91 */
    {  103, 122,   78, TRACK_NORMAL}, /* 0x92 */
    {  -98,   79, 121, TRACK_NORMAL}, /* 0x93 */
    {  177, 128,   72, TRACK_NORMAL}, /* 0x94 */
    {  -42,   90, 110, TRACK_NORMAL}, /* 0x95 */
    {   14, 103,   97, TRACK_NORMAL}, /* 0x96 */
    { -129,   76, 124, TRACK_NORMAL}, /* 0x97 */
    {  233, 133,   67, TRACK_NORMAL}, /* 0x98 */
    {    0, 100, 100, TRACK_NORMAL}, /* 0x99 */
    {   56, 113,   87, TRACK_NORMAL}, /* 0x9A */
    {  -95,   79, 121, TRACK_NORMAL}, /* 0x9B */
    {  112, 123,   78, TRACK_NORMAL}, /* 0x9C */
    {  -50,   88, 112, TRACK_NORMAL}, /* 0x9D */
    {   -6,   99, 101, TRACK_NORMAL}, /* 0x9E */
    { -121,   77, 123, TRACK_NORMAL}, /* 0x9F */
    {  476, 150,   50, TRACK_NORMAL}, /* 0xA0 */
    {   84, 120,   80, TRACK_NORMAL}, /* 0xA1 */
    {  159, 127,   73, TRACK_NORMAL}, /* 0xA2 */
    {  -56,   87, 113, TRACK_NORMAL}, /* 0xA3 */
    {  233, 133,   67, TRACK_NORMAL}, /* 0xA4 */
    {    0, 100, 100, TRACK_NORMAL}, /* 0xA5 */
    {   56, 113,   87, TRACK_NORMAL}, /* 0xA6 */
    {  -95,   79, 121, TRACK_NORMAL}, /* 0xA7 */
    {  289, 137,   63, TRACK_NORMAL}, /* 0xA8 */
    {   42, 110,   90, TRACK_NORMAL}, /* 0xA9 */
    {   98, 121,   79, TRACK_NORMAL}, /* 0xAA */
    {  -62,   85, 115, TRACK_NORMAL}, /* 0xAB */
    {  154, 126,   74, TRACK_NORMAL}, /* 0xAC */
    {  -17,   96, 104, TRACK_NORMAL}, /* 0xAD */
    {   28, 107,   93, TRACK_NORMAL}, /* 0xAE */
    {  -93,   79, 121, TRACK_NORMAL}, /* 0xAF */
    {  345, 141,   59, TRACK_NORMAL}, /* 0xB0 */
    {   84, 120,   80, TRACK_NORMAL}, /* 0xB1 */
    {  140, 125,   75, TRACK_NORMAL}, /* 0xB2 */
    {  -28,   93, 107, TRACK_NORMAL}, /* 0xB3 */
    {  196, 130,   70, TRACK_NORMAL}, /* 0xB4 */
    {   17, 104,   96, TRACK_NORMAL}, /* 0xB5 */
    {   62, 115,   85, TRACK_NORMAL}, /* 0xB6 */
    {  -65,   85, 115, TRACK_NORMAL}, /* 0xB7 */
    {  238, 134,   66, TRACK_NORMAL}, /* 0xB8 */
    {   50, 112,   88, TRACK_NORMAL}, /* 0xB9 */
    {   95, 121,   79, TRACK_NORMAL}, /* 0xBA */
    {  -37,   91, 109, TRACK_NORMAL}, /* 0xBB */
    {  140, 125,   75, TRACK_NORMAL}, /* 0xBC */
    {    0, 100, 100, TRACK_NORMAL}, /* 0xBD */
    {   37, 109,   91, TRACK_NORMAL}, /* 0xBE */
    {  -68,   84, 116, TRACK_NORMAL}, /* 0xBF */
    {  588, 165,   45, TRACK_NORMAL}, /* 0xC0 */
    {  159, 127,   73, TRACK_NORMAL}, /* 0xC1 */
    {  233, 133,   67, TRACK_NORMAL}, /* 0xC2 */
    {    0, 100, 100, TRACK_NORMAL}, /* 0xC3 */
    {  308, 139,   61, TRACK_NORMAL}, /* 0xC4 */
    {   56, 113,   87, TRACK_NORMAL}, /* 0xC5 */
    {  112, 123,   78, TRACK_NORMAL}, /* 0xC6 */
    {  -50,   88, 112, TRACK_NORMAL}, /* 0xC7 */
    {  364, 143,   58, TRACK_NORMAL}, /* 0xC8 */
    {   98, 121,   79, TRACK_NORMAL}, /* 0xC9 */
    {  154, 126,   74, TRACK_NORMAL}, /* 0xCA */
    {  -17,   96, 104, TRACK_NORMAL}, /* 0xCB */
    {  210, 131,   69, TRACK_NORMAL}, /* 0xCC */
    {   28, 107,   93, TRACK_NORMAL}, /* 0xCD */
    {   73, 117,   83, TRACK_NORMAL}, /* 0xCE */
    {  -56,   87, 113, TRACK_NORMAL}, /* 0xCF */
    {  420, 146,   54, TRACK_NORMAL}, /* 0xD0 */
    {  140, 125,   75, TRACK_NORMAL}, /* 0xD1 */
    {  196, 130,   70, TRACK_NORMAL}, /* 0xD2 */
    {   17, 104,   96, TRACK_NORMAL}, /* 0xD3 */
    {  252, 135,   65, TRACK_NORMAL}, /* 0xD4 */
    {   62, 115,   85, TRACK_NORMAL}, /* 0xD5 */
    {  106, 122,   78, TRACK_NORMAL}, /* 0xD6 */
    {  -28,   93, 107, TRACK_NORMAL}, /* 0xD7 */
    {  294, 138,   62, TRACK_NORMAL}, /* 0xD8 */
    {   95, 121,   79, TRACK_NORMAL}, /* 0xD9 */
    {  140, 125,   75, TRACK_NORMAL}, /* 0xDA */
    {    0, 100, 100, TRACK_NORMAL}, /* 0xDB */
    {  185, 129,   71, TRACK_NORMAL}, /* 0xDC */
    {   37, 109,   91, TRACK_NORMAL}, /* 0xDD */
    {   75, 118,   82, TRACK_NORMAL}, /* 0xDE */
    {  -36,   91, 109, TRACK_NORMAL}, /* 0xDF */
    {  476, 150,   50, TRACK_NORMAL}, /* 0xE0 */
    {  182, 129,   71, TRACK_NORMAL}, /* 0xE1 */
    {  238, 134,   66, TRACK_NORMAL}, /* 0xE2 */
    {   50, 112,   88, TRACK_NORMAL}, /* 0xE3 */
    {  294, 138,   62, TRACK_NORMAL}, /* 0xE4 */
    {   95, 121,   79, TRACK_NORMAL}, /* 0xE5 */
    {  140, 125,   75, TRACK_NORMAL}, /* 0xE6 */
    {    0, 100, 100, TRACK_NORMAL}, /* 0xE7 */
    {  336, 141,   59, TRACK_NORMAL}, /* 0xE8 */
    {  129, 124,   76, TRACK_NORMAL}, /* 0xE9 */
    {  174, 128,   72, TRACK_NORMAL}, /* 0xEA */
    {   28, 107,   93, TRACK_NORMAL}, /* 0xEB */
    {  218, 132,   68, TRACK_NORMAL}, /* 0xEC */
    {   65, 115,   85, TRACK_NORMAL}, /* 0xED */
    {  103, 122,   78, TRACK_NORMAL}, /* 0xEE */
    {  -12,   97, 103, TRACK_NORMAL}, /* 0xEF */
    {  378, 143,   57, TRACK_NORMAL}, /* 0xF0 */
    {  162, 127,   73, TRACK_NORMAL}, /* 0xF1 */
    {  207, 131,   69, TRACK_NORMAL}, /* 0xF2 */
    {   56, 113,   87, TRACK_NORMAL}, /* 0xF3 */
    {  252, 135,   65, TRACK_NORMAL}, /* 0xF4 */
    {   93, 121,   79, TRACK_NORMAL}, /* 0xF5 */
    {  131, 124,   76, TRACK_NORMAL}, /* 0xF6 */
    {   12, 103,   97, TRACK_NORMAL}, /* 0xF7 */
    {  286, 137,   63, TRACK_NORMAL}, /* 0xF8 */
    {  121, 123,   77, TRACK_NORMAL}, /* 0xF9 */
    {  159, 127,   73, TRACK_NORMAL}, /* 0xFA */
    {   36, 109,   91, TRACK_NORMAL}, /* 0xFB */
    {  196, 130,   70, TRACK_NORMAL}, /* 0xFC */
    {   68, 116,   84, TRACK_NORMAL}, /* 0xFD */
    {  100, 121,   79, TRACK_NORMAL}, /* 0xFE */
    {    0, 100, 100, TRACK_FINISH}  /* 0xFF */
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
 * @brief  初始化循迹模块 GPIO。
 * @note   当前引脚由 CubeMX 的 MX_GPIO_Init 统一配置，本函数保留为空实现。
 */
void IRTrack_GPIO_Init(void)
{
    /* GPIO configuration is generated by CubeMX in MX_GPIO_Init(). */
}

/**
 * @brief  初始化循迹数据并设置初始白线状态。
 * @param  mode 传感器模式；当前实现仅使用数字查表，模拟模式参数被忽略。
 */
void IRTrack_Init(IR_Mode_t mode)
{
    IRTrack_GPIO_Init();
    memset(&ir_data, 0, sizeof(ir_data));

    /* 传感器将黑线输出为高电平，因此软件 raw=0 表示黑线、raw=1 表示白线。 */
    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] = 1U;
    }

    (void)mode;
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
 * @brief  计算黑线图案、通道数量、加权位置和查表左右轮百分比。
 * @note   pattern 的 bit0-bit7 对应 IR1-IR8，黑线通道置 1。
 */
void IRTrack_Compute(void)
{
    int16_t sum = 0;
    uint8_t count = 0;
    uint8_t pattern = 0;
    const IRTrack_LutEntry_t *entry;

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

    entry = &ir_track_lut[pattern];
    ir_data.position = (float)entry->position_x100 / 100.0f;
    ir_data.left_percent = entry->left_percent;
    ir_data.right_percent = entry->right_percent;

    if ((entry->state != TRACK_FINISH) && (entry->state != TRACK_LOST)) {
        if (ir_data.position < 0.0f) {
            ir_data.last_side = -1;
        } else if (ir_data.position > 0.0f) {
            ir_data.last_side = 1;
        }
    }
}

/**
 * @brief  根据当前图案返回查表状态。
 * @return 当前图案对应的 TRACK_NORMAL、TRACK_FINISH 或 TRACK_LOST 等状态。
 */
IR_State_t IRTrack_DetectCrossroad(void)
{
    ir_data.state = (IR_State_t)ir_track_lut[ir_data.pattern].state;
    return ir_data.state;
}

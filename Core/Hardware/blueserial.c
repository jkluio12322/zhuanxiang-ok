/**
 * @file    blueserial.c
 * @brief   HC-05 蓝牙模块 UART 通信 + 环形缓冲区 + 协议解析
 * @note    使用 USART2 (PA2=TX, PA3=RX), 9600bps, 中断接收
 *
 *          环形缓冲区 (100 字节):
 *            - 中断中将收到的字节写入缓冲区
 *            - 主循环检测帧头 '[' 和帧尾 ']' 完成帧同步
 *
 *          蓝牙协议:
 *            摇杆模式: [joystick,<LH>,<LV>,<RH>,<RV>]
 *            按键模式: [key,<N>,<up/down>]
 */

#include "blueserial.h"
#include "pid_tune.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define BLUE_SERIAL_BUFFER_SIZE  100  /**< 环形缓冲区大小 */

/** 环形缓冲区 (ISR 写入, 主循环读取 → volatile 防止编译器缓存) */
static uint8_t BlueSerial_Buffer[BLUE_SERIAL_BUFFER_SIZE];
static volatile uint16_t BlueSerial_PutIndex = 0;  /**< 写入索引 (ISR→) */
static volatile uint16_t BlueSerial_GetIndex = 0;  /**< 读取索引 (←主循环) */

/** HAL 中断接收单字节缓冲 (与环形缓冲区分离, 防止数据冲突) */
static volatile uint8_t bt_rx_byte;

/** 协议解析结果: 最多 6 个字段, 每字段最长 20 字符 */
char BlueSerial_StringArray[6][20];
static char BlueSerial_String[100];  /**< 帧内字符串 (去除 '[' 和 ']') */

/**
 * @brief  初始化蓝牙串口: 清空缓冲区 + 启动中断接收
 */
void BlueSerial_Init(void)
{
    BlueSerial_ClearBuffer();
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&bt_rx_byte, 1);
}

/**
 * @brief  通过蓝牙发送一个字节
 */
void BlueSerial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart2, &Byte, 1, 10);
}

/**
 * @brief  通过蓝牙发送字节数组
 */
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit(&huart2, Array, Length, 1000);
}

/**
 * @brief  通过蓝牙发送字符串
 */
void BlueSerial_SendString(char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++)
        BlueSerial_SendByte(String[i]);
}

/**
 * @brief  格式化蓝牙打印
 */
void BlueSerial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    BlueSerial_SendString(String);
}

/**
 * @brief  清空环形缓冲区
 */
void BlueSerial_ClearBuffer(void)
{
    for (uint8_t i = 0; i < BLUE_SERIAL_BUFFER_SIZE; i++)
        BlueSerial_Buffer[i] = 0;
    BlueSerial_PutIndex = 0;
    BlueSerial_GetIndex = 0;
}

/**
 * @brief  向环形缓冲区写入一个字节 (生产者)
 * @param  Byte 待写入字节
 * @note   缓冲区满时静默丢弃
 */
static void BlueSerial_Put(uint8_t Byte)
{
    /* 缓冲区满则丢弃 */
    if ((BlueSerial_PutIndex + 1) % BLUE_SERIAL_BUFFER_SIZE == BlueSerial_GetIndex)
        return;
    BlueSerial_Buffer[BlueSerial_PutIndex] = Byte;
    if (BlueSerial_PutIndex >= BLUE_SERIAL_BUFFER_SIZE - 1)
        BlueSerial_PutIndex = 0;
    else
        BlueSerial_PutIndex++;
}

/**
 * @brief  从环形缓冲区读取一个字节 (消费者)
 * @param  Byte 输出参数: 读取的字节
 * @return 0=成功, 1=缓冲区空
 */
static uint8_t BlueSerial_Get(uint8_t *Byte)
{
    if (BlueSerial_GetIndex == BlueSerial_PutIndex) {
        *Byte = 0;
        return 1;  /* 缓冲区空 */
    }
    *Byte = BlueSerial_Buffer[BlueSerial_GetIndex];
    if (BlueSerial_GetIndex >= BLUE_SERIAL_BUFFER_SIZE - 1)
        BlueSerial_GetIndex = 0;
    else
        BlueSerial_GetIndex++;
    return 0;
}

/**
 * @brief  获取环形缓冲区中有效数据长度
 * @return 缓冲区中未读取的字节数
 */
static uint16_t BlueSerial_Length(void)
{
    return (BlueSerial_PutIndex + BLUE_SERIAL_BUFFER_SIZE - BlueSerial_GetIndex)
           % BLUE_SERIAL_BUFFER_SIZE;
}

/**
 * @brief  按索引读取缓冲区数据 (不移除)
 * @param  Index 从读取位置开始的偏移
 * @return 缓冲区中指定位置的字节
 */
static uint8_t BlueSerial_Read(uint16_t Index)
{
    return BlueSerial_Buffer[(BlueSerial_GetIndex + Index) % BLUE_SERIAL_BUFFER_SIZE];
}

/**
 * @brief  检测是否收到完整的数据帧 (帧头 '[' + 帧尾 ']')
 * @return 1=收到完整帧, 0=未收到
 */
uint8_t BlueSerial_ReceiveFlag(void)
{
    uint8_t Flag = 0;
    uint16_t Length = BlueSerial_Length();
    for (uint16_t i = 0; i < Length; i++) {
        if (BlueSerial_Read(i) == '[')
            Flag = 1;
        else if (BlueSerial_Read(i) == ']' && Flag == 1)
            return 1;  /* 找到完整帧 */
    }
    return 0;
}

/**
 * @brief  接收并解析蓝牙数据帧
 * @note   将 '[' 和 ']' 之间的内容按 ',' 分割, 存入 BlueSerial_StringArray
 *         例: "[joystick,0,50,0,0]" → {"joystick","0","50","0","0"}
 */
void BlueSerial_Receive(void)
{
    uint16_t p = 0, k = 0;
    uint8_t Flag = 0;
    uint16_t Length = BlueSerial_Length();

    /* 清除上一帧残留，防止旧字段值污染新帧解析 */
    memset(BlueSerial_StringArray, 0, sizeof(BlueSerial_StringArray));
    memset(BlueSerial_String, 0, sizeof(BlueSerial_String));

    /* 提取 '[' 和 ']' 之间的内容 */
    for (uint16_t i = 0; i < Length; i++) {
        uint8_t Byte;
        BlueSerial_Get(&Byte);
        if (Byte == '[') {
            Flag = 1;
            p = 0;
        } else if (Byte == ']' && Flag == 1) {
            if (p < sizeof(BlueSerial_String)) {
                BlueSerial_String[p] = '\0';
            }
            break;
        } else if (Flag == 1) {
            if (p < sizeof(BlueSerial_String) - 1) {
                BlueSerial_String[p] = Byte;
                p++;
            }
        }
    }

    /* 按逗号分割字段 */
    p = 0;
    k = 0;
    for (uint16_t i = 0; BlueSerial_String[i] != '\0'; i++) {
        if (BlueSerial_String[i] == ',') {
            if (k < 20) BlueSerial_StringArray[p][k] = '\0';
            p++;
            if (p >= 6) break;  /* 最多 6 个字段 */
            k = 0;
        } else {
            if (k < 19) {
                BlueSerial_StringArray[p][k] = BlueSerial_String[i];
                k++;
            }
        }
    }
    if (p < 6 && k < 20) {
        BlueSerial_StringArray[p][k] = '\0';
    }
}

/**
 * @brief  UART 接收完成回调 (中断上下文)
 * @note   每收到 1 字节即写入环形缓冲区, 并重新启动下一次接收
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        BlueSerial_Put(bt_rx_byte);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&bt_rx_byte, 1);
    }
    else if (huart->Instance == USART3) {
        PIDTune_RxCallback(pid_tune_rx_byte);
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&pid_tune_rx_byte, 1);
    }
}

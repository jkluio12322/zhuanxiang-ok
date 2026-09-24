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
#include "bytestream.h"
#include "pid_tune.h"
#include "ringbuf.h"
#include "serial.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

/* 蓝牙接收环形缓冲区容量，单位字节。 */
#define BLUE_SERIAL_BUFFER_SIZE  100

/** 环形缓冲区 (ISR 写入, 主循环读取 → volatile 防止编译器缓存) */
static uint8_t BlueSerial_Buffer[BLUE_SERIAL_BUFFER_SIZE];
static RingBuf BlueSerialRx;

/** HAL 中断接收单字节缓冲 (与环形缓冲区分离, 防止数据冲突) */
static volatile uint8_t bt_rx_byte;

/** 协议解析结果: 最多 6 个字段, 每字段最长 20 字符 */
char BlueSerial_StringArray[6][20];
static char BlueSerial_String[100];  /**< 帧内字符串 (去除 '[' 和 ']') */

/** 蓝牙单字节发送流，使用 10 ms HAL 发送超时。 */
static ByteStream_t blueByteStream;

/** 蓝牙数组和字符串发送流，使用 1000 ms HAL 发送超时。 */
static ByteStream_t blueArrayByteStream;

/**
 * @brief  蓝牙单字节底层写适配器。
 * @param  data 待发送数据指针。
 * @param  length 待发送字节数。
 * @return HAL 发送成功时返回 length，否则返回 0。
 * @note   单字节接口保持原有 10 ms 超时，避免阻塞控制循环过久。
 */
static uint16_t BlueSerial_ByteStreamWrite(const uint8_t *data,
                                           uint16_t length)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2,
                                                 (uint8_t *)data,
                                                 length,
                                                 10U);

    return (status == HAL_OK) ? length : 0U;
}

/**
 * @brief  蓝牙数组和字符串底层写适配器。
 * @param  data 待发送数据指针。
 * @param  length 待发送字节数。
 * @return HAL 发送成功时返回 length，否则返回 0。
 * @note   数组和字符串保持原有 1000 ms 超时；与单字节流分离，
 *         可在不改变公开接口的前提下保留两套超时行为。
 */
static uint16_t BlueSerial_ArrayByteStreamWrite(const uint8_t *data,
                                                uint16_t length)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2,
                                                 (uint8_t *)data,
                                                 length,
                                                 1000U);

    return (status == HAL_OK) ? length : 0U;
}

/**
 * @brief  初始化蓝牙串口: 清空缓冲区 + 启动中断接收。
 */
void BlueSerial_Init(void)
{
    blueByteStream.write = BlueSerial_ByteStreamWrite;
    blueArrayByteStream.write = BlueSerial_ArrayByteStreamWrite;
    RingBuf_Init(&BlueSerialRx, BlueSerial_Buffer,
                 BLUE_SERIAL_BUFFER_SIZE);
    BlueSerial_ClearBuffer();
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&bt_rx_byte, 1);
}

/**
 * @brief  清空环形缓冲区
 */
void BlueSerial_ClearBuffer(void)
{
    memset(BlueSerial_Buffer, 0, sizeof(BlueSerial_Buffer));
    RingBuf_Flush(&BlueSerialRx);
}

/**
 * @brief  通过蓝牙发送一个字节
 * @param  Byte 待发送字节。
 */
void BlueSerial_SendByte(uint8_t Byte)
{
    ByteStream_SendByte(&blueByteStream, Byte);
}

/**
 * @brief  通过蓝牙发送字节数组
 * @param  Array 数据指针。
 * @param  Length 数据长度，单位字节。
 */
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
    ByteStream_SendArray(&blueArrayByteStream, Array, Length);
}

/**
 * @brief  通过蓝牙发送字符串
 * @param  String 以 '\0' 结尾的字符串。
 */
void BlueSerial_SendString(char *String)
{
    ByteStream_SendString(&blueArrayByteStream, String);
}

/**
 * @brief  格式化蓝牙打印
 * @param  format printf 风格格式字符串。
 * @param  ... 可变参数列表。
 */
void BlueSerial_Printf(char *format, ...)
{
    va_list arg;

    va_start(arg, format);
    ByteStream_VPrintf(&blueArrayByteStream, format, arg);
    va_end(arg);
}

/**
 * @brief  检测是否收到完整的数据帧 (帧头 '[' + 帧尾 ']')
 * @return 1=收到完整帧, 0=未收到
 */
uint8_t BlueSerial_ReceiveFlag(void)
{
    uint8_t Flag = 0;
    uint16_t Length = RingBuf_Available(&BlueSerialRx);

    for (uint16_t i = 0; i < Length; i++) {
        uint8_t Byte;

        if (RingBuf_Peek(&BlueSerialRx, i, &Byte) < 0) break;

        if (Byte == '[')
            Flag = 1;
        else if (Byte == ']' && Flag == 1)
            return 1;  /* 找到完整帧 */
    }
    return 0;
}

/**
 * @brief  接收并解析蓝牙数据帧
 * @note   将 '[' 和 ']' 之间的内容按 ',' 分割, 存入 BlueSerial_StringArray
 *         例: "[joystick,0,50,0,0]" → {"joystick","0","50","0","0"}
 */
/**
 * @brief  从蓝牙接收缓冲区提取并解析一帧数据。
 * @note   输入帧使用 '[' 和 ']' 定界，最多解析 6 个逗号分隔字段；
 *         未完整到达的帧不会被消费。
 */
void BlueSerial_Receive(void)
{
    uint16_t p = 0, k = 0;
    uint8_t Flag = 0;
    uint16_t Length = RingBuf_Available(&BlueSerialRx);

    /* 清除上一帧残留，防止旧字段值污染新帧解析 */
    memset(BlueSerial_StringArray, 0, sizeof(BlueSerial_StringArray));
    memset(BlueSerial_String, 0, sizeof(BlueSerial_String));

    /* 提取 '[' 和 ']' 之间的内容 */
    for (uint16_t i = 0; i < Length; i++) {
        uint8_t Byte;
        if (RingBuf_Get(&BlueSerialRx, &Byte) < 0) break;
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
 * @param  huart 触发回调的 UART 句柄。
 * @note   USART2 每收到 1 字节即写入环形缓冲区并重新启动接收；USART1
 *         将字节交给在线调参模块缓存。不得在中断中解析命令或执行阻塞操作。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        (void)RingBuf_PutNB(&BlueSerialRx, bt_rx_byte);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&bt_rx_byte, 1);
    }
    else if (huart->Instance == USART1) {
        pidTuneRxCallback(pidTuneRxByte);
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&pidTuneRxByte, 1);
    }
}

/**
 * @brief  UART 发送完成回调 (中断上下文)
 * @param  huart 触发回调的 UART 句柄。
 * @note   USART1 TX 完成后接力发送下一块数据；不得在中断中调用阻塞发送。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Serial_TX_Done();
    }
}

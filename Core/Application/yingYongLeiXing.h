/**
 * @file    yingYongLeiXing.h
 * @brief   应用层公共类型定义。
 * @note    定义命令类型、命令消息结构和工作模式枚举。
 */
#ifndef __YING_YONG_LEI_XING_H
#define __YING_YONG_LEI_XING_H  /* 头文件保护宏，防止类型定义被重复包含。 */

#include <stdint.h>

/**
 * @brief 应用层控制命令类型。
 * @note 按键、蓝牙和串口调参均转换为该枚举后进入统一命令队列。
 */
typedef enum {
    commandNone = 0,       /* 空命令，表示没有有效控制动作。 */
    commandSpeed,          /* 设置左右轮相同目标速度。 */
    commandSpeedLeft,      /* 单独设置左轮目标速度。 */
    commandSpeedRight,     /* 单独设置右轮目标速度。 */
    commandTurn,           /* 设置左右轮差速转向量。 */
    commandPivot,          /* 设置一正一反原地转，正值右转、负值左转。 */
    commandStop,           /* 停车并清空运动状态。 */
    commandStart,          /* 允许车辆进入运行状态。 */
    commandGearUp,         /* 速度档位加一。 */
    commandGearDown,       /* 速度档位减一。 */
    commandLapCycle,       /* 循迹目标圈数按 1~5 循环一次。 */
    commandSetSpeedKp,     /* 同时设置左右增量式速度环 Kp。 */
    commandSetSpeedKi,     /* 同时设置左右增量式速度环 Ki。 */
    commandSetSpeedKd,     /* 同时设置左右增量式速度环 Kd。 */
    commandSetSpeedKf,     /* 旧前馈命令，增量式速度环不使用该参数。 */
    commandSetYawKp,       /* 设置航向角环 Kp。 */
    commandSetYawKi,       /* 设置航向角环 Ki。 */
    commandSetYawKd,       /* 设置航向角环 Kd。 */
    commandSetYawKf,       /* 设置航向角环前馈系数。 */
    commandYawTarget,      /* 设置航向角目标值。 */
    commandQuiet,          /* 暂停 VOFA 遥测输出。 */
    commandLog,            /* 恢复 VOFA 遥测输出。 */
    commandImuCalib,       /* 将当前航向角设置为零度。 */
    commandImuZero,        /* 航向角清零的兼容命令。 */
    commandModeTrack,      /* 切换到循迹模式。 */
    commandModeRun,        /* 切换到蓝牙/手动速度模式。 */
    commandModePwm         /* 切换到 PWM 开环测试模式。 */
} commandType_t;

/**
 * @brief 应用层命令消息。
 * @note 命令类型决定 value 的解释方式，例如速度、PID 系数或转向量。
 */
typedef struct {
    commandType_t type;  /* 命令类型。 */
    float value;         /* 命令参数值。 */
} commandMessage_t;

/**
 * @brief 应用层工作模式。
 */
typedef enum {
    modeRun = 0,  /* 蓝牙/手动速度闭环模式。 */
    modePwmSet,   /* PWM 开环测试模式。 */
    modeTrack     /* 8 路数字循迹模式。 */
} workMode_t;

#endif

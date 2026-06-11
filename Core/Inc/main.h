/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/**
 * @brief 命令类型枚举 — 协程架构的消息协议
 * @note  命令流: BT/Key/Serial → CmdQueue → Task_Control_50ms
 *        电机控制命令通过队列解耦, PID 参数命令直接生效
 */
typedef enum {
    CMD_NONE = 0,              /**< 空命令 (占位/无操作) */
    CMD_SPEED,                 /**< 设置 SpeedPID 目标 RPM */
    CMD_TURN,                  /**< 设置手动转向差速 RPM (TurnTarget) */
    CMD_STOP,                  /**< 停车: RunFlag=0, 清空 PID 状态 */
    CMD_START,                 /**< 启动: RunFlag=1 */
    CMD_GEAR_UP,               /**< 速度档位 +1 (1~6) */
    CMD_GEAR_DOWN,             /**< 速度档位 -1 (1~6) */
    CMD_SET_SPEED_KP,          /**< SpeedPID Kp 在线调参 */
    CMD_SET_SPEED_KI,          /**< SpeedPID Ki 在线调参 */
    CMD_SET_SPEED_KD,          /**< SpeedPID Kd 在线调参 */
    CMD_SET_SPEED_KF,          /**< SpeedPID Kf 在线调参 */
    CMD_SET_YAW_KP,            /**< YawPID Kp 在线调参 */
    CMD_SET_YAW_KI,            /**< YawPID Ki 在线调参 */
    CMD_SET_YAW_KD,            /**< YawPID Kd 在线调参 */
    CMD_SET_YAW_KF,            /**< YawPID Kf 在线调参 */
    CMD_YAW_TARGET,            /**< 设置 YawPID 目标角度 (°) */
    CMD_QUIET,                 /**< 静默模式: 关闭 VOFA 数据流 */
    CMD_LOG,                   /**< 恢复 VOFA 数据流输出 */
} CmdType;

/**
 * @brief 命令消息 — SPSC 队列元素
 * @note  type=命令类型, val=参数值 (RPM/角度/百分比 等, 语义由 type 决定)
 */
typedef struct {
    CmdType type;              /**< 命令类型 */
    float   val;               /**< 命令参数值 */
} CmdMsg;

/**
 * @brief 系统运行模式
 * @note  MODE_RUN:      级联 PID 速度/转向控制 (正常模式)
 *        MODE_PWM_SET:  直接 PWM 占空比输出 (调试模式, 按键切换)
 */
typedef enum { MODE_RUN, MODE_PWM_SET } Mode_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/** @brief 向命令队列投递消息 (生产者: BT/Key/Serial ISR/主循环) */
void CmdQueue_Put(CmdMsg msg);

/** @brief 电机运行标志: 0=停止, 1=运行中 */
extern uint8_t  RunFlag;
/** @brief PWM 直接输出占空比 (MODE_PWM_SET 下使用, 范围 ±100) */
extern int16_t  PWMDuty;
/** @brief 当前系统运行模式 */
extern Mode_t   Mode;
/** @brief 手动转向差速目标 RPM (蓝牙摇杆/按键控制) */
extern float    TurnTarget;
/** @brief YawPID 目标偏航角 (°) */
extern float    YawTarget;
/** @brief 上次收到蓝牙数据的时间戳 (ms, 超时 300ms 自动停车) */
extern uint32_t bt_last_rx;
/** @brief VOFA 遥测输出使能: 0=静默, 1=输出 CSV 数据流 */
extern uint8_t  VOFA_Enable;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* ── TB6612 电机驱动引脚 ────────────────────────── */
#define pwm1_1_Pin GPIO_PIN_1           /**< 左电机方向1 (PF1) */
#define pwm1_1_GPIO_Port GPIOF
#define pwm1_0_Pin GPIO_PIN_2           /**< 左电机方向2 (PF2) */
#define pwm1_0_GPIO_Port GPIOF
#define pwm2_1_Pin GPIO_PIN_3           /**< 右电机方向1 (PF3) */
#define pwm2_1_GPIO_Port GPIOF
#define pwm2_0_Pin GPIO_PIN_4           /**< 右电机方向2 (PF4) */
#define pwm2_0_GPIO_Port GPIOF
/* ── 蜂鸣器 ──────────────────────────────────────── */
#define buzzer_Pin GPIO_PIN_6           /**< 有源蜂鸣器 (PG6, 低电平响) */
#define buzzer_GPIO_Port GPIOG
/* ── 按键 (上拉输入, 按下低电平) ──────────────────── */
#define key3_Pin GPIO_PIN_6             /**< KEY3: PC6 (DEC/后退) */
#define key3_GPIO_Port GPIOC
#define key2_Pin GPIO_PIN_7             /**< KEY2: PC7 (ADD/前进) */
#define key2_GPIO_Port GPIOC
#define key1_Pin GPIO_PIN_8             /**< KEY1: PC8 (SET/确认) */
#define key1_GPIO_Port GPIOC
#define key4_Pin GPIO_PIN_9             /**< KEY4: PC9 (档位切换) */
#define key4_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

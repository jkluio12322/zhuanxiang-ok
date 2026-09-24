/**
 * @file    yingYongZhuangTai.h
 * @brief   应用层状态、速度档位和 PID 接口。
 */

#ifndef __YING_YONG_ZHUANG_TAI_H
#define __YING_YONG_ZHUANG_TAI_H  /* 头文件保护宏，防止状态接口重复声明。 */

#include "main.h"
#include "yingYongLeiXing.h"
#include "encoder.h"
#include "mpu6050.h"
#include "pid.h"

extern PID_t speedPidAverage;

/* 全局运行状态，任务之间通过这些变量交换控制意图。 */
extern workMode_t workMode;        /* 当前工作模式。 */
extern uint8_t runFlag;            /* 1=运行，0=停车。 */
extern int16_t pwmDuty;            /* PWM 测试模式占空比，范围 -100~100。 */
extern float turnTarget;           /* 手动差速量或原地转速度，正值右转。 */
extern float yawTarget;            /* 航向角目标值。 */
extern int16_t bluetoothLv;        /* 蓝牙左摇杆纵向值。 */
extern int16_t bluetoothRh;        /* 蓝牙右摇杆横向值。 */
extern uint32_t bluetoothLastRx;   /* 最近一次蓝牙数据接收时刻。 */
extern uint8_t vofaEnabled;        /* 1=允许 VOFA 输出。 */

/* PID、目标量和传感器对象由应用状态模块统一持有。 */
extern PID_t speedPidLeft;         /* 左轮增量式速度环。 */
extern PID_t speedPidRight;        /* 右轮增量式速度环。 */
extern float targetLeftRpm;        /* 左轮最终目标转速，单位 RPM。 */
extern float targetRightRpm;       /* 右轮最终目标转速，单位 RPM。 */
extern PID_t yawPid;               /* 航向角环。 */

extern Encoder encoderLeft;        /* 左轮编码器。 */
extern Encoder encoderRight;       /* 右轮编码器。 */
extern MPU6050 imu;                /* MPU6050 姿态数据。 */

/**
 * @brief  初始化编码器、PID 和默认运行状态。
 */
void appStateInit(void);

/**
 * @brief  立即复位运动状态并关停电机。
 * @note   清空左右轮目标、转向量、摇杆值以及两个增量速度环和航向环
 *         的历史量。
 */
void appStateResetMotion(void);

/**
 * @brief  清除航向角环的历史状态，保留在线调参得到的增益。
 */
void appStateResetYawPid(void);

/**
 * @brief  获取当前速度档位。
 * @return 当前档位编号，范围 1~6。
 */
uint8_t appStateGetSpeedLevel(void);

/**
 * @brief  设置当前速度档位。
 * @param  level 目标档位，越界时按 1~6 循环处理。
 */
void appStateSetSpeedLevel(uint8_t level);

/**
 * @brief  按方向调整当前速度档位。
 * @param  direction 大于 0 表示升档，小于 0 表示降档。
 * @note   循迹模式限制在 G1-G6 内，手动模式在 G1-G6 之间循环。
 */
void appStateStepSpeedLevel(int8_t direction);

/**
 * @brief  获取当前档位对应的中心速度。
 * @return 当前档位速度，单位 RPM。
 */
float appStateGetSpeedRpm(void);

/**
 * @brief  获取循迹目标圈数。
 * @return 1~5 圈。
 */
uint8_t appStateGetLapTarget(void);

/**
 * @brief  设置循迹目标圈数。
 * @param  lapTarget 目标圈数，越界时限制到 1~5。
 */
void appStateSetLapTarget(uint8_t lapTarget);

/**
 * @brief  将循迹目标圈数按 1~5 循环增加一次。
 */
void appStateCycleLapTarget(void);

/**
 * @brief  获取当前已完成的循迹圈数。
 */
uint8_t appStateGetCompletedLaps(void);

/**
 * @brief  增加已完成圈数。
 */
void appStateIncrementCompletedLaps(void);

/**
 * @brief  清零已完成圈数，保留目标圈数设置。
 */
void appStateResetLapProgress(void);

#endif

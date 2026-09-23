/**
 * @file    yingYongXunJi.h
 * @brief   八路数字循迹控制接口。
 */
#ifndef __YING_YONG_XUN_JI_H
#define __YING_YONG_XUN_JI_H

#include <stdint.h>

/**
 * @brief  循迹控制状态机状态。
 */
typedef enum {
    TRACK_CONTROL_FOLLOW = 0,  /* 普通查表循迹。 */
    TRACK_CONTROL_TURN_LEFT,   /* 正在执行左急弯。 */
    TRACK_CONTROL_TURN_RIGHT,  /* 正在执行右急弯。 */
    TRACK_CONTROL_REACQUIRE,   /* 急弯硬阈值后低速重捕并等待线路居中。 */
    TRACK_CONTROL_FINISH,      /* 已检测到终点图案。 */
    TRACK_CONTROL_LOST         /* 正在左右摆头找线。 */
} TrackControlState_t;

/**
 * @brief  循迹状态机诊断快照。
 */
typedef struct {
    TrackControlState_t state;       /* 当前控制状态。 */
    uint8_t pattern;                 /* 黑线图案，bit0 对应 IR1。 */
    uint8_t blackCount;              /* 检测到黑线的通道数量。 */
    int8_t candidateDirection;       /* 边缘候选方向，-1 左、+1 右。 */
    uint8_t candidateTicks;          /* 边缘候选保持计数。 */
    int8_t pendingCornerDirection;   /* 待确认的下一弯方向。 */
    uint8_t pendingCornerTicks;      /* 下一弯连续确认计数。 */
    uint8_t invalidLineTicks;        /* 无效线路连续确认计数。 */
    uint16_t turnTicks;              /* 当前急弯已运行周期数。 */
    uint16_t reacquireTicks;         /* 急弯后低速重捕已运行周期数。 */
} TrackDiagnostics_t;

/**
 * @brief  复位循迹状态机并清除红外数据。
 */
void appTrackingResetState(void);

/**
 * @brief  启动新的循迹任务并清零已完成圈数。
 */
void appTrackingStart(void);

/**
 * @brief  执行一次 20 ms 循迹任务。
 */
void appTrackingTask20ms(void);

/**
 * @brief  获取当前循迹状态。
 * @return 当前状态机枚举值。
 */
TrackControlState_t appTrackingGetState(void);

/**
 * @brief  获取归一化线路位置。
 * @return 负值偏左、正值偏右，范围约为 -7...+7。
 */
float appTrackingGetPosition(void);

/**
 * @brief  获取循迹差速输出。
 * @return 左右目标差的一半，单位 RPM。
 */
float appTrackingGetSteer(void);

/**
 * @brief  获取循迹状态机诊断快照。
 * @param  output 输出结构体指针。
 */
void appTrackingGetDiagnostics(TrackDiagnostics_t *output);

#endif

/**
 * @file    app_state.h
 * @brief   应用层唯一可变状态。
 * @note    本模块只持有数据：读走只读视图 AppState()，写走 AppState_SetXxx()。
 *          启停编排、模式切换等**动作**不在这里，它们属于控制层——否则
 *          app_state 和 app_testmode 会互相依赖形成环。
 *          本头只依赖 app_types.h 和 <stdint.h>，不含任何硬件头。
 */
#ifndef __APP_STATE_H
#define __APP_STATE_H

#include <stdint.h>
#include "app_types.h"

/* 只读状态视图。字段直接读，不为每个字段另配 getter。 */
typedef struct {
    workMode_t mode;            /* 当前工作模式。 */
    uint8_t    running;         /* 1 表示正在运行。 */
    uint8_t    vofa;            /* 1 表示允许发送 VOFA 遥测。 */
    uint8_t    gear;            /* 当前档位，1..SPEED_LEVEL_COUNT。 */
    float      gear_rpm;        /* 当前档位对应的目标转速，单位 RPM。 */
    uint8_t    lap_target;      /* 目标完成圈数。 */
    uint8_t    laps_done;       /* 已完成圈数。 */
    float      target_left_rpm; /* 左轮目标转速，单位 RPM。 */
    float      target_right_rpm;/* 右轮目标转速，单位 RPM。 */
} AppStateView_t;

/**
 * @brief  获取只读状态视图。
 */
const AppStateView_t *AppState(void);

/**
 * @brief  初始化状态到默认值。
 */
void AppState_Init(void);

/**
 * @brief  设置工作模式。
 */
void AppState_SetMode(workMode_t mode);

/**
 * @brief  设置运行标志。
 */
void AppState_SetRunning(uint8_t running);

/**
 * @brief  设置 VOFA 遥测开关。
 */
void AppState_SetVofa(uint8_t on);

/**
 * @brief  设置档位，越界时回绕。
 * @return 1 表示档位发生了变化。
 */
uint8_t AppState_SetGear(uint8_t gear);

/**
 * @brief  按方向增减档位，越界时回绕。
 */
void AppState_StepGear(int8_t direction);

/**
 * @brief  目标圈数循环加一。
 */
void AppState_CycleLapTarget(void);

/**
 * @brief  完成圈数加一，不超过目标圈数。
 */
void AppState_IncrementLap(void);

/**
 * @brief  清除圈数进度。
 */
void AppState_ResetLapProgress(void);

/**
 * @brief  设置左右轮目标转速。
 */
void AppState_SetWheelTargets(float left_rpm, float right_rpm);

/**
 * @brief  上报一次停车事件。
 * @param  reason 停车原因，stopReasonNone 会被忽略。
 * @note   只置标志，由遥测任务取走后打印 [STOP] 行。
 */
void AppState_ReportStop(appStopReason_t reason);

/**
 * @brief  取走待上报的停车事件并清除标志。
 * @param  reason 输出停车原因。
 * @return 1 表示有事件且 reason 有效。
 */
uint8_t AppState_TakeStopEvent(appStopReason_t *reason);

/**
 * @brief  清除停车事件标志。
 */
void AppState_ClearStopEvent(void);

/**
 * @brief  取走并清除「请求停车」标志。
 * @return 1 表示有模块请求停车。
 * @note   上报停车事件和真正执行停车是两件事：循迹模块只上报，
 *         由 50 ms 控制任务取走这个标志后执行停车，避免两边互相依赖。
 */
uint8_t AppState_TakeStopRequest(void);

#endif /* __APP_STATE_H */

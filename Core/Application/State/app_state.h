/* 应用层唯一可变状态。本模块只持有数据：读走只读视图 AppState()，写走 AppState_SetXxx()。
   启停编排、模式切换等**动作**不在这里，它们属于控制层——否则 app_state 和 app_testmode
   会互相依赖形成环。本头只依赖 app_types.h 和 <stdint.h>，不含任何硬件头。 */
#ifndef __APP_STATE_H
#define __APP_STATE_H

#include <stdint.h>
#include "app_types.h"

/* 只读状态视图。字段直接读，不为每个字段另配 getter。
   每个字段都带 volatile：它们由 TIM6 中断里的任务写、由主循环读。没有 volatile 时
   优化器可以把主循环里的一次字段读提到循环外反复使用。字段都是 32 位以内的标量，
   读写本身原子，所以不需要临界区。 */
typedef struct {
    volatile workMode_t mode;            /* 当前工作模式 */
    volatile uint8_t    running;         /* 1 表示正在运行 */
    volatile uint8_t    vofa;            /* 1 表示允许发送 VOFA 遥测 */
    volatile uint8_t    gear;            /* 当前档位，1..SPEED_LEVEL_COUNT */
    volatile float      gear_rpm;        /* 当前档位对应的目标转速，单位 RPM */
    volatile uint8_t    lap_target;      /* 目标完成圈数 */
    volatile uint8_t    laps_done;       /* 已完成圈数 */
    volatile float      target_left_rpm; /* 左轮目标转速，单位 RPM */
    volatile float      target_right_rpm;/* 右轮目标转速，单位 RPM */
} AppStateView_t;

const AppStateView_t *AppState(void);      /* 取只读状态视图 */
void AppState_Init(void);                  /* 初始化到默认值 */
void AppState_SetMode(workMode_t mode);    /* 设置工作模式 */

/* 取模式循环里的下一个：modeTrack -> modePwmSpeed -> modePwmTrack -> modeTrack。
   按键轮换和开机页轮换共用这一处，避免两套循环逻辑各自漂移。 */
workMode_t AppState_NextMode(void);

const char *AppModeName(workMode_t mode);  /* 模式名，OLED 和串口命令行共用 */

void AppState_SetRunning(uint8_t running); /* 设置运行标志 */
void AppState_SetVofa(uint8_t on);         /* 设置 VOFA 遥测开关 */

uint8_t AppState_SetGear(uint8_t gear);    /* 设置档位，越界回绕；返回 1 表示档位变了 */
void AppState_StepGear(int8_t direction);  /* 按方向增减档位，越界回绕 */
void AppState_CycleLapTarget(void);        /* 目标圈数循环加一 */
void AppState_IncrementLap(void);          /* 完成圈数加一，不超过目标圈数 */
void AppState_ResetLapProgress(void);      /* 清除圈数进度 */
void AppState_SetWheelTargets(float left_rpm, float right_rpm);

/* 上报一次停车事件。stopReasonNone 会被忽略；只置标志，
   由遥测任务取走后打印 [STOP] 行。 */
void AppState_ReportStop(appStopReason_t reason);

uint8_t AppState_TakeStopEvent(appStopReason_t *reason);  /* 取走并清除；1 = 有事件，reason 有效 */
void AppState_ClearStopEvent(void);                       /* 清除停车事件标志 */

/* 取走并清除「请求停车」标志。上报停车事件和真正执行停车是两件事：
   循迹模块只上报，由 50 ms 控制任务取走这个标志后执行停车，避免两边互相依赖。 */
uint8_t AppState_TakeStopRequest(void);

#endif

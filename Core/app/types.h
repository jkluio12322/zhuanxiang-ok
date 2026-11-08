/* 应用层共用类型。只保留实际用到的枚举，避免变成又一个 types 垃圾场。 */
#ifndef __TYPES_H
#define __TYPES_H

#include <stdint.h>

/* 运行模式 */
typedef enum {
    APP_MODE_IDLE = 0,   /* 待机，电机停 */
    APP_MODE_TRACK,      /* 循迹 */
    APP_MODE_MANUAL      /* 串口给固定轮速 */
} AppMode;

/* 停车原因：显示在 OLED 上，用来快速判断为什么停 */
typedef enum {
    STOP_NONE = 0,
    STOP_USER,           /* 串口 stop */
    STOP_LINE_LOST,      /* 丢线超过 TRACK_LOST_HOLD_MS */
    STOP_LAP_DONE,       /* 圈数达到目标 */
    STOP_CLOCK_FAIL      /* HSE 失效，时钟安全系统触发 */
} StopReason;

#endif /* __TYPES_H */

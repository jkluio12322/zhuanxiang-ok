/* VOFA+ 遥测接口。 */
#ifndef __TELEMETRY_H
#define __TELEMETRY_H  /* 头文件保护宏，防止遥测接口重复声明。 */

#include <stdint.h>

/* VOFA+ 遥测快照。轮速/差速单位 RPM，航向角单位度，mode 为 Mode 整数。 */
typedef struct {
    float targetLeftRpm;          /* 左轮目标速度，单位 RPM。 */
    float targetRightRpm;         /* 右轮目标速度，单位 RPM。 */
    float actualLeftRpm;          /* 左轮编码器速度，单位 RPM。 */
    float actualRightRpm;         /* 右轮编码器速度，单位 RPM。 */
    float filteredLeftRpm;        /* 左轮滤波后速度，单位 RPM。 */
    float filteredRightRpm;       /* 右轮滤波后速度，单位 RPM。 */
    float steerRpm;               /* 航向或循迹差速量，单位 RPM。 */
    float actualDiffRpm;          /* 左轮减右轮的实际速度差，单位 RPM。 */
    float yawAngleDeg;            /* IMU963RA 航向角，单位度。 */
    float trackError;             /* 循迹加权误差，负值偏左，正值偏右。 */
    int16_t mode;                 /* 当前 Mode 值。 */
    int16_t runFlag;              /* 1 表示运行，0 表示停车。 */
    int16_t gear;                 /* 当前速度档位。 */
    int16_t turnPhase;            /* TrackTurnPhase_t 整数。 */
    float commandVoltageLeft;     /* 左轮最终命令电压，单位 V。 */
    float commandVoltageRight;    /* 右轮最终命令电压，单位 V。 */
volatile int16_t fresh;       /* 1 表示有尚未发送的新数据。跨上下文：5 ms 中断置 1，
                                     主循环读并清零，故必须 volatile。 */
} telemetryData_t;

extern telemetryData_t telemetry_data;

/* 清零遥测快照。 */
void telemetry_init(void);

/* 发送开机横幅和 VOFA CSV 表头。字段顺序必须与 telemetry_send() 格式串一致，
   这里是表头唯一来源，不要在别处硬编码。 */
void telemetry_boot_header(void);

/* 用本周期控制结果填充遥测快照（中断上下文，5 ms 控制任务调用，只赋值不格式化）。
   参数单位 RPM/百分比。速度与姿态字段直接读只读视图，其余由调用方传入。最后置
   fresh=1 是通知主循环"有新快照"的唯一入口。 */
void telemetry_publish(float targetLeft, float targetRight,
                          int16_t leftPwm, int16_t rightPwm,
                          float steerRpm, float trackError,
                          int16_t turnPhase, float rpmDifference);

/* 将最新遥测快照格式化为 VOFA CSV 并发送（主循环调用，不在中断里）。 */
void telemetry_send(void);

#endif

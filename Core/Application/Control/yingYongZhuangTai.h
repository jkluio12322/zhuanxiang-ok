#ifndef __YING_YONG_ZHUANG_TAI_H
#define __YING_YONG_ZHUANG_TAI_H

#include "main.h"
#include "yingYongLeiXing.h"
#include "encoder.h"
#include "mpu6050.h"
#include "pid.h"

extern workMode_t workMode;
extern uint8_t runFlag;
extern uint8_t vofaEnabled;
extern PID_t speedPidLeft;
extern PID_t speedPidRight;
extern PID_t speedPidAverage;
extern float targetLeftRpm;
extern float targetRightRpm;
extern Encoder encoderLeft;
extern Encoder encoderRight;
extern MPU6050 imu;

void appStateInit(void);
void appStateResetMotion(void);
void appStateResetSpeedPidRuntime(void);
void appStateReportStop(appStopReason_t reason);
uint8_t appStateGetStopEvent(appStopReason_t *reason);
void appStateClearStopEvent(void);
uint8_t appStateGetSpeedLevel(void);
void appStateSetSpeedLevel(uint8_t level);
void appStateStepSpeedLevel(int8_t direction);
float appStateGetSpeedRpm(void);
uint8_t appStateGetLapTarget(void);
void appStateSetLapTarget(uint8_t lapTarget);
void appStateCycleLapTarget(void);
uint8_t appStateGetCompletedLaps(void);
void appStateIncrementCompletedLaps(void);
void appStateResetLapProgress(void);

#endif

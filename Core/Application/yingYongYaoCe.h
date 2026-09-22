/**
 * @file    yingYongYaoCe.h
 * @brief   VOFA telemetry interface.
 */
#ifndef __YING_YONG_YAO_CE_H
#define __YING_YONG_YAO_CE_H

#include <stdint.h>

/*
 * Canonical telemetry units:
 * - wheel speed and differential: RPM
 * - yaw angle: degrees
 * - work mode: workMode_t integer
 */
typedef struct {
    float targetLeftRpm;          /* Left wheel target speed, RPM. */
    float targetRightRpm;         /* Right wheel target speed, RPM. */
    float actualLeftRpm;          /* Left wheel encoder speed, RPM. */
    float actualRightRpm;         /* Right wheel encoder speed, RPM. */
    float filteredLeftRpm;        /* Left wheel filtered speed, RPM. */
    float filteredRightRpm;       /* Right wheel filtered speed, RPM. */
    float steerRpm;               /* Yaw or line-following steering differential, RPM. */
    float actualDiffRpm;          /* Actual left-right speed difference, RPM. */
    float yawAngleDeg;            /* MPU6050 yaw angle, degrees. */
    int16_t mode;                 /* Current workMode_t value. */
    int16_t runFlag;              /* 1=running, 0=stopped. */
    int16_t gear;                 /* Current speed level. */
    int16_t fresh;                /* 1=new telemetry sample. */
} telemetryData_t;

extern telemetryData_t telemetryData;

void appTelemetryInit(void);
void appTelemetrySend(void);

#endif

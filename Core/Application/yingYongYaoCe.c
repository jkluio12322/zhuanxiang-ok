/**
 * @file    yingYongYaoCe.c
 * @brief   VOFA+ CSV 遥测快照与发送。
 */

#include "yingYongYaoCe.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "serial.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

telemetryData_t telemetryData;

/**
 * @brief  初始化 VOFA 遥测数据结构。
 * @note   将全部遥测字段清零，保证首次发送前数据有效。
 */
void appTelemetryInit(void)
{
    memset(&telemetryData, 0, sizeof(telemetryData));
}

/**
 * @brief  将遥测快照格式化为 VOFA CSV 并发送。
 * @note   无新数据、VOFA 被关闭或车辆停车且转速接近 0 时不发送；
 *         PWM 百分比会换算为近似电压后输出。
 */
void appTelemetrySend(void)
{
    /* 无新数据、VOFA 已关闭或停车且转速接近 0 时不发送。 */
    if (!vofaEnabled || !telemetryData.fresh) return;
    if (!runFlag
        && fabsf(telemetryData.actualRightRpm) < TELEMETRY_STOP_RPM_EPSILON
        && fabsf(telemetryData.actualLeftRpm) < TELEMETRY_STOP_RPM_EPSILON) {
        return;
    }

    telemetryData.fresh = 0;

    char buffer[TELEMETRY_BUFFER_SIZE];
    int length;

    /*
     * CSV order and units:
     * tgtRpmL,rpmFL,rpmL,tgtRpmR,rpmFR,rpmR,
     * steerRpm,diffRpm,yawDeg,mode,run,gear
     */
    length = snprintf(buffer, sizeof(buffer),
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%d,%d,%d\r\n",
        telemetryData.targetLeftRpm,
        telemetryData.filteredLeftRpm,
        telemetryData.actualLeftRpm,
        telemetryData.targetRightRpm,
        telemetryData.filteredRightRpm,
        telemetryData.actualRightRpm,
        telemetryData.steerRpm,
        telemetryData.actualDiffRpm,
        telemetryData.yawAngleDeg,
        (int)telemetryData.mode,
        (int)telemetryData.runFlag,
        (int)telemetryData.gear);

    if (length > 0 && length < (int)sizeof(buffer)) {
        Serial_Write((const uint8_t *)buffer, (uint16_t)length);
    }
}

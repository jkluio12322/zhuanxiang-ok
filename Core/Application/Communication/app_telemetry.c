/**
 * @file    app_telemetry.c
 * @brief   VOFA+ CSV 遥测快照与发送。
 */

#include "app_telemetry.h"
#include "app_config.h"
#include "app_state.h"
#include "app_serial.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

telemetryData_t telemetryData;

/**
 * @brief  获取停车原因对应的 ASCII 名称。
 * @param  reason 停车原因。
 * @return 固定字符串名称；未知值返回 UNKNOWN。
 */
static const char *telemetryStopReasonName(appStopReason_t reason)
{
    switch (reason) {
    case stopReasonUser:
        return "USER";
    case stopReasonTargetLaps:
        return "TARGET_LAPS";
    case stopReasonAllWhite:
        return "ALL_WHITE";
    case stopReasonCornerTimeout:
        return "CORNER_TIMEOUT";
    case stopReasonModeSwitch:
        return "MODE_SWITCH";
    case stopReasonNone:
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief  初始化 VOFA 遥测数据结构。
 * @note   将全部遥测字段清零，保证首次发送前数据有效。
 */
void AppTelemetry_Init(void)
{
    memset(&telemetryData, 0, sizeof(telemetryData));
}

/**
 * @brief  将遥测快照格式化为 VOFA CSV 并发送。
 * @note   无新数据、VOFA 被关闭或车辆停车且转速接近 0 时不发送；
 *         PWM 百分比会换算为近似电压后输出。
 */
void AppTelemetry_BootHeader(void)
{
    /* 字段顺序与下面 Send 里的 snprintf 格式串一致，共 16 项。 */
    Serial_Printf("BOOT OK\r\n");
    Serial_Printf("#tgtRpmL,rpmFL,voltL,tgtRpmR,rpmFR,voltR,"
                  "steerRpm,diffRpm,yawDeg,trackError,"
                  "mode,run,gear,turnPhase,rpmL,rpmR\r\n");
}

void AppTelemetry_Send(void)
{
    appStopReason_t stopReason;

    if (!AppState()->vofa) return;

    /*
     * 停车事件优先于普通 CSV，并且绕过停车后零转速抑制。
     * 只有完整写入发送缓冲区后才清除事件；否则下个周期重试。
     */
    if (AppState_TakeStopEvent(&stopReason)) {
        char stopBuffer[TELEMETRY_BUFFER_SIZE];
        int stopLength = snprintf(stopBuffer,
                                  sizeof(stopBuffer),
                                  "[STOP] reason=%s code=%d\r\n",
                                  telemetryStopReasonName(stopReason),
                                  (int)stopReason);

        if ((stopLength > 0) && (stopLength < (int)sizeof(stopBuffer))) {
            int written = Serial_Write((const uint8_t *)stopBuffer,
                                       (uint16_t)stopLength);

            if (written == stopLength) {
                AppState_ClearStopEvent();
            }
        }

        return;
    }

    /* 无新数据或停车且转速接近 0 时不发送普通遥测。 */
    if (!telemetryData.fresh) return;
    if (!AppState()->running
        && fabsf(telemetryData.actualRightRpm) < TELEMETRY_STOP_RPM_EPSILON
        && fabsf(telemetryData.actualLeftRpm) < TELEMETRY_STOP_RPM_EPSILON) {
        return;
    }

    telemetryData.fresh = 0;

    char buffer[TELEMETRY_BUFFER_SIZE];
    int length;

    /*
     * CSV order and units:
     * tgtRpmL,rpmFL,voltL,tgtRpmR,rpmFR,voltR,
     * steerRpm,diffRpm,yawDeg,trackError,mode,run,gear,turnPhase,
     * rpmL,rpmR
     */
    length = snprintf(buffer, sizeof(buffer),
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%d,%d,%d,%d,%.1f,%.1f\r\n",
        telemetryData.targetLeftRpm,
        telemetryData.filteredLeftRpm,
        telemetryData.commandVoltageLeft,
        telemetryData.targetRightRpm,
        telemetryData.filteredRightRpm,
        telemetryData.commandVoltageRight,
        telemetryData.steerRpm,
        telemetryData.actualDiffRpm,
        telemetryData.yawAngleDeg,
        telemetryData.trackError,
        (int)telemetryData.mode,
        (int)telemetryData.runFlag,
        (int)telemetryData.gear,
        (int)telemetryData.turnPhase,
        telemetryData.actualLeftRpm,
        telemetryData.actualRightRpm);

    if (length > 0 && length < (int)sizeof(buffer)) {
        Serial_Write((const uint8_t *)buffer, (uint16_t)length);
    }
}

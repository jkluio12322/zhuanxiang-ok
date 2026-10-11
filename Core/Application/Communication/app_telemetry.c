/* VOFA CSV 遥测：50 ms 中断 AppTelemetry_Publish() 纯赋值填字段置 fresh，主循环
   AppTelemetry_Send() 临界区拷副本+清 fresh 再 snprintf 发送（格式化 16 个浮点要
   100~300 µs，放进中断会压住串口丢字节）。一致性靠临界区整体拷贝，fresh 只回答"有无
   新数据"。读 AppMotion()/AppImu_Get() 只读视图是只读跨层，避免填遥测传 10 个参数。 */

#include "app_telemetry.h"
#include "app_config.h"
#include "app_imu.h"
#include "app_motion.h"
#include "app_serial.h"
#include "app_state.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* 遥测快照本体。写它的只有本文件的 Publish()，读它的有 Send() 和显示模块。 */
telemetryData_t telemetryData;

static const char *telemetryStopReasonName(appStopReason_t reason);

/* 把遥测快照格式化为 VOFA CSV 并发送（主循环，AppDeferred_Run 调用）。发送优先级：
   VOFA 关不发；有停车事件发 [STOP]（绕过零转速抑制，只有整行写进发送缓冲才清事件）；
   无新快照不发；停车且左右转速接近 0 不发。读 17 个字段的整体拷贝+清标志在临界区做。 */
void AppTelemetry_Send(void)
{
    appStopReason_t stopReason;
    telemetryData_t snapshot;

    if (!AppState()->vofa) return;

    /* 停车事件优先于普通 CSV，并且绕过停车后零转速抑制。
       只有完整写入发送缓冲区后才清除事件；否则下个周期重试。 */
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

    /* 无新数据或停车且转速接近 0 时不发送普通遥测。
       这两次读只是"值不值得发"的判断，读到新旧混合无所谓，所以不保护。 */
    if (!telemetryData.fresh) return;
    if (!AppState()->running
        && fabsf(telemetryData.actualRightRpm) < TELEMETRY_STOP_RPM_EPSILON
        && fabsf(telemetryData.actualLeftRpm) < TELEMETRY_STOP_RPM_EPSILON) {
        return;
    }

    /* 整体拷贝 + 清标志放在同一个临界区里，之后一律从副本格式化。
       顺序很关键：先清标志再逐个读字段的话，标志根本保护不了快照。 */
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        snapshot = telemetryData;
        telemetryData.fresh = 0;
        __set_PRIMASK(primask);
    }

    char buffer[TELEMETRY_BUFFER_SIZE];
    int length;

    /* CSV 顺序与单位：tgtRpmL,rpmFL,voltL,tgtRpmR,rpmFR,voltR,steerRpm,diffRpm,
       yawDeg,trackError,mode,run,gear,turnPhase,rpmL,rpmR。 */
    length = snprintf(buffer, sizeof(buffer),
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%d,%d,%d,%d,%.1f,%.1f\r\n",
        snapshot.targetLeftRpm,
        snapshot.filteredLeftRpm,
        snapshot.commandVoltageLeft,
        snapshot.targetRightRpm,
        snapshot.filteredRightRpm,
        snapshot.commandVoltageRight,
        snapshot.steerRpm,
        snapshot.actualDiffRpm,
        snapshot.yawAngleDeg,
        snapshot.trackError,
        (int)snapshot.mode,
        (int)snapshot.runFlag,
        (int)snapshot.gear,
        (int)snapshot.turnPhase,
        snapshot.actualLeftRpm,
        snapshot.actualRightRpm);

    if (length > 0 && length < (int)sizeof(buffer)) {
        Serial_Write((const uint8_t *)buffer, (uint16_t)length);
    }
}

/* 获取停车原因对应的 ASCII 名称。未知值返回 UNKNOWN。 */
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

/* 用本周期控制结果填充遥测快照（中断上下文，50 ms 控制任务调用，只赋值不格式化）。
   参数单位 RPM/百分比；速度与姿态字段直接读只读视图，其余由调用方传入。最后置 fresh=1
   是通知主循环"有新快照可发"的唯一入口。commandVoltage 是 PWM 百分比按标称电压换算的
   近似值，仅供观察趋势（本工程无母线电压采样）。 */
void AppTelemetry_Publish(float targetLeft, float targetRight,
                          int16_t leftPwm, int16_t rightPwm,
                          float steerRpm, float trackError,
                          int16_t turnPhase, float rpmDifference)
{
    telemetryData.targetLeftRpm = targetLeft;
    telemetryData.targetRightRpm = targetRight;
    telemetryData.actualLeftRpm = AppMotion()->left_rpm_raw;
    telemetryData.actualRightRpm = AppMotion()->right_rpm_raw;
    telemetryData.filteredLeftRpm = AppMotion()->left_rpm;
    telemetryData.filteredRightRpm = AppMotion()->right_rpm;
    telemetryData.steerRpm = steerRpm;
    telemetryData.actualDiffRpm = rpmDifference;
    telemetryData.yawAngleDeg = AppImu_Get()->yaw;
    telemetryData.trackError = trackError;
    telemetryData.mode = (int16_t)AppState()->mode;
    telemetryData.runFlag = AppState()->running;
    telemetryData.gear = (int16_t)AppState()->gear;
    telemetryData.turnPhase = turnPhase;
    telemetryData.commandVoltageLeft =
        (float)leftPwm * MOTOR_SUPPLY_VOLTAGE / 100.0f;
    telemetryData.commandVoltageRight =
        (float)rightPwm * MOTOR_SUPPLY_VOLTAGE / 100.0f;
    telemetryData.fresh = 1;
}

/* 发送开机横幅和 VOFA CSV 表头。字段顺序必须与 AppTelemetry_Send() 格式串一致，
   这里是表头唯一来源，不要在别处硬编码。 */
void AppTelemetry_BootHeader(void)
{
    /* 字段顺序与 Send 里的 snprintf 格式串一致，共 16 项。 */
    Serial_Printf("BOOT OK\r\n");
    Serial_Printf("#tgtRpmL,rpmFL,voltL,tgtRpmR,rpmFR,voltR,"
                  "steerRpm,diffRpm,yawDeg,trackError,"
                  "mode,run,gear,turnPhase,rpmL,rpmR\r\n");
}

/* 初始化遥测快照：全部字段清零，保证首次发送前数据有效。 */
void AppTelemetry_Init(void)
{
    memset(&telemetryData, 0, sizeof(telemetryData));
}

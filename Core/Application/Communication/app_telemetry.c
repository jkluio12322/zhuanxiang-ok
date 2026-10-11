/**
 * @file    app_telemetry.c
 * @brief   VOFA+ CSV 遥测 —— 数据快照的填充与发送。
 *
 * @note    文件内排序：核心任务放最上方，辅助函数与对外接口放下面。
 *
 * @note    数据流（生产在中断、消费在主循环）：
 * @code
 *   50 ms 控制任务（中断）  AppTelemetry_Publish()  纯赋值填 17 个字段，置 fresh
 *   主循环                  AppTelemetry_Send()     临界区拷副本+清 fresh，再格式化发送
 * @endcode
 *          分开的原因：`snprintf` 格式化 16 个浮点要 100~300 µs，
 *          而 USART1 一个字节只要 86.8 µs——放进中断会压住串口接收丢字节。
 *
 * @note    快照的一致性是靠**临界区整体拷贝**保证的，不是靠 fresh 标志：
 *         标志只回答"有没有新数据"，它保护不了那 17 个字段的读取过程。
 *
 * @note    本模块读取 `AppMotion()` / `AppImu_Get()` 只读视图来填充快照。
 *         这是**只读跨层**：写操作严格按 Display/Comm → State → Control/Motion 分层，
 *          但只读视图允许多读，否则填遥测要传 10 个参数，反而更难维护。
 *
 * @see     README.md §8  遥测字段说明
 */

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

/* ── 辅助函数前置声明（定义在文件下方） ───────────────────────────── */
static const char *telemetryStopReasonName(appStopReason_t reason);

/**
 * @brief  把遥测快照格式化为 VOFA CSV 并发送。
 * @note   由 AppDeferred_Run() 在主循环调用，不在中断里。
 *
 * @note   发送条件（按优先级）：
 *         1. VOFA 被关闭 -> 直接不发；
 *         2. 有停车事件 -> 发一行 `[STOP] reason=...`，
 *            **绕过下面的零转速抑制**（停车恰恰就是转速为 0 的时刻）。
 *            只有整行都写进发送缓冲才清事件，否则下个周期重试；
 *         3. 没有新快照（`fresh` 为 0）-> 不发；
 *         4. 已停车且左右转速都接近 0 -> 不发，避免停车后刷屏。
 *
 * @note   本函数会读 `telemetryData` 的 17 个字段。写它的是 50 ms 中断任务，
 *         所以**整体拷贝 + 清标志**放在临界区里做，格式化一律从副本读。
 *         早先的写法是先清 fresh、再逐个读字段，那样标志什么也没保护住：
 *         中断会在格式化期间继续改结构体，发出去的是新旧混排的一行。
 *         当时的注释把窗口估成 0.0005%，按字段读取窗口 / 50 ms 算是 0.2~0.6%，
 *         差了约两个数量级。拷贝 60 来字节只要几微秒，值得做。
 */
void AppTelemetry_Send(void)
{
    appStopReason_t stopReason;
    telemetryData_t snapshot;

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

    /*
     * CSV order and units:
     * tgtRpmL,rpmFL,voltL,tgtRpmR,rpmFR,voltR,
     * steerRpm,diffRpm,yawDeg,trackError,mode,run,gear,turnPhase,
     * rpmL,rpmR
     */
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

/* ══════════════ 辅助函数 ══════════════ */

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

/* ══════════════ 对外接口 ══════════════ */

/**
 * @brief  用本周期的控制结果填充遥测快照。
 * @param  targetLeft    左轮目标转速，单位 RPM。
 * @param  targetRight   右轮目标转速，单位 RPM。
 * @param  leftPwm       左轮实际输出百分比。
 * @param  rightPwm      右轮实际输出百分比。
 * @param  steerRpm      本周期差速量，单位 RPM。
 * @param  trackError    本周期误差（e_y），仅在循迹/循迹测试下有效。
 * @param  turnPhase     循迹状态机的状态值。
 * @param  rpmDifference 左右实际速度差，单位 RPM。
 *
 * @note   **中断上下文**（由 50 ms 控制任务调用）。只做赋值，不做计算、
 *         不做格式化——`snprintf` 留给主循环的 Send()。
 *
 * @note   速度与姿态字段（左右实测/滤波转速、yaw）由本函数直接读只读视图取得；
 *         其余由调用方传入，因为它们只在控制任务里才算得出来。
 *
 * @note   最后置 `fresh = 1`，这是通知主循环"有新快照可发"的唯一入口。
 *         若忘记置位，遥测会静默停摆（Send 里第一道判断就返回）。
 *
 * @note   commandVoltage 是 PWM 百分比按**标称**电压换算的近似值，
 *         仅供观察趋势——本工程没有母线电压采样。
 */
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

/**
 * @brief  发送开机横幅和 VOFA CSV 表头。
 * @note   表头字段顺序必须和 AppTelemetry_Send() 里的格式串一致；
 *         这里是表头的**唯一来源**，不要在别处硬编码。
 */
void AppTelemetry_BootHeader(void)
{
    /* 字段顺序与 Send 里的 snprintf 格式串一致，共 16 项。 */
    Serial_Printf("BOOT OK\r\n");
    Serial_Printf("#tgtRpmL,rpmFL,voltL,tgtRpmR,rpmFR,voltR,"
                  "steerRpm,diffRpm,yawDeg,trackError,"
                  "mode,run,gear,turnPhase,rpmL,rpmR\r\n");
}

/**
 * @brief  初始化遥测快照。
 * @note   全部字段清零，保证首次发送前数据有效（此刻还没有任何采样）。
 */
void AppTelemetry_Init(void)
{
    memset(&telemetryData, 0, sizeof(telemetryData));
}

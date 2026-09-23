/**
 * @file    yingYongXianShi.c
 * @brief   OLED 开机配置页和运行状态页。
 */

#include "yingYongXianShi.h"
#include "yingYongKongZhi.h"
#include "yingYongMingLing.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "yingYongXunJi.h"
#include "yingYongYaoCe.h"
#include "irtrack.h"
#include "key.h"
#include "oled.h"
#include "timer.h"

#include <stdio.h>

/**
 * @brief  显示系统开机页。
 * @note   开机页显示车辆名称、模块化固件说明、通信方式和启动按键提示。
 */
void appDisplayShowBootScreen(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    const char *modeName;

    /* 开机页显示固件名称、通信方案和启动按键提示。 */
    OLED_Clear();
    OLED_ShowStringSmall(1, 1, "[BlueCar Init]");
    switch (workMode) {
    case modeTrack:
        modeName = "TRACK";
        break;
    case modePwmSet:
        modeName = "PWM";
        break;
    case modeRun:
    default:
        modeName = "RUN";
        break;
    }

    snprintf(buffer, sizeof(buffer), "Mode:%s G:%d",
             modeName, appStateGetSpeedLevel());
    OLED_ShowStringSmall(2, 1, buffer);

    snprintf(buffer, sizeof(buffer), "Y:%+05.1f", imu.yaw);
    OLED_ShowStringSmall(3, 1, buffer);
    OLED_ShowStringSmall(4, 1, "C8 Mode C9 Speed");
    OLED_ShowStringSmall(5, 1, "C6/C7 EXIT");
    OLED_Flush();
}

/**
 * @brief  在开机配置页调整速度档位。
 * @param  direction 大于 0 表示升档，小于 0 表示降档。
 */
static void appDisplayChangeBootGear(int8_t direction)
{
    uint8_t level = appStateGetSpeedLevel();
    uint8_t maxLevel = (workMode == modeTrack)
                       ? TRACK_MAX_SPEED_LEVEL
                       : SPEED_LEVEL_COUNT;

    if (level < 1U) level = 1U;
    if (level > maxLevel) level = maxLevel;

    if (workMode == modeTrack) {
        if (direction > 0 && level < maxLevel) level++;
        if (direction < 0 && level > 1U) level--;
    } else {
        int16_t next = (int16_t)level + direction;
        if (next > (int16_t)SPEED_LEVEL_COUNT) next = 1;
        if (next < 1) next = (int16_t)SPEED_LEVEL_COUNT;
        level = (uint8_t)next;
    }

    appStateSetSpeedLevel(level);
}

/**
 * @brief  按 RUN、TRACK、PWM 的顺序循环开机工作模式。
 * @note   切换后复位运动状态和 PWM 测试输出，并限制循迹模式档位。
 */
static void appDisplayCycleBootMode(void)
{
    if (workMode == modeRun) {
        workMode = modeTrack;
    } else if (workMode == modeTrack) {
        workMode = modePwmSet;
    } else {
        workMode = modeRun;
    }

    appControlResetModeState();
    pwmDuty = 0;

    if (workMode == modeTrack
        && appStateGetSpeedLevel() > TRACK_MAX_SPEED_LEVEL) {
        appStateSetSpeedLevel(TRACK_MAX_SPEED_LEVEL);
    }
}

/**
 * @brief  运行开机配置页，等待用户选择模式和启动按键。
 * @note   该函数在初始化阶段阻塞执行；退出时再次复位运动状态并清空旧命令。
 */
void appBootConfigRun(void)
{
    uint64_t lastDisplayTick = milliseconds;
    uint8_t exitRequested = 0;

    appStateResetMotion();
    pwmDuty = 0;
    appDisplayShowBootScreen();

    while (!exitRequested) {
        if (Key_CheckSingleOrLong(KEY_1)) {
            appDisplayCycleBootMode();
            appDisplayShowBootScreen();
            lastDisplayTick = milliseconds;
        }

        if (Key_Check(KEY_4, KEY_SINGLE)) {
            appDisplayChangeBootGear(1);
            appDisplayShowBootScreen();
            lastDisplayTick = milliseconds;
        }

        if (Key_Check(KEY_4, KEY_LONG)) {
            appDisplayChangeBootGear(-1);
            appDisplayShowBootScreen();
            lastDisplayTick = milliseconds;
        }

        if (Key_Check(KEY_2, KEY_SINGLE)
            || Key_Check(KEY_3, KEY_SINGLE)) {
            exitRequested = 1;
        }

        if ((milliseconds - lastDisplayTick) >= 100U) {
            lastDisplayTick = milliseconds;
            appDisplayShowBootScreen();
        }

        OLED_Flush();
    }

    appStateResetMotion();
    pwmDuty = 0;
    appControlResetModeState();
    appCommandInit();
    Key_Clear();
}

/**
 * @brief  根据当前工作模式刷新 OLED 运行状态页。
 * @note   循迹模式显示图案和状态；PWM 模式显示开环测试信息；
 *         蓝牙模式显示目标转速、实际转速和航向角。
 */
void appDisplayUpdate(void)
{
    OLED_Clear();

    /* 将 8 路红外原始电平转换为一行 0/1 字符串。 */
    char buffer[OLED_TEXT_BUFFER_SIZE];
    char irBits[IRTRACK_CHANNELS + 1U];

    for (uint8_t i = 0; i < IRTRACK_CHANNELS; i++) {
        irBits[i] = ir_data.raw[i] ? '1' : '0';
    }
    irBits[IRTRACK_CHANNELS] = '\0';

    /* 根据当前模式选择循迹、PWM 测试或蓝牙状态页面。 */
    if (workMode == modeTrack) {
        const char *trackStateName;

        switch (appTrackingGetState()) {
        case TRACK_CONTROL_TURN_LEFT:
            trackStateName = "TURN_L";
            break;
        case TRACK_CONTROL_TURN_RIGHT:
            trackStateName = "TURN_R";
            break;
        case TRACK_CONTROL_REACQUIRE:
            trackStateName = "REACQ";
            break;
        case TRACK_CONTROL_LOST:
            trackStateName = "LOST";
            break;
        case TRACK_CONTROL_FINISH:
            trackStateName = "FINISH";
            break;
        case TRACK_CONTROL_FOLLOW:
        default:
            trackStateName = "FOLLOW";
            break;
        }

        snprintf(buffer, sizeof(buffer), "TRK %s G%d L%u/%u",
                 runFlag ? "RUN" : "STOP",
                 appStateGetSpeedLevel(),
                 (unsigned int)appStateGetCompletedLaps(),
                 (unsigned int)appStateGetLapTarget());
        OLED_ShowStringSmall(1, 1, buffer);

        snprintf(buffer, sizeof(buffer), "IR:%s", irBits);
        OLED_ShowStringSmall(2, 1, buffer);

        /* S 显示 FOLLOW/TURN_L/TURN_R/LOST/FINISH/REACQ，P 显示八路图案。 */
        snprintf(buffer, sizeof(buffer), "S:%s P:%02X",
                 trackStateName, (unsigned int)ir_data.pattern);
        OLED_ShowStringSmall(3, 1, buffer);

        snprintf(buffer, sizeof(buffer), "TL:%+03.0f TR:%+03.0f",
                 telemetryData.targetLeftRpm, telemetryData.targetRightRpm);
        OLED_ShowStringSmall(4, 1, buffer);

        snprintf(buffer, sizeof(buffer), "Y:%+05.1f STR:%+05.1f",
                 imu.yaw,
                 appTrackingGetSteer());
        OLED_ShowStringSmall(5, 1, buffer);

        snprintf(buffer, sizeof(buffer), "RPM L:%+04.0f R:%+04.0f",
                 telemetryData.filteredLeftRpm,
                 telemetryData.filteredRightRpm);
        OLED_ShowStringSmall(6, 1, buffer);

        snprintf(buffer, sizeof(buffer), "ACT L:%+04.0f R:%+04.0f",
                 telemetryData.actualLeftRpm,
                 telemetryData.actualRightRpm);
        OLED_ShowStringSmall(7, 1, buffer);
        return;
    } else if (workMode == modePwmSet) {
        snprintf(buffer, sizeof(buffer), "[PWM] %s",
                 runFlag ? "RUN" : "STOP");
        OLED_ShowString(1, 1, buffer);

        snprintf(buffer, sizeof(buffer), "RPM L:%+04.0f R:%+04.0f",
                 telemetryData.filteredLeftRpm,
                 telemetryData.filteredRightRpm);
        OLED_ShowString(2, 1, buffer);

        snprintf(buffer, sizeof(buffer), "ACT L:%+04.0f R:%+04.0f",
                 telemetryData.actualLeftRpm,
                 telemetryData.actualRightRpm);
        OLED_ShowString(3, 1, buffer);

        snprintf(buffer, sizeof(buffer), "Y:%+05.1f", imu.yaw);
        OLED_ShowString(4, 1, buffer);
        return;
    } else {
        snprintf(buffer, sizeof(buffer), "[BT] %s G%d",
                 runFlag ? "RUN" : "STOP", appStateGetSpeedLevel());
        OLED_ShowString(1, 1, buffer);

        snprintf(buffer, sizeof(buffer), "T L:%+04.0f R:%+04.0f",
                 telemetryData.targetLeftRpm,
                 telemetryData.targetRightRpm);
        OLED_ShowString(2, 1, buffer);

        snprintf(buffer, sizeof(buffer), "RPM L:%+04.0f R:%+04.0f",
                 telemetryData.actualLeftRpm,
                 telemetryData.actualRightRpm);
        OLED_ShowString(3, 1, buffer);

        snprintf(buffer, sizeof(buffer), "Y:%+05.1f", imu.yaw);
        OLED_ShowString(4, 1, buffer);
        return;
    }

}

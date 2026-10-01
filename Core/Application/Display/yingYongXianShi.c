#include "yingYongXianShi.h"
#include "yingYongKongZhi.h"
#include "yingYongMingLing.h"
#include "app_config.h"
#include "yingYongZhuangTai.h"
#include "yingYongXunJi.h"
#include "yingYongYaoCe.h"
#include "irtrack.h"
#include "key.h"
#include "oled.h"
#include "timer.h"

#include <stdio.h>

static void appDisplayCycleBootMode(void)
{
    if (workMode == modeTrack) workMode = modePwmSpeed;
    else if (workMode == modePwmSpeed) workMode = modePwmTrack;
    else workMode = modeTrack;

    appControlResetModeState();
}

void appDisplayShowBootScreen(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    const char *name;

    OLED_Clear();
    OLED_ShowStringSmall(1, 1, "[BlueCar]");
    if (workMode == modePwmSpeed) name = "PWM-SPEED";
    else if (workMode == modePwmTrack) name = "PWM-TRACK";
    else name = "TRACK";
    snprintf(buffer, sizeof(buffer), "Mode:%s G:%d", name, appStateGetSpeedLevel());
    OLED_ShowStringSmall(2, 1, buffer);
    OLED_ShowStringSmall(4, 1, "KEY1 Mode");
    OLED_ShowStringSmall(5, 1, "KEY2/3 Exit");
    OLED_Flush();
}

void appBootConfigRun(void)
{
    uint8_t exitRequested = 0;
    uint64_t lastTick = milliseconds;

    appStateResetMotion();
    appDisplayShowBootScreen();

    while (!exitRequested) {
        if (Key_CheckSingleOrLong(KEY_1)) {
            appDisplayCycleBootMode();
            appDisplayShowBootScreen();
            lastTick = milliseconds;
        }
        if (Key_Check(KEY_2, KEY_SINGLE) || Key_Check(KEY_3, KEY_SINGLE)) {
            exitRequested = 1;
        }
        if ((milliseconds - lastTick) >= 100U) {
            lastTick = milliseconds;
            appDisplayShowBootScreen();
        }
        OLED_Flush();
    }

    appControlResetModeState();
    appCommandInit();
    Key_Clear();
}

void appDisplayUpdate(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    char irBits[IRTRACK_CHANNELS + 1U];
    uint8_t i;

    OLED_Clear();
    for (i = 0; i < IRTRACK_CHANNELS; i++) {
        irBits[i] = ir_data.raw[i] ? '1' : '0';
    }
    irBits[IRTRACK_CHANNELS] = '\0';

    if (workMode == modeTrack) {
        snprintf(buffer, sizeof(buffer), "TRK %s G%d L%u/%u",
                 runFlag ? "RUN" : "STOP", appStateGetSpeedLevel(),
                 (unsigned int)appStateGetCompletedLaps(),
                 (unsigned int)appStateGetLapTarget());
        OLED_ShowStringSmall(1, 1, buffer);
        snprintf(buffer, sizeof(buffer), "IR:%s", irBits);
        OLED_ShowStringSmall(2, 1, buffer);
        snprintf(buffer, sizeof(buffer), "STR:%+05.1f", appTrackingGetSteer());
        OLED_ShowStringSmall(3, 1, buffer);
        snprintf(buffer, sizeof(buffer), "V L:%+.2f R:%+.2f",
                 telemetryData.commandVoltageLeft,
                 telemetryData.commandVoltageRight);
        OLED_ShowStringSmall(4, 1, buffer);
        return;
    }

    if (workMode == modePwmSpeed) {
        float target = appStateGetSpeedRpm();
        if (appControlGetPwmSpeedPhase() == PWM_TEST_PHASE_REVERSE) target = -target;
        snprintf(buffer, sizeof(buffer), "[PWM-SPD] %s G%d",
                 runFlag ? "RUN" : "STOP", appStateGetSpeedLevel());
        OLED_ShowStringSmall(1, 1, buffer);
        snprintf(buffer, sizeof(buffer), "PH:%s TGT:%+05.0f",
                 appControlGetPwmSpeedPhase() == PWM_TEST_PHASE_REVERSE
                 ? "REVERSE" : "FORWARD", target);
        OLED_ShowStringSmall(2, 1, buffer);
        snprintf(buffer, sizeof(buffer), "V L:%+.2f R:%+.2f",
                 telemetryData.commandVoltageLeft,
                 telemetryData.commandVoltageRight);
        OLED_ShowStringSmall(3, 1, buffer);
        return;
    }

    snprintf(buffer, sizeof(buffer), "[PWM-TRK] %s A:%u",
             runFlag ? "RUN" : "STOP",
             (unsigned int)appControlPwmTrackGetAngle());
    OLED_ShowStringSmall(1, 1, buffer);
    snprintf(buffer, sizeof(buffer), "PH:%s SRC:%s",
             appControlPwmTrackGetPhase() == PWM_TEST_PHASE_RIGHT ? "RIGHT" : "LEFT",
             appControlPwmTrackGetSimEnabled() ? "SIM" : "IR");
    OLED_ShowStringSmall(2, 1, buffer);
    snprintf(buffer, sizeof(buffer), "BASE:%d%% ERR:%+05.1f",
             appControlGetPwmTrackBasePwm(), telemetryData.trackError);
    OLED_ShowStringSmall(3, 1, buffer);
    snprintf(buffer, sizeof(buffer), "V L:%+.2f R:%+.2f",
             telemetryData.commandVoltageLeft,
             telemetryData.commandVoltageRight);
    OLED_ShowStringSmall(4, 1, buffer);
}

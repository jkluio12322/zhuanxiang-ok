#include "yingYongXianShi.h"
#include "app_config.h"
#include "app_speedloop.h"
#include "app_state.h"
#include "app_testmode.h"
#include "yingYongMingLing.h"
#include "app_tracking.h"
#include "yingYongYaoCe.h"
#include "irtrack.h"
#include "key.h"
#include "oled.h"
#include "timer.h"

#include <stdio.h>

static void appDisplayCycleBootMode(void)
{
    if (AppState()->mode == modeTrack) {
        AppState_SetMode(modePwmSpeed);
    } else if (AppState()->mode == modePwmSpeed) {
        AppState_SetMode(modePwmTrack);
    } else {
        AppState_SetMode(modeTrack);
    }

    AppSpeedLoop_ResetModeState();
}

void appDisplayShowBootScreen(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    const char *name;

    OLED_Clear();
    OLED_ShowStringSmall(1, 1, "[BlueCar]");
    if (AppState()->mode == modePwmSpeed) name = "PWM-SPEED";
    else if (AppState()->mode == modePwmTrack) name = "PWM-TRACK";
    else name = "TRACK";
    snprintf(buffer, sizeof(buffer), "Mode:%s G:%d", name, AppState()->gear);
    OLED_ShowStringSmall(2, 1, buffer);
    OLED_ShowStringSmall(4, 1, "KEY1 Mode");
    OLED_ShowStringSmall(5, 1, "KEY2/3 Exit");
    OLED_Flush();
}

void appBootConfigRun(void)
{
    uint8_t exitRequested = 0;
    uint64_t lastTick = milliseconds;

    AppSpeedLoop_ResetMotion();
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

    AppSpeedLoop_ResetModeState();
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

    if (AppState()->mode == modeTrack) {
        snprintf(buffer, sizeof(buffer), "TRK %s G%d L%u/%u",
                 AppState()->running ? "RUN" : "STOP", AppState()->gear,
                 (unsigned int)AppState()->laps_done,
                 (unsigned int)AppState()->lap_target);
        OLED_ShowStringSmall(1, 1, buffer);
        snprintf(buffer, sizeof(buffer), "IR:%s", irBits);
        OLED_ShowStringSmall(2, 1, buffer);
        snprintf(buffer, sizeof(buffer), "STR:%+05.1f", AppTracking_GetSteer());
        OLED_ShowStringSmall(3, 1, buffer);
        snprintf(buffer, sizeof(buffer), "V L:%+.2f R:%+.2f",
                 telemetryData.commandVoltageLeft,
                 telemetryData.commandVoltageRight);
        OLED_ShowStringSmall(4, 1, buffer);
        return;
    }

    if (AppState()->mode == modePwmSpeed) {
        float target = AppState()->gear_rpm;
        if (AppTestMode_GetSpeedPhase() == PWM_TEST_PHASE_REVERSE) target = -target;
        snprintf(buffer, sizeof(buffer), "[PWM-SPD] %s G%d",
                 AppState()->running ? "RUN" : "STOP", AppState()->gear);
        OLED_ShowStringSmall(1, 1, buffer);
        snprintf(buffer, sizeof(buffer), "PH:%s TGT:%+05.0f",
                 AppTestMode_GetSpeedPhase() == PWM_TEST_PHASE_REVERSE
                 ? "REVERSE" : "FORWARD", target);
        OLED_ShowStringSmall(2, 1, buffer);
        snprintf(buffer, sizeof(buffer), "V L:%+.2f R:%+.2f",
                 telemetryData.commandVoltageLeft,
                 telemetryData.commandVoltageRight);
        OLED_ShowStringSmall(3, 1, buffer);
        return;
    }

    snprintf(buffer, sizeof(buffer), "[PWM-TRK] %s A:%u",
             AppState()->running ? "RUN" : "STOP",
             (unsigned int)AppTestMode_GetAngle());
    OLED_ShowStringSmall(1, 1, buffer);
    snprintf(buffer, sizeof(buffer), "PH:%s SRC:%s",
             AppTestMode_GetTrackPhase() == PWM_TEST_PHASE_RIGHT ? "RIGHT" : "LEFT",
             AppTestMode_GetSimEnabled() ? "SIM" : "IR");
    OLED_ShowStringSmall(2, 1, buffer);
    snprintf(buffer, sizeof(buffer), "BASE:%d%% ERR:%+05.1f",
             (int)AppTestMode_GetBasePwm(), telemetryData.trackError);
    OLED_ShowStringSmall(3, 1, buffer);
    snprintf(buffer, sizeof(buffer), "V L:%+.2f R:%+.2f",
             telemetryData.commandVoltageLeft,
             telemetryData.commandVoltageRight);
    OLED_ShowStringSmall(4, 1, buffer);
}

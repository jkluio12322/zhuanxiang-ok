/**
 * @file    app_display.c
 * @brief   OLED 开机页与三个模式状态页。
 */

#include "app_display.h"
#include "app_cmd.h"
#include "app_config.h"
#include "app_speedloop.h"
#include "app_state.h"
#include "app_testmode.h"
#include "app_tracking.h"
#include "app_telemetry.h"
#include "irtrack.h"
#include "key.h"
#include "oled.h"
#include "timer.h"

#include <stdio.h>

/* 三个模式页共用的电压行，避免同一段代码抄三遍。 */
static void showVoltageLine(uint8_t row)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];

    snprintf(buffer, sizeof(buffer), "V L:%+.2f R:%+.2f",
             telemetryData.commandVoltageLeft,
             telemetryData.commandVoltageRight);
    OLED_ShowStringSmall(row, 1, buffer);
}

/* 模式循环只有这一处实现，按键和开机页都用它。 */
static void cycleBootMode(void)
{
    AppState_SetMode(AppState_NextMode());
    AppSpeedLoop_ResetModeState();
}

void AppDisplay_ShowBoot(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];

    OLED_Clear();
    OLED_ShowStringSmall(1, 1, "[BlueCar]");
    snprintf(buffer, sizeof(buffer), "Mode:%s G:%d",
             AppModeName(AppState()->mode), AppState()->gear);
    OLED_ShowStringSmall(2, 1, buffer);
    OLED_ShowStringSmall(4, 1, "KEY1 Mode");
    OLED_ShowStringSmall(5, 1, "KEY2/3 Exit");
    OLED_Flush();
}

void AppDisplay_BootConfig(void)
{
    uint8_t exitRequested = 0;
    uint64_t lastTick = milliseconds;

    AppSpeedLoop_ResetMotion();
    AppDisplay_ShowBoot();

    while (!exitRequested) {
        if (Key_CheckSingleOrLong(KEY_1)) {
            cycleBootMode();
            AppDisplay_ShowBoot();
            lastTick = milliseconds;
        }
        if (Key_Check(KEY_2, KEY_SINGLE) || Key_Check(KEY_3, KEY_SINGLE)) {
            exitRequested = 1;
        }
        if ((milliseconds - lastTick) >= 100U) {
            lastTick = milliseconds;
            AppDisplay_ShowBoot();
        }
        OLED_Flush();
    }

    AppSpeedLoop_ResetModeState();
    AppCmd_Init();
    Key_Clear();
}

/* 循迹页。 */
static void showTrackPage(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    char irBits[IRTRACK_CHANNELS + 1U];

    for (uint8_t i = 0; i < IRTRACK_CHANNELS; i++) {
        irBits[i] = ir_data.raw[i] ? '1' : '0';
    }
    irBits[IRTRACK_CHANNELS] = '\0';

    snprintf(buffer, sizeof(buffer), "TRK %s G%d L%u/%u",
             AppState()->running ? "RUN" : "STOP", AppState()->gear,
             (unsigned int)AppState()->laps_done,
             (unsigned int)AppState()->lap_target);
    OLED_ShowStringSmall(1, 1, buffer);

    snprintf(buffer, sizeof(buffer), "IR:%s", irBits);
    OLED_ShowStringSmall(2, 1, buffer);

    snprintf(buffer, sizeof(buffer), "STR:%+05.1f", AppTracking_GetSteer());
    OLED_ShowStringSmall(3, 1, buffer);

    showVoltageLine(4);
}

/* PWM 速度测试页。 */
static void showPwmSpeedPage(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    float target = AppState()->gear_rpm;

    if (AppTestMode_GetSpeedPhase() == PWM_TEST_PHASE_REVERSE) {
        target = -target;
    }

    snprintf(buffer, sizeof(buffer), "[PWM-SPD] %s G%d",
             AppState()->running ? "RUN" : "STOP", AppState()->gear);
    OLED_ShowStringSmall(1, 1, buffer);

    snprintf(buffer, sizeof(buffer), "PH:%s TGT:%+05.0f",
             AppPhaseName(AppTestMode_GetSpeedPhase()), target);
    OLED_ShowStringSmall(2, 1, buffer);

    showVoltageLine(3);
}

/* PWM 循迹测试页。 */
static void showPwmTrackPage(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];

    snprintf(buffer, sizeof(buffer), "[PWM-TRK] %s A:%u",
             AppState()->running ? "RUN" : "STOP",
             (unsigned int)AppTestMode_GetAngle());
    OLED_ShowStringSmall(1, 1, buffer);

    snprintf(buffer, sizeof(buffer), "PH:%s SRC:%s",
             AppPhaseName(AppTestMode_GetTrackPhase()),
             AppTestMode_GetSimEnabled() ? "SIM" : "IR");
    OLED_ShowStringSmall(2, 1, buffer);

    snprintf(buffer, sizeof(buffer), "BASE:%d%% ERR:%+05.1f",
             (int)AppTestMode_GetBasePwm(), telemetryData.trackError);
    OLED_ShowStringSmall(3, 1, buffer);

    showVoltageLine(4);
}

void AppDisplay_Task100ms(void)
{
    OLED_Clear();

    if (AppState()->mode == modeTrack) {
        showTrackPage();
    } else if (AppState()->mode == modePwmSpeed) {
        showPwmSpeedPage();
    } else {
        showPwmTrackPage();
    }
}

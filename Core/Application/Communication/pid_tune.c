#include "pid_tune.h"
#include "yingYongMingLing.h"
#include "app_config.h"
#include "app_state.h"
#include "app_tracking.h"
#include "app_speedloop.h"
#include "app_testmode.h"
#include "yingYongYaoCe.h"
#include "app_pid.h"
#include "serial.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PID_TUNE_RESPONSE_BUFFER_SIZE 384U
#define PID_TUNE_LINE_BUFFER_SIZE     32U
#define PID_TUNE_TOKEN_BUFFER_SIZE    16U

typedef enum {
    TUNE_SKP,
    TUNE_SKI,
    TUNE_SKD,
    TUNE_LKP,
    TUNE_LKI,
    TUNE_LKD,
    TUNE_RKP,
    TUNE_RKI,
    TUNE_RKD,
    TUNE_TKP,
    TUNE_TKI,
    TUNE_TKD,
    TUNE_TKG,
    TUNE_START,
    TUNE_STOP,
    TUNE_SPDGEAR,
    TUNE_SIMLEVEL,
    TUNE_BASE,
    TUNE_SIM,
    TUNE_QUIET,
    TUNE_LOG,
    TUNE_SHOW,
    TUNE_HELP
} TuneAction_t;

typedef struct {
    const char *name;
    uint8_t needsArg;
    TuneAction_t action;
} TuneCommand_t;

static const TuneCommand_t tuneTable[] = {
    {"skp", 1, TUNE_SKP},
    {"ski", 1, TUNE_SKI},
    {"skd", 1, TUNE_SKD},
    {"lkp", 1, TUNE_LKP},
    {"lki", 1, TUNE_LKI},
    {"lkd", 1, TUNE_LKD},
    {"rkp", 1, TUNE_RKP},
    {"rki", 1, TUNE_RKI},
    {"rkd", 1, TUNE_RKD},
    {"tkp", 1, TUNE_TKP},
    {"tki", 1, TUNE_TKI},
    {"tkd", 1, TUNE_TKD},
    {"tkg", 1, TUNE_TKG},
    {"start", 0, TUNE_START},
    {"stop", 0, TUNE_STOP},
    {"spdgear", 1, TUNE_SPDGEAR},
    {"simlevel", 1, TUNE_SIMLEVEL},
    {"base", 1, TUNE_BASE},
    {"sim", 1, TUNE_SIM},
    {"quiet", 0, TUNE_QUIET},
    {"log", 0, TUNE_LOG},
    {"show", 0, TUNE_SHOW},
    {"help", 0, TUNE_HELP},
    {"?", 0, TUNE_HELP}
};

extern UART_HandleTypeDef huart1;
volatile uint8_t pidTuneRxByte;

static volatile uint8_t tuneResponsePending = 0;
static volatile uint8_t tuneLineReady = 0;
static char tuneResponseBuffer[PID_TUNE_RESPONSE_BUFFER_SIZE];
static char tuneLine[PID_TUNE_LINE_BUFFER_SIZE];
static uint8_t tuneRxIndex = 0;

static void pidTuneRespond(const char *format, ...)
{
    va_list arguments;
    if (tuneResponsePending) return;
    va_start(arguments, format);
    vsnprintf(tuneResponseBuffer, sizeof(tuneResponseBuffer), format, arguments);
    va_end(arguments);
    tuneResponsePending = 1;
}

void pidTuneInit(void)
{
    tuneRxIndex = 0;
    tuneResponsePending = 0;
    tuneLineReady = 0;
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&pidTuneRxByte, 1);
}

void pidTuneRxCallback(uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (tuneRxIndex > 0) {
            tuneLine[tuneRxIndex] = '\0';
            tuneLineReady = 1;
            tuneRxIndex = 0;
        }
    } else if (tuneRxIndex < sizeof(tuneLine) - 1U) {
        tuneLine[tuneRxIndex++] = byte;
    }
}

static const TuneCommand_t *pidTuneFind(const char *name)
{
    uint8_t i;
    for (i = 0; i < sizeof(tuneTable) / sizeof(tuneTable[0]); i++) {
        if (strcmp(tuneTable[i].name, name) == 0) return &tuneTable[i];
    }
    return 0;
}

static const char *pidTunePhaseName(void)
{
    if (AppState()->mode == modePwmSpeed) {
        return (AppTestMode_GetSpeedPhase() == PWM_TEST_PHASE_REVERSE)
               ? "REVERSE" : "FORWARD";
    }
    if (AppState()->mode == modePwmTrack) {
        return (AppTestMode_GetTrackPhase() == PWM_TEST_PHASE_RIGHT)
               ? "RIGHT" : "LEFT";
    }
    return "TRACK";
}

static const char *pidTuneModeName(void)
{
    if (AppState()->mode == modePwmSpeed) return "PWM-SPEED";
    if (AppState()->mode == modePwmTrack) return "PWM-TRACK";
    return "TRACK";
}

void pidTuneProcess(void)
{
    char token[PID_TUNE_TOKEN_BUFFER_SIZE];
    char argument[PID_TUNE_TOKEN_BUFFER_SIZE];
    char *command;
    const TuneCommand_t *entry;
    float value = 0.0f;
    int count;

    if (!tuneLineReady) return;
    token[0] = '\0';
    argument[0] = '\0';
    command = tuneLine;
    while (*command == ' ') command++;

    count = sscanf(command, "%15s %15s", token, argument);
    if (count < 1) {
        tuneLineReady = 0;
        return;
    }

    entry = pidTuneFind(token);
    if (entry == 0) {
        pidTuneRespond("? '%s' (type 'help')\r\n", token);
        tuneLineReady = 0;
        return;
    }
    if (entry->needsArg && count < 2) {
        pidTuneRespond("? %s needs a value\r\n", token);
        tuneLineReady = 0;
        return;
    }
    if (count >= 2) value = strtof(argument, 0);

    switch (entry->action) {
    /* 左右同时设置三个增益。 */
    case TUNE_SKP:
    case TUNE_SKI:
    case TUNE_SKD: {
        PidField_t field = (entry->action == TUNE_SKP) ? PID_FIELD_KP
                           : (entry->action == TUNE_SKI) ? PID_FIELD_KI
                           : PID_FIELD_KD;
        AppSpeedLoop_SetGain(SPEED_CH_LEFT, field, value);
        AppSpeedLoop_SetGain(SPEED_CH_RIGHT, field, value);
        break;
    }
    /* 只设左轮。 */
    case TUNE_LKP:
    case TUNE_LKI:
    case TUNE_LKD: {
        PidField_t field = (entry->action == TUNE_LKP) ? PID_FIELD_KP
                           : (entry->action == TUNE_LKI) ? PID_FIELD_KI
                           : PID_FIELD_KD;
        AppSpeedLoop_SetGain(SPEED_CH_LEFT, field, value);
        break;
    }
    /* 只设右轮。 */
    case TUNE_RKP:
    case TUNE_RKI:
    case TUNE_RKD: {
        PidField_t field = (entry->action == TUNE_RKP) ? PID_FIELD_KP
                           : (entry->action == TUNE_RKI) ? PID_FIELD_KI
                           : PID_FIELD_KD;
        AppSpeedLoop_SetGain(SPEED_CH_RIGHT, field, value);
        break;
    }
    case TUNE_TKP:
    case TUNE_TKI:
    case TUNE_TKD: {
        PidField_t field = (entry->action == TUNE_TKP) ? PID_FIELD_KP
                           : (entry->action == TUNE_TKI) ? PID_FIELD_KI
                           : PID_FIELD_KD;
        AppTracking_SetGain(field, value);
        break;
    }
    case TUNE_TKG:
        AppTracking_SetSteerRpmToPwmGain(value);
        break;
    case TUNE_START:
        appCommandPushValue(commandStart, 0.0f);
        break;
    case TUNE_STOP:
        appCommandPushValue(commandStop, 0.0f);
        break;
    case TUNE_SPDGEAR:
        if (value < 1.0f || value > 6.0f) {
            pidTuneRespond("[PWM-SPEED] invalid gear 1..6\r\n");
        } else {
            appCommandPushValue(commandSetSpeedGear, value);
        }
        break;
    case TUNE_SIMLEVEL: {
        int level = (int)value;
        if ((level != 5) && (level != 10) && (level != 45) && (level != 90)) {
            pidTuneRespond("[PWM-TRACK] invalid simlevel\r\n");
        } else {
            appCommandPushValue(commandPwmTrackSetAngle, (float)level);
        }
        break;
    }
    case TUNE_BASE:
        appCommandPushValue(commandPwmTrackBaseStep,
                            value - (float)AppTestMode_GetBasePwm());
        break;
    case TUNE_SIM:
        if (strcmp(argument, "on") == 0) {
            appCommandPushValue(commandPwmTrackSim, 1.0f);
        } else if (strcmp(argument, "off") == 0) {
            appCommandPushValue(commandPwmTrackSim, 0.0f);
        } else {
            pidTuneRespond("[PWM-TRACK] use sim on/off\r\n");
        }
        break;
    case TUNE_QUIET:
        appCommandPushValue(commandQuiet, 0.0f);
        break;
    case TUNE_LOG:
        appCommandPushValue(commandLog, 0.0f);
        break;
    case TUNE_SHOW: {
        SpeedPidInfo_t infoL;
        SpeedPidInfo_t infoR;
        float kp = AppTracking_GetGain(PID_FIELD_KP);
        float ki = AppTracking_GetGain(PID_FIELD_KI);
        float kd = AppTracking_GetGain(PID_FIELD_KD);
        AppSpeedLoop_GetInfo(SPEED_CH_LEFT, &infoL);
        AppSpeedLoop_GetInfo(SPEED_CH_RIGHT, &infoR);
        pidTuneRespond(
            "Mode=%s Run=%d Phase=%s Gear=%d Base=%d%%\r\n"
            "SpeedL Kp=%.2f Ki=%.2f Kd=%.2f Tgt=%.1f Out=%.1f\r\n"
            "SpeedR Kp=%.2f Ki=%.2f Kd=%.2f Tgt=%.1f Out=%.1f\r\n"
            "Track Kp=%.2f Ki=%.2f Kd=%.2f tkg=%.2f\r\n"
            "VoltL=%.2fV VoltR=%.2fV\r\n",
            pidTuneModeName(), AppState()->running, pidTunePhaseName(),
            AppState()->gear, (int)AppTestMode_GetBasePwm(),
            infoL.kp, infoL.ki, infoL.kd, infoL.target, infoL.out,
            infoR.kp, infoR.ki, infoR.kd, infoR.target, infoR.out,
            kp, ki, kd, AppTracking_GetSteerRpmToPwmGain(),
            telemetryData.commandVoltageLeft,
            telemetryData.commandVoltageRight);
        break;
    }
    case TUNE_HELP:
        pidTuneRespond(
            "Commands: start stop spdgear simlevel base sim on|off\r\n"
            "Speed: skp/ski/skd lkp/lki/lkd rkp/rki/rkd\r\n"
            "Track: tkp/tki/tkd/tkg\r\n"
            "VOFA: quiet log  Info: show help\r\n");
        break;
    }

    tuneLineReady = 0;
}

void pidTuneProcessResponse(void)
{
    if (!tuneResponsePending) return;
    __disable_irq();
    Serial_SendString(tuneResponseBuffer);
    __enable_irq();
    tuneResponsePending = 0;
}

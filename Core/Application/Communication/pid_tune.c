#include "pid_tune.h"
#include "yingYongMingLing.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "yingYongXunJi.h"
#include "yingYongKongZhi.h"
#include "yingYongYaoCe.h"
#include "pid.h"
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

static void pidTuneSetSpeedField(PID_t *pid, PID_Field_t field, float value)
{
    PID_SetField(pid, field, value);
}

static const char *pidTunePhaseName(void)
{
    if (workMode == modePwmSpeed) {
        return (appControlGetPwmSpeedPhase() == PWM_TEST_PHASE_REVERSE)
               ? "REVERSE" : "FORWARD";
    }
    if (workMode == modePwmTrack) {
        return (appControlPwmTrackGetPhase() == PWM_TEST_PHASE_RIGHT)
               ? "RIGHT" : "LEFT";
    }
    return "TRACK";
}

static const char *pidTuneModeName(void)
{
    if (workMode == modePwmSpeed) return "PWM-SPEED";
    if (workMode == modePwmTrack) return "PWM-TRACK";
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
    case TUNE_SKP:
        pidTuneSetSpeedField(&speedPidLeft, PID_FIELD_KP, value);
        pidTuneSetSpeedField(&speedPidRight, PID_FIELD_KP, value);
        break;
    case TUNE_SKI:
        pidTuneSetSpeedField(&speedPidLeft, PID_FIELD_KI, value);
        pidTuneSetSpeedField(&speedPidRight, PID_FIELD_KI, value);
        break;
    case TUNE_SKD:
        pidTuneSetSpeedField(&speedPidLeft, PID_FIELD_KD, value);
        pidTuneSetSpeedField(&speedPidRight, PID_FIELD_KD, value);
        break;
    case TUNE_LKP:
        pidTuneSetSpeedField(&speedPidLeft, PID_FIELD_KP, value);
        break;
    case TUNE_LKI:
        pidTuneSetSpeedField(&speedPidLeft, PID_FIELD_KI, value);
        break;
    case TUNE_LKD:
        pidTuneSetSpeedField(&speedPidLeft, PID_FIELD_KD, value);
        break;
    case TUNE_RKP:
        pidTuneSetSpeedField(&speedPidRight, PID_FIELD_KP, value);
        break;
    case TUNE_RKI:
        pidTuneSetSpeedField(&speedPidRight, PID_FIELD_KI, value);
        break;
    case TUNE_RKD:
        pidTuneSetSpeedField(&speedPidRight, PID_FIELD_KD, value);
        break;
    case TUNE_TKP:
    case TUNE_TKI:
    case TUNE_TKD: {
        float kp;
        float ki;
        float kd;
        appTrackingGetPidGains(&kp, &ki, &kd);
        if (entry->action == TUNE_TKP) kp = value;
        else if (entry->action == TUNE_TKI) ki = value;
        else kd = value;
        appTrackingSetPidGains(kp, ki, kd);
        break;
    }
    case TUNE_TKG:
        appTrackingSetSteerRpmToPwmGain(value);
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
                            value - (float)appControlGetPwmTrackBasePwm());
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
        float kp;
        float ki;
        float kd;
        appTrackingGetPidGains(&kp, &ki, &kd);
        pidTuneRespond(
            "Mode=%s Run=%d Phase=%s Gear=%d Base=%d%%\r\n"
            "SpeedL Kp=%.2f Ki=%.2f Kd=%.2f Tgt=%.1f Out=%.1f\r\n"
            "SpeedR Kp=%.2f Ki=%.2f Kd=%.2f Tgt=%.1f Out=%.1f\r\n"
            "Track Kp=%.2f Ki=%.2f Kd=%.2f tkg=%.2f\r\n"
            "VoltL=%.2fV VoltR=%.2fV\r\n",
            pidTuneModeName(), runFlag, pidTunePhaseName(),
            appStateGetSpeedLevel(), appControlGetPwmTrackBasePwm(),
            speedPidLeft.Kp, speedPidLeft.Ki, speedPidLeft.Kd,
            speedPidLeft.Target, speedPidLeft.Out,
            speedPidRight.Kp, speedPidRight.Ki, speedPidRight.Kd,
            speedPidRight.Target, speedPidRight.Out,
            kp, ki, kd, appTrackingGetSteerRpmToPwmGain(),
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

#include "yingYongMingLing.h"
#include "app_config.h"
#include "app_state.h"
#include "key.h"
#include "pid_tune.h"

static commandMessage_t commandQueue[COMMAND_QUEUE_SIZE];
static volatile uint8_t commandHead = 0;
static volatile uint8_t commandTail = 0;

void appCommandInit(void)
{
    commandHead = 0;
    commandTail = 0;
}

void appCommandQueuePush(commandMessage_t message)
{
    uint8_t next = (uint8_t)((commandHead + 1U) % COMMAND_QUEUE_SIZE);
    if (next == commandTail) return;
    commandQueue[commandHead] = message;
    commandHead = next;
}

void appCommandPushValue(commandType_t type, float value)
{
    commandMessage_t message;
    message.type = type;
    message.value = value;
    appCommandQueuePush(message);
}

int appCommandQueuePop(commandMessage_t *message)
{
    if (commandHead == commandTail) return -1;
    *message = commandQueue[commandTail];
    commandTail = (uint8_t)((commandTail + 1U) % COMMAND_QUEUE_SIZE);
    return 0;
}

uint8_t appCommandQueueAvailable(void)
{
    return (uint8_t)((commandHead + COMMAND_QUEUE_SIZE - commandTail)
                     % COMMAND_QUEUE_SIZE);
}

void appCommandProcessBluetooth(void)
{
}

static commandType_t appNextMode(void)
{
    if (AppState()->mode == modeTrack) return commandModePwmSpeed;
    if (AppState()->mode == modePwmSpeed) return commandModePwmTrack;
    return commandModeTrack;
}

void appCommandProcessKeys(void)
{
    if (Key_CheckSingleOrLong(KEY_1)) {
        appCommandPushValue(appNextMode(), 0.0f);
    }

    if (AppState()->mode == modeTrack) {
        if (Key_Check(KEY_4, KEY_SINGLE) && !AppState()->running) {
            appCommandPushValue(commandLapCycle, 0.0f);
        }
        (void)Key_CheckSingleOrLong(KEY_4);
        if (Key_Check(KEY_2, KEY_SINGLE)) {
            appCommandPushValue(AppState()->running ? commandStop : commandStart, 0.0f);
        }
        if (Key_Check(KEY_3, KEY_SINGLE)) {
            appCommandPushValue(commandStop, 0.0f);
        }
        return;
    }

    if (AppState()->mode == modePwmTrack) {
        if (Key_Check(KEY_4, KEY_SINGLE)) {
            appCommandPushValue(commandPwmTrackBaseStep,
                                (float)PWM_TRACK_BASE_STEP_PERCENT);
        }
        if (Key_Check(KEY_4, KEY_LONG)) {
            appCommandPushValue(commandPwmTrackBaseStep,
                                -(float)PWM_TRACK_BASE_STEP_PERCENT);
        }
        (void)Key_CheckSingleOrLong(KEY_4);
        if (Key_Check(KEY_2, KEY_SINGLE)) {
            appCommandPushValue(commandPwmTrackNext, 0.0f);
        }
        if (Key_Check(KEY_3, KEY_SINGLE)) {
            appCommandPushValue(AppState()->running ? commandStop : commandStart, 0.0f);
        }
        return;
    }

    (void)Key_CheckSingleOrLong(KEY_4);
    if (Key_Check(KEY_2, KEY_SINGLE)) {
        appCommandPushValue(commandPwmSpeedNextGear, 0.0f);
    }
    if (Key_Check(KEY_3, KEY_SINGLE)) {
        appCommandPushValue(AppState()->running ? commandStop : commandStart, 0.0f);
    }
}

void appCommandProcessTune(void)
{
    pidTuneProcess();
}

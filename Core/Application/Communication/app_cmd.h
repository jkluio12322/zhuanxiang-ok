#ifndef __APP_CMD_H
#define __APP_CMD_H

#include "app_types.h"

void AppCmd_Init(void);
void AppCmd_PushValue(commandType_t type, float value);
void AppCmd_Push(commandMessage_t message);
int AppCmd_Pop(commandMessage_t *message);
uint8_t AppCmd_Available(void);
void AppCmd_Task20ms(void);

#endif

#ifndef __YING_YONG_MING_LING_H
#define __YING_YONG_MING_LING_H

#include "yingYongLeiXing.h"

void appCommandInit(void);
void appCommandPushValue(commandType_t type, float value);
void appCommandQueuePush(commandMessage_t message);
int appCommandQueuePop(commandMessage_t *message);
uint8_t appCommandQueueAvailable(void);
void appCommandProcessBluetooth(void);
void appCommandProcessKeys(void);
void appCommandProcessTune(void);

#endif

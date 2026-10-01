#ifndef __YING_YONG_KONG_ZHI_H
#define __YING_YONG_KONG_ZHI_H

#include <stdint.h>
#include "yingYongLeiXing.h"

void appControlResetModeState(void);
PwmTestPhase_t appControlGetPwmSpeedPhase(void);
uint8_t appControlGetPwmTrackBasePwm(void);
void appControlPwmTrackTask20ms(void);
void appControlPwmTrackStart(void);
void appControlPwmTrackSetSimEnabled(uint8_t enabled);
uint8_t appControlPwmTrackGetSimEnabled(void);
void appControlPwmTrackSetAngle(uint8_t angleDeg);
void appControlPwmTrackNextAngle(void);
uint8_t appControlPwmTrackGetAngle(void);
PwmTestPhase_t appControlPwmTrackGetPhase(void);
void appControlTask50ms(void);

#endif

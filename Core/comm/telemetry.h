/* VOFA+ 遥测：把 yaw 用 FireWater 文本协议发出去（"yaw:123.45\n"）。 */
#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include <stdint.h>

void telemetry_task(void);      /* 主循环调用，20 ms 一次 */
void telemetry_banner(void);

#endif /* __TELEMETRY_H */

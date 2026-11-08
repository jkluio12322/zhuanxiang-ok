/* 计圈：每检测到一个直角弯记一次，4 个弯一圈。
 * 用确认计数 + 弯间最小间隔双重保护，避免同一个弯被数成两圈。 */
#ifndef __LAPS_H
#define __LAPS_H

#include <stdint.h>

void laps_init(uint8_t target);
void laps_update(uint8_t corner_now);   /* 每 5 ms 调用，参数=本拍是否处于弯道 */
uint8_t laps_count(void);
uint8_t laps_target(void);
void laps_set_target(uint8_t target);
void laps_reset(void);

#endif /* __LAPS_H */

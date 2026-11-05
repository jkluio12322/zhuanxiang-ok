#ifndef __TRACKBIN_H
#define __TRACKBIN_H

#include <stdint.h>

void trackbin_init(void);
uint8_t trackbin_start(uint32_t now_ms);
void trackbin_stop(void);
uint8_t trackbin_is_active(void);
uint8_t trackbin_has_fault(void);
void trackbin_host_heartbeat(uint32_t now_ms);
void trackbin_capture_track(uint8_t pattern, uint8_t valid,
                            float line_error, uint16_t exec_20us,
                            uint16_t interval_10us);
void trackbin_capture_motion(int16_t left_delta, int16_t right_delta);
void trackbin_capture_control(uint16_t exec_20us,
                              uint16_t interval_10us);
void trackbin_tick_5ms(uint32_t timestamp_ms);
void trackbin_process(void);

#endif

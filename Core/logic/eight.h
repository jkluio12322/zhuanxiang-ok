#ifndef __EIGHT_H
#define __EIGHT_H

#include <stdint.h>

typedef enum {
    EIGHT_IDLE = 0,
    EIGHT_PRECHECK,
    EIGHT_TURN_A_C,
    EIGHT_LEG_A_C,
    EIGHT_TURN_C_ARC,
    EIGHT_ARC_C_B,
    EIGHT_TURN_B_D,
    EIGHT_LEG_B_D,
    EIGHT_TURN_D_ARC,
    EIGHT_ARC_D_A,
    EIGHT_FINISH,
    EIGHT_FAULT
} EightPhase_t;

typedef enum {
    EIGHT_FAULT_NONE = 0,
    EIGHT_FAULT_IMU_NOT_READY,
    EIGHT_FAULT_IMU_STALE,
    EIGHT_FAULT_IMU_GAP,
    EIGHT_FAULT_ARC_LOST,
    EIGHT_FAULT_ROUTE_TIMEOUT
} EightFault_t;

typedef struct {
    volatile EightPhase_t phase;
    volatile EightFault_t fault;
    volatile uint8_t speed_index;
    volatile float speed_rpm;
    volatile float progress_cm;
    volatile float target_distance_cm;
    volatile float yaw_deg;
    volatile float target_yaw_deg;
    volatile float line_error;
    volatile uint8_t ir_pattern;
    volatile uint32_t elapsed_ms;
} EightView_t;

void eight_init(void);
void eight_reset(void);
uint8_t eight_start(uint32_t now_ms);
void eight_next_speed(void);
void eight_task(uint32_t now_ms);

const EightView_t *eight_view(void);
const char *eight_phase_name(EightPhase_t phase);
const char *eight_fault_name(EightFault_t fault);

#endif

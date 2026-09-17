/**
 * @file    irtrack.h
 * @brief   Digital eight-channel line tracking sensor interface.
 */
#ifndef __IRTRACK_H
#define __IRTRACK_H

#include "main.h"

#define IRTRACK_CHANNELS  8

typedef enum {
    IR_MODE_DIGITAL = 0,
    IR_MODE_ANALOG  = 1,
} IR_Mode_t;

typedef enum {
    TRACK_NORMAL     = 0,
    TRACK_HARD_LEFT  = 1,
    TRACK_HARD_RIGHT = 2,
    TRACK_FINISH     = 3,
    TRACK_LOST       = 4,
} IR_State_t;

typedef struct {
    uint8_t   raw[IRTRACK_CHANNELS];    /* 0=white, 1=black */
    int8_t    weight_sum;
    float     position;                 /* Normalized position, about -7..+7 */
    uint8_t   left_percent;             /* Direct lookup target, 40..180 */
    uint8_t   right_percent;            /* Direct lookup target, 40..180 */
    uint8_t   black_count;
    IR_State_t state;
    uint8_t   pattern;                  /* bit0=IR1(left) ... bit7=IR8(right) */
    int8_t    last_side;                /* -1=left, 0=unknown, +1=right */
} IRTrack_Data_t;

extern IRTrack_Data_t ir_data;

void IRTrack_Init(IR_Mode_t mode);
void IRTrack_Read(void);
void IRTrack_Compute(void);
IR_State_t IRTrack_DetectCrossroad(void);
void IRTrack_GPIO_Init(void);

#endif

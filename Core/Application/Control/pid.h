#ifndef __BSP_PID_H
#define __BSP_PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Target;
    float Actual;
    float Error0;
    float Error1;
    float Error2;
    float Out;
    float OutMax;
    float OutMin;
    float LastOutput;
} PID_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float ErrorPrevious;
    float Integral;
} TrackPid_t;

typedef enum {
    PID_FIELD_KP = 0,
    PID_FIELD_KI,
    PID_FIELD_KD
} PID_Field_t;

void PID_Init(PID_t *p);
void PID_SetParam(PID_t *p, float kp, float ki, float kd);
void PID_SetField(PID_t *p, PID_Field_t field, float value);
void PID_SetTarget(PID_t *p, float target);
void PID_ResetRuntime(PID_t *p);
void PID_ClampOutput(PID_t *p);
void PID_IncrementalUpdate(PID_t *p);
float PID_UpdateSpeed(PID_t *p, float target, float actual);

void TrackPid_Init(TrackPid_t *p);
void TrackPid_SetGains(TrackPid_t *p, float kp, float ki, float kd);
void TrackPid_GetGains(const TrackPid_t *p, float *kp, float *ki, float *kd);
void TrackPid_Reset(TrackPid_t *p);
float TrackPid_Update(TrackPid_t *p, float error, float dt, float diffLimit);

#endif

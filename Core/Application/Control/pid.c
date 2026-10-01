#include "pid.h"

#define PID_SPEED_STOP_HOLD_RPM 0.5f

void PID_Init(PID_t *p)
{
    p->Kp = 0.0f;
    p->Ki = 0.0f;
    p->Kd = 0.0f;
    p->Target = 0.0f;
    p->Actual = 0.0f;
    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->Error2 = 0.0f;
    p->Out = 0.0f;
    p->OutMax = 100.0f;
    p->OutMin = -100.0f;
    p->LastOutput = 0.0f;
}

void PID_SetParam(PID_t *p, float kp, float ki, float kd)
{
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
}

void PID_SetField(PID_t *p, PID_Field_t field, float value)
{
    if (field == PID_FIELD_KP) p->Kp = value;
    else if (field == PID_FIELD_KI) p->Ki = value;
    else if (field == PID_FIELD_KD) p->Kd = value;
}

void PID_SetTarget(PID_t *p, float target)
{
    int oldSign = (p->Target > 0.0f) ? 1 : ((p->Target < 0.0f) ? -1 : 0);
    int newSign = (target > 0.0f) ? 1 : ((target < 0.0f) ? -1 : 0);

    p->Target = target;
    if (oldSign != newSign) {
        p->Error0 = 0.0f;
        p->Error1 = 0.0f;
        p->Error2 = 0.0f;
        p->Out = 0.0f;
        p->LastOutput = 0.0f;
    }
}

void PID_ResetRuntime(PID_t *p)
{
    p->Target = 0.0f;
    p->Actual = 0.0f;
    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->Error2 = 0.0f;
    p->Out = 0.0f;
    p->LastOutput = 0.0f;
}

void PID_ClampOutput(PID_t *p)
{
    if (p->Out > p->OutMax) p->Out = p->OutMax;
    if (p->Out < p->OutMin) p->Out = p->OutMin;
    p->LastOutput = p->Out;
}

void PID_IncrementalUpdate(PID_t *p)
{
    float delta;

    p->Error2 = p->Error1;
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    delta = p->Kp * (p->Error0 - p->Error1)
            + p->Ki * p->Error0
            + p->Kd * (p->Error0 - 2.0f * p->Error1 + p->Error2);
    p->Out += delta;
    PID_ClampOutput(p);
}

float PID_UpdateSpeed(PID_t *p, float target, float actual)
{
    PID_SetTarget(p, target);
    p->Actual = actual;
    PID_IncrementalUpdate(p);
    if (target > -PID_SPEED_STOP_HOLD_RPM && target < PID_SPEED_STOP_HOLD_RPM) {
        p->Out = 0.0f;
    }
    PID_ClampOutput(p);
    return p->Out;
}

void TrackPid_Init(TrackPid_t *p)
{
    p->Kp = 0.0f;
    p->Ki = 0.0f;
    p->Kd = 0.0f;
    p->ErrorPrevious = 0.0f;
    p->Integral = 0.0f;
}

void TrackPid_SetGains(TrackPid_t *p, float kp, float ki, float kd)
{
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
}

void TrackPid_GetGains(const TrackPid_t *p, float *kp, float *ki, float *kd)
{
    if (kp != 0) *kp = p->Kp;
    if (ki != 0) *ki = p->Ki;
    if (kd != 0) *kd = p->Kd;
}

void TrackPid_Reset(TrackPid_t *p)
{
    p->ErrorPrevious = 0.0f;
    p->Integral = 0.0f;
}

float TrackPid_Update(TrackPid_t *p, float error, float dt, float diffLimit)
{
    float derivative = error - p->ErrorPrevious;
    float output;

    p->Integral += p->Ki * error * dt;
    if (p->Integral > diffLimit) p->Integral = diffLimit;
    if (p->Integral < -diffLimit) p->Integral = -diffLimit;

    output = p->Kp * error + p->Integral + p->Kd * derivative;
    p->ErrorPrevious = error;
    if (output > diffLimit) output = diffLimit;
    if (output < -diffLimit) output = -diffLimit;
    return output;
}

/* 编译期速度标定表。默认值来自当前台架近似曲线，正式标定后由主机工具生成。 */

#include "speed_calibration.h"
#include "config.h"

static const float dutyPoints[SPEED_CAL_POINT_COUNT] = {
    0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f,
    60.0f, 70.0f, 80.0f, 90.0f, 100.0f
};

static const float rpmFwdLeft[SPEED_CAL_POINT_COUNT] = {
    0.0f, 0.0f, 18.0f, 39.0f, 59.0f, 78.0f,
    97.0f, 116.0f, 135.0f, 153.0f, 170.0f
};

static const float rpmFwdRight[SPEED_CAL_POINT_COUNT] = {
    0.0f, 0.0f, 20.0f, 41.0f, 61.0f, 80.0f,
    99.0f, 118.0f, 137.0f, 154.0f, 168.0f
};

static const float rpmRevLeft[SPEED_CAL_POINT_COUNT] = {
    0.0f, 0.0f, 19.0f, 40.0f, 60.0f, 79.0f,
    98.0f, 117.0f, 136.0f, 154.0f, 168.0f
};

static const float rpmRevRight[SPEED_CAL_POINT_COUNT] = {
    0.0f, 0.0f, 18.0f, 39.0f, 59.0f, 79.0f,
    98.0f, 117.0f, 136.0f, 153.0f, 167.0f
};

static const float deadzoneStart[2][2] = {
    {22.0f, 21.0f},
    {21.0f, 22.0f}
};

static const float deadzoneSustain[2][2] = {
    {18.0f, 17.0f},
    {17.0f, 18.0f}
};

static const float *curve_for(int8_t direction, SpeedCalSide_t side)
{
    if (direction > 0) {
        return (side == SPEED_CAL_LEFT) ? rpmFwdLeft : rpmFwdRight;
    }
    return (side == SPEED_CAL_LEFT) ? rpmRevLeft : rpmRevRight;
}

static float interpolate(float x, float x0, float x1, float y0, float y1)
{
    if (x1 <= x0) {
        return y1;
    }
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

static float curve_eval(float duty, const float *rpm)
{
    uint8_t i;

    if (duty <= dutyPoints[0]) return rpm[0];
    if (duty >= dutyPoints[SPEED_CAL_POINT_COUNT - 1U]) {
        return rpm[SPEED_CAL_POINT_COUNT - 1U];
    }

    for (i = 1U; i < SPEED_CAL_POINT_COUNT; i++) {
        if (duty <= dutyPoints[i]) {
            return interpolate(duty, dutyPoints[i - 1U], dutyPoints[i],
                               rpm[i - 1U], rpm[i]);
        }
    }
    return rpm[SPEED_CAL_POINT_COUNT - 1U];
}

static float rpm_to_duty(float target_rpm, const float *rpm)
{
    uint8_t i;
    uint8_t first_motion = SPEED_CAL_POINT_COUNT;

    if (target_rpm <= 0.0f) return 0.0f;

    for (i = 1U; i < SPEED_CAL_POINT_COUNT; i++) {
        if ((rpm[i] > 0.0f) && (first_motion == SPEED_CAL_POINT_COUNT)) {
            first_motion = i;
        }
        if (target_rpm <= rpm[i]) {
            /* A real motor does not move between 0% and breakaway duty.
               Avoid interpolating through a synthetic zero-RPM segment. */
            if ((i == first_motion) || (rpm[i - 1U] <= 0.0f)) {
                return dutyPoints[i];
            }
            return interpolate(target_rpm, rpm[i - 1U], rpm[i],
                               dutyPoints[i - 1U], dutyPoints[i]);
        }
    }
    return 100.0f;
}

float speed_calibration_predict_rpm(float signed_duty, SpeedCalSide_t side)
{
    float magnitude;
    const float *curve;
    int8_t direction;

    if (signed_duty > 0.0f) {
        direction = 1;
        magnitude = signed_duty;
    } else if (signed_duty < 0.0f) {
        direction = -1;
        magnitude = -signed_duty;
    } else {
        return 0.0f;
    }
    if (magnitude > 100.0f) magnitude = 100.0f;

    curve = curve_for(direction, side);
    return (float)direction * curve_eval(magnitude, curve);
}

float speed_calibration_feedforward(float target_rpm, SpeedCalSide_t side)
{
    float duty;
    int8_t direction;
    float magnitude;
    const float *curve;

    if (target_rpm > 0.0f) {
        direction = 1;
        magnitude = target_rpm;
    } else if (target_rpm < 0.0f) {
        direction = -1;
        magnitude = -target_rpm;
    } else {
        return 0.0f;
    }

    curve = curve_for(direction, side);
    duty = rpm_to_duty(magnitude, curve);
    if (magnitude < SPEED_CAL_FF_TAPER_RPM) {
        duty *= magnitude / SPEED_CAL_FF_TAPER_RPM;
    }
    return (float)direction * duty;
}

float speed_calibration_compensated_duty(float requested_duty,
                                         int8_t direction,
                                         SpeedCalSide_t side)
{
    float requested;
    float left_rpm;
    float right_rpm;
    float desired_rpm;
    const float *curve;

    if (direction == 0) return 0.0f;
    requested = (requested_duty < 0.0f) ? -requested_duty : requested_duty;
    if (requested > 100.0f) requested = 100.0f;

    left_rpm = speed_calibration_predict_rpm((float)direction * requested,
                                              SPEED_CAL_LEFT);
    right_rpm = speed_calibration_predict_rpm((float)direction * requested,
                                               SPEED_CAL_RIGHT);
    desired_rpm = 0.5f * (left_rpm + right_rpm);
    if (desired_rpm <= 0.0f) return 0.0f;

    curve = curve_for(direction, side);
    return (float)direction * rpm_to_duty(desired_rpm, curve);
}

float speed_calibration_start_deadzone(int8_t direction, SpeedCalSide_t side)
{
    return deadzoneStart[(direction < 0) ? 1U : 0U][(side == SPEED_CAL_LEFT) ? 0U : 1U];
}

float speed_calibration_sustain_deadzone(int8_t direction, SpeedCalSide_t side)
{
    return deadzoneSustain[(direction < 0) ? 1U : 0U][(side == SPEED_CAL_LEFT) ? 0U : 1U];
}

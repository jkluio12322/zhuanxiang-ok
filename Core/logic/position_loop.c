/* 轮端位置外环。Kd 作用于实际 RPM，阶跃目标不会产生微分冲击。 */

#include "position_loop.h"
#include "config.h"
#include <math.h>

typedef struct {
    float target_deg;
    float reference_deg;
    float actual_deg;
    float error_deg;
    float velocity_ff_rpm;
    float actual_rpm;
    float rpm_cmd;
    uint8_t holding;
    uint8_t settling;
} PositionAxisState_t;

static PositionAxisState_t axis_state[2];
static PositionLoopView_t view;
static float positionKp[2];
static float positionKd[2];
static float positionTrackKp[2];
static float positionTrackKd[2];
static float positionDeadbandEnterDeg[2];
static float positionDeadbandReleaseDeg[2];
static float positionCreepRpm[2];

static float clampf(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

static void sync_view(void)
{
    for (uint8_t i = 0u; i < 2u; i++) {
        view.axis[i].target_deg = axis_state[i].target_deg;
        view.axis[i].reference_deg = axis_state[i].reference_deg;
        view.axis[i].actual_deg = axis_state[i].actual_deg;
        view.axis[i].error_deg = axis_state[i].error_deg;
        view.axis[i].velocity_ff_rpm = axis_state[i].velocity_ff_rpm;
        view.axis[i].actual_rpm = axis_state[i].actual_rpm;
        view.axis[i].rpm_cmd = axis_state[i].rpm_cmd;
        view.axis[i].holding = axis_state[i].holding;
    }
}

void position_loop_init(void)
{
    positionKp[POSITION_LEFT] = POSITION_LEFT_KP;
    positionKd[POSITION_LEFT] = POSITION_LEFT_KD;
    positionDeadbandEnterDeg[POSITION_LEFT] = POSITION_LEFT_ENTER_DEG;
    positionDeadbandReleaseDeg[POSITION_LEFT] = POSITION_LEFT_RELEASE_DEG;
    positionCreepRpm[POSITION_LEFT] = POSITION_LEFT_CREEP_RPM;

    positionKp[POSITION_RIGHT] = POSITION_RIGHT_KP;
    positionKd[POSITION_RIGHT] = POSITION_RIGHT_KD;
    positionDeadbandEnterDeg[POSITION_RIGHT] = POSITION_RIGHT_ENTER_DEG;
    positionDeadbandReleaseDeg[POSITION_RIGHT] = POSITION_RIGHT_RELEASE_DEG;
    positionCreepRpm[POSITION_RIGHT] = POSITION_RIGHT_CREEP_RPM;
    positionTrackKp[POSITION_LEFT] = POSITION_TRACK_LEFT_KP;
    positionTrackKd[POSITION_LEFT] = POSITION_TRACK_LEFT_KD;
    positionTrackKp[POSITION_RIGHT] = POSITION_TRACK_RIGHT_KP;
    positionTrackKd[POSITION_RIGHT] = POSITION_TRACK_RIGHT_KD;
    position_loop_reset();
}

void position_loop_reset(void)
{
    for (uint8_t i = 0u; i < 2u; i++) {
        axis_state[i].target_deg = 0.0f;
        axis_state[i].reference_deg = 0.0f;
        axis_state[i].actual_deg = 0.0f;
        axis_state[i].error_deg = 0.0f;
        axis_state[i].velocity_ff_rpm = 0.0f;
        axis_state[i].actual_rpm = 0.0f;
        axis_state[i].rpm_cmd = 0.0f;
        axis_state[i].holding = 0u;
        axis_state[i].settling = 0u;
    }
    sync_view();
}

float position_loop_get_track_kp(PositionSide_t side)
{
    return positionTrackKp[side];
}

float position_loop_get_track_kd(PositionSide_t side)
{
    return positionTrackKd[side];
}

uint8_t position_loop_set_track_gains(PositionSide_t side,
                                      float kp, float kd)
{
    if ((side > POSITION_RIGHT)
        || !((kp >= 0.0f) && (kp <= 10.0f))
        || !((kd >= 0.0f) && (kd <= 10.0f))) {
        return 0U;
    }
    positionTrackKp[side] = kp;
    positionTrackKd[side] = kd;
    return 1U;
}

void position_loop_begin(float target_left_deg, float target_right_deg)
{
    axis_state[POSITION_LEFT].target_deg =
        clampf(target_left_deg, -POSITION_LIMIT_DEG, POSITION_LIMIT_DEG);
    axis_state[POSITION_RIGHT].target_deg =
        clampf(target_right_deg, -POSITION_LIMIT_DEG, POSITION_LIMIT_DEG);
    axis_state[POSITION_LEFT].holding = 0u;
    axis_state[POSITION_RIGHT].holding = 0u;
    axis_state[POSITION_LEFT].settling = 0u;
    axis_state[POSITION_RIGHT].settling = 0u;
}

void position_loop_track_begin(float actual_left_deg, float actual_right_deg)
{
    position_loop_reset();
    axis_state[POSITION_LEFT].reference_deg = actual_left_deg;
    axis_state[POSITION_RIGHT].reference_deg = actual_right_deg;
    axis_state[POSITION_LEFT].actual_deg = actual_left_deg;
    axis_state[POSITION_RIGHT].actual_deg = actual_right_deg;
    sync_view();
}

static float wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static void track_axis(PositionSide_t side, float desired_rpm,
                       float actual_deg, float actual_rpm,
                       float out_min, float out_max)
{
    PositionAxisState_t *axis = &axis_state[side];
    float output;

    axis->actual_deg = actual_deg;
    axis->actual_rpm = actual_rpm;
    axis->velocity_ff_rpm = desired_rpm;
    axis->error_deg = wrap_deg(axis->reference_deg - actual_deg);
    axis->holding = 0U;
    axis->settling = 0U;

    output = desired_rpm
             + positionTrackKp[side] * axis->error_deg
             + positionTrackKd[side] * (desired_rpm - actual_rpm);
    axis->rpm_cmd = clampf(output, out_min, out_max);
    axis->reference_deg = wrap_deg(
        axis->reference_deg
        + desired_rpm * 6.0f * ((float)CONTROL_PERIOD_MS * 0.001f));
}

void position_loop_track_step(float desired_left_rpm, float desired_right_rpm,
                              float actual_left_deg, float actual_right_deg,
                              float actual_left_rpm, float actual_right_rpm,
                              float out_min, float out_max)
{
    track_axis(POSITION_LEFT, desired_left_rpm,
               actual_left_deg, actual_left_rpm, out_min, out_max);
    track_axis(POSITION_RIGHT, desired_right_rpm,
               actual_right_deg, actual_right_rpm, out_min, out_max);
    sync_view();
}

static float update_axis(PositionSide_t side,
                         PositionAxisState_t *axis,
                         float reference_deg,
                         float velocity_ff_rpm,
                         float actual_deg,
                         float actual_rpm,
                         float limit_rpm,
                         float approach_accel_rpm_s,
                         float brake_accel_rpm_s)
{
    float magnitude;
    float approach_speed;
    float limited_ff;
    float braking_speed;
    float output;

    axis->reference_deg = reference_deg;
    axis->actual_deg = actual_deg;
    axis->velocity_ff_rpm = velocity_ff_rpm;
    axis->actual_rpm = actual_rpm;
    axis->error_deg = axis->target_deg - actual_deg;
    magnitude = fabsf(axis->error_deg);

    if (axis->holding == 0u) {
        if (magnitude <= positionDeadbandEnterDeg[side]) {
            axis->holding = 1u;
            axis->settling = 1u;
        }
    } else if (magnitude >= positionDeadbandReleaseDeg[side]) {
        axis->holding = 0u;
    }

    if (axis->settling != 0u) {
        axis->rpm_cmd = 0.0f;
        if (fabsf(actual_rpm) <= POSITION_SETTLE_RPM_MAX) {
            if (magnitude < positionDeadbandReleaseDeg[side]) {
                axis->holding = 1u;
            }
            axis->settling = 0u;
        }
        return 0.0f;
    }

    if (axis->holding != 0u) {
        axis->rpm_cmd = 0.0f;
        return 0.0f;
    }

    limited_ff = velocity_ff_rpm;
    if ((limited_ff * axis->error_deg > 0.0f) && (brake_accel_rpm_s > 0.0f)) {
        braking_speed = sqrtf(brake_accel_rpm_s * magnitude / 3.0f);
        if (braking_speed > limit_rpm) braking_speed = limit_rpm;
        if (fabsf(limited_ff) > braking_speed) {
            limited_ff = (limited_ff > 0.0f) ? braking_speed : -braking_speed;
        }
    }

    if ((fabsf(limited_ff) < 0.01f) && (approach_accel_rpm_s > 0.0f)) {
        approach_speed = sqrtf(approach_accel_rpm_s * magnitude / 3.0f);
        if (approach_speed > limit_rpm) approach_speed = limit_rpm;
        output = ((axis->error_deg > 0.0f) ? approach_speed : -approach_speed)
                 - positionKd[side] * actual_rpm;
    } else {
        output = limited_ff
                 + positionKp[side] * axis->error_deg
                 - positionKd[side] * actual_rpm;
    }
    if ((output > -positionCreepRpm[side])
        && (output < positionCreepRpm[side])
        && (axis->error_deg != 0.0f)) {
        output = (axis->error_deg > 0.0f)
                 ? positionCreepRpm[side] : -positionCreepRpm[side];
    }
    axis->rpm_cmd = clampf(output, -limit_rpm, limit_rpm);
    return axis->rpm_cmd;
}

void position_loop_step(float reference_left_deg, float reference_right_deg,
                        float velocity_left_rpm, float velocity_right_rpm,
                        float actual_left_deg, float actual_right_deg,
                        float actual_left_rpm, float actual_right_rpm,
                        float output_limit_rpm, float approach_accel_rpm_s,
                        float brake_accel_rpm_s)
{
    (void)update_axis(POSITION_LEFT, &axis_state[POSITION_LEFT], reference_left_deg,
                      velocity_left_rpm, actual_left_deg, actual_left_rpm,
                      output_limit_rpm, approach_accel_rpm_s, brake_accel_rpm_s);
    (void)update_axis(POSITION_RIGHT, &axis_state[POSITION_RIGHT], reference_right_deg,
                      velocity_right_rpm, actual_right_deg, actual_right_rpm,
                      output_limit_rpm, approach_accel_rpm_s, brake_accel_rpm_s);
    sync_view();
}

const PositionLoopView_t *position_loop_view(void)
{
    return &view;
}

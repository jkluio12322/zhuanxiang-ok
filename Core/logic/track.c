/* 八路数字循迹：稳定分母归一化质心、位置式转向 PD、陀螺仪阻尼和计圈。
   本模块只生成左右轮速度意图，位置与速度闭环由 position_loop/speed_loop 执行。 */

#include "track.h"
#include "config.h"
#include "imu.h"
#include "motion.h"
#include "position_loop.h"
#include "sched.h"
#include "state.h"
#include "irtrack.h"

#include <math.h>

typedef struct {
    uint8_t pattern;
    uint8_t black_count;
    uint8_t contiguous;
    uint8_t valid;
    float   line_error;
} LineSample_t;

static LineSample_t sample;
static TrackState_t trackState = TRK_STOPPED;
static TrackOutput_t output;
static ConfirmCounter cornerCounter;
static uint16_t cornerGap;
static uint8_t lapCorners;

static float trackKp = TRACK_PID_KP;
static float trackKd = TRACK_PID_KD;
static float trackGyroKp = TRACK_GYRO_KP;
static float lineErrorPrevious;
static float derivativeFiltered;
static uint8_t derivativeInitialized;
static float lastSteerRatio;
static float baseRpm;

static float lostLastYaw;
static float lostYawDeg;
static uint8_t lostYawInitialized;
static float steerRpmToPwmGain = TRACK_STEER_RPM_TO_PWM_DEFAULT;

static void    evaluate_line(void);
static void    sample_line(void);
static void    inject_pattern(uint8_t pattern);
static uint8_t run_line_quality(uint32_t now_ms);
static uint8_t line_is_contiguous(uint8_t pattern);
static float   wrap_yaw_delta(float delta);
static uint8_t update_lost_heading(uint32_t now_ms);
static float   slew_base_rpm(float current, float target);
static float   update_steering(float line_error, uint32_t now_ms,
                               uint8_t allow_gyro);
static void    track_apply_velocity(float desired_left_rpm,
                                    float desired_right_rpm);

static float clampf(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

static uint8_t imu_damping_ready(const imu_data_t *imu, uint32_t now_ms)
{
    return (uint8_t)((imu->ok != 0U)
                     && (imu->cal_status == IMU_CAL_READY)
                     && ((uint32_t)(now_ms - imu->sample_ms)
                         <= TRACK_LOST_IMU_STALE_MS));
}

static void reset_steering_runtime(void)
{
    derivativeInitialized = 0U;
    lineErrorPrevious = 0.0f;
    derivativeFiltered = 0.0f;
    lastSteerRatio = 0.0f;
    output.steer_ratio = 0.0f;
    output.steer_rpm = 0.0f;
}

static float slew_base_rpm(float current, float target)
{
    float step = TRACK_BASE_ACCEL_RPM_S
                 * ((float)CONTROL_PERIOD_MS * 0.001f);

    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

static float update_steering(float line_error, uint32_t now_ms,
                             uint8_t allow_gyro)
{
    const imu_data_t *imu = imu_get();
    float control_error = line_error * TRACK_STEERING_SIGN;
    float derivative_raw;
    float derivative;
    float gyro_damping = 0.0f;
    float command;
    float alpha;
    float dt = (float)CONTROL_PERIOD_MS * 0.001f;

    if (derivativeInitialized == 0U) {
        lineErrorPrevious = control_error;
        derivativeFiltered = 0.0f;
        derivativeInitialized = 1U;
    }
    derivative_raw = (control_error - lineErrorPrevious) / dt;
    lineErrorPrevious = control_error;
    alpha = dt / (((float)TRACK_D_FILTER_MS * 0.001f) + dt);
    derivativeFiltered += alpha * (derivative_raw - derivativeFiltered);
    derivative = derivativeFiltered;

    output.imu_active = 0U;
    if ((allow_gyro != 0U) && (imu_damping_ready(imu, now_ms) != 0U)) {
        output.imu_active = 1U;
        gyro_damping = TRACK_GYRO_SIGN * trackGyroKp * imu->gyro_dps[2];
        output.yaw_rate_dps = imu->gyro_dps[2];
    } else {
        output.yaw_rate_dps = 0.0f;
    }

    command = trackKp * control_error
              + trackKd * derivative
              - gyro_damping;
    command = clampf(command,
                     -TRACK_STEER_RATIO_LIMIT,
                     TRACK_STEER_RATIO_LIMIT);
    lastSteerRatio = command;
    output.steer_ratio = command;
    return command;
}

static void differential_from_ratio(float base, float ratio,
                                    float *left, float *right)
{
    float magnitude = fabsf(ratio);

    if (ratio >= 0.0f) {
        *left = base * (1.0f - magnitude);
        *right = base * (1.0f + TRACK_OUTER_WHEEL_GAIN * magnitude);
    } else {
        *left = base * (1.0f + TRACK_OUTER_WHEEL_GAIN * magnitude);
        *right = base * (1.0f - magnitude);
    }

    *left = clampf(*left, 0.0f, SPEED_TARGET_LIMIT_RPM);
    *right = clampf(*right, 0.0f, SPEED_TARGET_LIMIT_RPM);
}

void track_wheel_targets_from_ratio(float base_rpm, float ratio,
                                    float *left_rpm, float *right_rpm)
{
    if ((left_rpm == 0) || (right_rpm == 0)) {
        return;
    }
    differential_from_ratio(base_rpm, ratio, left_rpm, right_rpm);
}

static void track_apply_velocity(float desired_left_rpm,
                                 float desired_right_rpm)
{
    const MotionView_t *motion = motion_view();
    const PositionLoopView_t *position;

    position_loop_track_step(desired_left_rpm, desired_right_rpm,
                             motion->left_angle_deg, motion->right_angle_deg,
                             motion->left_rpm, motion->right_rpm,
                             0.0f, SPEED_TARGET_LIMIT_RPM);
    position = position_loop_view();
    output.desired_left_rpm = desired_left_rpm;
    output.desired_right_rpm = desired_right_rpm;
    output.target_left_rpm = position->axis[POSITION_LEFT].rpm_cmd;
    output.target_right_rpm = position->axis[POSITION_RIGHT].rpm_cmd;
    output.steer_rpm =
        0.5f * (output.desired_right_rpm - output.desired_left_rpm);
    state_set_wheel_targets(output.target_left_rpm, output.target_right_rpm);
}

void track_task(void)
{
    uint32_t now_ms = HAL_GetTick();
    float desired_left;
    float desired_right;

    if (state_view()->mode != modeTrack) {
        return;
    }

    sample_line();

    if (!state_view()->running) {
        trackState = TRK_STOPPED;
        baseRpm = 0.0f;
        reset_steering_runtime();
        output.base_rpm = 0.0f;
        output.line_error = sample.line_error;
        output.lost_yaw_deg = 0.0f;
        output.state = trackState;
        output.pattern = sample.pattern;
        state_set_wheel_targets(0.0f, 0.0f);
        return;
    }

    if (run_line_quality(now_ms) != 0U) {
        output.base_rpm = 0.0f;
        output.line_error = sample.line_error;
        output.state = trackState;
        output.pattern = sample.pattern;
        state_set_wheel_targets(0.0f, 0.0f);
        return;
    }

    baseRpm = slew_base_rpm(baseRpm, state_view()->gear_rpm);

    if (sample.valid != 0U) {
        (void)update_steering(sample.line_error, now_ms, 1U);

        if (cornerGap > 0U) {
            cornerGap--;
        } else if (confirm_update(&cornerCounter,
                                  (uint8_t)(fabsf(sample.line_error)
                                            >= TRACK_CORNER_ERROR_MIN)) != 0U) {
            confirm_reset(&cornerCounter, TRACK_CORNER_CONFIRM_TICKS);
            cornerGap = TRACK_CORNER_GAP_TICKS;
            if (++lapCorners >= TRACK_CORNERS_PER_LAP) {
                lapCorners = 0U;
                state_increment_lap();
            }
        }
    } else {
        output.steer_ratio = lastSteerRatio;
    }

    differential_from_ratio(baseRpm, lastSteerRatio,
                            &desired_left, &desired_right);
    track_apply_velocity(desired_left, desired_right);

    output.base_rpm = baseRpm;
    output.line_error = sample.line_error;
    output.state = trackState;
    output.pattern = sample.pattern;
}

static uint8_t line_is_contiguous(uint8_t pattern)
{
    uint8_t first = 0U;
    uint8_t last = 0U;
    uint8_t found = 0U;

    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        if ((pattern & (uint8_t)(1U << i)) != 0U) {
            if (found == 0U) {
                first = i;
                found = 1U;
            }
            last = i;
        }
    }

    if (found == 0U) {
        return 0U;
    }

    for (uint8_t i = first; i <= last; i++) {
        if ((pattern & (uint8_t)(1U << i)) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static void evaluate_line(void)
{
    sample.pattern = ir_data.pattern;
    sample.black_count = ir_data.black_count;
    sample.line_error = ir_data.normalized_position;
    sample.contiguous = line_is_contiguous(sample.pattern);
    sample.valid = (uint8_t)((sample.black_count >= 1U)
                             && (sample.black_count
                                 <= TRACK_LINE_VALID_MAX_BLACK)
                             && (sample.contiguous != 0U));
}

static void sample_line(void)
{
    ir_read();
    ir_compute();
    evaluate_line();
}

uint8_t track_sample_line(float *line_error, uint8_t *pattern)
{
    sample_line();
    output.line_error = sample.line_error;
    output.pattern = sample.pattern;
    if (line_error != 0) *line_error = sample.line_error;
    if (pattern != 0) *pattern = sample.pattern;
    return sample.valid;
}

float track_follow_update(float line_error)
{
    (void)update_steering(line_error, HAL_GetTick(), 1U);
    trackState = TRK_FOLLOW;
    output.state = trackState;
    return lastSteerRatio;
}

void track_follow_reset(void)
{
    reset_steering_runtime();
    trackState = TRK_STOPPED;
    output.state = trackState;
}

static void inject_pattern(uint8_t pattern)
{
    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        ir_data.raw[i] = ((pattern & (uint8_t)(1U << i)) != 0U) ? 0U : 1U;
    }
    ir_compute();
}

static float wrap_yaw_delta(float delta)
{
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

static uint8_t update_lost_heading(uint32_t now_ms)
{
    const imu_data_t *imu = imu_get();

    if (imu_damping_ready(imu, now_ms) == 0U) {
        state_report_stop(stopReasonCornerTimeout);
        return 1U;
    }

    if (lostYawInitialized == 0U) {
        lostLastYaw = imu->yaw;
        lostYawDeg = 0.0f;
        lostYawInitialized = 1U;
    } else {
        lostYawDeg += wrap_yaw_delta(imu->yaw - lostLastYaw);
        lostLastYaw = imu->yaw;
        output.lost_yaw_deg = lostYawDeg;
        if (fabsf(lostYawDeg) >= TRACK_LOST_YAW_LIMIT_DEG) {
            state_report_stop(stopReasonCornerTimeout);
            return 1U;
        }
    }

    return 0U;
}

static uint8_t run_line_quality(uint32_t now_ms)
{
    if (sample.valid != 0U) {
        if (trackState == TRK_LOST) {
            derivativeInitialized = 0U;
            lostYawInitialized = 0U;
            lostYawDeg = 0.0f;
            output.lost_yaw_deg = 0.0f;
        }
        trackState = TRK_FOLLOW;
        return 0U;
    }

    trackState = TRK_LOST;
    confirm_reset(&cornerCounter, TRACK_CORNER_CONFIRM_TICKS);
    return update_lost_heading(now_ms);
}

const TrackOutput_t *track_output(void)
{
    return &output;
}

void track_init(void)
{
    track_reset_state();
}

void track_reset_state(void)
{
    confirm_reset(&cornerCounter, TRACK_CORNER_CONFIRM_TICKS);
    cornerGap = 0U;
    lapCorners = 0U;
    baseRpm = 0.0f;
    lostLastYaw = 0.0f;
    lostYawDeg = 0.0f;
    lostYawInitialized = 0U;
    trackState = TRK_STOPPED;
    reset_steering_runtime();

    output.base_rpm = 0.0f;
    output.steer_rpm = 0.0f;
    output.steer_ratio = 0.0f;
    output.line_error = 0.0f;
    output.yaw_rate_dps = 0.0f;
    output.desired_left_rpm = 0.0f;
    output.desired_right_rpm = 0.0f;
    output.target_left_rpm = 0.0f;
    output.target_right_rpm = 0.0f;
    output.lost_yaw_deg = 0.0f;
    output.imu_active = 0U;
    output.state = trackState;
    output.pattern = 0U;
    state_set_wheel_targets(0.0f, 0.0f);
}

void track_start(void)
{
    const MotionView_t *motion = motion_view();

    track_reset_state();
    position_loop_track_begin(motion->left_angle_deg, motion->right_angle_deg);
    trackState = TRK_FOLLOW;
    output.state = trackState;
}

float track_get_gain(PidField_t field)
{
    if (field == PID_FIELD_KP) return trackKp;
    if (field == PID_FIELD_KI) return 0.0f;
    if (field == PID_FIELD_KD) return trackKd;
    return 0.0f;
}

uint8_t track_set_gain(PidField_t field, float value)
{
    if (!((value >= 0.0f) && (value <= 1000.0f))) {
        return 0U;
    }

    if (field == PID_FIELD_KP) trackKp = value;
    else if (field == PID_FIELD_KI) return 0U;
    else if (field == PID_FIELD_KD) trackKd = value;
    else return 0U;

    reset_steering_runtime();
    return 1U;
}

float track_get_gyro_gain(void)
{
    return trackGyroKp;
}

uint8_t track_set_gyro_gain(float value)
{
    if (!((value >= 0.0f) && (value <= 10.0f))) {
        return 0U;
    }
    trackGyroKp = value;
    return 1U;
}

void track_set_steer_rpm_to_pwm_gain(float gain)
{
    steerRpmToPwmGain = gain;
}

float track_get_steer_rpm_to_pwm_gain(void)
{
    return steerRpmToPwmGain;
}

uint8_t track_test_step(uint8_t pattern)
{
    uint32_t now_ms = HAL_GetTick();

    if (!state_view()->running) {
        trackState = TRK_STOPPED;
        reset_steering_runtime();
        output.state = trackState;
        state_set_wheel_targets(0.0f, 0.0f);
        return 1U;
    }

    inject_pattern(pattern);
    evaluate_line();

    if (sample.valid != 0U) {
        if (trackState == TRK_LOST) {
            derivativeInitialized = 0U;
        }
        trackState = TRK_FOLLOW;
    } else {
        trackState = TRK_LOST;
    }

    if (sample.valid != 0U) {
        float ratio = update_steering(sample.line_error, now_ms, 0U);
        track_set_pwm_steer(ratio * TRACK_PWM_TEST_STEER_SCALE_RPM);
    }

    output.base_rpm = 0.0f;
    output.line_error = sample.line_error;
    output.state = trackState;
    output.pattern = sample.pattern;
    return 0U;
}

void track_set_pwm_steer(float steerRpm)
{
    output.steer_rpm = steerRpm;
}

float track_get_steer(void)
{
    return output.steer_rpm;
}

float track_get_steer_ratio(void)
{
    return lastSteerRatio;
}

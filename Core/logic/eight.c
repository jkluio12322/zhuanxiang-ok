#include <math.h>

#include "config.h"
#include "eight.h"
#include "imu.h"
#include "motion.h"
#include "position_loop.h"
#include "state.h"
#include "track.h"

static EightView_t view;
static const float speed_levels[EIGHT_SPEED_LEVEL_COUNT] = {30.0f, 40.0f, 50.0f};

static uint32_t route_start_ms;
static uint32_t last_imu_sample_ms;
static uint32_t line_lost_ms;
static float previous_imu_yaw;
static float yaw_total;
static float turn_target_yaw;
static float straight_target_yaw;
static float phase_start_left_deg;
static float phase_start_right_deg;
static uint8_t yaw_initialized;
static uint8_t line_lost;

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float wrap_delta(float delta)
{
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

static float distance_from_angles(float left_deg, float right_deg)
{
    const float cm_per_degree =
        (6.28318530718f * ENCODER_WHEEL_RADIUS_M * 100.0f) / 360.0f;
    return 0.5f * (left_deg + right_deg) * cm_per_degree;
}

static void publish_targets(float left_rpm, float right_rpm)
{
    const MotionView_t *motion = motion_view();
    const PositionLoopView_t *position;

    position_loop_track_step(left_rpm, right_rpm,
                             motion->left_angle_deg, motion->right_angle_deg,
                             motion->left_rpm, motion->right_rpm,
                             -view.speed_rpm, view.speed_rpm);
    position = position_loop_view();
    state_set_wheel_targets(position->axis[POSITION_LEFT].rpm_cmd,
                            position->axis[POSITION_RIGHT].rpm_cmd);
}

static void set_phase(EightPhase_t phase, uint32_t now_ms)
{
    const MotionView_t *motion = motion_view();
    float turn_delta = 0.0f;

    (void)now_ms;
    view.phase = phase;
    view.progress_cm = 0.0f;
    view.target_distance_cm = 0.0f;
    phase_start_left_deg = motion->left_angle_deg;
    phase_start_right_deg = motion->right_angle_deg;
    line_lost = 0U;

    switch (phase) {
    case EIGHT_TURN_A_C:
        turn_delta = EIGHT_START_TURN_DEG;
        break;
    case EIGHT_TURN_C_ARC:
        turn_delta = EIGHT_TURN_C_ARC_DEG;
        break;
    case EIGHT_TURN_B_D:
        turn_delta = EIGHT_TURN_B_D_DEG;
        break;
    case EIGHT_TURN_D_ARC:
        turn_delta = EIGHT_TURN_D_ARC_DEG;
        break;
    case EIGHT_LEG_A_C:
        view.target_distance_cm = EIGHT_LEG_A_C_CM;
        straight_target_yaw = yaw_total;
        break;
    case EIGHT_LEG_B_D:
        view.target_distance_cm = EIGHT_LEG_B_D_CM;
        straight_target_yaw = yaw_total;
        break;
    case EIGHT_ARC_C_B:
        view.target_distance_cm = EIGHT_ARC_LENGTH_CM;
        straight_target_yaw = yaw_total;
        track_follow_reset();
        break;
    case EIGHT_ARC_D_A:
        view.target_distance_cm = EIGHT_ARC_LENGTH_CM;
        straight_target_yaw = yaw_total;
        track_follow_reset();
        break;
    default:
        break;
    }

    if (turn_delta != 0.0f) {
        turn_target_yaw = yaw_total + turn_delta;
        view.target_yaw_deg = turn_target_yaw;
    } else if ((phase == EIGHT_LEG_A_C) || (phase == EIGHT_LEG_B_D)) {
        view.target_yaw_deg = straight_target_yaw;
    } else if (phase == EIGHT_ARC_C_B) {
        view.target_yaw_deg = yaw_total + (180.0f * EIGHT_ARC_C_B_YAW_SIGN);
    } else if (phase == EIGHT_ARC_D_A) {
        view.target_yaw_deg = yaw_total + (180.0f * EIGHT_ARC_D_A_YAW_SIGN);
    }
}

static void enter_next_phase(EightPhase_t next_phase, uint32_t now_ms)
{
    set_phase(next_phase, now_ms);
    if (next_phase == EIGHT_FINISH) {
        publish_targets(0.0f, 0.0f);
        state_report_stop(stopReasonEightComplete);
    }
}

static void fail_route(EightFault_t fault)
{
    if (view.phase == EIGHT_FAULT) {
        return;
    }

    view.phase = EIGHT_FAULT;
    view.fault = fault;
    view.target_distance_cm = 0.0f;
    publish_targets(0.0f, 0.0f);
    state_set_running(0U);
    state_report_stop(stopReasonEightFault);
}

static uint8_t update_yaw(uint32_t now_ms)
{
    const imu_data_t *imu = imu_get();
    uint32_t sample_ms;
    uint32_t sample_dt;
    float delta;

    if ((imu->ok == 0U) || (imu->cal_status != IMU_CAL_READY)) {
        fail_route(EIGHT_FAULT_IMU_NOT_READY);
        return 0U;
    }
    if ((uint32_t)(now_ms - imu->sample_ms) > EIGHT_IMU_STALE_MS) {
        fail_route(EIGHT_FAULT_IMU_STALE);
        return 0U;
    }

    sample_ms = imu->sample_ms;
    if (sample_ms == last_imu_sample_ms) {
        return 1U;
    }

    sample_dt = (uint32_t)(sample_ms - last_imu_sample_ms);
    if (sample_dt > EIGHT_IMU_MAX_GAP_MS) {
        fail_route(EIGHT_FAULT_IMU_GAP);
        return 0U;
    }

    delta = wrap_delta(imu->yaw - previous_imu_yaw);
    yaw_total += delta * EIGHT_YAW_SIGN;
    previous_imu_yaw = imu->yaw;
    last_imu_sample_ms = sample_ms;
    view.yaw_deg = yaw_total;
    return 1U;
}

static void update_elapsed(uint32_t now_ms)
{
    view.elapsed_ms = now_ms - route_start_ms;
    if (view.elapsed_ms >= EIGHT_TOTAL_TIMEOUT_MS) {
        fail_route(EIGHT_FAULT_ROUTE_TIMEOUT);
    }
}

static float drive_speed(float remaining_cm, float minimum_rpm)
{
    float remaining_m;
    float speed_mps;
    float rpm;
    const float wheel_circumference_m =
        6.28318530718f * ENCODER_WHEEL_RADIUS_M;

    if (remaining_cm <= EIGHT_DISTANCE_TOLERANCE_CM) {
        return 0.0f;
    }

    remaining_m = (remaining_cm - EIGHT_DISTANCE_TOLERANCE_CM) * 0.01f;
    speed_mps = sqrtf(2.0f * EIGHT_DRIVE_DECEL_MPS2 * remaining_m);
    rpm = speed_mps * 60.0f / wheel_circumference_m;
    rpm = clampf(rpm, minimum_rpm, view.speed_rpm);
    return rpm;
}

static float straight_heading_correction(float left_progress_deg,
                                         float right_progress_deg)
{
    const imu_data_t *imu = imu_get();
    float error = straight_target_yaw - yaw_total;
    float gyro = imu->gyro_dps[2] * EIGHT_YAW_SIGN;
    float cm_per_degree =
        (6.28318530718f * ENCODER_WHEEL_RADIUS_M * 100.0f) / 360.0f;
    float wheel_difference_cm =
        (right_progress_deg - left_progress_deg) * cm_per_degree;
    float gyro_correction = EIGHT_HEADING_KP * error - EIGHT_HEADING_KD * gyro;
    float physical_correction =
        (EIGHT_YAW_SIGN * gyro_correction)
        - (EIGHT_ENCODER_HEADING_KP * wheel_difference_cm);

    return clampf(physical_correction,
                  -EIGHT_HEADING_CORRECTION_MAX_RPM,
                  EIGHT_HEADING_CORRECTION_MAX_RPM);
}

static float turn_command(void)
{
    const imu_data_t *imu = imu_get();
    float error = turn_target_yaw - yaw_total;
    float gyro = imu->gyro_dps[2] * EIGHT_YAW_SIGN;
    float command = EIGHT_TURN_KP * error - EIGHT_TURN_KD * gyro;

    command = clampf(command, -EIGHT_TURN_MAX_RPM, EIGHT_TURN_MAX_RPM);
    if ((fabsf(error) > 3.0f) && (fabsf(command) < EIGHT_TURN_MIN_RPM)) {
        command = (error > 0.0f) ? EIGHT_TURN_MIN_RPM : -EIGHT_TURN_MIN_RPM;
    }
    return command;
}

static void update_turn(uint32_t now_ms, float *left_target, float *right_target,
                        EightPhase_t next_phase)
{
    float error = turn_target_yaw - yaw_total;
    float command;

    if (fabsf(error) <= EIGHT_TURN_TOLERANCE_DEG) {
        enter_next_phase(next_phase, now_ms);
        return;
    }

    command = turn_command();
    command *= EIGHT_YAW_SIGN;
    *left_target = -command;
    *right_target = command;
}

static EightPhase_t next_after_arc(EightPhase_t phase)
{
    return (phase == EIGHT_ARC_C_B) ? EIGHT_TURN_B_D : EIGHT_FINISH;
}

static void update_arc(uint32_t now_ms, float *left_target, float *right_target,
                       float e_y, uint8_t valid, uint8_t pattern)
{
    const MotionView_t *motion = motion_view();
    float steer_ratio = track_get_steer_ratio();
    float left_progress = motion->left_angle_deg - phase_start_left_deg;
    float right_progress = motion->right_angle_deg - phase_start_right_deg;
    float distance = distance_from_angles(left_progress, right_progress);
    float yaw_progress = yaw_total - straight_target_yaw;
    float yaw_sign = (view.phase == EIGHT_ARC_C_B)
                     ? EIGHT_ARC_C_B_YAW_SIGN : EIGHT_ARC_D_A_YAW_SIGN;
    float oriented_yaw = yaw_progress * yaw_sign;
    float remaining = EIGHT_ARC_LENGTH_CM - distance;
    float base;
    view.progress_cm = distance;
    view.line_error = e_y;
    view.ir_pattern = pattern;
    if (valid != 0U) {
        steer_ratio = track_follow_update(e_y);
        line_lost = 0U;
    } else {
        if (line_lost == 0U) {
            line_lost = 1U;
            line_lost_ms = now_ms;
        }

        if ((distance >= EIGHT_ARC_ENDPOINT_MIN_CM)
            && (oriented_yaw >= EIGHT_ARC_ENDPOINT_MIN_YAW_DEG)) {
            enter_next_phase(next_after_arc((EightPhase_t)view.phase), now_ms);
            return;
        }
        if ((uint32_t)(now_ms - line_lost_ms) >= EIGHT_LINE_LOST_TIMEOUT_MS) {
            fail_route(EIGHT_FAULT_ARC_LOST);
            return;
        }
    }

    if ((valid == 0U) && (line_lost != 0U)) {
        base = EIGHT_ARC_SEARCH_RPM;
        if (fabsf(steer_ratio) < 0.1f) {
            steer_ratio = -yaw_sign * (EIGHT_ARC_SEARCH_STEER_RPM
                                      / EIGHT_ARC_SEARCH_RPM);
        }
        steer_ratio = clampf(steer_ratio, -0.65f, 0.65f);
    } else {
        base = drive_speed(remaining, EIGHT_DRIVE_MIN_RPM);
        if (remaining <= EIGHT_DISTANCE_TOLERANCE_CM) {
            base = EIGHT_DRIVE_MIN_RPM;
        }
    }

    track_wheel_targets_from_ratio(base, steer_ratio,
                                   left_target, right_target);
    *left_target = clampf(*left_target, 0.0f, view.speed_rpm);
    *right_target = clampf(*right_target, 0.0f, view.speed_rpm);
}

void eight_init(void)
{
    view.speed_index = 0U;
    view.speed_rpm = speed_levels[0];
    eight_reset();
}

void eight_reset(void)
{
    view.phase = EIGHT_IDLE;
    view.fault = EIGHT_FAULT_NONE;
    view.progress_cm = 0.0f;
    view.target_distance_cm = 0.0f;
    view.yaw_deg = 0.0f;
    view.target_yaw_deg = 0.0f;
    view.line_error = 0.0f;
    view.ir_pattern = 0U;
    view.elapsed_ms = 0U;
    route_start_ms = 0U;
    last_imu_sample_ms = 0U;
    line_lost_ms = 0U;
    yaw_initialized = 0U;
    line_lost = 0U;
    publish_targets(0.0f, 0.0f);
}

uint8_t eight_start(uint32_t now_ms)
{
    const imu_data_t *imu = imu_get();

    if ((imu->ok == 0u) || (imu->cal_status != IMU_CAL_READY)) {
        publish_targets(0.0f, 0.0f);
        view.phase = EIGHT_FAULT;
        view.fault = EIGHT_FAULT_IMU_NOT_READY;
        view.target_distance_cm = 0.0f;
        state_report_stop(stopReasonEightFault);
        return 0u;
    }

    motion_reset_position();
    position_loop_track_begin(0.0f, 0.0f);
    route_start_ms = now_ms;
    view.fault = EIGHT_FAULT_NONE;
    view.phase = EIGHT_PRECHECK;
    view.speed_rpm = speed_levels[view.speed_index];
    view.progress_cm = 0.0f;
    view.target_distance_cm = 0.0f;
    view.elapsed_ms = 0U;
    yaw_initialized = 0U;
    line_lost = 0U;
    previous_imu_yaw = imu->yaw;
    yaw_total = 0.0f;
    view.yaw_deg = 0.0f;
    view.target_yaw_deg = 0.0f;
    publish_targets(0.0f, 0.0f);
    return 1u;
}

void eight_next_speed(void)
{
    if (state_view()->running != 0U) {
        return;
    }
    view.speed_index++;
    if (view.speed_index >= EIGHT_SPEED_LEVEL_COUNT) {
        view.speed_index = 0U;
    }
    view.speed_rpm = speed_levels[view.speed_index];
}

void eight_task(uint32_t now_ms)
{
    const MotionView_t *motion;
    const imu_data_t *imu;
    float left_target = 0.0f;
    float right_target = 0.0f;
    float e_y = 0.0f;
    uint8_t pattern = 0U;
    uint8_t line_valid;

    if (state_view()->running == 0U) {
        publish_targets(0.0f, 0.0f);
        return;
    }
    if ((view.phase == EIGHT_FAULT) || (view.phase == EIGHT_FINISH)
        || (view.phase == EIGHT_IDLE)) {
        publish_targets(0.0f, 0.0f);
        return;
    }

    update_elapsed(now_ms);
    if (view.phase == EIGHT_FAULT) {
        return;
    }

    line_valid = track_sample_line(&e_y, &pattern);
    view.line_error = e_y;
    view.ir_pattern = pattern;
    imu = imu_get();

    if (view.phase == EIGHT_PRECHECK) {
        if ((imu->ok == 0U) || (imu->cal_status != IMU_CAL_READY)) {
            fail_route(EIGHT_FAULT_IMU_NOT_READY);
            return;
        }
        if ((uint32_t)(now_ms - imu->sample_ms) > EIGHT_IMU_STALE_MS) {
            fail_route(EIGHT_FAULT_IMU_STALE);
            return;
        }
        if (pattern == 0U) {
            publish_targets(0.0f, 0.0f);
            return;
        }

        previous_imu_yaw = imu->yaw;
        last_imu_sample_ms = imu->sample_ms;
        yaw_total = 0.0f;
        view.yaw_deg = 0.0f;
        yaw_initialized = 1U;
        set_phase(EIGHT_TURN_A_C, now_ms);
        return;
    }

    if ((yaw_initialized == 0U) || (update_yaw(now_ms) == 0U)) {
        return;
    }
    if (view.phase == EIGHT_FAULT) {
        return;
    }

    motion = motion_view();

    switch (view.phase) {
    case EIGHT_TURN_A_C:
        update_turn(now_ms, &left_target, &right_target, EIGHT_LEG_A_C);
        break;
    case EIGHT_LEG_A_C:
    case EIGHT_LEG_B_D: {
        float left_progress = motion->left_angle_deg - phase_start_left_deg;
        float right_progress = motion->right_angle_deg - phase_start_right_deg;
        float distance = distance_from_angles(left_progress, right_progress);
        float remaining = view.target_distance_cm - distance;
        float base = drive_speed(remaining, 0.0f);
        float correction;
        EightPhase_t next =
            (view.phase == EIGHT_LEG_A_C) ? EIGHT_TURN_C_ARC : EIGHT_TURN_D_ARC;

        view.progress_cm = distance;
        if (remaining <= EIGHT_DISTANCE_TOLERANCE_CM) {
            enter_next_phase(next, now_ms);
            break;
        }

        correction = straight_heading_correction(left_progress, right_progress);
        left_target = clampf(base - correction, 0.0f, view.speed_rpm);
        right_target = clampf(base + correction, 0.0f, view.speed_rpm);
        break;
    }
    case EIGHT_TURN_C_ARC:
        update_turn(now_ms, &left_target, &right_target, EIGHT_ARC_C_B);
        break;
    case EIGHT_ARC_C_B:
        view.ir_pattern = pattern;
        view.line_error = e_y;
        update_arc(now_ms, &left_target, &right_target, e_y, line_valid, pattern);
        break;
    case EIGHT_TURN_B_D:
        update_turn(now_ms, &left_target, &right_target, EIGHT_LEG_B_D);
        break;
    case EIGHT_TURN_D_ARC:
        update_turn(now_ms, &left_target, &right_target, EIGHT_ARC_D_A);
        break;
    case EIGHT_ARC_D_A:
        view.ir_pattern = pattern;
        view.line_error = e_y;
        update_arc(now_ms, &left_target, &right_target, e_y, line_valid, pattern);
        break;
    default:
        break;
    }

    publish_targets(left_target, right_target);
}

const EightView_t *eight_view(void)
{
    return &view;
}

const char *eight_phase_name(EightPhase_t phase)
{
    static const char *const names[] = {
        "IDLE", "PRECHECK", "TURN A-C", "LEG A-C", "TURN C",
        "ARC C-B", "TURN B-D", "LEG B-D", "TURN D", "ARC D-A",
        "FINISH", "FAULT"
    };
    return ((uint32_t)phase < (uint32_t)(sizeof(names) / sizeof(names[0])))
           ? names[phase] : "UNKNOWN";
}

const char *eight_fault_name(EightFault_t fault)
{
    static const char *const names[] = {
        "NONE", "IMU NOT READY", "IMU STALE", "IMU GAP",
        "ARC LOST", "ROUTE TIME"
    };
    return ((uint32_t)fault < (uint32_t)(sizeof(names) / sizeof(names[0])))
           ? names[fault] : "UNKNOWN";
}

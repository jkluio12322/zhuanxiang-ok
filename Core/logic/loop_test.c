/* LOOP-TEST 50 ms 状态机与确定性假对象。 */

#include "loop_test.h"
#include "config.h"
#include "motion.h"
#include "speed_calibration.h"
#include "state.h"
#include "position_loop.h"
#include "timer.h"
#include <math.h>
#include <stdio.h>

typedef struct {
    float position;
    float target;
    float velocity;
    float max_rpm;
    float accel_rpm_s;
    float decel_scale;
    float approach_accel_rpm_s;
    float brake_accel_rpm_s;
    uint8_t done;
} TrapezoidProfile_t;

typedef struct {
    float rpm_true;
    float angle_true;
} FakeAxis_t;

static LoopTestProgram_t program;
static uint8_t option;
static uint8_t running;
static uint8_t phase;
static uint8_t phase_configured;
static uint32_t start_ms;
static uint32_t phase_ms;
static uint32_t hold_ms;
static LoopTestView_t view;
static TrapezoidProfile_t profile[2];
static FakeAxis_t fake_axis[2];
static uint16_t stall_count;
static uint8_t position_begin_pending;
static uint8_t side_mask;

/* SWD-only bench control. 1=program, 2=option, 3=start, 4=stop. */
volatile uint8_t loop_test_debug_request;
volatile uint8_t loop_test_debug_argument;

static const float speed_low[3] = {30.0f, 45.0f, 55.0f};
static const float speed_mid[3] = {30.0f, 80.0f, 105.0f};
static const float speed_high[3] = {30.0f, 80.0f, 140.0f};

static float clampf(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

static const float *speed_group(void)
{
    if (option == 0u) return speed_low;
    if (option == 1u) return speed_mid;
    return speed_high;
}

static void position_level(float *step_deg, float *long_deg,
                           float *max_rpm, float *accel,
                           float *decel_scale, float *approach_scale)
{
    if (option == 0u) {
        *step_deg = POSITION_LEVEL_LOW_STEP_DEG;
        *long_deg = POSITION_LEVEL_LOW_LONG_DEG;
        *max_rpm = POSITION_LEVEL_LOW_RPM;
        *accel = POSITION_LEVEL_LOW_ACCEL;
        *decel_scale = POSITION_LEVEL_LOW_DECEL_SCALE;
        *approach_scale = POSITION_LEVEL_LOW_APPROACH_SCALE;
    } else if (option == 1u) {
        *step_deg = POSITION_LEVEL_MID_STEP_DEG;
        *long_deg = POSITION_LEVEL_MID_LONG_DEG;
        *max_rpm = POSITION_LEVEL_MID_RPM;
        *accel = POSITION_LEVEL_MID_ACCEL;
        *decel_scale = POSITION_LEVEL_MID_DECEL_SCALE;
        *approach_scale = POSITION_LEVEL_MID_APPROACH_SCALE;
    } else {
        *step_deg = POSITION_LEVEL_HIGH_STEP_DEG;
        *long_deg = POSITION_LEVEL_HIGH_LONG_DEG;
        *max_rpm = POSITION_LEVEL_HIGH_RPM;
        *accel = POSITION_LEVEL_HIGH_ACCEL;
        *decel_scale = POSITION_LEVEL_HIGH_DECEL_SCALE;
        *approach_scale = POSITION_LEVEL_HIGH_APPROACH_SCALE;
    }
}

static void profile_init(const float position[2], const float target[2],
                         float max_rpm, float accel,
                         float decel_scale, float approach_scale)
{
    for (uint8_t i = 0u; i < 2u; i++) {
        profile[i].position = position[i];
        profile[i].target = target[i];
        profile[i].velocity = 0.0f;
        profile[i].max_rpm = max_rpm;
        profile[i].accel_rpm_s = accel;
        profile[i].decel_scale = decel_scale;
        profile[i].approach_accel_rpm_s = accel * approach_scale;
        profile[i].brake_accel_rpm_s = accel * decel_scale;
        profile[i].done = 0u;
    }
}

static void profile_step(float dt)
{
    for (uint8_t i = 0u; i < 2u; i++) {
        TrapezoidProfile_t *p = &profile[i];
        float remaining = p->target - p->position;
        float magnitude = fabsf(remaining);
        float direction = (remaining >= 0.0f) ? 1.0f : -1.0f;
        float desired_speed;
        float delta_speed;

        if (magnitude <= 0.01f) {
            p->position = p->target;
            p->velocity = 0.0f;
            p->done = 1u;
            continue;
        }

        /* RPM -> deg/s adds a factor of 6; solve v^2 = 2*a*s in degree units. */
        desired_speed = sqrtf(p->accel_rpm_s * magnitude / 3.0f);
        if (fabsf(p->velocity) > desired_speed) {
            /* Leave braking margin for the velocity loop's response lag. */
            desired_speed = sqrtf(p->decel_scale
                                  * p->accel_rpm_s * magnitude / 3.0f);
        }
        if (desired_speed > p->max_rpm) desired_speed = p->max_rpm;
        desired_speed *= direction;

        delta_speed = desired_speed - p->velocity;
        if (delta_speed > (p->accel_rpm_s * dt)) {
            p->velocity += p->accel_rpm_s * dt;
        } else if (delta_speed < -(p->accel_rpm_s * dt)) {
            p->velocity -= p->accel_rpm_s * dt;
        } else {
            p->velocity = desired_speed;
        }
        p->velocity = clampf(p->velocity, -p->max_rpm, p->max_rpm);

        p->position += p->velocity * 6.0f * dt;
        if (((remaining > 0.0f) && (p->position >= p->target))
            || ((remaining < 0.0f) && (p->position <= p->target))) {
            p->position = p->target;
            p->velocity = 0.0f;
            p->done = 1u;
        }
    }
}

static void clear_command(LoopTestCommand_t *command)
{
    command->action = LOOP_ACTION_STOP;
    command->fake = 0u;
    command->open_dir = 0;
    command->open_raw = 1u;
    command->open_duty = 0.0f;
    command->speed_target[0] = 0.0f;
    command->speed_target[1] = 0.0f;
    command->target_deg[0] = profile[0].target;
    command->target_deg[1] = profile[1].target;
    command->reference_deg[0] = profile[0].position;
    command->reference_deg[1] = profile[1].position;
    command->velocity_ff_rpm[0] = 0.0f;
    command->velocity_ff_rpm[1] = 0.0f;
    command->max_rpm = POSITION_MAX_RPM;
    command->approach_accel_rpm_s =
        POSITION_LEVEL_LOW_ACCEL * POSITION_LEVEL_LOW_APPROACH_SCALE;
    command->brake_accel_rpm_s =
        POSITION_LEVEL_LOW_ACCEL * POSITION_LEVEL_LOW_DECEL_SCALE;
    command->position_begin = 0u;
}

static void enter_phase(uint8_t next, uint32_t now_ms)
{
    phase = next;
    phase_ms = now_ms;
    hold_ms = 0u;
    phase_configured = 0u;
}

static void configure_position_phase(void)
{
    float step_deg;
    float long_deg;
    float max_rpm;
    float accel;
    float decel_scale;
    float approach_scale;
    uint8_t direct = 0u;
    float target;
    float position[2];
    float goal[2];

    position_level(&step_deg, &long_deg, &max_rpm, &accel,
                   &decel_scale, &approach_scale);
    switch (phase) {
    case 0u: target = step_deg; direct = 1u; break;
    case 1u: target = long_deg; break;
    case 2u: target = 0.0f; break;
    case 3u: target = -step_deg; direct = 1u; break;
    case 4u: target = -long_deg; break;
    default: target = 0.0f; break;
    }

    position[0] = view.actual_angle_deg[0];
    position[1] = view.actual_angle_deg[1];
    goal[0] = ((side_mask & 0x01u) != 0u) ? target : 0.0f;
    goal[1] = ((side_mask & 0x02u) != 0u) ? target : 0.0f;
    profile_init(position, goal, max_rpm, accel, decel_scale, approach_scale);

    if (direct != 0u) {
        for (uint8_t i = 0u; i < 2u; i++) {
            if ((side_mask & (uint8_t)(1u << i)) != 0u) {
                profile[i].position = goal[i];
                profile[i].velocity = 0.0f;
                profile[i].done = 1u;
            }
        }
    }
    position_begin_pending = 1u;
    phase_configured = 1u;
}

static uint8_t position_phase_settled(void)
{
    const PositionLoopView_t *position = position_loop_view();

    if (position_begin_pending != 0u) return 0u;

    for (uint8_t i = 0u; i < 2u; i++) {
        if ((side_mask & (uint8_t)(1u << i)) == 0u) continue;
        if ((profile[i].done == 0u)
            || (position->axis[i].holding == 0u)
            || (fabsf(view.actual_rpm[i]) > POSITION_SETTLE_RPM_MAX)) {
            return 0u;
        }
    }
    return 1u;
}

void loop_test_init(void)
{
    program = LOOP_PROG_ENC_CAL;
    option = 0u;
    side_mask = 0x03u;
    loop_test_reset();
}

uint8_t loop_test_debug_poll(void)
{
    uint8_t request = loop_test_debug_request;
    uint8_t result = 0u;

    if (request == 0u) return 0u;
    loop_test_debug_request = 0u;

    if (request == 1u) {
        (void)loop_test_set_program((LoopTestProgram_t)loop_test_debug_argument);
        position_loop_reset();
        motion_reset_position();
    } else if (request == 2u) {
        (void)loop_test_set_option(loop_test_debug_argument);
        position_loop_reset();
    } else if (request == 3u) {
        state_set_mode(modeLoopTest);
        (void)loop_test_start(HAL_GetTick());
        position_loop_reset();
        state_set_running(1u);
        result = 1u;
    } else if (request == 4u) {
        loop_test_stop();
        state_set_running(0u);
        motion_stop();
        result = 2u;
    } else if (request == 5u) {
        (void)loop_test_set_side_mask(loop_test_debug_argument);
        position_loop_reset();
        result = 3u;
    }
    return result;
}

void loop_test_reset(void)
{
    running = 0u;
    phase = 0u;
    phase_configured = 0u;
    start_ms = 0u;
    phase_ms = 0u;
    hold_ms = 0u;
    stall_count = 0u;
    position_begin_pending = 0u;
    for (uint8_t i = 0u; i < 2u; i++) {
        profile[i].position = 0.0f;
        profile[i].velocity = 0.0f;
        profile[i].done = 1u;
        fake_axis[i].rpm_true = 0.0f;
        fake_axis[i].angle_true = 0.0f;
    }
    view.program = program;
    view.option = option;
    view.running = 0u;
    view.fake = 0u;
    view.phase = 0u;
    view.elapsed_ms = 0u;
    for (uint8_t i = 0u; i < 2u; i++) {
        view.target_angle_deg[i] = 0.0f;
        view.actual_angle_deg[i] = 0.0f;
        view.target_rpm[i] = 0.0f;
        view.actual_rpm[i] = 0.0f;
        view.pwm[i] = 0.0f;
        view.position_error_deg[i] = 0.0f;
    }
}

uint8_t loop_test_start(uint32_t now_ms)
{
    loop_test_reset();
    running = 1u;
    start_ms = now_ms;
    enter_phase(0u, now_ms);
    motion_reset_position();
    return 1u;
}

void loop_test_stop(void)
{
    running = 0u;
    view.running = 0u;
}

void loop_test_next_program(void)
{
    if (running != 0u) return;
    program = (LoopTestProgram_t)(((uint8_t)program + 1u) % (uint8_t)LOOP_PROG_COUNT);
    option = 0u;
    loop_test_reset();
}

void loop_test_next_option(void)
{
    uint8_t count;

    if (running != 0u) return;
    if (program == LOOP_PROG_OPEN_EQ) count = 3u;
    else if ((program == LOOP_PROG_SPEED_CL) || (program == LOOP_PROG_POSITION_CL)) count = 3u;
    else count = 2u;
    option = (uint8_t)((option + 1u) % count);
    loop_test_reset();
}

uint8_t loop_test_set_program(LoopTestProgram_t new_program)
{
    if ((running != 0u) || ((uint8_t)new_program >= (uint8_t)LOOP_PROG_COUNT)) return 0u;
    program = new_program;
    option = 0u;
    loop_test_reset();
    return 1u;
}

uint8_t loop_test_set_option(uint8_t new_option)
{
    uint8_t count;
    if (running != 0u) return 0u;
    if (program == LOOP_PROG_OPEN_EQ) count = 3u;
    else if ((program == LOOP_PROG_SPEED_CL) || (program == LOOP_PROG_POSITION_CL)) count = 3u;
    else count = 2u;
    if (new_option >= count) return 0u;
    option = new_option;
    loop_test_reset();
    return 1u;
}

uint8_t loop_test_set_side_mask(uint8_t mask)
{
    if ((running != 0u) || ((mask & 0x03u) == 0u)) return 0u;
    side_mask = (uint8_t)(mask & 0x03u);
    loop_test_reset();
    return 1u;
}

LoopTestCommand_t loop_test_step50ms(uint32_t now_ms)
{
    LoopTestCommand_t command;
    uint32_t elapsed;
    const float *group;

    clear_command(&command);
    if (running == 0u) return command;

    elapsed = now_ms - start_ms;
    view.elapsed_ms = elapsed;
    view.running = 1u;
    if (elapsed >= LOOP_TEST_TIMEOUT_MS) {
        running = 0u;
        view.running = 0u;
        state_report_stop(stopReasonTestTimeout);
        return command;
    }

    if (program == LOOP_PROG_ENC_CAL) {
        if (option == 0u) {
            return command;
        }
        if ((now_ms - phase_ms) >= 3000u && (phase == 0u || phase == 2u)) {
            enter_phase((uint8_t)(phase + 1u), now_ms);
        } else if ((now_ms - phase_ms) >= 1000u && (phase == 1u || phase == 3u)) {
            enter_phase((uint8_t)((phase + 1u) % 4u), now_ms);
        }
        command.action = ((phase == 1u) || (phase == 3u))
                         ? LOOP_ACTION_STOP : LOOP_ACTION_OPEN;
        command.open_raw = 1u;
        command.open_duty = 20.0f;
        command.open_dir = (phase < 2u) ? 1 : -1;
        return command;
    }

    if (program == LOOP_PROG_OPEN_EQ) {
        static const float duty[3] = {30.0f, 50.0f, 80.0f};
        if ((now_ms - phase_ms) >= LOOP_TEST_SEGMENT_MS) {
            enter_phase((uint8_t)((phase + 1u) % 6u), now_ms);
        }
        command.action = ((phase == 2u) || (phase == 5u))
                         ? LOOP_ACTION_STOP : LOOP_ACTION_OPEN;
        command.open_duty = duty[option];
        command.open_dir = (phase < 3u) ? 1 : -1;
        command.open_raw = (uint8_t)((phase == 0u) || (phase == 3u));
        return command;
    }

    if (program == LOOP_PROG_SPEED_CL) {
        group = speed_group();
        if ((now_ms - phase_ms) >= LOOP_TEST_SPEED_SEGMENT_MS) {
            enter_phase((uint8_t)((phase + 1u) % 8u), now_ms);
        }
        command.action = LOOP_ACTION_SPEED;
        if (phase == 0u) command.speed_target[0] = command.speed_target[1] = group[0];
        else if (phase == 1u) command.speed_target[0] = command.speed_target[1] = group[1];
        else if (phase == 2u) command.speed_target[0] = command.speed_target[1] = group[2];
        else if (phase == 3u || phase == 7u) command.speed_target[0] = command.speed_target[1] = 0.0f;
        else if (phase == 4u) command.speed_target[0] = command.speed_target[1] = -group[0];
        else if (phase == 5u) command.speed_target[0] = command.speed_target[1] = -group[1];
        else command.speed_target[0] = command.speed_target[1] = -group[2];
        return command;
    }

    if (program == LOOP_PROG_POSITION_CL || program == LOOP_PROG_FAKE) {
        uint8_t phases = 6u;

        if (program == LOOP_PROG_FAKE) {
            if (option == 0u) {
                /* Fake speed uses a compact +80/0/-80 profile. */
                if ((now_ms - phase_ms) >= LOOP_TEST_SPEED_SEGMENT_MS) {
                    enter_phase((uint8_t)((phase + 1u) % 4u), now_ms);
                }
                command.action = LOOP_ACTION_SPEED;
                command.fake = 1u;
                command.speed_target[0] = command.speed_target[1] =
                    (phase == 0u) ? 80.0f : ((phase == 2u) ? -80.0f : 0.0f);
                return command;
            }
        } else {
            phases = 6u;
        }

        if (phase >= phases) enter_phase(0u, now_ms);
        if (phase_configured == 0u) {
            configure_position_phase();
            phase_configured = 1u;
        }
        if ((profile[0].done == 0u) || (profile[1].done == 0u)) {
        profile_step(CONTROL_SLOW_PERIOD_MS / 1000.0f);
            hold_ms = 0u;
        } else {
            if (position_phase_settled() == 0u) {
                hold_ms = 0u;
            } else if (hold_ms == 0u) {
                hold_ms = now_ms;
            }

            if ((hold_ms != 0u)
                && ((now_ms - hold_ms) >= LOOP_TEST_DWELL_MS)) {
                enter_phase((uint8_t)((phase + 1u) % phases), now_ms);
                configure_position_phase();
            }
        }

        if ((now_ms - phase_ms) >= LOOP_TEST_POSITION_PHASE_TIMEOUT_MS) {
            running = 0u;
            view.running = 0u;
            state_report_stop(stopReasonTestTimeout);
            command.action = LOOP_ACTION_STOP;
            return command;
        }

        command.action = LOOP_ACTION_POSITION;
        command.fake = (program == LOOP_PROG_FAKE) ? 1u : 0u;
        command.target_deg[0] = profile[0].target;
        command.target_deg[1] = profile[1].target;
        command.reference_deg[0] = profile[0].position;
        command.reference_deg[1] = profile[1].position;
        command.velocity_ff_rpm[0] = profile[0].velocity;
        command.velocity_ff_rpm[1] = profile[1].velocity;
        command.max_rpm = profile[0].max_rpm;
        command.approach_accel_rpm_s = profile[0].approach_accel_rpm_s;
        command.brake_accel_rpm_s = profile[0].brake_accel_rpm_s;
        command.position_begin = position_begin_pending;
        position_begin_pending = 0u;
        return command;
    }

    return command;
}

void loop_test_record(float target_left_deg, float target_right_deg,
                      float actual_left_deg, float actual_right_deg,
                      float target_left_rpm, float target_right_rpm,
                      float actual_left_rpm, float actual_right_rpm,
                      float left_pwm, float right_pwm,
                      float left_position_error, float right_position_error)
{
    view.program = program;
    view.option = option;
    view.running = running;
    view.phase = phase;
    view.fake = (program == LOOP_PROG_FAKE) ? 1u : 0u;
    view.target_angle_deg[0] = target_left_deg;
    view.target_angle_deg[1] = target_right_deg;
    view.actual_angle_deg[0] = actual_left_deg;
    view.actual_angle_deg[1] = actual_right_deg;
    view.target_rpm[0] = target_left_rpm;
    view.target_rpm[1] = target_right_rpm;
    view.actual_rpm[0] = actual_left_rpm;
    view.actual_rpm[1] = actual_right_rpm;
    view.pwm[0] = left_pwm;
    view.pwm[1] = right_pwm;
    view.position_error_deg[0] = left_position_error;
    view.position_error_deg[1] = right_position_error;
}

const LoopTestView_t *loop_test_view(void)
{
    return &view;
}

void loop_test_fake_feedback(float *left_angle_deg, float *right_angle_deg,
                             float *left_rpm, float *right_rpm)
{
    float rpm_quantum = 1200.0f / (ENCODER_MULTIPLE * ENCODER_REDUCTION_RATIO * ENCODER_PPR);
    *left_rpm = roundf(fake_axis[0].rpm_true / rpm_quantum) * rpm_quantum;
    *right_rpm = roundf(fake_axis[1].rpm_true / rpm_quantum) * rpm_quantum;
    *left_angle_deg = roundf(fake_axis[0].angle_true / ENCODER_COUNT_ANGLE_DEG)
                      * ENCODER_COUNT_ANGLE_DEG;
    *right_angle_deg = roundf(fake_axis[1].angle_true / ENCODER_COUNT_ANGLE_DEG)
                       * ENCODER_COUNT_ANGLE_DEG;
}

void loop_test_fake_apply_output(float left_pwm, float right_pwm,
                                 float dt_seconds)
{
    float target[2];
    float alpha = dt_seconds / (SPEED_FAKE_TAU_S + dt_seconds);

    target[0] = speed_calibration_predict_rpm(left_pwm, SPEED_CAL_LEFT);
    target[1] = speed_calibration_predict_rpm(right_pwm, SPEED_CAL_RIGHT);
    for (uint8_t i = 0u; i < 2u; i++) {
        fake_axis[i].rpm_true += alpha * (target[i] - fake_axis[i].rpm_true);
        fake_axis[i].angle_true += fake_axis[i].rpm_true * 6.0f * dt_seconds;
    }
}

uint8_t loop_test_check_fault(float left_pwm, float right_pwm)
{
    const LoopTestView_t *v = &view;
    uint8_t stalled;

    if (running == 0u || v->fake != 0u) return 0u;
    if ((fabsf(v->actual_angle_deg[0]) >= POSITION_LIMIT_DEG)
        || (fabsf(v->actual_angle_deg[1]) >= POSITION_LIMIT_DEG)) {
        state_report_stop(stopReasonPositionLimit);
        return 1u;
    }

    stalled = (uint8_t)(
        ((fabsf(v->target_rpm[0]) >= MOTOR_STALL_TARGET_MIN_RPM)
         && (fabsf(v->actual_rpm[0]) <= MOTOR_STALL_ACTUAL_MAX_RPM))
        || ((fabsf(v->target_rpm[1]) >= MOTOR_STALL_TARGET_MIN_RPM)
            && (fabsf(v->actual_rpm[1]) <= MOTOR_STALL_ACTUAL_MAX_RPM)));
    stalled = (uint8_t)(stalled
                        && ((fabsf(left_pwm) >= MOTOR_STALL_OUTPUT_MIN_PERCENT)
                            || (fabsf(right_pwm) >= MOTOR_STALL_OUTPUT_MIN_PERCENT)));
    if (stalled != 0u) {
        if (++stall_count >= MOTOR_STALL_CONFIRM_TICKS) {
            state_report_stop(stopReasonEncoderStall);
            return 1u;
        }
    } else {
        stall_count = 0u;
    }
    return 0u;
}

const char *loop_test_program_name(LoopTestProgram_t value)
{
    switch (value) {
    case LOOP_PROG_ENC_CAL: return "ENC-CAL";
    case LOOP_PROG_OPEN_EQ: return "OPEN-EQ";
    case LOOP_PROG_SPEED_CL: return "SPEED-CL";
    case LOOP_PROG_POSITION_CL: return "POS-CL";
    case LOOP_PROG_FAKE: return "FAKE";
    default: return "?";
    }
}

const char *loop_test_option_name(LoopTestProgram_t value, uint8_t opt)
{
    static const char *const enc[] = {"MANUAL", "AUTO"};
    static const char *const open[] = {"30%", "50%", "80%"};
    static const char *const level[] = {"LOW", "MID", "HIGH"};
    static const char *const fake[] = {"FAKE-SPD", "FAKE-POS"};

    if (value == LOOP_PROG_ENC_CAL) return enc[opt & 1u];
    if (value == LOOP_PROG_OPEN_EQ) return open[opt % 3u];
    if ((value == LOOP_PROG_SPEED_CL) || (value == LOOP_PROG_POSITION_CL)) {
        return level[opt % 3u];
    }
    return fake[opt & 1u];
}

const char *loop_test_phase_name(const LoopTestView_t *v)
{
    static char text[20];
    if (v == 0) return "?";
    if (v->program == LOOP_PROG_OPEN_EQ) {
        static const char *const names[] = {
            "FWD-RAW", "FWD-COMP", "FWD-STOP",
            "REV-RAW", "REV-COMP", "REV-STOP"
        };
        return names[v->phase % 6u];
    }
    if (v->program == LOOP_PROG_SPEED_CL) {
        static const char *const names[] = {
            "+LOW", "+MID", "+HIGH", "STOP",
            "-LOW", "-MID", "-HIGH", "STOP"
        };
        return names[v->phase % 8u];
    }
    if (v->program == LOOP_PROG_ENC_CAL) return (v->option == 0u) ? "MANUAL" : "MOTOR";
    (void)snprintf(text, sizeof(text), "STEP%d", (int)v->phase);
    return text;
}

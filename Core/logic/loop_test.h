/* LOOP-TEST：开环标定、速度闭环、位置闭环和假对象测试。 */
#ifndef __LOOP_TEST_H
#define __LOOP_TEST_H

#include <stdint.h>

typedef enum {
    LOOP_PROG_ENC_CAL = 0,
    LOOP_PROG_OPEN_EQ,
    LOOP_PROG_SPEED_CL,
    LOOP_PROG_POSITION_CL,
    LOOP_PROG_FAKE,
    LOOP_PROG_COUNT
} LoopTestProgram_t;

typedef enum {
    LOOP_ACTION_STOP = 0,
    LOOP_ACTION_OPEN,
    LOOP_ACTION_SPEED,
    LOOP_ACTION_POSITION
} LoopTestAction_t;

typedef struct {
    LoopTestAction_t action;
    uint8_t fake;
    int8_t open_dir;
    uint8_t open_raw;
    float open_duty;
    float speed_target[2];
    float target_deg[2];
    float reference_deg[2];
    float velocity_ff_rpm[2];
    float max_rpm;
    float approach_accel_rpm_s;
    float brake_accel_rpm_s;
    uint8_t position_begin;
} LoopTestCommand_t;

typedef struct {
    LoopTestProgram_t program;
    uint8_t option;
    uint8_t running;
    uint8_t fake;
    uint8_t phase;
    uint32_t elapsed_ms;
    float target_angle_deg[2];
    float actual_angle_deg[2];
    float target_rpm[2];
    float actual_rpm[2];
    float pwm[2];
    float position_error_deg[2];
} LoopTestView_t;

void loop_test_init(void);
void loop_test_reset(void);
uint8_t loop_test_debug_poll(void);

uint8_t loop_test_start(uint32_t now_ms);
void loop_test_stop(void);

void loop_test_next_program(void);
void loop_test_next_option(void);
uint8_t loop_test_set_program(LoopTestProgram_t program);
uint8_t loop_test_set_option(uint8_t option);
uint8_t loop_test_set_side_mask(uint8_t mask);

LoopTestCommand_t loop_test_step50ms(uint32_t now_ms);
void loop_test_record(float target_left_deg, float target_right_deg,
                      float actual_left_deg, float actual_right_deg,
                      float target_left_rpm, float target_right_rpm,
                      float actual_left_rpm, float actual_right_rpm,
                      float left_pwm, float right_pwm,
                      float left_position_error, float right_position_error);

const LoopTestView_t *loop_test_view(void);

void loop_test_fake_feedback(float *left_angle_deg, float *right_angle_deg,
                             float *left_rpm, float *right_rpm);
void loop_test_fake_apply_output(float left_pwm, float right_pwm,
                                 float dt_seconds);

uint8_t loop_test_check_fault(float left_pwm, float right_pwm);

const char *loop_test_program_name(LoopTestProgram_t program);
const char *loop_test_option_name(LoopTestProgram_t program, uint8_t option);
const char *loop_test_phase_name(const LoopTestView_t *view);

#endif

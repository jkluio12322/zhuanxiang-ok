#include "trackbin.h"

#include "config.h"
#include "imu.h"
#include "motion.h"
#include "position_loop.h"
#include "sched.h"
#include "serial.h"
#include "state.h"

#include <string.h>

#define TRACKBIN_VERSION 1U
#define TRACKBIN_CRC_SIZE 2U

typedef struct {
    uint16_t sequence;
    uint32_t timestamp_ms;
    uint8_t mode;
    uint8_t flags;
    uint8_t laps;
    uint8_t ir_count;
    uint8_t patterns[SQUARE_TRACK_BIN_IR_SAMPLES];
    uint8_t ir_valid_mask;
    int16_t line_error;
    int16_t yaw;
    int16_t yaw_rate;
    int16_t target_left;
    int16_t target_right;
    int16_t actual_left;
    int16_t actual_right;
    int16_t encoder_left;
    int16_t encoder_right;
    int8_t pwm_left;
    int8_t pwm_right;
    int16_t position_error_left;
    int16_t position_error_right;
    uint8_t track_exec_20us;
    uint8_t control_exec_20us;
    uint16_t track_interval_10us;
    uint16_t control_interval_10us;
} TrackBinRecord_t;

static volatile uint8_t active;
static volatile uint8_t fault_latched;
static volatile uint8_t queue_head;
static volatile uint8_t queue_tail;
static volatile uint8_t heartbeat_seen;
static volatile uint32_t last_heartbeat_ms;
static uint16_t next_sequence;
static int32_t encoder_sum_left;
static int32_t encoder_sum_right;
static uint8_t track_exec_max_20us;
static uint8_t control_exec_max_20us;
static uint16_t track_interval_10us;
static uint16_t control_interval_10us;
static TrackBinRecord_t record_queue[SQUARE_TRACK_BIN_QUEUE_SIZE];

static int16_t scaled_i16(float value, float scale)
{
    float scaled = value * scale;

    if (!(scaled >= -32768.0f)) {
        return (scaled < -32768.0f) ? (int16_t)-32768 : 0;
    }
    if (scaled > 32767.0f) {
        return 32767;
    }
    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static uint8_t scaled_u8(uint16_t value)
{
    return (value > 255U) ? 255U : (uint8_t)value;
}

static uint16_t scaled_interval(uint16_t value)
{
    return value;
}

static void put_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

static uint16_t crc16_ccitt(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint8_t i = 0U; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void build_frame(const TrackBinRecord_t *record, uint8_t frame[SQUARE_TRACK_BIN_FRAME_SIZE])
{
    uint16_t crc;

    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = TRACKBIN_VERSION;
    frame[3] = SQUARE_TRACK_BIN_FRAME_SIZE;
    put_u16(&frame[4], record->sequence);
    put_u32(&frame[6], record->timestamp_ms);
    frame[10] = record->mode;
    frame[11] = record->flags;
    frame[12] = record->laps;
    frame[13] = record->ir_count;
    memcpy(&frame[14], record->patterns, SQUARE_TRACK_BIN_IR_SAMPLES);
    frame[19] = record->ir_valid_mask;
    put_u16(&frame[20], (uint16_t)record->line_error);
    put_u16(&frame[22], (uint16_t)record->yaw);
    put_u16(&frame[24], (uint16_t)record->yaw_rate);
    put_u16(&frame[26], (uint16_t)record->target_left);
    put_u16(&frame[28], (uint16_t)record->target_right);
    put_u16(&frame[30], (uint16_t)record->actual_left);
    put_u16(&frame[32], (uint16_t)record->actual_right);
    put_u16(&frame[34], (uint16_t)record->encoder_left);
    put_u16(&frame[36], (uint16_t)record->encoder_right);
    frame[38] = (uint8_t)record->pwm_left;
    frame[39] = (uint8_t)record->pwm_right;
    put_u16(&frame[40], (uint16_t)record->position_error_left);
    put_u16(&frame[42], (uint16_t)record->position_error_right);
    frame[44] = record->track_exec_20us;
    frame[45] = record->control_exec_20us;
    put_u16(&frame[46], scaled_interval(record->track_interval_10us));
    put_u16(&frame[48], scaled_interval(record->control_interval_10us));
    crc = crc16_ccitt(frame, SQUARE_TRACK_BIN_FRAME_SIZE - TRACKBIN_CRC_SIZE);
    put_u16(&frame[50], crc);
}

void trackbin_init(void)
{
    active = 0U;
    fault_latched = 0U;
    queue_head = 0U;
    queue_tail = 0U;
    heartbeat_seen = 0U;
    last_heartbeat_ms = 0U;
    next_sequence = 0U;
    encoder_sum_left = 0;
    encoder_sum_right = 0;
    track_exec_max_20us = 0U;
    control_exec_max_20us = 0U;
    track_interval_10us = 0U;
    control_interval_10us = 0U;
    memset(record_queue, 0, sizeof(record_queue));
}

uint8_t trackbin_start(uint32_t now_ms)
{
    uint32_t primask;

    if (state_view()->running != 0U) {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    queue_head = 0U;
    queue_tail = 0U;
    next_sequence = 0U;
    fault_latched = 0U;
    heartbeat_seen = 0U;
    last_heartbeat_ms = now_ms;
    encoder_sum_left = 0;
    encoder_sum_right = 0;
    track_exec_max_20us = 0U;
    control_exec_max_20us = 0U;
    track_interval_10us = 0U;
    control_interval_10us = 0U;
    active = 1U;
    __set_PRIMASK(primask);
    return 1U;
}

void trackbin_stop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    active = 0U;
    queue_head = 0U;
    queue_tail = 0U;
    __set_PRIMASK(primask);
}

uint8_t trackbin_is_active(void)
{
    return active;
}

uint8_t trackbin_has_fault(void)
{
    return fault_latched;
}

void trackbin_host_heartbeat(uint32_t now_ms)
{
    last_heartbeat_ms = now_ms;
    heartbeat_seen = 1U;
}

void trackbin_capture_track(uint8_t pattern, uint8_t valid,
                            float line_error, uint16_t exec_20us,
                            uint16_t interval_10us)
{
    TrackBinRecord_t *record;
    uint8_t next;

    if (active == 0U) {
        return;
    }
    record = &record_queue[queue_head];
    if (record->ir_count < SQUARE_TRACK_BIN_IR_SAMPLES) {
        uint8_t index = record->ir_count++;
        record->patterns[index] = pattern;
        if (valid != 0U) {
            record->ir_valid_mask |= (uint8_t)(1U << index);
        }
    } else {
        fault_latched = 1U;
    }
    record->line_error = scaled_i16(line_error, 10000.0f);
    if (exec_20us > track_exec_max_20us) {
        track_exec_max_20us = scaled_u8(exec_20us);
    }
    track_interval_10us = interval_10us;
    next = (uint8_t)((queue_head + 1U) % SQUARE_TRACK_BIN_QUEUE_SIZE);
    (void)next;
}

void trackbin_capture_motion(int16_t left_delta, int16_t right_delta)
{
    if (active != 0U) {
        encoder_sum_left += left_delta;
        encoder_sum_right += right_delta;
    }
}

void trackbin_capture_control(uint16_t exec_20us, uint16_t interval_10us)
{
    if (active == 0U) {
        return;
    }
    if (exec_20us > control_exec_max_20us) {
        control_exec_max_20us = scaled_u8(exec_20us);
    }
    control_interval_10us = interval_10us;
}

void trackbin_tick_5ms(uint32_t timestamp_ms)
{
    TrackBinRecord_t *record;
    const StateView *state;
    const MotionView_t *motion;
    const PositionLoopView_t *position;
    const imu_data_t *imu;
    uint8_t next;
    uint8_t flags = 0U;

    if (active == 0U) {
        return;
    }

    if ((state_view()->running != 0U)
        && ((uint32_t)(timestamp_ms - last_heartbeat_ms)
            > SQUARE_TRACK_HOST_WATCHDOG_MS)) {
        fault_latched = 1U;
        state_report_stop(stopReasonSerialTimeout);
    }

    next = (uint8_t)((queue_head + 1U) % SQUARE_TRACK_BIN_QUEUE_SIZE);
    if (next == queue_tail) {
        fault_latched = 1U;
        state_report_stop(stopReasonSerialOverflow);
        return;
    }

    record = &record_queue[queue_head];
    state = state_view();
    motion = motion_view();
    position = position_loop_view();
    imu = imu_get();

    record->sequence = next_sequence++;
    record->timestamp_ms = timestamp_ms;
    record->mode = (uint8_t)state->mode;
    record->laps = state->laps_done;
    if (state->running != 0U) flags |= 0x01U;
    if (record->ir_valid_mask != 0U) flags |= 0x02U;
    if ((imu->ok != 0U) && (imu->cal_status == IMU_CAL_READY)) flags |= 0x04U;
    if (fault_latched != 0U) flags |= 0x08U;
    if (((uint32_t)track_exec_max_20us * 20U * 10U)
        > ((uint32_t)sched_track_period_ms() * 1000U * 7U)) flags |= 0x10U;
    if (((uint32_t)control_exec_max_20us * 20U * 10U)
        > ((uint32_t)sched_control_period_ms() * 1000U * 7U)) flags |= 0x20U;
    record->flags = flags;
    record->ir_count = (record->ir_count > SQUARE_TRACK_BIN_IR_SAMPLES)
        ? SQUARE_TRACK_BIN_IR_SAMPLES : record->ir_count;
    record->line_error = scaled_i16(record->line_error / 10000.0f, 10000.0f);
    record->yaw = scaled_i16(imu->yaw, 100.0f);
    record->yaw_rate = scaled_i16(imu->gyro_dps[2], 10.0f);
    record->target_left = scaled_i16(state->target_left_rpm, 10.0f);
    record->target_right = scaled_i16(state->target_right_rpm, 10.0f);
    record->actual_left = scaled_i16(motion->left_rpm, 10.0f);
    record->actual_right = scaled_i16(motion->right_rpm, 10.0f);
    record->encoder_left = scaled_i16((float)encoder_sum_left, 1.0f);
    record->encoder_right = scaled_i16((float)encoder_sum_right, 1.0f);
    record->pwm_left = (int8_t)scaled_i16(motion->left_pwm, 1.0f);
    record->pwm_right = (int8_t)scaled_i16(motion->right_pwm, 1.0f);
    record->position_error_left =
        scaled_i16(position->axis[POSITION_LEFT].error_deg, 100.0f);
    record->position_error_right =
        scaled_i16(position->axis[POSITION_RIGHT].error_deg, 100.0f);
    record->track_exec_20us = track_exec_max_20us;
    record->control_exec_20us = control_exec_max_20us;
    record->track_interval_10us = track_interval_10us;
    record->control_interval_10us = control_interval_10us;

    next = (uint8_t)((queue_head + 1U) % SQUARE_TRACK_BIN_QUEUE_SIZE);
    queue_head = next;
    if (queue_head != queue_tail) {
        TrackBinRecord_t *fresh = &record_queue[queue_head];
        memset(fresh, 0, sizeof(*fresh));
    }
    encoder_sum_left = 0;
    encoder_sum_right = 0;
    track_exec_max_20us = 0U;
    control_exec_max_20us = 0U;
    track_interval_10us = 0U;
    control_interval_10us = 0U;
}

void trackbin_process(void)
{
    TrackBinRecord_t record;
    uint8_t frame[SQUARE_TRACK_BIN_FRAME_SIZE];

    while (active != 0U) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (queue_tail == queue_head) {
            __set_PRIMASK(primask);
            break;
        }
        record = record_queue[queue_tail];
        queue_tail = (uint8_t)((queue_tail + 1U) % SQUARE_TRACK_BIN_QUEUE_SIZE);
        __set_PRIMASK(primask);

        build_frame(&record, frame);
        if (serial_write(frame, sizeof(frame)) != (int)sizeof(frame)) {
            fault_latched = 1U;
            state_report_stop(stopReasonSerialOverflow);
            break;
        }
    }
}

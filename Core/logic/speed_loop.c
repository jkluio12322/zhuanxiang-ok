/* 5 ms 控制任务：速度环、模式分发、电机输出。全工程唯一直接驱动电机的函数。
   循迹用级联（差速在目标转速空间闭环），不用「平均环 + PWM 差速」（那是开环，轮子卡住差速就失真）。 */

#include "speed_loop.h"
#include "config.h"
#include "imu.h"
#include "motion.h"
#include "state.h"
#include "testmode.h"
#include "cmd.h"
#include "track.h"
#include "telemetry.h"
#include "speed_calibration.h"
#include "position_loop.h"
#include "loop_test.h"
#include "eight.h"

/* 左右各一个增量式速度环，参数来自配置。 */
static Pid_t pidLeft;
static Pid_t pidRight;
static LoopTestCommand_t loopTestCommand;

/* 速度环配置：增量式 PI。Kd = 0（测速噪声会被微分放大）；目标反号清运行时；目标近 0 强制输出 0。 */
static const PidConfig_t leftSpeedPidConfig = {
    .kp = SPEED_PID_LEFT_KP,
    .ki = SPEED_PID_LEFT_KI,
    .kd = SPEED_PID_LEFT_KD,
    .dt = 0.0f,
    .out_min = SPEED_PID_OUT_MIN,
    .out_max = SPEED_PID_OUT_MAX,
    .zero_target_hold = SPEED_PID_ZERO_HOLD_RPM,
    .out_offset = SPEED_PID_OUT_OFFSET_PERCENT,
    .mode = PID_MODE_INCREMENTAL,
    .reset_on_sign_change = 1U,
    .derivative_on_measurement = 1U,
};
static const PidConfig_t rightSpeedPidConfig = {
    .kp = SPEED_PID_RIGHT_KP,
    .ki = SPEED_PID_RIGHT_KI,
    .kd = SPEED_PID_RIGHT_KD,
    .dt = 0.0f,
    .out_min = SPEED_PID_OUT_MIN,
    .out_max = SPEED_PID_OUT_MAX,
    .zero_target_hold = SPEED_PID_ZERO_HOLD_RPM,
    .out_offset = SPEED_PID_OUT_OFFSET_PERCENT,
    .mode = PID_MODE_INCREMENTAL,
    .reset_on_sign_change = 1U,
    .derivative_on_measurement = 1U,
};

static float clampSpeedTarget(float target);
static float speedControlStep(Pid_t *pid, SpeedCalSide_t side,
                              float target, float actual,
                              float out_min, float out_max,
                              uint8_t allow_zero_hold);
static int16_t roundPwmForTelemetry(float pwm);
static void  startPwmSpeedTest(void);
static void  resetBeforeModeChange(void);
static void  handleCommand(commandMessage_t message);

/* 执行一次 5 ms 控制任务。TIM6 中断里。顺序：读编码器→消费命令→处理停车→按模式算 PWM→输出。
   三条路径：循迹=级联（差速在目标转速空间闭环，PID 下限收 0 禁反转）；PWM 循迹=开环基础±差速（下限夹 0）；
   手动/PWM 速度=左右目标独立（下限放开 -100，原地转需一正一反）。 */
void speed_loop_task(void)
{
    float   leftPwm = 0.0f;
    float   rightPwm = 0.0f;
    float   targetLeft = 0.0f;
    float   targetRight = 0.0f;
    float   steerRpm = 0.0f;
    float   trackError = 0.0f;
    int16_t turnPhase = (int16_t)TRK_FOLLOW;
    float   rpmDifference;
    uint8_t driveMotors = 1U;

    while (cmd_available()) {
        commandMessage_t message;
        if (cmd_pop(&message) < 0) {
            break;
        }
        handleCommand(message);
    }

    {
        uint8_t debug_action = loop_test_debug_poll();
        if ((debug_action == 1U) || (debug_action == 2U)) {
            speed_loop_ResetRuntime();
        }
    }

    rpmDifference = motion_view()->left_rpm_raw - motion_view()->right_rpm_raw;

    /* 循迹模块请求停车时在这里落地，避免它直接依赖速度环。 */
    if (state_take_stop_request() != 0U) {
        speed_loop_ResetMotion();
    }

    if (!state_view()->running) {
        speed_loop_ResetRuntime();   /* 停止时清积分，避免下次启动窜车 */
        motion_stop();
        state_set_wheel_targets(0.0f, 0.0f);
        telemetry_publish(0.0f, 0.0f, 0, 0, 0.0f, 0.0f,
                             (int16_t)TRK_FOLLOW, rpmDifference);
        return;
    }

    if (state_view()->mode == modePwmTrack) {
        /* 开环测试：只把差速 PWM 叠到基础占空比上，不走速度环。 */
        float steerPwm = track_get_steer()
                         * track_get_steer_rpm_to_pwm_gain();
        int16_t base = testmode_get_base_pwm();

        leftPwm = (int16_t)((float)base + steerPwm);
        rightPwm = (int16_t)((float)base - steerPwm);
        if (leftPwm > MOTOR_OUTPUT_MAX_PERCENT) leftPwm = MOTOR_OUTPUT_MAX_PERCENT;
        if (leftPwm < 0) leftPwm = 0;
        if (rightPwm > MOTOR_OUTPUT_MAX_PERCENT) rightPwm = MOTOR_OUTPUT_MAX_PERCENT;
        if (rightPwm < 0) rightPwm = 0;

        steerRpm = track_get_steer();
        trackError = track_output()->line_error;
        turnPhase = (int16_t)track_output()->state;

    } else if (state_view()->mode == modeTrack) {
        /* 路线层已完成位置串速度级联，本层只执行左右速度 PI。 */
        targetLeft = clampSpeedTarget(state_view()->target_left_rpm);
        targetRight = clampSpeedTarget(state_view()->target_right_rpm);

        /* 循迹时不允许轮子反转：输出下限收到 0，由 PID 自己限幅，不在外面把负数改成 0
           ——那会让内部输出停在 -100，出弯爬不回来。 */
        pid_set_limits(&pidLeft, 0.0f, SPEED_PID_OUT_MAX);
        pid_set_limits(&pidRight, 0.0f, SPEED_PID_OUT_MAX);

        leftPwm = speedControlStep(&pidLeft, SPEED_CAL_LEFT, targetLeft,
                                   motion_view()->left_rpm,
                                   0.0f, SPEED_PID_OUT_MAX, 1U);
        rightPwm = speedControlStep(&pidRight, SPEED_CAL_RIGHT, targetRight,
                                    motion_view()->right_rpm,
                                    0.0f, SPEED_PID_OUT_MAX, 1U);

        state_set_wheel_targets(targetLeft, targetRight);

        steerRpm = track_get_steer();
        trackError = track_output()->line_error;
        turnPhase = (int16_t)track_output()->state;

    } else if ((state_view()->mode == modePwmSpeed)
               && (testmode_get_pwm_speed_dir() != 0)) {
        /* PWM-SPEED 手动开环：占空比直接给左右轮，不经过速度环。专门用来验证接线
           （轮向、编码器符号、驱动板真值表）——闭环跑飞时先切到这里看轮子往哪边转。 */
        float duty = testmode_get_pwm_speed_duty();
        int8_t  dir  = testmode_get_pwm_speed_dir();

        if (testmode_get_pwm_speed_raw() != 0U) {
            leftPwm = duty * (float)dir;
            rightPwm = leftPwm;
        } else {
            leftPwm = speed_calibration_compensated_duty(
                duty, dir, SPEED_CAL_LEFT);
            rightPwm = speed_calibration_compensated_duty(
                duty, dir, SPEED_CAL_RIGHT);
        }

        /* 开环期间速度环积分保持 0：切回闭环或停车时不会带着旧积分窜车。 */
        speed_loop_ResetRuntime();
        state_set_wheel_targets(0.0f, 0.0f);

        targetLeft  = 0.0f;
        targetRight = 0.0f;
    } else if (state_view()->mode == modeLoopTest) {
        LoopTestCommand_t command = loopTestCommand;
        float actualAngleLeft = motion_view()->left_angle_deg;
        float actualAngleRight = motion_view()->right_angle_deg;
        float actualRpmLeft = motion_view()->left_rpm;
        float actualRpmRight = motion_view()->right_rpm;
        float targetAngleLeft = 0.0f;
        float targetAngleRight = 0.0f;
        float positionErrorLeft;
        float positionErrorRight;

        if (command.fake != 0U) {
            loop_test_fake_feedback(&actualAngleLeft, &actualAngleRight,
                                    &actualRpmLeft, &actualRpmRight);
            driveMotors = 0U;
        }

        if (command.action == LOOP_ACTION_OPEN) {
            if (command.open_raw != 0U) {
                leftPwm = (float)command.open_dir * command.open_duty;
                rightPwm = leftPwm;
            } else {
                leftPwm = speed_calibration_compensated_duty(
                    command.open_duty, command.open_dir, SPEED_CAL_LEFT);
                rightPwm = speed_calibration_compensated_duty(
                    command.open_duty, command.open_dir, SPEED_CAL_RIGHT);
            }
            speed_loop_ResetRuntime();
        } else if (command.action == LOOP_ACTION_SPEED) {
            targetLeft = clampSpeedTarget(command.speed_target[0]);
            targetRight = clampSpeedTarget(command.speed_target[1]);
            leftPwm = speedControlStep(&pidLeft, SPEED_CAL_LEFT, targetLeft,
                                       actualRpmLeft, SPEED_PID_OUT_MIN,
                                       SPEED_PID_OUT_MAX, 1U);
            rightPwm = speedControlStep(&pidRight, SPEED_CAL_RIGHT, targetRight,
                                        actualRpmRight, SPEED_PID_OUT_MIN,
                                        SPEED_PID_OUT_MAX, 1U);
        } else if (command.action == LOOP_ACTION_POSITION) {
            if (command.position_begin != 0U) {
                position_loop_begin(command.target_deg[0], command.target_deg[1]);
            }
            position_loop_step(command.reference_deg[0], command.reference_deg[1],
                               command.velocity_ff_rpm[0], command.velocity_ff_rpm[1],
                               actualAngleLeft, actualAngleRight,
                               actualRpmLeft, actualRpmRight,
                               command.max_rpm, command.approach_accel_rpm_s,
                               command.brake_accel_rpm_s);
            targetAngleLeft = command.target_deg[0];
            targetAngleRight = command.target_deg[1];
            targetLeft = clampSpeedTarget(
                position_loop_view()->axis[POSITION_LEFT].rpm_cmd);
            targetRight = clampSpeedTarget(
                position_loop_view()->axis[POSITION_RIGHT].rpm_cmd);
            leftPwm = speedControlStep(&pidLeft, SPEED_CAL_LEFT, targetLeft,
                                       actualRpmLeft, SPEED_PID_OUT_MIN,
                                       SPEED_PID_OUT_MAX, 0U);
            rightPwm = speedControlStep(&pidRight, SPEED_CAL_RIGHT, targetRight,
                                        actualRpmRight, SPEED_PID_OUT_MIN,
                                        SPEED_PID_OUT_MAX, 0U);
            positionErrorLeft =
                position_loop_view()->axis[POSITION_LEFT].error_deg;
            positionErrorRight =
                position_loop_view()->axis[POSITION_RIGHT].error_deg;
        } else {
            targetLeft = 0.0f;
            targetRight = 0.0f;
            leftPwm = 0.0f;
            rightPwm = 0.0f;
            speed_loop_ResetRuntime();
            positionErrorLeft = targetAngleLeft - actualAngleLeft;
            positionErrorRight = targetAngleRight - actualAngleRight;
        }
        if (command.action != LOOP_ACTION_POSITION) {
            positionErrorLeft = targetAngleLeft - actualAngleLeft;
            positionErrorRight = targetAngleRight - actualAngleRight;
        }

        loop_test_record(targetAngleLeft, targetAngleRight,
                         actualAngleLeft, actualAngleRight,
                         targetLeft, targetRight,
                         actualRpmLeft, actualRpmRight,
                         leftPwm, rightPwm,
                         positionErrorLeft, positionErrorRight);
        if (command.fake != 0U) {
            loop_test_fake_apply_output(leftPwm, rightPwm,
                                        (float)CONTROL_PERIOD_MS / 1000.0f);
        }
    } else if (state_view()->mode == modeEight) {
        const EightView_t *route = eight_view();
        targetLeft = clampSpeedTarget(state_view()->target_left_rpm);
        targetRight = clampSpeedTarget(state_view()->target_right_rpm);
        leftPwm = speedControlStep(&pidLeft, SPEED_CAL_LEFT, targetLeft,
                                   motion_view()->left_rpm,
                                   SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, 1U);
        rightPwm = speedControlStep(&pidRight, SPEED_CAL_RIGHT, targetRight,
                                    motion_view()->right_rpm,
                                    SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, 1U);
        steerRpm = 0.5f * (targetLeft - targetRight);
        trackError = route->line_error;
        turnPhase = (int16_t)route->phase;
    } else {
        /* 手动模式：左右各自目标，允许反转（原地转需一正一反）。PWM 速度测试也走这条分支。 */
        targetLeft = clampSpeedTarget(state_view()->target_left_rpm);
        targetRight = clampSpeedTarget(state_view()->target_right_rpm);

        leftPwm = speedControlStep(&pidLeft, SPEED_CAL_LEFT, targetLeft,
                                   motion_view()->left_rpm,
                                   SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, 1U);
        rightPwm = speedControlStep(&pidRight, SPEED_CAL_RIGHT, targetRight,
                                    motion_view()->right_rpm,
                                    SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, 1U);
    }

    state_set_wheel_targets(targetLeft, targetRight);
    if (driveMotors != 0U) {
        motion_apply(leftPwm, rightPwm);
    } else {
        motion_stop();
    }
    telemetry_publish(targetLeft, targetRight,
                         roundPwmForTelemetry(leftPwm),
                         roundPwmForTelemetry(rightPwm),
                         steerRpm, trackError, turnPhase, rpmDifference);
}

void speed_loop_slow_task(void)
{
    if ((state_view()->mode == modePwmSpeed)
        && state_view()->running
        && (testmode_get_pwm_speed_dir() == 0)) {
        if (testmode_tick_pwm_speed() != 0U) {
            float target = state_view()->gear_rpm
                           * (float)testmode_get_pwm_speed_sign();
            speed_loop_ResetRuntime();
            state_set_wheel_targets(target, target);
        }
        return;
    }

    if (state_view()->mode != modeLoopTest) {
        loopTestCommand.action = LOOP_ACTION_STOP;
        return;
    }

    loopTestCommand = loop_test_step50ms(HAL_GetTick());
    if (loop_test_check_fault(loop_test_view()->pwm[0],
                              loop_test_view()->pwm[1]) != 0U) {
        speed_loop_ResetMotion();
    }
}

/* 把目标转速夹到 ±SPEED_TARGET_LIMIT_RPM。夹的是目标不是 PID 输出（目标可能来自串口）。 */
static float clampSpeedTarget(float target)
{
    if (target > SPEED_TARGET_LIMIT_RPM) return SPEED_TARGET_LIMIT_RPM;
    if (target < -SPEED_TARGET_LIMIT_RPM) return -SPEED_TARGET_LIMIT_RPM;
    return target;
}

static float speedControlStep(Pid_t *pid, SpeedCalSide_t side,
                              float target, float actual,
                              float out_min, float out_max,
                              uint8_t allow_zero_hold)
{
    float feedforward = speed_calibration_feedforward(target, side);
    float saved_zero_hold;
    float correction;
    float output;

    if (feedforward < out_min) feedforward = out_min;
    if (feedforward > out_max) feedforward = out_max;

    pid_set_limits(pid, out_min - feedforward, out_max - feedforward);
    saved_zero_hold = pid->cfg.zero_target_hold;
    if (allow_zero_hold == 0U) {
        pid->cfg.zero_target_hold = 0.0f;
    }
    correction = pid_update_scaled(
        pid, target, actual,
        (float)CONTROL_PERIOD_MS / SPEED_PID_REFERENCE_PERIOD_MS);
    pid->cfg.zero_target_hold = saved_zero_hold;
    output = feedforward + correction;

    if (output < out_min) output = out_min;
    if (output > out_max) output = out_max;
    return output;
}

/* Keep the legacy integer telemetry/display field without truncating the actual motor output. */
static int16_t roundPwmForTelemetry(float pwm)
{
    return (int16_t)((pwm >= 0.0f) ? (pwm + 0.5f) : (pwm - 0.5f));
}

/* 启动 PWM 速度测试：按当前档位设左右同向目标。 */
static void startPwmSpeedTest(void)
{
    float target = state_view()->gear_rpm;

    testmode_reset_pwm_speed();
    speed_loop_ResetRuntime();
    state_set_wheel_targets(target, target);
}

/* 模式切换前：运行中先报一次停车原因（让遥测打印一行 [STOP]，能看出是自停不是出错），再整体复位。 */
static void resetBeforeModeChange(void)
{
    if (state_view()->running) {
        state_report_stop(stopReasonModeSwitch);
    }
    speed_loop_ResetModeState();
}

/* 执行一条命令。在中断里调，不能有阻塞/长格式化，动作都是「改状态 + 复位」。 */
static void handleCommand(commandMessage_t message)
{
    switch (message.type) {
    case commandStart:
        state_clear_stop_event();
        if (!state_view()->running) {
            if (state_view()->mode == modeTrack) {
                motion_reset_position();
                track_start();
                speed_loop_ResetRuntime();
            } else if (state_view()->mode == modePwmSpeed) {
                /* 手动开环：启动 = 正转，占空比取当前设定值。 */
                testmode_set_pwm_speed_dir(1);
                speed_loop_ResetRuntime();
                state_set_wheel_targets(0.0f, 0.0f);
            } else if (state_view()->mode == modePwmTrack) {
                track_reset_state();
                testmode_reset_pwm_track();
            } else if (state_view()->mode == modeLoopTest) {
                loopTestCommand.action = LOOP_ACTION_STOP;
                (void)loop_test_start(HAL_GetTick());
                position_loop_reset();
                speed_loop_ResetRuntime();
            } else {
                if (eight_start(HAL_GetTick()) == 0U) {
                    speed_loop_ResetRuntime();
                    break;
                }
                speed_loop_ResetRuntime();
            }
        }
        state_set_running(1U);
        break;

    case commandStop:
        if (state_view()->running) {
            state_report_stop(stopReasonUser);
        }
        speed_loop_ResetModeState();
        break;

    case commandLapCycle:
        if ((state_view()->mode == modeTrack) && !state_view()->running) {
            state_cycle_lap_target();
        }
        break;

    case commandModeTrack:
        resetBeforeModeChange();
        state_set_mode(modeTrack);
        motion_reset_position();
        track_start();
        break;

    case commandModePwmSpeed:
        resetBeforeModeChange();
        state_set_mode(modePwmSpeed);
        /* 切进手动开环：先停住，等 KEY3/KEY4 起转。 */
        testmode_reset_pwm_speed();
        speed_loop_ResetRuntime();
        state_set_wheel_targets(0.0f, 0.0f);
        break;

    case commandModePwmTrack:
        resetBeforeModeChange();
        state_set_mode(modePwmTrack);
        track_reset_state();
        testmode_reset_pwm_track();
        break;

    case commandModeLoopTest:
        resetBeforeModeChange();
        state_set_mode(modeLoopTest);
        loopTestCommand.action = LOOP_ACTION_STOP;
        loop_test_reset();
        position_loop_reset();
        motion_reset_position();
        break;

    case commandModeEight:
        resetBeforeModeChange();
        state_set_mode(modeEight);
        eight_reset();
        track_follow_reset();
        motion_reset_position();
        break;

    case commandEightNextSpeed:
        eight_next_speed();
        break;

    case commandLoopTestNextProgram:
        loop_test_next_program();
        position_loop_reset();
        motion_reset_position();
        break;

    case commandLoopTestNextOption:
        loop_test_next_option();
        position_loop_reset();
        break;

    case commandLoopTestSetProgram:
        (void)loop_test_set_program((LoopTestProgram_t)message.value);
        position_loop_reset();
        motion_reset_position();
        break;

    case commandLoopTestSetOption:
        (void)loop_test_set_option((uint8_t)message.value);
        position_loop_reset();
        break;

    /* 台架手动转速：只改指定的那一路，另一路保持当前值。l/t 改左轮，r/t 改右轮。
       模式检查在命令行侧已完成（那里才能回提示），这里直接应用。 */
    case commandSetLeftRpm:
    case commandSetRightRpm:
    case commandSetBothRpm: {
        float leftTarget = state_view()->target_left_rpm;
        float rightTarget = state_view()->target_right_rpm;

        /* 关掉 5 秒自动换向，把控制权交给命令。stop 与模式切换会走 ResetPwmSpeed 自动恢复。 */
        testmode_set_auto_phase(0U);
        testmode_set_pwm_speed_dir(0);

        if (message.type != commandSetRightRpm) {
            leftTarget = message.value;
        }
        if (message.type != commandSetLeftRpm) {
            rightTarget = message.value;
        }

        state_set_wheel_targets(leftTarget, rightTarget);
        speed_loop_ResetRuntime();
        state_set_running(1U);
        break;
    }

    /* PWM-SPEED 手动开环起转：置方向 + 置运行标志，输出在下一次 5 ms 任务里生效。 */
    case commandPwmSpeedForward:
    case commandPwmSpeedReverse:
        if (state_view()->mode == modePwmSpeed) {
            testmode_set_pwm_speed_dir((message.type == commandPwmSpeedForward) ? 1 : -1);
            speed_loop_ResetRuntime();
            state_set_wheel_targets(0.0f, 0.0f);
            state_set_running(1U);
        }
        break;

    /* 改开环占空比：不停机、不重新起转，下一拍直接生效。 */
    case commandPwmSpeedDutyStep:
        if (state_view()->mode == modePwmSpeed) {
            testmode_step_pwm_speed_duty(message.value);
        }
        break;

    /* 循迹模式调速：短按升档、长按降档。运行中改档即时生效（循迹 base_rpm 每拍取自档位）。 */
    case commandGearStep:
        if (state_view()->mode == modeTrack) {
            state_step_gear((int8_t)message.value);
        }
        break;
    case commandPwmSpeedNextGear:
        state_step_gear(1);
        if (state_view()->mode == modePwmSpeed) {
            startPwmSpeedTest();
        }
        break;

    case commandPwmTrackNext:
        if (state_view()->mode == modePwmTrack) {
            testmode_next_pattern();
        }
        break;

    case commandPwmTrackBaseStep:
        if (state_view()->mode == modePwmTrack) {
            testmode_step_base_pwm((int16_t)message.value);
        }
        break;

    case commandQuiet:
        state_set_vofa(0U);
        break;

    case commandLog:
        state_set_vofa(1U);
        break;

    default:
        break;
    }
}

/* 初始化左右两个速度环。须在中断放行前调。 */
void speed_loop_Init(void)
{
    pid_init(&pidLeft, &leftSpeedPidConfig);
    pid_init(&pidRight, &rightSpeedPidConfig);
    position_loop_init();
    loop_test_init();
    eight_init();
}

/* 清空速度环运行时状态，保留增益。停车时清积分是必须的，否则下次启动积分带旧值，表现为起步窜车。 */
void speed_loop_ResetRuntime(void)
{
    pid_reset(&pidLeft);
    pid_reset(&pidRight);
}

/* 停车落地：清运行标志、清目标、复位速度环、关电机。循迹模块只上报停车，由 5 ms 任务取走标志后执行。 */
void speed_loop_ResetMotion(void)
{
    state_set_running(0U);
    state_set_wheel_targets(0.0f, 0.0f);
    speed_loop_ResetRuntime();
    motion_stop();
}

/* 复位整个模式：停车 + 复位循迹 + 复位两个测试模式。手动转速模式恢复自动换向也走这里。 */
void speed_loop_ResetModeState(void)
{
    speed_loop_ResetMotion();
    track_reset_state();
    testmode_reset_pwm_speed();
    testmode_reset_pwm_track();
    loop_test_reset();
    position_loop_reset();
    eight_reset();
}

/* 读速度环增益与运行状态，供 show 打印。只读。整体拷贝放临界区：target/actual/out 由 5 ms 中断
   在改，逐字段读会拷出「新 target + 旧 out」混排值。 */
void speed_loop_GetInfo(SpeedChannel_t channel, SpeedPidInfo_t *out)
{
    const Pid_t *pid = (channel == SPEED_CH_LEFT) ? &pidLeft : &pidRight;
    uint32_t primask;

    if (out == 0) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    out->kp = pid->cfg.kp;
    out->ki = pid->cfg.ki;
    out->kd = pid->cfg.kd;
    out->target = pid->target;
    out->actual = pid->actual;
    out->out = pid->out;

    __set_PRIMASK(primask);
}

uint8_t speed_loop_SetGain(SpeedChannel_t channel, PidField_t field, float value)
{
    Pid_t *pid = (channel == SPEED_CH_LEFT) ? &pidLeft : &pidRight;
    uint32_t primask;
    uint8_t ok;

    primask = __get_PRIMASK();
    __disable_irq();
    ok = pid_set_gain(pid, field, value);
    if (ok != 0U) {
        pid_reset(pid);
    }
    __set_PRIMASK(primask);
    return ok;
}

/* 50 ms 控制任务：速度环、模式分发、电机输出。全工程唯一直接驱动电机的函数，在 TIM6 中断里 50 ms 一次。
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

/* 左右各一个增量式速度环，参数来自配置。 */
static Pid_t pidLeft;
static Pid_t pidRight;

/* 速度环配置：增量式 PI。Kd = 0（测速噪声会被微分放大）；目标反号清运行时；目标近 0 强制输出 0。 */
static const PidConfig_t speedPidConfig = {
    .kp = SPEED_PID_KP,
    .ki = SPEED_PID_KI,
    .kd = SPEED_PID_KD,
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
static void  startPwmSpeedTest(void);
static void  resetBeforeModeChange(void);
static void  handleCommand(commandMessage_t message);

/* 执行一次 50 ms 控制任务。TIM6 中断里。顺序：读编码器滤波→消费命令→处理停车→按模式算 PWM→输出。
   三条路径：循迹=级联（差速在目标转速空间闭环，PID 下限收 0 禁反转）；PWM 循迹=开环基础±差速（下限夹 0）；
   手动/PWM 速度=左右目标独立（下限放开 -100，原地转需一正一反）。 */
void speed_loop_task(void)
{
    int16_t leftPwm = 0;
    int16_t rightPwm = 0;
    float   targetLeft = 0.0f;
    float   targetRight = 0.0f;
    float   steerRpm = 0.0f;
    float   trackError = 0.0f;
    int16_t turnPhase = (int16_t)TRK_FOLLOW;
    float   rpmDifference;

    motion_update();

    while (cmd_available()) {
        commandMessage_t message;
        if (cmd_pop(&message) < 0) {
            break;
        }
        handleCommand(message);
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
        trackError = track_output()->e_y;
        turnPhase = (int16_t)track_output()->state;

    } else if (state_view()->mode == modeTrack) {
        /* 级联：循迹环出目标转速，左右速度环各自闭环到 PWM。差速是闭环的，轮子跟不上时内环自己补。 */
        float base = track_output()->base_rpm;
        float steer = track_get_steer();

        targetLeft = clampSpeedTarget(base + steer);
        targetRight = clampSpeedTarget(base - steer);

        /* 循迹时不允许轮子反转：输出下限收到 0，由 PID 自己限幅，不在外面把负数改成 0
           ——那会让内部输出停在 -100，出弯爬不回来。 */
        pid_set_limits(&pidLeft, 0.0f, SPEED_PID_OUT_MAX);
        pid_set_limits(&pidRight, 0.0f, SPEED_PID_OUT_MAX);

        leftPwm = (int16_t)pid_update(&pidLeft, targetLeft, motion_view()->left_rpm);
        rightPwm = (int16_t)pid_update(&pidRight, targetRight, motion_view()->right_rpm);

        state_set_wheel_targets(targetLeft, targetRight);

        steerRpm = steer;
        trackError = track_output()->e_y;
        turnPhase = (int16_t)track_output()->state;

    } else {
        /* 手动模式：左右各自目标，允许反转（原地转需一正一反）。PWM 速度测试也走这条分支。 */
        if (state_view()->mode == modePwmSpeed) {
            /* 到点换向：必须复位速度环并重设目标，否则带着上个方向的积分继续跑会换向迟滞。
               手动模式下自动相位已关，TickPwmSpeed 直接返回 0。 */
            if (testmode_tick_pwm_speed() != 0U) {
                float target = state_view()->gear_rpm
                               * (float)testmode_get_pwm_speed_sign();
                speed_loop_ResetRuntime();
                state_set_wheel_targets(target, target);
            }
        }

        targetLeft = clampSpeedTarget(state_view()->target_left_rpm);
        targetRight = clampSpeedTarget(state_view()->target_right_rpm);

        pid_set_limits(&pidLeft, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);
        pid_set_limits(&pidRight, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);

        leftPwm = (int16_t)pid_update(&pidLeft, targetLeft, motion_view()->left_rpm);
        rightPwm = (int16_t)pid_update(&pidRight, targetRight, motion_view()->right_rpm);
    }

    motion_apply(leftPwm, rightPwm);
    telemetry_publish(targetLeft, targetRight, leftPwm, rightPwm,
                         steerRpm, trackError, turnPhase, rpmDifference);
}

/* 把目标转速夹到 ±SPEED_TARGET_LIMIT_RPM。夹的是目标不是 PID 输出（目标可能来自串口）。 */
static float clampSpeedTarget(float target)
{
    if (target > SPEED_TARGET_LIMIT_RPM) return SPEED_TARGET_LIMIT_RPM;
    if (target < -SPEED_TARGET_LIMIT_RPM) return -SPEED_TARGET_LIMIT_RPM;
    return target;
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
                track_start();
            } else if (state_view()->mode == modePwmSpeed) {
                startPwmSpeedTest();
            } else {
                track_reset_state();
                testmode_reset_pwm_track();
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
        track_start();
        break;

    case commandModePwmSpeed:
        resetBeforeModeChange();
        state_set_mode(modePwmSpeed);
        startPwmSpeedTest();
        break;

    case commandModePwmTrack:
        resetBeforeModeChange();
        state_set_mode(modePwmTrack);
        track_reset_state();
        testmode_reset_pwm_track();
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
    pid_init(&pidLeft, &speedPidConfig);
    pid_init(&pidRight, &speedPidConfig);
}

/* 清空速度环运行时状态，保留增益。停车时清积分是必须的，否则下次启动积分带旧值，表现为起步窜车。 */
void speed_loop_ResetRuntime(void)
{
    pid_reset(&pidLeft);
    pid_reset(&pidRight);
}

/* 停车落地：清运行标志、清目标、复位速度环、关电机。循迹模块只上报停车，由 50 ms 任务取走标志后执行。 */
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
}

/* 读速度环增益与运行状态，供 show 打印。只读。整体拷贝放临界区：target/actual/out 由 50 ms 中断
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

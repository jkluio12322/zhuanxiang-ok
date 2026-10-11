/**
 * @file    app_speedloop.c
 * @brief   50 ms 控制任务 — 速度环、模式分发、电机输出。
 *
 * @note    文件内排序：核心任务放最上方，辅助函数与对外接口放下面。
 *
 * @note    **这是全工程唯一直接驱动电机的函数。** 其它模块只提供目标或差速，
 *          由这里统一算成 PWM 并交给 app_motion.c 输出。
 *
 * @note    三个模式走三条不同的输出路径（见 AppSpeedLoop_Task50ms）：
 * @code
 *   modeTrack     级联：循迹环出目标转速 -> 左右速度环各自闭环 -> PWM
 *   modePwmSpeed  手动：目标转速由 AppState 给出 -> 左右速度环 -> PWM（允许反转）
 *   modePwmTrack  开环：基础占空比 ± 差速PWM，**旁路速度环**
 * @endcode
 *
 * @note    为什么是级联而不是「平均环 + PWM 差速」：
 *          差速放在目标转速空间是**闭环**的——某个轮子跟不上目标时，
 *          它自己的速度环会把 PWM 顶上去补；而直接加到 PWM 上是开环，
 *          轮子被卡住差速就失真。详见 README §4.3.3。
 *
 * @note    执行上下文：AppSpeedLoop_Task50ms() 在 TIM6 中断里，50 ms 一次。
 *          本函数里的浮点与 PID 运算都是为「固定周期」设计的，
 *          挪到主循环会因抖动而算错（测速分母是常数 50 ms）。
 */

#include "app_speedloop.h"
#include "app_config.h"
#include "app_imu.h"
#include "app_motion.h"
#include "app_state.h"
#include "app_testmode.h"
#include "app_cmd.h"
#include "app_tracking.h"
#include "app_telemetry.h"

/* 左右各一个增量式速度环，参数来自配置。 */
static Pid_t pidLeft;
static Pid_t pidRight;

/* 速度环配置：增量式 PI。
   - Kd = 0：测速有噪声，微分会把它放大成抖动；
   - derivative_on_measurement：即便将来加 Kd，也让它作用于测量值，
     避免改目标时产生微分冲击；
   - reset_on_sign_change：目标反号时清运行时，防止原地转的内侧轮
     保留着转弯前的正输出；
   - zero_target_hold：目标接近 0 时强制输出 0，避免停车后残留输出顶电机。 */
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

/* ── 辅助函数前置声明（定义在文件下方） ───────────────────────────── */
static float clampSpeedTarget(float target);
static void  startPwmSpeedTest(void);
static void  resetBeforeModeChange(void);
static void  handleCommand(commandMessage_t message);

/**
 * @brief  执行一次 50 ms 控制任务 —— 本文件的核心。
 *
 * @note   在 TIM6 中断里执行。整体顺序：
 * @code
 *   1. 读编码器并做中值滤波
 *   2. 消费命令队列（含启停、模式切换、手动转速）
 *   3. 处理停车请求
 *   4. 未运行 -> 清积分、关电机、发一帧停止遥测，返回
 *   5. 按模式算左右 PWM
 *   6. 输出到电机并填充遥测快照
 * @endcode
 *
 * @note   第 1 步放在最前面是必须的：后面的分支都会用到速度反馈，
 *         保证「一个周期内的速度采样只取一次」。
 *
 * @note   三条输出路径的差别：
 *         - **PWM 循迹**：不走速度环，直接「基础占空比 ± 差速 PWM」，
 *           下限夹 0（测试用，不允许反转基础速度）；
 *         - **循迹**：级联。差速在**目标转速**空间生效，左右速度环各自闭环，
 *           并把 PID 输出下限收到 0 禁止反转；
 *         - **手动/PWM 速度**：左右目标独立，输出下限放开到 -100，
 *           原地转需要一正一反。
 *
 * @note   为什么禁止反转要用 PID 的 out_min 而不是事后夹一下：
 *         增量式 PID 的「积分」就是输出本身。在外面把负数改成 0，
 *         内部输出仍会停在 -100，出弯时要一路爬回来，表现为内轮反应迟钝。
 *         把 out_min 设成 0 则输出天然停在 0，出弯立刻能往上走。
 */
void AppSpeedLoop_Task50ms(void)
{
    int16_t leftPwm = 0;
    int16_t rightPwm = 0;
    float   targetLeft = 0.0f;
    float   targetRight = 0.0f;
    float   steerRpm = 0.0f;
    float   trackError = 0.0f;
    int16_t turnPhase = (int16_t)TRK_FOLLOW;
    float   rpmDifference;

    AppMotion_Update();

    while (AppCmd_Available()) {
        commandMessage_t message;
        if (AppCmd_Pop(&message) < 0) {
            break;
        }
        handleCommand(message);
    }

    rpmDifference = AppMotion()->left_rpm_raw - AppMotion()->right_rpm_raw;

    /* 循迹模块请求停车时在这里落地，避免它直接依赖速度环。 */
    if (AppState_TakeStopRequest() != 0U) {
        AppSpeedLoop_ResetMotion();
    }

    if (!AppState()->running) {
        AppSpeedLoop_ResetRuntime();   /* 停止时清积分，避免下次启动窜车 */
        AppMotion_Stop();
        AppState_SetWheelTargets(0.0f, 0.0f);
        AppTelemetry_Publish(0.0f, 0.0f, 0, 0, 0.0f, 0.0f,
                             (int16_t)TRK_FOLLOW, rpmDifference);
        return;
    }

    if (AppState()->mode == modePwmTrack) {
        /* 开环测试：只把差速 PWM 叠到基础占空比上，不走速度环。 */
        float steerPwm = AppTracking_GetSteer()
                         * AppTracking_GetSteerRpmToPwmGain();
        int16_t base = AppTestMode_GetBasePwm();

        leftPwm = (int16_t)((float)base + steerPwm);
        rightPwm = (int16_t)((float)base - steerPwm);
        if (leftPwm > MOTOR_OUTPUT_MAX_PERCENT) leftPwm = MOTOR_OUTPUT_MAX_PERCENT;
        if (leftPwm < 0) leftPwm = 0;
        if (rightPwm > MOTOR_OUTPUT_MAX_PERCENT) rightPwm = MOTOR_OUTPUT_MAX_PERCENT;
        if (rightPwm < 0) rightPwm = 0;

        steerRpm = AppTracking_GetSteer();
        trackError = AppTracking_Output()->e_y;
        turnPhase = (int16_t)AppTracking_Output()->state;

    } else if (AppState()->mode == modeTrack) {
        /* 级联：循迹环出目标转速，左右速度环各自闭环到 PWM。
           差速是闭环的，某个轮子跟不上目标时内环会自己补。 */
        float base = AppTracking_Output()->base_rpm;
        float steer = AppTracking_GetSteer();

        targetLeft = clampSpeedTarget(base + steer);
        targetRight = clampSpeedTarget(base - steer);

        /* 循迹时不允许轮子反转：把输出下限收到 0，由 PID 自己限幅，
           不在外面把负数改成 0——那会让内部输出停在 -100，出弯爬不回来。 */
        Pid_SetLimits(&pidLeft, 0.0f, SPEED_PID_OUT_MAX);
        Pid_SetLimits(&pidRight, 0.0f, SPEED_PID_OUT_MAX);

        leftPwm = (int16_t)Pid_Update(&pidLeft, targetLeft, AppMotion()->left_rpm);
        rightPwm = (int16_t)Pid_Update(&pidRight, targetRight, AppMotion()->right_rpm);

        AppState_SetWheelTargets(targetLeft, targetRight);

        steerRpm = steer;
        trackError = AppTracking_Output()->e_y;
        turnPhase = (int16_t)AppTracking_Output()->state;

    } else {
        /* 手动模式：左右各自目标，允许反转（原地转需要一正一反）。
           PWM 速度测试也走这条分支，只是目标由测试相位或串口命令决定。 */
        if (AppState()->mode == modePwmSpeed) {
            /* 到点换向：必须复位速度环并重设目标，否则会带着上个方向的
               积分继续跑，换向瞬间出现明显迟滞。
               手动模式下自动相位已被关掉，TickPwmSpeed 会直接返回 0。 */
            if (AppTestMode_TickPwmSpeed() != 0U) {
                float target = AppState()->gear_rpm
                               * (float)AppTestMode_GetPwmSpeedSign();
                AppSpeedLoop_ResetRuntime();
                AppState_SetWheelTargets(target, target);
            }
        }

        targetLeft = clampSpeedTarget(AppState()->target_left_rpm);
        targetRight = clampSpeedTarget(AppState()->target_right_rpm);

        Pid_SetLimits(&pidLeft, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);
        Pid_SetLimits(&pidRight, SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);

        leftPwm = (int16_t)Pid_Update(&pidLeft, targetLeft, AppMotion()->left_rpm);
        rightPwm = (int16_t)Pid_Update(&pidRight, targetRight, AppMotion()->right_rpm);
    }

    AppMotion_Apply(leftPwm, rightPwm);
    AppTelemetry_Publish(targetLeft, targetRight, leftPwm, rightPwm,
                         steerRpm, trackError, turnPhase, rpmDifference);
}

/* ══════════════ 辅助函数 ══════════════ */

/**
 * @brief  把目标转速夹到绝对上限内。
 * @param  target 目标转速，单位 RPM。
 * @return 夹取后的值，范围 ±SPEED_TARGET_LIMIT_RPM。
 * @note   这里夹的是**目标**，不是 PID 输出。目标可能来自串口命令，
 *         必须在这里兜住，不能让电机被要求跑到超限转速。
 */
static float clampSpeedTarget(float target)
{
    if (target > SPEED_TARGET_LIMIT_RPM) return SPEED_TARGET_LIMIT_RPM;
    if (target < -SPEED_TARGET_LIMIT_RPM) return -SPEED_TARGET_LIMIT_RPM;
    return target;
}

/**
 * @brief  启动 PWM 速度测试：按当前档位设好左右同向目标。
 * @note   相位回到正转、自动换向恢复、速度环复位。
 */
static void startPwmSpeedTest(void)
{
    float target = AppState()->gear_rpm;

    AppTestMode_ResetPwmSpeed();
    AppSpeedLoop_ResetRuntime();
    AppState_SetWheelTargets(target, target);
}

/**
 * @brief  模式切换前的统一处理：运行中先报一次停车原因，再整体复位。
 * @note   报 stopReasonModeSwitch 是为了让遥测打印一行 [STOP]，
 *         现场能看出「车是自己停的，不是出错停的」。
 */
static void resetBeforeModeChange(void)
{
    if (AppState()->running) {
        AppState_ReportStop(stopReasonModeSwitch);
    }
    AppSpeedLoop_ResetModeState();
}

/**
 * @brief  执行一条命令。
 * @param  message 从命令队列取出的命令。
 * @note   由 AppSpeedLoop_Task50ms() 在中断里调用，所以这里**不能有阻塞操作**
 *         和长耗时的格式化。命令涉及的动作全部是「改状态 + 复位」这类短操作。
 *
 * @note   各命令的处理要点：
 * @code
 *   start        先按当前模式做启动前初始化，再置运行标志
 *   stop         报停车原因（若在运行），再整体复位
 *   lapCycle     只在循迹模式且**停车时**允许改目标圈数
 *   modeXxx      先报停车原因并复位，再改模式并做该模式的初始化
 *   setXxxRpm    l/r/t：切手动并设目标，见下方说明
 *   pwmSpeedNextGear
 *                PWM 速度模式下换档要重启测试，否则新旧目标混在一起
 *   quiet / log  只切 VOFA 开关，不动控制
 * @endcode
 */
static void handleCommand(commandMessage_t message)
{
    switch (message.type) {
    case commandStart:
        AppState_ClearStopEvent();
        if (!AppState()->running) {
            if (AppState()->mode == modeTrack) {
                AppTracking_Start();
            } else if (AppState()->mode == modePwmSpeed) {
                startPwmSpeedTest();
            } else {
                AppTracking_ResetState();
                AppTestMode_ResetPwmTrack();
            }
        }
        AppState_SetRunning(1U);
        break;

    case commandStop:
        if (AppState()->running) {
            AppState_ReportStop(stopReasonUser);
        }
        AppSpeedLoop_ResetModeState();
        break;

    case commandLapCycle:
        if ((AppState()->mode == modeTrack) && !AppState()->running) {
            AppState_CycleLapTarget();
        }
        break;

    case commandModeTrack:
        resetBeforeModeChange();
        AppState_SetMode(modeTrack);
        AppTracking_Start();
        break;

    case commandModePwmSpeed:
        resetBeforeModeChange();
        AppState_SetMode(modePwmSpeed);
        startPwmSpeedTest();
        break;

    case commandModePwmTrack:
        resetBeforeModeChange();
        AppState_SetMode(modePwmTrack);
        AppTracking_ResetState();
        AppTestMode_ResetPwmTrack();
        break;

    /* 台架手动转速：只改指定的那一路，另一路保持当前值。
       `l`/`t` 改左轮，`r`/`t` 改右轮。
       模式检查在命令行侧已完成（那里才能回提示），这里直接应用。 */
    case commandSetLeftRpm:
    case commandSetRightRpm:
    case commandSetBothRpm: {
        float leftTarget = AppState()->target_left_rpm;
        float rightTarget = AppState()->target_right_rpm;

        /* 关掉 5 秒自动换向，把控制权交给命令。
           stop 与模式切换都会走到 AppTestMode_ResetPwmSpeed()，自动恢复。 */
        AppTestMode_SetAutoPhase(0U);

        if (message.type != commandSetRightRpm) {
            leftTarget = message.value;
        }
        if (message.type != commandSetLeftRpm) {
            rightTarget = message.value;
        }

        AppState_SetWheelTargets(leftTarget, rightTarget);
        AppSpeedLoop_ResetRuntime();
        AppState_SetRunning(1U);
        break;
    }

    case commandPwmSpeedNextGear:
        AppState_StepGear(1);
        if (AppState()->mode == modePwmSpeed) {
            startPwmSpeedTest();
        }
        break;

    case commandPwmTrackNext:
        if (AppState()->mode == modePwmTrack) {
            AppTestMode_NextPattern();
        }
        break;

    case commandPwmTrackBaseStep:
        if (AppState()->mode == modePwmTrack) {
            AppTestMode_StepBasePwm((int16_t)message.value);
        }
        break;

    case commandQuiet:
        AppState_SetVofa(0U);
        break;

    case commandLog:
        AppState_SetVofa(1U);
        break;

    default:
        break;
    }
}

/* ══════════════ 对外接口 ══════════════ */

/**
 * @brief  初始化左右两个速度环。
 * @note   必须在中断放行之前调用；Pid_Init() 会把运行时状态一并清零。
 */
void AppSpeedLoop_Init(void)
{
    Pid_Init(&pidLeft, &speedPidConfig);
    Pid_Init(&pidRight, &speedPidConfig);
}

/**
 * @brief  清空速度环运行时状态，保留增益。
 * @note   在「停止」「模式切换」「换向」「改目标」几处调用。
 *         停车时清积分是必须的：否则下次启动时积分项带着旧值进来，
 *         输出会突然不为零，表现为**起步窜车**。
 */
void AppSpeedLoop_ResetRuntime(void)
{
    Pid_Reset(&pidLeft);
    Pid_Reset(&pidRight);
}

/**
 * @brief  停车：清运行标志、清目标、复位速度环、关电机。
 * @note   这是「上报停车」之后真正落地的一步。循迹模块只负责上报
 *         （AppState_ReportStop），由 50 ms 任务取走标志后执行这里的动作，
 *         这样循迹模块不需要反向依赖速度环。
 */
void AppSpeedLoop_ResetMotion(void)
{
    AppState_SetRunning(0U);
    AppState_SetWheelTargets(0.0f, 0.0f);
    AppSpeedLoop_ResetRuntime();
    AppMotion_Stop();
}

/**
 * @brief  复位整个模式：停车 + 复位循迹状态机 + 复位两个测试模式。
 * @note   模式切换、开机页轮换、从开机页退出时都会调用。
 *         只复位自己域内的东西——三处状态分属三个模块，任何一处漏掉都会
 *         在切换后留下残留输出。
 * @note   这里也是「手动转速模式 → 恢复自动换向」的复位点：
 *         AppTestMode_ResetPwmSpeed() 会把自动相位开关重新打开。
 */
void AppSpeedLoop_ResetModeState(void)
{
    AppSpeedLoop_ResetMotion();
    AppTracking_ResetState();
    AppTestMode_ResetPwmSpeed();
    AppTestMode_ResetPwmTrack();
}

/**
 * @brief  读取速度环的增益与运行状态。
 * @param  channel SPEED_CH_LEFT 或 SPEED_CH_RIGHT。
 * @param  out     输出结构体，为空指针时直接返回。
 * @note   供串口 show 命令打印。只读——本工程不再提供在线改增益的接口，
 *         参数在 app_config.h 里改后重新编译。
 * @note   target/actual/out 由 50 ms 中断任务在改，所以整体拷贝放进
 *         保存/恢复 PRIMASK 的临界区：逐字段读会拷出「新 target + 旧 out」
 *         这种混排值，show 打出来前后矛盾。cfg 的三个增益初始化后不再变，
 *         本来不需要保护，放在同一个临界区里只是为了少一层分支。
 */
void AppSpeedLoop_GetInfo(SpeedChannel_t channel, SpeedPidInfo_t *out)
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

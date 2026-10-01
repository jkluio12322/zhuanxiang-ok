/**
 * @file    app_config.h
 * @brief   应用层全部可调参数，唯一入口。
 * @note    所有宏都是单行的 `#define 名字 数值`，因为 tools/ 下的 Python 脚本
 *          用正则从这里解析参数做离线仿真。不要写成表达式或跨行。
 *          电机方向引脚别名需要 CubeMX 的类型定义，单独放在 board.h。
 */
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* ========================================================================= */
/* 1. 任务周期与启动时序                                                     */
/* ========================================================================= */

/* MPU6050 姿态采样任务周期，单位毫秒。 */
#define APP_SENSOR_PERIOD_MS          10U

/* 左右速度环和电机控制任务周期，单位毫秒。 */
#define APP_CONTROL_PERIOD_MS         50U

/* 循迹采样、查表和状态机任务周期，单位毫秒。 */
#define APP_TRACK_PERIOD_MS           20U

/* OLED 状态页刷新周期，单位毫秒。 */
#define APP_DISPLAY_PERIOD_MS         100U

/* 按键状态机扫描周期，单位毫秒。 */
#define APP_KEY_TICK_PERIOD_MS        20U

/* OLED 上电初始化后的稳定等待时间，单位毫秒。 */
#define APP_OLED_BOOT_DELAY_MS        100U

/* 启动蜂鸣器鸣叫时间，单位毫秒。 */
#define APP_BOOT_BEEP_MS              200U

/* ========================================================================= */
/* 2. 时基与中断优先级                                                       */
/* ========================================================================= */

/* 1 kHz 时基预分频值，72 MHz / 72 = 1 MHz 计数时钟。 */
#define TIMER_PRESCALER_1KHZ          71U

/* 1 kHz 时基自动重装载值，每 1000 个计数产生一次中断。 */
#define TIMER_PERIOD_1KHZ             999U

/* TIM5 毫秒计数中断的抢占优先级，数值越小优先级越高。 */
#define TIMER_TIM5_IRQ_PREEMPTION     4U

/* TIM6 应用任务调度中断的抢占优先级。 */
#define TIMER_TIM6_IRQ_PREEMPTION     0U

/* 按键 EXTI 中断抢占优先级。 */
#define KEY_EXTI_IRQ_PREEMPTION       3U

/* MPU6050 数据就绪中断抢占优先级。 */
#define IMU_EXTI_IRQ_PREEMPTION       1U

/* USART 接收中断抢占优先级。 */
#define SERIAL_IRQ_PREEMPTION         2U

/* ========================================================================= */
/* 3. 串口波特率                                                             */
/* ========================================================================= */

/* USART1 调试、VOFA 和在线调参使用的波特率。 */
#define SERIAL_DEBUG_BAUD_RATE        115200U

/* USART2 保留给蓝牙模块，当前应用未使用。 */
#define BLUETOOTH_BAUD_RATE           9600U

/* ========================================================================= */
/* 4. TIM1 电机 PWM 几何                                                     */
/* ========================================================================= */

/* TIM1 预分频值，0 表示 72 MHz 时钟不进一步分频。 */
#define MOTOR_PWM_PRESCALER           0U

/* TIM1 自动重装载值，72 MHz / 4500 = 16 kHz PWM 频率。 */
#define MOTOR_PWM_PERIOD              4500U

/* PWM 比较寄存器最大值，对应 100% 占空比。 */
#define MOTOR_PWM_CCR_MAX             4500U

/* 每 1% 占空比对应的 CCR 增量，4500 / 100 = 45。 */
#define MOTOR_PWM_CCR_PER_PERCENT     45U

/* 电机电源固定标称电压，用于 VOFA 最终命令电压换算。 */
#define MOTOR_SUPPLY_VOLTAGE          12.0f

/* ========================================================================= */
/* 5. 电机输出限幅与编码器速度滤波                                           */
/* ========================================================================= */

/* 电机 PWM 输出的百分比上限。 */
#define MOTOR_OUTPUT_MAX_PERCENT      100

/* 电机 PWM 输出的百分比下限，手动模式原地转需要一正一反。 */
#define MOTOR_OUTPUT_MIN_PERCENT      (-100)

/* 编码器速度中值滤波窗口长度。 */
#define ENCODER_FILTER_WINDOW         3U

/* 编码器速度一阶低通滤波的旧值权重。 */
#define ENCODER_FILTER_PREVIOUS       0.75f

/* 编码器速度一阶低通滤波的新值权重。 */
#define ENCODER_FILTER_CURRENT        0.25f

/* ========================================================================= */
/* 6. TIM2/TIM3 编码器寄存器参数与轮参数                                     */
/* ========================================================================= */

/* 编码器定时器预分频值，0 表示编码器信号直接计数。 */
#define ENCODER_TIMER_PRESCALER       0U

/* 编码器定时器计数上限，使用 16 位最大值。 */
#define ENCODER_TIMER_PERIOD          65535U

/* 编码器定时器输入数字滤波等级，数值越大抗干扰越强。 */
#define ENCODER_INPUT_FILTER          15U

/* 正交编码器四倍频系数，用于将计数值换算为完整脉冲数。 */
#define ENCODER_MULTIPLE              4U

/* JGB37-520 12V 178 RPM 版本对应 56:1 减速箱。 */
#define ENCODER_REDUCTION_RATIO       56.0f

/* 编码器每转脉冲数，用于计算轮速。 */
#define ENCODER_PPR                   11U

/* 车轮半径，单位米，用于把角速度换算为线速度。 */
#define ENCODER_WHEEL_RADIUS_M        0.065f

/* 左右轮目标速度绝对上限，单位 RPM。 */
#define SPEED_TARGET_LIMIT_RPM        190.0f

/* ========================================================================= */
/* 7. 左右轮增量式速度环（内环，PI）                                         */
/* ========================================================================= */

/* 速度环比例增益。 */
#define SPEED_PID_KP                  0.25f

/* 速度环积分增益，用于消除摩擦、负载和电池掉压造成的稳态速度误差。 */
#define SPEED_PID_KI                  0.020f

/* 速度环微分增益；测速有噪声，默认不给微分。 */
#define SPEED_PID_KD                  0.0f

/* 速度环输出百分比上限。 */
#define SPEED_PID_OUT_MAX             100.0f

/* 速度环输出百分比下限，手动模式允许反转。 */
#define SPEED_PID_OUT_MIN             (-100.0f)

/* 速度环输出的静摩擦补偿百分比；0 表示关闭。 */
#define SPEED_PID_OUT_OFFSET_PERCENT  0.0f

/* 目标转速绝对值低于该值时输出强制为零，单位 RPM。 */
#define SPEED_PID_ZERO_HOLD_RPM       0.5f

/* ========================================================================= */
/* 8. 速度档位                                                               */
/* ========================================================================= */

/* 可用速度档位数量，当前为 G1~G6 共 6 档。 */
#define SPEED_LEVEL_COUNT             6U

/* 上电默认档位。 */
#define SPEED_LEVEL_DEFAULT           2U

/* G1 目标速度。 */
#define SPEED_TABLE_G1_RPM            30.0f

/* G2 目标速度。 */
#define SPEED_TABLE_G2_RPM            55.0f

/* G3 目标速度。 */
#define SPEED_TABLE_G3_RPM            80.0f

/* G4 目标速度。 */
#define SPEED_TABLE_G4_RPM            105.0f

/* G5 目标速度。 */
#define SPEED_TABLE_G5_RPM            130.0f

/* G6 目标速度。 */
#define SPEED_TABLE_G6_RPM            170.0f

/* ========================================================================= */
/* 9. 循迹位置环（外环，PD）                                                 */
/* ========================================================================= */

/* 循迹环比例增益。 */
#define TRACK_PID_KP                  1.2f

/* 循迹环积分增益。 */
#define TRACK_PID_KI                  0.5f

/* 循迹环微分增益，用于阻尼和预判。 */
#define TRACK_PID_KD                  0.2f

/* 循迹环控制周期，与循迹任务调用周期一致，单位秒。 */
#define TRACK_PID_PERIOD_SECONDS      0.02f

/* +1 保持当前逻辑转向方向，-1 可反向修正传感器或安装方向。 */
#define TRACK_STEERING_SIGN           1.0f

/* 差速 RPM 换算为 PWM 百分比时使用的默认增益。 */
#define TRACK_STEER_RPM_TO_PWM_DEFAULT 1.0f

/* 普通循迹差速限制，单位 RPM。 */
#define TRACK_FOLLOW_DIFF_LIMIT_RPM   60.0f

/* 循迹模式允许使用的最高档位。 */
#define TRACK_MAX_SPEED_LEVEL         6U

/* 1.0 表示循迹使用所选档位的完整基准 RPM。 */
#define TRACK_DRIVE_SPEED_SCALE       1.00f

/* ========================================================================= */
/* 10. 八路循迹：线路判定与保护                                              */
/* ========================================================================= */

/* 普通有效线路图案允许的最大压黑通道数，再多视为全黑无效。 */
#define TRACK_LINE_VALID_MAX_BLACK    7U

/* 无效线路连续确认周期数后才进入丢线状态。 */
#define TRACK_LINE_INVALID_CONFIRM_TICKS 3U

/* 八路全白连续保持周期数，50 个 20 ms 周期即 1 秒后停车。 */
#define TRACK_ALL_WHITE_STOP_TICKS    50U

/* 八路加权误差限幅。 */
#define TRACK_WEIGHTED_ERROR_LIMIT    40.0f

/* 中间 IR4/IR5 掩码仅用于区分遥测阶段；误差统一由八路权重计算。 */
#define TRACK_CENTER_LEFT_MASK        0x08U

/* 中间右侧 IR5 的通道位掩码，对应 bit4。 */
#define TRACK_CENTER_RIGHT_MASK       0x10U

/* 中间两路 IR4/IR5 的组合位掩码。 */
#define TRACK_CENTER_SENSOR_MASK      0x18U

/* ========================================================================= */
/* 11. 弯道预降速与丢线保护                                                  */
/* ========================================================================= */

/* 弯道预降速开始生效的八路权重绝对误差。 */
#define TRACK_CURVE_SPEED_ERROR_START 6.0f

/* 达到最低速度比例时对应的八路权重绝对误差。 */
#define TRACK_CURVE_SPEED_ERROR_FULL  40.0f

/* 强误差时允许使用的最低基础速度，单位 RPM。 */
#define TRACK_CURVE_SPEED_MIN_RPM     45.0f

/* 强误差时基础速度相对当前档位速度的最低比例。 */
#define TRACK_CURVE_SPEED_MIN_SCALE   0.35f

/* 线路恢复居中后基础速度每周期允许的最大回升量，单位 RPM。 */
#define TRACK_SPEED_RECOVERY_STEP_RPM 10.0f

/* 线路确认丢失后允许使用的最高平均基础速度，单位 RPM。 */
#define TRACK_LOST_LINE_MAX_BASE_RPM  45.0f

/* 急弯标记期间确认线路丢失后要求的最低绝对差速，单位 RPM。 */
#define TRACK_CORNER_LOST_LINE_MIN_DIFF_RPM 40.0f

/* 急弯标记期间确认线路丢失后允许使用的最高平均基础速度，单位 RPM。 */
#define TRACK_CORNER_LOST_LINE_MAX_BASE_RPM 45.0f

/* 急弯标记期间连续无效线路达到该周期数后触发超时停车。 */
#define TRACK_CORNER_LOST_LINE_STOP_TICKS 100U

/* ========================================================================= */
/* 12. 弯道检测与计圈                                                        */
/* ========================================================================= */

/* 判定线路已经偏到边缘所需的最小归一化位置绝对值。 */
#define TRACK_CORNER_EDGE_POSITION_MIN 4.5f

/* 判断边缘候选所需的最少压线通道数。 */
#define TRACK_CORNER_EDGE_MIN_BLACK   1U

/* 无需等待离线即可直接触发急弯所需的单侧最少压线通道数。 */
#define TRACK_CORNER_DIRECT_MIN_BLACK 3U

/* 边缘候选保持周期数，5 个周期约为 100 ms。 */
#define TRACK_CORNER_CANDIDATE_HOLD_TICKS 5U

/* 下一个弯需要连续确认的次数，避免单帧噪声触发连续转向。 */
#define TRACK_CORNER_PENDING_CONFIRM_TICKS 2U

/* 急弯标记后的最短持续时间，6 个周期约为 120 ms。 */
#define TRACK_CORNER_MIN_TICKS        6U

/* 八路位置确认离开中心所需的连续周期数。 */
#define TRACK_CORNER_CENTER_CLEAR_TICKS 2U

/* 八路位置确认离开中心的最小绝对位置阈值。 */
#define TRACK_CORNER_CENTER_CLEAR_POSITION_MIN 0.5f

/* 八路位置重新居中所需连续周期数，5 次约为 100 ms。 */
#define TRACK_CORNER_REACQUIRE_CENTER_TICKS 5U

/* 判定完整八路位置已经居中的最大归一化位置绝对值。 */
#define TRACK_CORNER_REACQUIRE_POSITION_MAX 0.5f

/* 每圈完成终点计圈前必须通过的同向直角弯数量。 */
#define TRACK_CORNERS_PER_LAP         4U

/* 终点横线判定所需的最少压黑通道数，7 路用于容忍斜穿时未全部压线。 */
#define TRACK_FINISH_BLACK_COUNT_MIN  7U

/* 终点图案连续确认周期数。 */
#define TRACK_FINISH_CONFIRM_TICKS    1U

/* 非最终圈过终点后的短时丢线宽限周期数，25 个周期约为 500 ms。 */
#define TRACK_POST_LAP_LINE_GRACE_TICKS 25U

/* 起步后直行保持的周期数，50 个周期约为 1 秒。 */
#define TRACK_START_FORWARD_TICKS     50U

/* 上电默认的目标完成圈数。 */
#define TRACK_LAP_TARGET_DEFAULT      1U

/* 在线设置目标圈数时允许的最大值。 */
#define TRACK_LAP_TARGET_MAX          5U

/* ========================================================================= */
/* 13. PWM 测试模式                                                          */
/* ========================================================================= */

/* PWM 测试模式每次按键增加的占空比百分比。 */
#define PWM_TEST_STEP_PERCENT         10

/* PWM 测试模式允许的最大占空比百分比。 */
#define PWM_TEST_MAX_PERCENT          100

/* PWM 测试模式允许的最小占空比百分比。 */
#define PWM_TEST_MIN_PERCENT          (-100)

/* PWM-SPEED 自动测试的单方向持续时间，单位毫秒。 */
#define PWM_SPEED_TEST_PHASE_MS       5000U

/* PWM-TRACK 自动测试的单方向持续时间，单位毫秒。 */
#define PWM_TRACK_TEST_PHASE_MS       5000U

/* PWM-TRACK 默认基础 PWM 百分比。 */
#define PWM_TRACK_BASE_DEFAULT_PERCENT 50

/* PWM-TRACK 每次调整基础 PWM 的百分比。 */
#define PWM_TRACK_BASE_STEP_PERCENT   10

/* PWM-TRACK 模拟角度到循迹加权误差的线性满量程映射。 */
#define PWM_TRACK_ERROR_FULL_SCALE    40.0f

/* PWM-TRACK 模拟角度的满量程，单位度。 */
#define PWM_TRACK_ANGLE_FULL_SCALE    90.0f

/* ========================================================================= */
/* 14. 遥测与 OLED                                                           */
/* ========================================================================= */

/* VOFA CSV 发送缓冲区大小，单位字节。 */
#define TELEMETRY_BUFFER_SIZE         224U

/* 判断车辆已经停稳的转速误差阈值，单位 RPM。 */
#define TELEMETRY_STOP_RPM_EPSILON    0.5f

/* OLED 单行文本格式化缓冲区大小，单位字节。 */
#define OLED_TEXT_BUFFER_SIZE         24U

/* ========================================================================= */
/* 15. 电机堵转保护                                                          */
/* ========================================================================= */

/* 电机堵转保护总开关，1 表示启用，0 表示关闭。 */
#define MOTOR_STALL_PROTECT_ENABLE    1U

/* 触发堵转判断所需的最小目标转速绝对值，单位 RPM。 */
#define MOTOR_STALL_TARGET_MIN_RPM    20.0f

/* 判定电机没有转动的实际转速阈值，单位 RPM。 */
#define MOTOR_STALL_ACTUAL_MAX_RPM    2.0f

/* 判定 PID 已经持续增大输出所需的 PWM 阈值，单位百分比。 */
#define MOTOR_STALL_OUTPUT_MIN_PERCENT 25.0f

/* 堵转条件连续成立的控制周期数，25 个周期约为 1.25 秒。 */
#define MOTOR_STALL_CONFIRM_TICKS     25U

/* ========================================================================= */
/* 16. 命令队列                                                              */
/* ========================================================================= */

/* 按键和串口共用命令队列的容量。 */
#define COMMAND_QUEUE_SIZE            8U

#endif /* __APP_CONFIG_H */

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

/* USART1 调试、VOFA 和串口命令使用的波特率。 */
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

/* 这里原有两个一阶低通 EMA 的权重宏（PREVIOUS 0.75 / CURRENT 0.25）。
   那套「中值-3 + EMA」级联已被纯中值取代——EMA 的群延迟约 200 ms
   （4 个控制周期）是起步 judder 的成因之一，见 app_motion.c 的文件头。
   权重宏随之删除：它们零引用，留着只会让人以为滤波还是 EMA。 */

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

/* 循迹环比例增益。
   误差信号已从「八路权重和」换成归一化质心 position，
   量纲差了约 3.6 倍（单路压黑时 25 : 7），所以初值由 1.2 放大到 4.3。 */
#define TRACK_PID_KP                  4.3f

/* 循迹环积分增益。
   位置环没有稳态误差可言，加积分只会在出弯积一堆造成超调，默认给 0。
   若实测发现持续偏向一侧（比如传感器安装歪了），再引入一点点。 */
#define TRACK_PID_KI                  0.0f

/* 循迹环微分增益，用于阻尼和预判。同样按 3.6 倍折算。
   注意 e_y 是阶梯量化值，Kd 放大的是量化台阶，实车抖动就往下调。 */
#define TRACK_PID_KD                  0.7f

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
/* 10. 八路循迹：线路判定                                                    */
/* ========================================================================= */

/* 普通有效线路图案允许的最大压黑通道数，再多视为全黑无效。 */
#define TRACK_LINE_VALID_MAX_BLACK    7U

/* 无效线路连续确认周期数后才进入丢线状态。 */
#define TRACK_LINE_INVALID_CONFIRM_TICKS 3U

/* 八路全白连续保持周期数，50 个 20 ms 周期即 1 秒后停车。 */
#define TRACK_ALL_WHITE_STOP_TICKS    50U

/* ========================================================================= */
/* 11. 循迹：丢线保护与计圈                                                  */
/* ========================================================================= */

/* 起步后直行保持的周期数，50 个周期约为 1 秒。 */
#define TRACK_START_FORWARD_TICKS     50U

/* 丢线连续保持该周期数后停车（100 个 20 ms 周期约为 2 秒）。 */
#define TRACK_LINE_LOST_STOP_TICKS    100U

/* 判定「车在过弯」所需的归一化横向偏差绝对值。
   e_y 全量程 ±7；8 路全压黑时 e_y≈0，所以起点横线天然不会计入。 */
#define TRACK_CORNER_E_Y_MIN          3.0f

/* 越过该阈值需要连续保持的周期数。
   取 3（60 ms）：起点横线一闪而过只有约 1.5 个周期，会被滤掉；
   而转弯持续几十到几百毫秒，能抓住。 */
#define TRACK_CORNER_CONFIRM_TICKS    3U

/* 计一个弯之后的最小间隔周期数，防止一个弯被计成多个。 */
#define TRACK_CORNER_GAP_TICKS        10U

/* 每圈必须通过的直角弯数量。 */
#define TRACK_CORNERS_PER_LAP         4U

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

/* 模拟输入不再是「角度 + 量纲映射」：PWM-TRACK 把八路图案直接注入驱动层，
   由驱动层算归一化质心当误差，所以不需要任何手工标定的满量程系数。
   图案表本身在 app_testmode.c 的 patternTable[] 里。 */

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

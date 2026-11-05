/* 应用层全部可调参数，唯一入口。所有宏都是单行 `#define 名字 数值`，tools/ 的 Python
   脚本用正则从这里解析参数做离线仿真，不要写成表达式或跨行。电机方向引脚别名需要
   CubeMX 类型定义，单独放 board.h。 */
#ifndef __CONFIG_H
#define __CONFIG_H

/* IMU963RA 姿态采样任务周期，单位毫秒。 */
#define SENSOR_PERIOD_MS          10U

/* 电机速度环和输出周期，单位毫秒。 */
#define CONTROL_PERIOD_MS          5U
/* 测试阶段推进、遥测和慢速状态处理周期。 */
#define CONTROL_SLOW_PERIOD_MS    50U
#define SPEED_PID_REFERENCE_PERIOD_MS 50U
#define SPEED_MEASURE_WINDOW_MS   50U
#define SPEED_MEASURE_WINDOW_MAX_MS 50U
#define SQUARE_TRACK_BIN_PERIOD_MS 5U
#define SQUARE_TRACK_BIN_IR_SAMPLES 5U
#define SQUARE_TRACK_BIN_FRAME_SIZE 52U
#define SQUARE_TRACK_BIN_QUEUE_SIZE 8U
#define SQUARE_TRACK_HOST_WATCHDOG_MS 750U
#define TRACK_TASK_PERIOD_MIN_MS 1U
#define TRACK_TASK_PERIOD_MAX_MS 5U
#define CONTROL_TASK_PERIOD_MIN_MS 1U
#define CONTROL_TASK_PERIOD_MAX_MS 5U
#define SPEED_MEASURE_WINDOW_MIN_MS 10U
#define SPEED_MEASURE_WINDOW_MAX_CONFIG_MS 50U

/* 循迹采样、查表和状态机任务周期，单位毫秒。 */
#define TRACK_PERIOD_MS           5U
#define TRACK_MS_TO_TICKS(ms)     (((ms) + (TRACK_PERIOD_MS) - 1U) / (TRACK_PERIOD_MS))

/* OLED 状态页刷新周期，单位毫秒。 */
#define DISPLAY_PERIOD_MS         100U

/* 按键状态机扫描周期，单位毫秒。 */
#define KEY_TICK_PERIOD_MS        20U

/* OLED 上电初始化后的稳定等待时间，单位毫秒。 */
#define OLED_BOOT_DELAY_MS        100U

/* OLED 初始化总超时，防止 I2C 外设等待状态拖死整机启动。 */
#define OLED_INIT_TIMEOUT_MS      1500U

/* 启动蜂鸣器鸣叫时间，单位毫秒。 */
#define BOOT_BEEP_MS              200U

/* 1 kHz 时基预分频值，72 MHz / 72 = 1 MHz 计数时钟。 */
#define TIMER_PRESCALER_1KHZ          71U

/* 1 kHz 时基自动重装载值，每 1000 个计数产生一次中断。 */
#define TIMER_PERIOD_1KHZ             999U

/* TIM5 毫秒计数中断的抢占优先级，数值越小优先级越高。 */
#define TIMER_TIM5_IRQ_PREEMPTION     4U

/* TIM6 应用任务调度中断的抢占优先级。 */
#define TIMER_TIM6_IRQ_PREEMPTION     0U

/* IMU963RA（LSM6DSR）采样与零偏标定参数。SPI2 片选 PB12，10 ms 采样一帧。 */
#define IMU_SAMPLE_MS                  SENSOR_PERIOD_MS
#define IMU_SPI_TIMEOUT_MS             1U
#define IMU_BIAS_SAMPLES               100U
/* 上电后先等待陀螺沉降再标定，避免开机残差 */
#define IMU_SETTLE_MS            2000U
#define IMU_CAL_TIMEOUT_MS       8000U
#define IMU_CAL_MAX_SAMPLE_GAP_MS 25U
#define IMU_CAL_MAX_GYRO_MEAN_DPS 3.0f
#define IMU_CAL_MAX_GYRO_STD_DPS  0.35f
#define IMU_CAL_MAX_GYRO_DEV_DPS  2.0f
#define IMU_CAL_MAX_ACCEL_STD_G   0.05f
#define IMU_CAL_ENCODER_RPM_MAX   0.25f
#define IMU_CAL_ENCODER_ANGLE_EPS_DEG 0.30f
#define IMU_FRAME_RETRY_COUNT     3U
#define IMU_RUN_FAIL_LIMIT        5U

/* IMU963RA 丝印轴 -> 车辆右手坐标系（+X 车头、+Y 车左、+Z 车顶）：
   Xv=-Zs，Yv=+Xs，Zv=-Ys。源轴编号 0=X，1=Y，2=Z。 */
#define IMU_VEHICLE_X_SOURCE          2U
#define IMU_VEHICLE_Y_SOURCE          0U
#define IMU_VEHICLE_Z_SOURCE          1U
#define IMU_VEHICLE_X_SIGN       (-1.0f)
#define IMU_VEHICLE_Y_SIGN       (+1.0f)
#define IMU_VEHICLE_Z_SIGN       (-1.0f)

/* 映射后平放时加速度为 (0,0,+1g)，roll/pitch 机械零位均为 0。 */
#define IMU_ROLL_OFFSET   (0.0f)
#define IMU_PITCH_OFFSET  (0.0f)
#define IMU_ACC_LSB_G                  0.000122f
#define IMU_GYRO_LSB_DPS               0.035f
#define IMU_ACC_MOD_MIN_G              0.5f
#define IMU_ACC_MOD_MAX_G              1.5f
#define IMU_REPROBE_MS                 500U

/* QMC5883L 仅做后台采集和诊断，不参与 yaw、姿态或运动控制。 */
#define IMU_MAG_ENABLE                 1U
#define IMU_MAG_SAMPLE_MS              20U
#define IMU_MAG_TIMEOUT_MS             50U

/* EIGHT single-lap route. Verify yaw sign and measured track geometry before driving. */
#define EIGHT_TOTAL_TIMEOUT_MS          40000U
#define EIGHT_IMU_STALE_MS                 50U
#define EIGHT_IMU_MAX_GAP_MS               50U
#define EIGHT_YAW_SIGN                    (+1.0f)
#define EIGHT_START_TURN_DEG              (-38.7f)
#define EIGHT_TURN_C_ARC_DEG               38.7f
#define EIGHT_TURN_B_D_DEG                 38.7f
#define EIGHT_TURN_D_ARC_DEG              (-38.7f)
#define EIGHT_ARC_C_B_YAW_SIGN             (+1.0f)
#define EIGHT_ARC_D_A_YAW_SIGN             (-1.0f)
#define EIGHT_LEG_A_C_CM                  128.1f
#define EIGHT_LEG_B_D_CM                  128.1f
#define EIGHT_ARC_LENGTH_CM               125.7f
#define EIGHT_ARC_ENDPOINT_MIN_YAW_DEG    150.0f
#define EIGHT_ARC_ENDPOINT_MIN_CM         100.0f
#define EIGHT_DISTANCE_TOLERANCE_CM         2.0f
#define EIGHT_DRIVE_DECEL_MPS2              0.20f
#define EIGHT_DRIVE_MIN_RPM                  5.0f
#define EIGHT_TURN_KP                        0.70f
#define EIGHT_TURN_KD                        0.08f
#define EIGHT_TURN_MAX_RPM                  18.0f
#define EIGHT_TURN_MIN_RPM                   4.0f
#define EIGHT_TURN_TOLERANCE_DEG             1.5f
#define EIGHT_HEADING_KP                     0.80f
#define EIGHT_HEADING_KD                     0.05f
/* Differential encoder-distance correction, in RPM per centimeter of wheel mismatch. */
#define EIGHT_ENCODER_HEADING_KP             0.15f
#define EIGHT_HEADING_CORRECTION_MAX_RPM    10.0f
#define EIGHT_ARC_SEARCH_RPM                10.0f
#define EIGHT_ARC_SEARCH_STEER_RPM           5.0f
#define EIGHT_LINE_LOST_TIMEOUT_MS           300U
#define EIGHT_SPEED_LEVEL_COUNT                 3U

/* USART 接收中断抢占优先级。 */
#define SERIAL_IRQ_PREEMPTION         2U

/* USART1 调试、VOFA 和串口命令使用的波特率。 */
#define SERIAL_DEBUG_BAUD_RATE        115200U

/* USART2 保留给蓝牙模块，当前应用未使用。 */
#define BLUETOOTH_BAUD_RATE           9600U

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

/* 电机 PWM 输出的百分比上限。 */
#define MOTOR_OUTPUT_MAX_PERCENT      100

/* 电机 PWM 输出的百分比下限，手动模式原地转需要一正一反。 */
#define MOTOR_OUTPUT_MIN_PERCENT      (-100)

/* 编码器速度中值滤波窗口长度。 */

/* 编码器输出轮等效角度的单计数分辨率，360 / (4 * 56 * 11)。 */
#define ENCODER_COUNT_ANGLE_DEG       0.1461f

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

/* 车体方向极性（±1）。+1 = 该轮「命令为正 → 车往前」「往前 → 编码器读数为正」。
   电机线接反就取 -1；编码器 A/B 接反就取 -1。判定方法：PWM-SPD 页按 K3 看 ENC 行，
   轮子往前转但 ENC 是负 → 改 ENCODER_x_SIGN；轮子往后转（ENC 为正）→ 改 MOTOR_x_SIGN。
   命令与反馈符号相反时闭环会一直放大误差，积分 2~4 秒顶满 = 上电就全速。 */
#define MOTOR_LEFT_SIGN                (+1)
#define MOTOR_RIGHT_SIGN               (+1)
#define ENCODER_LEFT_SIGN              (+1)
#define ENCODER_RIGHT_SIGN             (+1)

/* 速度环比例增益；台架阶跃整定值。 */
#define SPEED_PID_KP                  0.45f

/* 速度环积分增益，用于消除摩擦、负载和电池掉压造成的稳态速度误差。 */
#define SPEED_PID_KI                  0.12f

/* 速度环微分增益；测速有噪声，默认不给微分。 */
#define SPEED_PID_KD                  0.0f

/* 左右速度环独立增益，后续分别整定。 */
#define SPEED_PID_LEFT_KP             0.45f
#define SPEED_PID_LEFT_KI             0.12f
#define SPEED_PID_LEFT_KD             0.0f
#define SPEED_PID_RIGHT_KP            0.45f
#define SPEED_PID_RIGHT_KI            0.12f
#define SPEED_PID_RIGHT_KD            0.0f

/* 速度环输出百分比上限。 */
#define SPEED_PID_OUT_MAX             100.0f

/* 速度环输出百分比下限，手动模式允许反转。 */
#define SPEED_PID_OUT_MIN             (-100.0f)

/* 速度环输出的静摩擦补偿百分比；0 表示关闭。 */
#define SPEED_PID_OUT_OFFSET_PERCENT  0.0f

/* 目标转速绝对值低于该值时输出强制为零，单位 RPM。 */
#define SPEED_PID_ZERO_HOLD_RPM       0.5f

/* 每侧、每方向速度前馈 LUT 的点数，占空比点为 0,10,...,100%。 */
#define SPEED_CAL_POINT_COUNT         11U

/* 低速前馈从 0 渐入到完整 LUT 的转速范围，避免微小目标触发启动死区 PWM。 */
#define SPEED_CAL_FF_TAPER_RPM        10.0f

/* 假速度对象的一阶时间常数，单位秒。 */
#define SPEED_FAKE_TAU_S              0.18f

/* 轮端位置 PD。Kp 单位 RPM/deg；Kd 对实际 RPM 做阻尼。 */

/* 左右轮位置外环独立参数。 */
#define POSITION_LEFT_KP              0.12f
#define POSITION_LEFT_KD              0.20f
#define POSITION_LEFT_ENTER_DEG       0.8f
#define POSITION_LEFT_RELEASE_DEG     2.0f
#define POSITION_LEFT_CREEP_RPM       1.5f
#define POSITION_RIGHT_KP             0.08f
#define POSITION_RIGHT_KD             0.20f
#define POSITION_RIGHT_ENTER_DEG      0.8f
#define POSITION_RIGHT_RELEASE_DEG    2.0f
#define POSITION_RIGHT_CREEP_RPM      1.5f

/* 非保持状态下用于跨越静摩擦的最小爬行目标，单位 RPM。 */

/* 位置进入/释放死区，最终值根据编码器重复性和回差实测固化。 */

/* 连续轨迹位置环：期望速度前馈、位置误差修正和速度误差阻尼。 */
#define POSITION_TRACK_LEFT_KP        0.35f
#define POSITION_TRACK_LEFT_KD        0.37f
#define POSITION_TRACK_RIGHT_KP       0.35f
#define POSITION_TRACK_RIGHT_KD       0.37f

/* 位置测试安全边界和默认最高轮速。 */
#define POSITION_LIMIT_DEG            1440.0f
#define POSITION_MAX_RPM              140.0f

#define POSITION_LEVEL_LOW_DECEL_SCALE  0.70f
#define POSITION_LEVEL_MID_DECEL_SCALE  0.35f
#define POSITION_LEVEL_HIGH_DECEL_SCALE 0.45f
#define POSITION_LEVEL_LOW_APPROACH_SCALE  0.15f
#define POSITION_LEVEL_MID_APPROACH_SCALE  0.03f
#define POSITION_LEVEL_HIGH_APPROACH_SCALE 0.015f

/* A position phase may advance only after both wheels are nearly stopped. */
#define POSITION_SETTLE_RPM_MAX        2.0f

/* LOOP-TEST 阶段和超时。 */
#define LOOP_TEST_TIMEOUT_MS          60000U
#define LOOP_TEST_SEGMENT_MS          3000U
#define LOOP_TEST_SPEED_SEGMENT_MS    2000U
#define LOOP_TEST_DWELL_MS            2000U
#define LOOP_TEST_POSITION_PHASE_TIMEOUT_MS 15000U

/* POS-CL 三档：阶跃角度、梯形长行程、最高 RPM、加速度。 */
#define POSITION_LEVEL_LOW_STEP_DEG   30.0f
#define POSITION_LEVEL_LOW_LONG_DEG   360.0f
#define POSITION_LEVEL_LOW_RPM        20.0f
#define POSITION_LEVEL_LOW_ACCEL      40.0f
#define POSITION_LEVEL_MID_STEP_DEG   45.0f
#define POSITION_LEVEL_MID_LONG_DEG   720.0f
#define POSITION_LEVEL_MID_RPM        40.0f
#define POSITION_LEVEL_MID_ACCEL      80.0f
#define POSITION_LEVEL_HIGH_STEP_DEG  60.0f
#define POSITION_LEVEL_HIGH_LONG_DEG  1080.0f
#define POSITION_LEVEL_HIGH_RPM       140.0f
#define POSITION_LEVEL_HIGH_ACCEL     160.0f

/* 可用速度档位数量，当前为 G1~G6 共 6 档。 */
#define SPEED_LEVEL_COUNT             6U

/* 上电默认档位。 */
#define SPEED_LEVEL_DEFAULT           1U

/* G1 目标速度。 */
#define SPEED_TABLE_G1_RPM            60.0f

/* G2 目标速度。 */
#define SPEED_TABLE_G2_RPM            80.0f

/* G3 目标速度。 */
#define SPEED_TABLE_G3_RPM           100.0f

/* G4 目标速度。 */
#define SPEED_TABLE_G4_RPM           120.0f

/* G5 目标速度。 */
#define SPEED_TABLE_G5_RPM           145.0f

/* G6 目标速度。 */
#define SPEED_TABLE_G6_RPM            170.0f

/* 转向比例环增益，输入是稳定分母归一化误差 [-1,+1]。 */
#define TRACK_PID_KP                  0.80f

/* 归一化误差微分的增益；输入 D 项已经过 20 ms 低通。 */
#define TRACK_PID_KD                  0.02f

/* +1 保持当前逻辑转向方向，-1 可反向修正传感器或安装方向。 */
#define TRACK_STEERING_SIGN           (-1.0f)

/* 陀螺仪阻尼方向；手转标定后若阻尼帮助收敛则保持 +1。 */
#define TRACK_GYRO_SIGN                (+1.0f)

/* 陀螺仪阻尼增益，单位转向比例/(deg/s)。 */
#define TRACK_GYRO_KP                  0.001f

/* 转向 D 项低通时间常数。 */
#define TRACK_D_FILTER_MS             20U

/* 归一化质心稳定分母偏置，0.5 个通道。 */
#define TRACK_ERROR_DENOM_BIAS        0.5f

/* 转向比例限幅，以及外轮只增加内轮修正量的 20%。 */
#define TRACK_STEER_RATIO_LIMIT       0.65f
#define TRACK_OUTER_WHEEL_GAIN        0.20f

/* 基础目标速度的起步/换档加速度，单位 RPM/s。 */
#define TRACK_BASE_ACCEL_RPM_S        120.0f

/* 差速 RPM 换算为 PWM 百分比时使用的默认增益。 */
#define TRACK_STEER_RPM_TO_PWM_DEFAULT 1.0f

/* PWM-TRACK 测试把无量纲转向比例换算成差速 RPM 的显示/测试刻度。 */
#define TRACK_PWM_TEST_STEER_SCALE_RPM 35.0f

/* 循迹模式允许使用的最高档位。 */
#define TRACK_MAX_SPEED_LEVEL         6U

/* 普通有效线路图案允许的最大压黑通道数，再多视为全黑无效。 */
#define TRACK_LINE_VALID_MAX_BLACK    7U

/* 丢线后保持原舵量搜索，车辆净转角达到该值仍无黑线则停车。 */
#define TRACK_LOST_YAW_LIMIT_DEG      180.0f

/* 丢线转弯依赖 IMU，样本超过该时间未更新则立即停车。 */
#define TRACK_LOST_IMU_STALE_MS       100U

/* 判定「车在过弯」所需的归一化横向偏差绝对值。 */
#define TRACK_CORNER_ERROR_MIN         0.42f

/* 计圈事件连续保持时间；只影响计数，不参与转向。 */
#define TRACK_CORNER_CONFIRM_MS       20U
#define TRACK_CORNER_CONFIRM_TICKS    TRACK_MS_TO_TICKS(TRACK_CORNER_CONFIRM_MS)

/* 计一个弯之后的最小间隔周期数，防止一个弯被计成多个。 */
#define TRACK_CORNER_GAP_MS           200U
#define TRACK_CORNER_GAP_TICKS        TRACK_MS_TO_TICKS(TRACK_CORNER_GAP_MS)

/* 每圈必须通过的直角弯数量。 */
#define TRACK_CORNERS_PER_LAP         4U

/* 上电默认的目标完成圈数。 */
#define TRACK_LAP_TARGET_DEFAULT      1U

/* 在线设置目标圈数时允许的最大值。 */
#define TRACK_LAP_TARGET_MAX          5U

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

/* 手动 PWM-SPEED 开环测试的默认占空比百分比（按 KEY3/KEY4 起转时的输出强度）。 */
#define PWM_SPEED_DUTY_DEFAULT_PERCENT 30

/* 手动 PWM-SPEED 开环测试每次按 KEY2 调整的占空比百分比。 */
#define PWM_SPEED_DUTY_STEP_PERCENT   10

/* VOFA CSV 发送缓冲区大小，单位字节。 */
#define TELEMETRY_BUFFER_SIZE         224U

/* OLED 单行文本格式化缓冲区大小，单位字节。 */
#define OLED_TEXT_BUFFER_SIZE         24U

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

/* 按键和串口共用命令队列的容量。 */
#define COMMAND_QUEUE_SIZE            8U

#endif /* __CONFIG_H */

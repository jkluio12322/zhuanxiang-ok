/**
 * @file    yingYongPeiZhi.h
 * @brief   应用层集中配置头文件。
 * @note    所有任务周期、PID 默认值、速度档位、循迹参数、通信参数
 *          和电机方向 IO 语义别名都集中在这里，便于统一调整。
 */
#ifndef __YING_YONG_PEI_ZHI_H
#define __YING_YONG_PEI_ZHI_H  /* 头文件保护宏，防止配置被重复包含。 */

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 应用任务周期与启动时序                                                    */
/* ------------------------------------------------------------------------- */

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

/* ------------------------------------------------------------------------- */
/* TIM1 电机 PWM                                                             */
/* ------------------------------------------------------------------------- */

/* TIM1 预分频值，0 表示 72 MHz 时钟不进一步分频。 */
#define MOTOR_PWM_PRESCALER           0U

/* TIM1 自动重装载值，72 MHz / 4500 = 16 kHz PWM 频率。 */
#define MOTOR_PWM_PERIOD              4500U

/* PWM 比较寄存器最大值，对应 100% 占空比。 */
#define MOTOR_PWM_CCR_MAX             4500U

/* 每 1% 占空比对应的 CCR 增量，4500 / 100 = 45。 */
#define MOTOR_PWM_CCR_PER_PERCENT     45U

/* 电机输出限制和低速补偿参数已移到 yingYongKongZhi.c 顶部。 */

/* PWM 测试模式每次按键增加的占空比百分比。 */
#define PWM_TEST_STEP_PERCENT         10

/* PWM 测试模式允许的最大占空比百分比。 */
#define PWM_TEST_MAX_PERCENT          100

/* PWM 测试模式允许的最小占空比百分比。 */
#define PWM_TEST_MIN_PERCENT          (-100)

/* ------------------------------------------------------------------------- */
/* 电机堵转保护                                                              */
/* ------------------------------------------------------------------------- */

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

/* ------------------------------------------------------------------------- */
/* TIM2/TIM3 编码器寄存器参数                                                */
/* ------------------------------------------------------------------------- */

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

/* ------------------------------------------------------------------------- */
/* TIM5/TIM6 时基与中断优先级                                                */
/* ------------------------------------------------------------------------- */

/* 1 kHz 时基预分频值，72 MHz / 72 = 1 MHz 计数时钟。 */
#define TIMER_PRESCALER_1KHZ          (72U - 1U)

/* 1 kHz 时基自动重装载值，每 1000 个计数产生一次中断。 */
#define TIMER_PERIOD_1KHZ             (1000U - 1U)

/* TIM5 毫秒计数中断的抢占优先级，数值越小优先级越高。 */
#define TIMER_TIM5_IRQ_PREEMPTION     4U

/* TIM6 应用任务调度中断的抢占优先级，保持最高用于周期调度。 */
#define TIMER_TIM6_IRQ_PREEMPTION     0U

/* 按键 EXTI 中断抢占优先级。 */
#define KEY_EXTI_IRQ_PREEMPTION       3U

/* MPU6050 数据就绪中断抢占优先级。 */
#define IMU_EXTI_IRQ_PREEMPTION       1U

/* USART 接收中断抢占优先级。 */
#define SERIAL_IRQ_PREEMPTION         2U

/* USART1 调试、VOFA 和在线调参使用的波特率。 */
#define SERIAL_DEBUG_BAUD_RATE        115200U

/* USART2 蓝牙模块通信波特率，HC-05 默认使用 9600。 */
#define BLUETOOTH_BAUD_RATE           9600U

/* ------------------------------------------------------------------------- */
/* 电机方向 IO 语义别名                                                      */
/* ------------------------------------------------------------------------- */

/* 电机 A 正转方向控制 GPIO 端口，实际端口由 CubeMX main.h 定义。 */
#define MOTOR_A_FORWARD_GPIO_PORT     pwm1_0_GPIO_Port

/* 电机 A 正转方向控制 GPIO 引脚，实际引脚由 CubeMX main.h 定义。 */
#define MOTOR_A_FORWARD_PIN           pwm1_0_Pin

/* 电机 A 反转方向控制 GPIO 端口。 */
#define MOTOR_A_REVERSE_GPIO_PORT     pwm1_1_GPIO_Port

/* 电机 A 反转方向控制 GPIO 引脚。 */
#define MOTOR_A_REVERSE_PIN           pwm1_1_Pin

/* 电机 B 正转方向控制 GPIO 端口。 */
#define MOTOR_B_FORWARD_GPIO_PORT     pwm2_1_GPIO_Port

/* 电机 B 正转方向控制 GPIO 引脚。 */
#define MOTOR_B_FORWARD_PIN           pwm2_1_Pin

/* 电机 B 反转方向控制 GPIO 端口。 */
#define MOTOR_B_REVERSE_GPIO_PORT     pwm2_0_GPIO_Port

/* 电机 B 反转方向控制 GPIO 引脚。 */
#define MOTOR_B_REVERSE_PIN           pwm2_0_Pin

/* ------------------------------------------------------------------------- */
/* 编码器滤波与速度限制                                                      */
/* ------------------------------------------------------------------------- */

/* 编码器滤波参数已移到 yingYongKongZhi.c 顶部。 */

/* 左右轮目标速度绝对上限，单位 RPM。 */
#define SPEED_TARGET_LIMIT_RPM        190.0f

/* ------------------------------------------------------------------------- */
/* 速度档位                                                                  */
/* ------------------------------------------------------------------------- */

/* 可用速度档位数量，当前为 G1~G6 共 6 档。 */
#define SPEED_LEVEL_COUNT             6U

/* 默认档位和 G1~G6 速度表已移到 yingYongZhuangTai.c 顶部。 */

/* ------------------------------------------------------------------------- */
/* 左右轮增量式速度 PID                                                      */
/* ------------------------------------------------------------------------- */

/* 左右速度环 PID 参数已移到 yingYongZhuangTai.c 顶部。 */

/* ------------------------------------------------------------------------- */
/* 航向角环 PID                                                              */
/* ------------------------------------------------------------------------- */

/* 航向角环 PID 增益和输出限制已移到 yingYongZhuangTai.c 顶部。 */

/* 航向角环分频数；2 表示每两个 50 ms 控制周期更新一次，即 100 ms。 */
#define YAW_PID_PERIOD_DIVIDER        2U

/* 判定左右基础目标相同所需的误差阈值，单位 RPM。 */
#define YAW_STRAIGHT_EPSILON_RPM      0.01f

/* 平均目标绝对值低于该值时，不启用直行航向闭环，单位 RPM。 */
#define YAW_MIN_TARGET_RPM            1.0f

/* ------------------------------------------------------------------------- */
/* 循迹位置式 PID 兼容定义                                                   */
/* ------------------------------------------------------------------------- */

/* 以下定义为旧接口兼容项，当前数字查表循迹不读取这些值。
 * 保留宏名是为了避免影响外部旧代码，调参时不要修改这些数值。 */
#define TRACK_PID_KP                  4.5f
#define TRACK_PID_KI                  0.0f
#define TRACK_PID_KD                  0.8f
#define TRACK_PID_OUT_MAX             25.0f
#define TRACK_PID_OUT_MIN             (-25.0f)
#define TRACK_PID_INTEGRAL_LIMIT      8.0f
#define TRACK_PID_DEADBAND            0.0f
#define TRACK_PID_RATE_LIMIT          12.0f
#define TRACK_USE_IMU_YAW             0U

#define TRACK_POSITION_FILTER_PREVIOUS 0.55f
#define TRACK_POSITION_FILTER_CURRENT  0.45f

/* 位置一阶低通滤波权重。 */
/* ------------------------------------------------------------------------- */
/* 8 路数字循迹策略                                                          */
/* ------------------------------------------------------------------------- */

/* 循迹模式允许使用的最高档位。 */
#define TRACK_MAX_SPEED_LEVEL         6U

/* 1.0 表示循迹使用所选档位的完整基准 RPM。 */
#define TRACK_DRIVE_SPEED_SCALE       1.00f

/* +1 保持当前逻辑转向方向，-1 可反向修正传感器或安装方向。 */
#define TRACK_STEERING_SIGN           1.0f

/* 普通有效线路图案必须包含 1~6 个黑线通道。 */
#define TRACK_LINE_VALID_MAX_BLACK    7U

/* 无效线路连续确认 3 个 20 ms 周期后才进入丢线状态。 */
#define TRACK_LINE_INVALID_CONFIRM_TICKS 3U

/* 急弯锁存后的最短持续时间，6 个 20 ms 周期约为 120 ms。 */
#define TRACK_CORNER_MIN_TICKS        6U

/* 新线路重新居中所需连续周期数，3 次约为 60 ms。 */
#define TRACK_CORNER_REACQUIRE_CENTER_TICKS 3U

/* 判定新线路已经居中的最大归一化位置绝对值。 */
#define TRACK_CORNER_REACQUIRE_POSITION_MAX 2.0f

/* 急弯检测、转弯速度、急弯计数阈值和重新找线参数已分别移到
 * yingYongXunJi.c 与 zhanShuBao.c 顶部。 */

/* 全灭计圈、全亮丢线及左右摆头找线参数。 */
#define TRACK_FINISH_PATTERN          0xFFU
#define TRACK_LOST_PATTERN            0x00U
#define TRACK_FINISH_CONFIRM_TICKS    3U
#define TRACK_FINISH_CLEAR_TICKS      10U
#define TRACK_SWEEP_HALF_TICKS        10U
#define TRACK_SWEEP_ROUNDS            3U
#define TRACK_START_FORWARD_TICKS     50U
#define TRACK_LAP_TARGET_DEFAULT      1U
#define TRACK_LAP_TARGET_MAX          5U

/* ------------------------------------------------------------------------- */
/* 遥测与 OLED                                                               */
/* ------------------------------------------------------------------------- */

/* VOFA CSV 发送缓冲区大小，单位字节。 */
#define TELEMETRY_BUFFER_SIZE         128U

/* 判断车辆已经停稳的转速误差阈值，单位 RPM。 */
#define TELEMETRY_STOP_RPM_EPSILON    0.5f

/* OLED 单行文本格式化缓冲区大小，单位字节。 */
#define OLED_TEXT_BUFFER_SIZE         24U

/* ------------------------------------------------------------------------- */
/* 蓝牙命令换算                                                              */
/* ------------------------------------------------------------------------- */

/* 蓝牙摇杆原始值到速度比例的除数，摇杆范围按 -100~100 处理。 */
#define BLUETOOTH_STICK_SCALE         100.0f

/* 右摇杆横向值到差速量的比例，限制转向幅度。 */
#define BLUETOOTH_TURN_SCALE          0.5f

/* 蓝牙右摇杆横向绝对值达到该百分比时，忽略纵向输入并执行一正一反原地转。 */
#define BLUETOOTH_PIVOT_THRESHOLD_PERCENT 80

/* 按键、蓝牙和串口共用命令队列的容量。 */
#define COMMAND_QUEUE_SIZE            8U

#endif

/* BlueCar 全部可调参数。
   上半段是应用参数（循迹/速度环/计圈/调度），下半段是驱动层参数，由硬件实测得出，不要随手改。 */
#ifndef __CONFIG_H
#define __CONFIG_H

/* ===================== 应用参数 ===================== */

/* 调度节拍（ms）：两个独立硬件时基 —— TIM4 出采样拍，TIM7 出控制拍。
   两档不再有整除关系，也不需要公倍拍：采样拍（NVIC 0）随时插进控制拍（NVIC 1）之间，
   控制拍每次读到的都是「刚被采样拍更新过」的编码器窗口。 */
#define TASK_SAMPLE_MS          2U      /* 采样拍：读编码器、推进测速窗口（并跑速度环） */
#define TASK_CONTROL_MS         5U      /* 控制拍：红外 → 循迹决策 → 差速目标 */
#define TASK_IMU_MS              4U     /* IMU 采样周期（ODR 416 Hz → 2.4 ms 一个新样本） */
#define TASK_VOFA_MS            20U     /* VOFA 遥测 50 Hz */
#define TASK_OLED_MS            100U    /* OLED 刷新 10 Hz */
#define SCHED_OVERRUN_US        1000UL  /* 控制拍耗时预算（周期的 20%），超过计一次 overrun */
#define DIAG_PUSH_MS            1000U   /* 串口自动诊断推送周期 ms（0 = 关闭；串口 'q' 可临时开关） */
#define TASK_TRACE_MS           20U     /* 转向环采集帧周期（必须是 TASK_CONTROL_MS 的整数倍，20 = 50 Hz） */
#define PIDTRACE_DEFAULT_ON     1U      /* 上电即发采集帧；串口 'P' 关闭，'C' 开注入测试 */

/* 速度环执行位置：1 = 放进 2 ms 采样拍（TIM4，抢占 0），0 = 留在 5 ms 控制拍。
   放在 2 ms 拍的理由：测速窗本身就是 2 ms 一节，速度环每拍都有新反馈，
   等效采样率 500 Hz，比 5 ms（200 Hz）的等效延迟少 1.5 ms，且与控制拍解耦。 */
#define SPEED_LOOP_IN_SAMPLE_ISR  1U

/* 拍计数：所有时间常数先换算成控制拍数再用，中断里不做毫秒累加 */
#define TICKS_OF_MS(ms)         (((uint32_t)(ms)) / (uint32_t)TASK_CONTROL_MS)

#if ((TASK_SAMPLE_MS) == 0U) || ((TASK_CONTROL_MS) == 0U)
#error "TASK_SAMPLE_MS and TASK_CONTROL_MS must be greater than zero"
#endif


/* ---------------- 循迹：位置环（PD + 陀螺阻尼 + 车速解耦） ----------------
   steer = clamp((KP*e + KD*d + GYRO_SIGN*GYRO_KP*wz) * kv, ±STEER_LIMIT)
   e  = 归一化偏差 -1..+1（线在右为正，车要右转）
   d  = e 的微分，先过 D_FILTER_HZ 一阶低通：8 档离散位置直接求导全是尖峰
   wz = 车体 +Z 角速度 °/s（右转为负）
   kv = BASE / |v|，限幅 [MIN, MAX]：转弯增益与车速成正比，高速必须压下来
   KP 只需在标称 BASE_RPM 下整定，其它车速由 kv 自动折算。 */
#define TRACK_KP                 0.80f
#define TRACK_KD                 0.02f
#define TRACK_GYRO_KP            0.0010f /* 角速度 → 转向量，1 °/s 折 0.001 */
#define TRACK_GYRO_SIGN         (-1.0f)  /* +Z 朝上、右转为负角速度，故取 -1 */
#define TRACK_D_FILTER_HZ         8.0f   /* 微分低通截止，0 = 不滤波 */
#define TRACK_BASE_RPM          100.0f   /* 直道基础轮速，也是 kv 的标称点 */
#define TRACK_STEER_LIMIT         0.85f  /* 转向量限幅，±1 为满差速 */
#define TRACK_GAIN_SCALE_MIN      0.60f  /* kv 下限：高速最多把增益压到 60% */
#define TRACK_GAIN_SCALE_MAX      1.80f  /* kv 上限：低速最多放到 180% */

/* 非对称差速整形：内轮减满、外轮只加 OUTER。
   对称差速在打满时内轮会掉到 35 RPM 以下，实际接近堵转；
   手册 8.8 也明确反对内轮反转（偏航加速度过大、伤电机）。 */
#define TRACK_DIFF_LIMIT          0.65f  /* 整形后转向量上限（内轮剩 35%） */
#define TRACK_DIFF_OUTER          0.20f  /* 外轮最多只加 20% */

/* 入弯降速：|e| 连续 CONFIRM 拍超过 ENTER_E 就压到 SCALE 倍，
   离开后按 HOLD 时间缓慢恢复（防止弯中反复加速-减速抖成锯齿）。 */
#define TRACK_CURVE_ENTER_E       0.45f
#define TRACK_CURVE_CONFIRM       2U
#define TRACK_CURVE_SCALE         0.75f
#define TRACK_CURVE_HOLD_MS     300U

/* ---------------- 全白丢线策略 ----------------
   0 = STOP：当拍（或确认拍后）停车，最安全，默认
   1 = HOLD：保持上次转向量 HOLD_MS 再停
   2 = ARC ：按入弯前记录的航向转 CORNER_ANGLE_DEG 找线（车大、扫线宽，一次 90° 弯常常全白）
   3 = TURN：从当前航向转 CORNER_ANGLE_DEG（不依赖入弯上下文）
   运行期用串口 p 命令切换，默认值在这里。 */
#define TRACK_LOSS_POLICY_DEFAULT   0U
#define TRACK_LINE_LOST_CONFIRM     0U   /* 0 = 当拍停车；>0 = 连续 N 拍确认（防线缝误停） */
#define TRACK_CORNER_ANGLE_DEG     90.0f
#define TRACK_ARC_MAX_DEG         120.0f /* 找线最多转这么多，超了就认输（物理上限，不是时钟） */
#define TRACK_ARC_STALL_MS       1500U   /* 连续这么久误差没再缩小才算卡住：慢转不该被判死 */
#define TRACK_ARC_PROGRESS_DEG      1.0f /* 一次「有效靠近」至少缩这么多度 */
#define TRACK_ARC_CONTEXT_MS     1200U   /* 入弯上下文有效期，超时就不许再用 */
#define TRACK_LOST_HOLD_MS        400U   /* HOLD 策略的保持时长 */

/* ---------------- 陀螺闭环转角（原地转，只看 yaw 不看红外） ----------------
   误差 err = yaw_now - yaw_target，顺时针时 err > 0；
   u = KP*err + KI*Err积分 + KD*wz，左轮 +u、右轮 -u → 顺时针为正。
   KI 不参与精度：err 剩 2 度时 KP 只算出 1.6 rpm 差速，轮子转不动（静摩擦），
   实测每步就停在容差边界欠转 2 度、8 步累计 16 度。积分把这点误差按时间累起来
   把轮子顶动，限幅在 I_LIMIT_RPM 内，防止积分自己变成过冲源。 */
#define TRACK_TURN_RPM           180.0f  /* 转角差速轮速上限（左右各 ±180 → 差速 360 RPM），比首版快 6 倍 */
#define TRACK_TURN_KP              4.0f  /* 与限幅同步放大：只放大限幅而 KP 不动，末段指数收敛时间常数不变，整体快不了 */
#define TRACK_TURN_I_ENABLE         1U   /* 1 = 用积分项（V2/V3 原样） */
#define TRACK_TURN_KI              3.0f  /* rpm/(度*秒) */
#define TRACK_TURN_I_LIMIT_RPM     4.0f  /* 积分项最多贡献的差速 rpm */
#define TRACK_TURN_KD             0.30f  /* 角速度阻尼，随速度同步放大，保持阻尼比不变 */
#define TRACK_TURN_DEADBAND_RPM    4.0f  /* 末段命令小于它直接给 0：轮子在 3~4 rpm 以下只能
                                            一跳一停地爬，左右还不对称（实测右轮比左轮快
                                            1.01~1.17 倍），留着这点命令就是让车身慢慢蹭偏 */
#define TRACK_TURN_TOL_DEG         0.8f  /* 到位判据：|err| < 0.8 度 */
#define TRACK_TURN_DONE_DPS        5.0f  /* 到位还必须「真的停住」：角速度低于 20 dps 才算，
                                           否则一进容差就断电，车按惯性滑 5~9 度过去 */
#define TRACK_TURN_SETTLE          2U    /* 连续 2 拍在容差内才算到位 */
/* 转角到位判据只有角度：|err| < TOL 连续 SETTLE 拍。这里不再挂任何时钟，
   找线路径的放弃条件在下方 TRACK_ARC_MAX_DEG + TRACK_ARC_STALL_MS。 */

/* 计圈：4 个直角弯一圈 */
#define LAP_CORNER_ERROR        0.55f   /* 判定进入弯道的误差门限 */
#define LAP_CORNER_CONFIRM      3U      /* 连续确认次数（3 × 5 ms） */
#define LAP_CORNER_GAP_MS       500U    /* 两弯最小间隔，防重复计数 */
#define LAP_DEFAULT_TARGET      5U
#define LAP_MAX_TARGET          99U
#define LAP_STOP_ENABLE          0U     /* 1 = 跑够目标圈数自动停车；0 = 只计数不停车 */

/* 速度环：增量式 PI（没有 D——差分对量化噪声太敏感，手册 8.7 也不加）
   du = KP*(e - e_prev) + KI*e*dt_s，out = clamp(out + du, ±OUT_MAX)
   KP/KI 是「每拍增量」量纲，与位置式的 KP_pos/KI_pos 关系：KP = KP_pos、KI = KI_pos*dt。
   0.45 / 0.12 是从位置式沿用的等效值，实车标定后可直接调。 */
#define SPEED_PID_KP              0.45f
#define SPEED_PID_KI              0.12f
#define SPEED_PID_OUT_MAX        100.0f
#define SPEED_TARGET_LIMIT_RPM   190.0f
#define SPEED_RAMP_RPM_PER_S     300.0f  /* 目标斜坡：100 RPM 用 0.33 s 爬上来，防起步打滑/翘头 */
#define SPEED_DEADZONE_PCT         0.0f  /* 死区前馈：串口 m 1 1 读出稳速占空比后填，0 = 关闭 */
#define SPEED_STOP_RPM             6.0f  /* 停车锁存门限：目标为 0 时，实际轮速连续降到这个值以下
                                           就认定「已经停住」，直接清积分、断 PWM、上刹车。
                                           取 6 而不是 2，是因为电机的物理最低稳速约 3.2 rpm，
                                           残留转速在 0~4 rpm 之间时速度环会一直在「反向刹-松-再刹」，
                                           肉眼就是停稳后轮子间歇蠕动。 */
#define SPEED_MIN_TARGET_RPM       3.0f  /* 目标绝对值小于它 → 当成 0：实测轮子最低稳速约 3.2 rpm
                                           （m 1 1 命令下去实际跑到 3.2 rpm），再小的命令只会让
                                           两轮一停一转，车身被慢慢带偏。 */

/* ---------------- 战术包（mission） ---------------- */
#define MISSION_SPIN_RPM          30.0f  /* 顺时针原地转的轮速 */
#define MISSION_SPIN_KP            0.6f
#define MISSION_SPIN_KD           0.04f
#define MISSION_SPIN_TOL_DEG       2.0f
/* 转角不设总时长上限：转 360° 就该花 360° 的时间，用钟卡死是替物理做决定。
   只判「不再靠近目标」——连续 MISSION_SPIN_STALL_MS 里误差没有缩小
   MISSION_SPIN_PROGRESS_DEG，才认输停下。堵转、陀螺失效、被打断都会立刻
   表现成「不靠近」，所以这一个判据就够，不需要第二个超时。 */
#define MISSION_SPIN_STALL_MS    1500U
#define MISSION_SPIN_PROGRESS_DEG  1.0f
#define MISSION_ANGLE_DEG_DEFAULT 90.0f
#define MISSION_SELFTEST_ENABLE     0U   /* 1 = 上电自动跑一次顺时针 90°（架空车轮验证用） */

/* ---------------- 战术包（tactic）：比 mission 高一层的「打什么」 ---------------- */
#define TACTIC_STEP_DEG_DEFAULT   45.0f  /* 战术 2 的每步转角，初始 45° */
#define TACTIC_STEP_DEG_BUMP      45.0f  /* K3 每按一次加多少度 */
#define TACTIC_STEP_MAX_DEG      360.0f  /* 步长上限，到顶就停住不再涨（不绕回 45，K4 才回默认） */
#define TACTIC_STEP_GAP_MS      2000U    /* 转到位后停这么久，再转下一步 */
#define TACTIC_LAP_STEP            1U    /* K3 在战术 1 里每次加几圈 */

/* ---------------- 显示事件横幅的持续时间 ---------------- */
#define UI_BANNER_MS_TACTIC     1500U    /* 切战术时第一行确认条/横幅持续多久 */
#define UI_BANNER_MS_DONE       2000U    /* 任务完成/超时横幅持续多久 */

/* 通信缓冲 */
#define OLED_TEXT_SIZE          24U
#define OLED_DMA_TX              1U     /* 1 = OLED 帧用 I2C2+DMA 发送；0 = 回退到中断模式 */
#define TELEMETRY_TEXT_SIZE    128U
#define SERIAL_DEBUG_BAUD_RATE  115200U

/* ===================== 驱动层参数（勿改） ===================== */
#define ENCODER_LEFT_SIGN              (+1)
#define ENCODER_MULTIPLE              4U
#define ENCODER_PPR                   11U
#define ENCODER_REDUCTION_RATIO       56.0f
#define ENCODER_RIGHT_SIGN             (+1)
#define ENCODER_WHEEL_RADIUS_M        0.065f
#define IMU_ACC_LSB_G                  0.000122f
#define IMU_ACC_MOD_MAX_G              1.5f
#define IMU_ACC_MOD_MIN_G              0.5f
#define IMU_BIAS_SAMPLES               100U
#define IMU_CAL_ENCODER_ANGLE_EPS_DEG 0.30f
#define IMU_CAL_ENCODER_RPM_MAX   0.25f
#define IMU_CAL_MAX_ACCEL_STD_G   0.05f
#define IMU_CAL_MAX_GYRO_DEV_DPS  2.0f
#define IMU_CAL_MAX_GYRO_MEAN_DPS 3.0f
#define IMU_CAL_MAX_GYRO_STD_DPS  0.35f
#define IMU_CAL_MAX_SAMPLE_GAP_MS 25U
#define IMU_CAL_TIMEOUT_MS       8000U
#define IMU_FRAME_RETRY_COUNT     3U
#define IMU_GYRO_LSB_DPS               0.035f
#define IMU_MAG_ENABLE                 1U
#define IMU_MAG_INIT_TIMEOUT_MS       500U     /* 磁力计初始化整体超时，超了就放弃不重试 */
#define IMU_MAG_RETRY_MAX               3U     /* 单个步骤失败重试次数 */
#define IMU_MAG_SAMPLE_MS              20U
#define IMU_MAG_TIMEOUT_MS              2U     /* 单次 hub 等待上限：主循环里不能久等 */
#define IMU_PITCH_OFFSET  (0.0f)
#define IMU_REPROBE_MS                 500U
#define IMU_ROLL_OFFSET   (0.0f)
#define IMU_SAMPLE_MS                  TASK_IMU_MS
#define IMU_SETTLE_MS            2000U
#define IMU_SPI_TIMEOUT_MS             1U
#define IMU_VEHICLE_X_SIGN       (-1.0f)
#define IMU_VEHICLE_X_SOURCE          2U
#define IMU_VEHICLE_Y_SIGN       (+1.0f)
#define IMU_VEHICLE_Y_SOURCE          0U
#define IMU_VEHICLE_Z_SIGN       (-1.0f)
#define IMU_VEHICLE_Z_SOURCE          1U
#define MOTOR_LEFT_SIGN                (+1)
#define MOTOR_OUTPUT_MAX_PERCENT      100
#define MOTOR_OUTPUT_MIN_PERCENT      (-100)
/* 停车方式：TB6612 的真值表里 IN1=IN2=0 是 coast（输出高阻，靠摩擦滑停），
   IN1=IN2=1 是 short brake（电机绕组短接，动能立刻变成制动电流）。
   实测两轮摩擦不等：同样的占空比右轮比左轮快 1.01~1.17 倍，滑停时右轮要多转一会儿，
   车身就被带得微微转。1 = 停车用短路制动，0 = 原来的滑行。 */
#define MOTOR_BRAKE_ON_STOP     1U

#define MOTOR_PWM_CCR_MAX             4500U
#define MOTOR_PWM_CCR_PER_PERCENT     45U
#define MOTOR_RIGHT_SIGN               (+1)
#define OLED_BOOT_DELAY_MS        100U
#define OLED_INIT_TIMEOUT_MS      1500U
#define SPEED_SAMPLE_MAX           128U     /* 2 ms 采样、200 ms 上限窗约 100 个样本 */
#define SPEED_WINDOW_MAX_US     200000UL    /* 窗口上限，也是低速扩窗的硬顶 */
#define SPEED_WINDOW_MIN_US       5000UL
#define SPEED_WINDOW_US          20000UL    /* 基础窗 20 ms：55 RPM 下约 45 个脉冲（±1.1%） */
#define SPEED_MIN_COUNTS            12U     /* 窗内脉冲少于此数就自动扩窗，压住量化误差 */
#define SPEED_FILTER_HZ           10.0f     /* 速度一阶低通截止频率，0 = 不滤波 */
#define TRACK_ERROR_DENOM_BIAS        0.5f

/* ===================== 自检（必须放在所有宏定义之后） ===================== */
/* 再分频只有整除才不丢拍：采样拍分频出 IMU，控制拍分频出遥测与刷屏 */
#if (((TASK_IMU_MS) % (TASK_SAMPLE_MS)) != 0U)
#error "TASK_IMU_MS must be an integer multiple of TASK_SAMPLE_MS"
#endif
#if (((TASK_VOFA_MS) % (TASK_CONTROL_MS)) != 0U)
#error "TASK_VOFA_MS must be an integer multiple of TASK_CONTROL_MS"
#endif
#if (((TASK_OLED_MS) % (TASK_CONTROL_MS)) != 0U)
#error "TASK_OLED_MS must be an integer multiple of TASK_CONTROL_MS"
#endif
#if ((SPEED_SAMPLE_MAX) < 4U)
#error "SPEED_SAMPLE_MAX too small for the speed window"
#endif
#if ((TELEMETRY_TEXT_SIZE) < 96U)
#error "TELEMETRY_TEXT_SIZE too small for the 7-channel FireWater line"
#endif

#endif /* __CONFIG_H */

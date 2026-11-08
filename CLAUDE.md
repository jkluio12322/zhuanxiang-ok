# BlueCar — STM32F103ZET6 循迹小车

STM32F103ZET6 + STM32 HAL 裸机工程（无 RTOS）。**2026-11-05 应用层全量重写**，
2026-11-06 升级控制层：双硬件时基（2 ms 采样 / 5 ms 控制）、增量式 PI 速度环、陀螺阻尼转向、
非对称差速整形、里程积分、全白四策略、陀螺闭环原地转角。
旧架构（五模式 / 39 条命令 / trackbin / 位置环 / 八路模式）已整体移除。

## 目录（按职责分 7 类，22 个 .c / 26 个 .h）

| 目录 | 职责 | 内容 |
| --- | --- | --- |
| `Core/app/` | 入口与全局定义 | main.c、config.h、types.h |
| `Core/periph/` | 外设：执行器 / 时基 / 中断 | board.h、motor、buzzer、timer、isr |
| `Core/sensor/` | 传感器 | imu、irtrack、encoder |
| `Core/comm/` | 数据通信 | serial、telemetry、console、ringbuf |
| `Core/algorithm/` | 算法处理 | pid、track、laps、motion |
| `Core/task/` | 任务处理 | sched、mission、control、state |
| `Core/tactic/` | 战术包：决定「这一轮打什么、参数多少」 | tactic |
| `Core/ui/` | UI 显示 | display、oled、oled_font.h、oled_font_small.h |
| `Core/Inc/` | CubeMX 生成头文件，**main.h 里的引脚宏是唯一引脚来源** | — |
| `Core/Src/` | CubeMX 生成外设初始化，保持原样不动 | — |

`Core/Inc/` 在头文件搜索路径最前，所以任何同名头文件都优先命中它。
目录名用英文：AC5 对非 ASCII 路径不稳，中文只在文档里做映射。

### 分层规则（依赖只能向下）

```
app → tactic → task → algorithm → {sensor, periph}
                        ↘ comm ↗
ui 只读 state / motion / imu / irtrack，不反向调用
```

两处向上的调用是既有的、有意为之的例外：`periph/isr.c` 与 `periph/timer.c` 调 `task/sched.c`
的两个中断入口（中断向量是总编排点）。这两个文件之外再出现向上 include 就是 bug。

## 构建 / 烧录 / 校验

```bash
# Keil（AC5）—— 工程目录是 MDK-ARM，-o 的相对基准也是它
cd MDK-ARM && "F:/kilv5/UV4/UV4.exe" -r bluecar.uvprojx -j0 -o build.log
# 产物：MDK-ARM/bluecar/bluecar.hex

# 烧录 A：ST-Link（有线）
"C:/Users/Lenovo/.eide/tools/st_cube_programer/bin/STM32_Programmer_CLI.exe" \
  -c port=SWD freq=4000 mode=UR reset=HWrst \
  -d "E:/32/32/STM32HAL/f103zet6/bluecar/MDK-ARM/bluecar/bluecar.hex" -v

# 烧录 B：无线 DAP（CMSIS-DAP v2，本机常用；同一根 USB 还带一路 CDC 串口 COM19）
OCD=C:/Users/Lenovo/.eide/tools/openocd_7a1adfbec_mingw32
"$OCD/bin/openocd.exe" -s "$OCD/share/openocd/scripts" \
  -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg -c "adapter speed 1000" \
  -c "program E:/32/32/STM32HAL/f103zet6/bluecar/MDK-ARM/bluecar/bluecar.hex verify reset exit"

# 探针自检（只看 ID、不烧）
"$OCD/bin/openocd.exe" -s "$OCD/share/openocd/scripts" \
  -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg \
  -c "adapter speed 1000" -c "init" -c "targets" -c "shutdown"

# EIDE（VSCode 里点构建，会按 eide.yml 重新生成 builder.params）
"C:/Users/Lenovo/.vscode/extensions/cl.eide-3.27.2/res/tools/win32/unify_builder/unify_builder.exe" \
  -p "e:/32/32/STM32HAL/f103zet6/bluecar/MDK-ARM/build/bluecar/builder.params" --rebuild

# 单文件语法快检（不产出固件，最快的错误检查手段）
arm-none-eabi-gcc -fsyntax-only -c -std=c11 -mcpu=cortex-m3 -DSTM32F103xE -DUSE_HAL_DRIVER \
  -I Core/Inc -I Core/app -I Core/periph -I Core/sensor -I Core/comm \
  -I Core/algorithm -I Core/task -I Core/ui \
  -I Drivers/STM32F1xx_HAL_Driver/Inc -I Drivers/STM32F1xx_HAL_Driver/Inc/Legacy \
  -I Drivers/CMSIS/Device/ST/STM32F1xx/Include -I Drivers/CMSIS/Include \
  Core/app/main.c
```

`compile_commands.json` 每次构建自动刷新，不要手动编辑。

## 四个必须知道的陷阱

### 1. 实际编译器是 ARMCC v5，不是 GCC

Keil（`bluecar.uvprojx`，`<uAC6>0</uAC6>`）和 EIDE（`toolchain: AC5`）都用 **ARMCC v5**
（`F:\kilv5\ARM\ARMCC`）。GCC 只用于 clangd 解析和上面的语法快检。不要写只在 GCC 下成立的扩展。

### 2. `isr.c` 和 `Core/Src/stm32f1xx_it.c` 只能选一个

两者定义同一批中断向量（NMI/HardFault/SysTick/TIM5/TIM6/USART1/USART2/I2C2 等）。同时进工程
报 15 个 `L6200E: multiply defined`。**当前工程只用 `Core/periph/isr.c`**，`stm32f1xx_it.c`
不进工程文件，也不要往里加代码。

`isr.c` 现在必须自己实现这些向量，漏一个就落进 startup 的 `Default_Handler` 死循环：
`TIM4_IRQHandler`（2 ms 采样拍）、`TIM7_IRQHandler`（5 ms 控制拍）、`TIM5_IRQHandler`（ms 计数）、
`DMA1_Channel4_IRQHandler`（OLED I2C2 TX）、`DMA1_Channel5_IRQHandler`（USART1 RX）、
`NMI_Handler`（含 CSS 时钟失效分支）。加新 DMA/定时器时，先在这里补入口再使能。

### 3. 增删改名任何源文件，两个工程文件都要改

`MDK-ARM/bluecar.uvprojx` 和 `MDK-ARM/.eide/eide.yml` 都要同步，包括 `IncludePath`/`incList`。
EIDE 出现过回退文件的情况，两个都视为 source of truth。还有第三条必须一起看的：
**`MDK-ARM/build/bluecar/builder.params`**（EIDE 真正的编译清单，`incDirs` + `sourceList`）。

EIDE 的实际行为（2026-11-08 实测）：
- 打开着工程时改 `eide.yml`，EIDE 会在数秒内按内存模型写回旧内容（改完必须 `grep` 复查）。
- 只改 `eide.yml` 不改 `builder.params`，点构建仍按旧清单编译，报 `#5: cannot open source input file`。
- `virtualFolder` 只写 7 个自写组、丢了 `Startup`/`CubeMX`/`Drivers`，编译能过但**链接**报
  `L6236E: No section matches selector - no section to be FIRST/LAST`（没有 startup 复位段）。
- `sourceList` 只放 `.c` / `.s`；把 `.h` 也塞进去会当源文件编译。

正确顺序：关掉/重载 EIDE 窗口 → 改 `eide.yml`（10 组：App/Periph/Sensor/Comm/Algorithm/Task/Ui/Startup/CubeMX/Drivers，73 条）
→ 用同一份清单重写 `builder.params` → 用下面的命令行构建验证 → 再回 EIDE 点构建。

### 4. 中断里绝不能放 `snprintf`、浮点长扫或 I2C I/O

中断里只有两档活：**2 ms 采样拍（TIM4）**读编码器、推进测速窗；**5 ms 控制拍（TIM7）**读红外
GPIO、一次转向计算、左右两个 PID、写 PWM。OLED 刷屏（1032 B ≈ 23 ms）走 I2C2 DMA 在主循环
推进，IMU 的 SPI 采样也在主循环。

两条硬经验（都是实测踩出来的）：
- M3 没有 FPU，`double` 全靠软件模拟，写进度环/测速会整段变慢（encoder.c 改 `float` 后少 1.2 KB、
  采样拍从 441 us 降到 134 us）。中断里只许 `float`，而且越少越好。
- 循环里的浮点乘加比整数贵一个数量级：测速窗的 100 项回溯改成整数后，同一处从 134 us 降到 50 us。

## 时序模型：两个独立硬件时基

| 定时器 | 配置 | 抢占 | 用途 |
| --- | --- | --- | --- |
| TIM4 | PSC 71 / ARR 1999 → **2.000 ms** | 0 | `sched_sample_isr()`：读编码器、推测速窗、置 IMU 标志 |
| TIM7 | PSC 71 / ARR 4999 → **5.000 ms** | 1 | `sched_control_isr()`：红外 → 循迹 → PID → PWM |
| TIM5 | PSC 71 / ARR 999 → 1 ms | 4 | 只 `milliseconds++`，不参与调度 |
| TIM1 / TIM2 / TIM3 | — | — | PWM / 编码器左 / 编码器右，不接管调度 |

启动顺序（`main.c`）：先双清 `TIM4->CNT` 与 `TIM7->CNT`，再 `HAL_TIM_Base_Start_IT(&htim4)`，
最后启 TIM7 —— 采样拍先跑，控制拍第一拍就有一份新鲜窗口。

两档不需要整除，也不需要公倍拍：采样拍随时插进控制拍之间，采样拍中断里已经把计数写进测速窗，
控制拍读窗口（`motion_snapshot()`，seqlock 保护）永远拿到自洽快照。

| 周期 | 任务 | 位置 |
| --- | --- | --- |
| 2 ms | `motion_update(now_us, dt_us)`：TIM2/TIM3 增量 → 时间加权测速窗 → `control_speed_task(dt_us)`（左右两个增量式 PI 直接出 PWM） | **TIM4 ISR** |
| 5 ms | `ir_read` → `ir_compute` → `mission_tick(dt_us)`：`track_task` 算转向、写左右目标 RPM | **TIM7 ISR** |
| 4 ms | IMU 采样（SPI 12 次单字节读约 25 us，ODR 416 Hz） | 主循环（采样拍置标志） |
| 20 ms | yaw + PID 目标/实际 → VOFA 文本（定点手拼，无 `%f`） | 主循环 |
| 100 ms | OLED 刷新：1 行大字 + 6 行小字（整帧 1025 B 走 I2C2 DMA） | 主循环 |
| — | 串口 RX（DMA+IDLE）/ TX（IT 128 B 分块）、命令行、OLED 状态机 | 主循环 |

**所有 dt 都是 DWT->CYCCNT 换算的真实微秒**（`sched_now_us()`，72 MHz、13.9 ns 分辨率、
59.6 s 回绕，一律用无符号减法求差），没有一处用"假设的 5 ms"。中断延迟、被更高优先级抢占的
时间都因此自动吸收进 PI 的 dt 里。

看门狗：IWDG 由 LSI 40 kHz / PSC 64 = 625 Hz 驱动，RLR 625 ≈ **1.0 s**（`main.c` 的 `watchdog_init()` 实测值，早期文档写的 1600 ≈ 2.56 s 是错的）；主循环每圈喂狗。

### 时间轴自检（全部是实测值）

全部走串口：每秒自动推两行中文（`q` 开关），`d` 打完整英文体检页：

| 字段 | 含义 |
| --- | --- |
| `smp_lat=/max` | TIM4 溢出 → ISR 入口延迟（`TIM4->CNT` 直读） |
| `ctl_lat=/max` | TIM7 溢出 → ISR 入口延迟（`TIM7->CNT` 直读） |
| `dt` / `dev` | 控制拍实测间隔 / 与 5 ms 的最大偏差 |
| `ctl=/max` / `ovr` | 控制拍耗时 / 超 `SCHED_OVERRUN_US`(1000 us) 的次数 |
| `smp=/max` | 采样拍耗时（含速度环） |
| `spd=/max` | 速度环单次耗时，跑在 2 ms 拍里，直接吃采样拍预算 |
| `win` / `retry` | 当前测速窗 ms / 快照撕裂回退次数 |
| `loop=/max` / `o1ms` | 主循环单圈耗时 / 超过 1 ms 的圈数 |
| `imu=/max` | IMU 标志置位 → `imu_task` 执行的排队延迟 |
| `rxov` / `txov` | 串口环形缓冲溢出计数，长期必须是 0 |

2026-11-06 静止实测基线：`smp_lat=1/2 ctl_lat=2/25 dt≈5000 dev=30 smp=50/50 ctl=25/25 ovr=0
loop=15/1784 imu=57/88`。

## 对外接口

**串口 USART1 / 115200**（8N1，接收 DMA + 空闲中断，发送 IT 128 B 分块）：

| 命令 | 作用 |
| --- | --- |
| `s` | 启动循迹任务（`mission_start_track()`） |
| `x` | 停车 / 中止任务（`mission_abort()`） |
| `y` | yaw 清零 |
| `m <左> <右>` | 手动**轮速目标 RPM**（不是占空比），例如 `m 60 60` |
| `k <KP> <KI>` | 在线改速度环增量式 PI 增益 |
| `G <KP> <KD> <KG>` | 在线改转向环：位置 P / 微分 D / 陀螺阻尼 |
| `L <LIM> <OUT>` | 差速整形：内轮减到 `1-LIM`、外轮只加 `OUT` 倍 |
| `p <0-3>` | 全白（丢线）策略：0=停 1=保持 2=圆弧 3=按角度转 |
| `r <DEG>` | 原地顺时针转 DEG 度（只用陀螺闭环，验证轴向/符号） |
| `t <圈数>` | 设目标圈数 1–99 |
| `w <ms>` | 改基础测速窗，5–200 ms |
| `f <Hz>` | 速度低通截止频率，0–100 Hz（0 = 关） |
| `o` | 打任务页：mode/stop/policy/line/lost、yaw/gy/kv/dist、三增益、mission 状态 |
| `d` | 打时间轴体检（5 行数据） |
| `q` | 开关每秒自动诊断推送（中文两行） |
| `?` | 帮助 |

三路串口都是 115200（USART1 调试口 / USART2 / USART3-PD8/PD9）。

**VOFA+ 用 FireWater 协议**，一行 **7 通道**，**定点手拼不用 `%f`**（ARMCC 的 `%f` 会链进完整浮点
printf，约 +6 KB Flash、单次 100–300 us）：

```
lt:55.0,la:54.3,rt:55.0,ra:54.8,yaw:12.34,st:0.42,gy:-31.5\n
```

通道顺序固定（顺序变了已保存的 VOFA 布局会错位）：`lt`/`la` 左轮目标+实际、`rt`/`ra` 右轮、
`yaw` 航向、`st` 转向量（-1..1，正=右转）、`gy` 车体 +Z 角速度 deg/s（右转为负）。

**按键**（CubeMX 已配，全部上拉输入、低有效，`Core/periph/key.c` 15 ms 去抖）：

| 键 | 引脚 | 作用 |
| --- | --- | --- |
| K1 | PG2 | **开始 / 停止**：在跑就停（等同 `x`），停着就按当前战术起跑（等同 `s`） |
| K2 | PG3 | **切战术包**：战术 1 循迹 ↔ 战术 2 步进转角（同时停掉正在跑的动作） |
| K3 | PD14 | **加量**：战术 1 目标圈数 +1；战术 2 每步角度 +45°（上限 360°） |
| K4 | PD15 | **姿态清零**：停车 + 圈数/弯数清零 + 航向归零 + 步进计数清零 |

战术包在 `Core/tactic/tactic.c`：战术 1 = 循迹（`mission_start_track`），战术 2 = 转到位后停 `TACTIC_STEP_GAP_MS`（默认 2000 ms）再原地顺时针转 `tactic_step_deg()`（默认 45°，K3 +45°）。串口等价命令：`T` 切战术、`A DEG` 设步长、`N` 加量。

`key_tick()` 挂在 5 ms 控制拍里只做扫描并生成「按下沿」位图，`key_handle()` 在主循环消费并动作；
中断里不调用 mission/track，避免与速度环抢状态。

**OLED 界面**：第 1 行 **8×16 大字**（16 字符）当战术条，其余 6 行 6×8 小字（21 字符）放数据。
为什么混排：跑车时人离车 2~3 m，6×8 读不出来；远处只需看清「哪个战术、步长多少、在不在跑」。

```
T1 TRACK   RUN             战术号 + 战术名 + 动作状态（IDLE/RUN/DONE/TOUT/ABRT）
T2 STEP 45  RUN            战术 2 紧跟步长（度），K3 每次 +45
IR 00111100  L 2/5         红外 8 路（1=压黑线，左→右）+ 圈数/目标
Y +12.3 P  -3.1 R  +0.8    三轴角度，±0.1°，IMU 未就绪时显示 ---
T  55/ 55  A  54/ 56       目标 RPM(左/右) | 实际 RPM(左/右)
PWM 62 63   D  12.40m      双轮占空比 % | 里程
S3 45d    gy -31.5         战术2：已完成步数+步长 | 车体角速度（战术1 换成 LAP RUN/DONE）
== MISSION DONE            事件横幅：丢线/陀螺校准 > 任务结束(2 s) > 切战术(1.5 s) > 提示
```

切战术时第一行整行换成 `>> TACTIC 2 STEP` 持续 1.5 s（`UI_BANNER_MS_TACTIC`），横幅同时提示；
横幅事件全部由 `display.c` 自己盯 `state()` 推出，任务层不知道 UI 存在（分层：ui 是侧枝）。

**诊断信息只走串口，不上屏**：`console_task()` 每秒自动往 USART1 推两行中文状态（`DIAG_PUSH_MS`
=1000，`0` 关闭，串口 `q` 临时开关），出问题时直接看收发区/抓日志；`d` 打完整时间轴体检、
`o` 打任务页。**串口助手要设 UTF-8 编码**，固件里的中文字面量是 UTF-8 字节。
AC5 对 UTF-8 字面量报 `#870-D`，已在 `builder.params` 的 C_FLAGS 加 `--diag_suppress=870`。

## 参数在哪

全部宏在 `Core/app/config.h`（79 个：应用层 29 + 驱动层 50）。改行为先改这里，不要在 .c 里写魔数。
转向符号约定写在 `track.c` 顶部注释里。

几个必须记住的：
- `TASK_SAMPLE_MS`=2 / `TASK_CONTROL_MS`=5 / `TASK_IMU_MS`=4（三个都靠 TIM4/TIM7 硬件时基保证）。
- `TICKS_OF_MS(ms)`：毫秒 → 控制拍数。`TRACK_LOST_HOLD_MS`、`LAP_CORNER_GAP_MS` 这类时间常数
  在代码里一律先换算成拍数再比较，中断里不做毫秒累加（也不调 `HAL_GetTick()`）。
- `SPEED_WINDOW_US`=20000（基础窗 20 ms）、`SPEED_MIN_COUNTS`=12（不够就自动扩窗到
  `SPEED_WINDOW_MAX_US`=200 ms）、`SPEED_FILTER_HZ`=10（一阶低通）。
- `SPEED_LOOP_IN_SAMPLE_ISR`=1：速度环挂在 2 ms 采样拍，测速与出 PWM 同一拍，不再隔 5 ms。
- `SCHED_OVERRUN_US`=1000：控制拍预算，超了 `ovr` 自增，长期必须为 0。
- 改周期宏后必须满足 config.h 末尾的 `#error` 断言（IMU/采样、VOFA/控制、OLED/控制 整除，
  `TELEMETRY_TEXT_SIZE` ≥ 96）。

## 控制算法（2026-11-06 版）

红外 8 路 → `ir_compute` 输出 `normalized_position ∈ [-1,1]`（0 = 线在正中，>0 = 线偏右）→
8 Hz 一阶低通微分 → 转向量 `u = (TRACK_KP·e + TRACK_KD·d + TRACK_GYRO_SIGN·TRACK_GYRO_KP·gyro_z) · kv`，
其中 `kv = TRACK_BASE_RPM / |v_actual|` 限幅 0.6~1.8（低速放大、高速收敛）→ 非对称差速整形
（内轮 `base·(1-|u|)`、外轮 `base·(1+TRACK_DIFF_OUTER·|u|)`，外轮少加防偏航过冲）→ 左右目标 RPM
→ 2 ms 增量式 PI：`Δu = KP·(e-e₋₁) + KI·e·dt`，`PWM += Δu`，限幅 ±100 %。

- **转向环是 PD + 陀螺阻尼，不用 I**：位置环加积分会在弯道累积、出弯甩头。陀螺直接测角速度，
  比"对位置差分"噪声小一个量级，所以 `TRACK_KD` 只留 0.02 做相位补偿，阻尼交给 `TRACK_GYRO_KP`。
- **速度环是增量式 PI**：状态即输出（PWM），天然抗积分饱和；`SPEED_PID_KD` 已删除——增量式的
  D 项作用在 Δ误差上，对 ±1 % 的测速噪声过于敏感，只会抖占空比。
- **入弯降速**：连续 `TRACK_CURVE_CONFIRM` 拍 |e| > `TRACK_CURVE_ENTER_E` 时基础速度乘
  `TRACK_CURVE_SCALE`，保持 `TRACK_CURVE_HOLD_MS` 后自动恢复。
- **全白四策略**（`p` 命令切换）：0 停（`STOP_LINE_LOST`）/ 1 保持上一拍转向 /
  2 圆弧（按入弯方向继续转，最长 `TRACK_ARC_MAX_DEG`）/ 3 按 `TRACK_CORNER_ANGLE_DEG` 用陀螺闭环转向。
- **里程**：`motion.c` 对滤波后轮速积分（左右平均、取绝对值），写 `motionView.distance_m`。
- **所有转角只有一个时钟原则，没有死超时**：转角到位判据是 `|err| < TRACK_TURN_TOL_DEG` 连续
  `TRACK_TURN_SETTLE` 拍；放弃判据是「不再靠近目标」——连续 `MISSION_SPIN_STALL_MS`（原地转）或
  `TRACK_ARC_STALL_MS`（丢线找线）误差没再缩小 `*_PROGRESS_DEG`，另外找线还有
  `TRACK_ARC_MAX_DEG` 这个物理角度上限。转 45° 和转 360° 走同一套判据，慢就多花时间，不会被钟判死。
- **IMU 丢帧不改状态**：SPI 单帧读失败保持 `IMU_ST_RUN` 与已标定的零偏，姿态停在上一帧，
  丢掉的这段时间留给下一帧积分；只有连续静默 `IMU_REPROBE_MS`(500 ms) 才回 OFF 重探。
  读晚 4 ms 就重走 2 s 沉降 + 标定，是把统计波动当成物理故障。
- **低速命令有一道 3 rpm 死区**：`SPEED_MIN_TARGET_RPM` 作用在**目标**上（不是斜坡后的命令），
  低于它的目标直接当 0。原因是电机最低稳速约 3.2 rpm，两轮阻力不等（右轮在同样占空比下快 1.01~1.17 倍），
  1 rpm 的命令只会让省力的那只轮子慢慢爬、另一只抖着不动，车身被带着微转。
  死区写在斜坡后的命令上会把斜坡钉死在 0（每拍只走 1 rpm 就立刻被清零，`m 30 -30` 完全不动）——已踩过。
- **停车用 TB6612 的 short brake**（`MOTOR_BRAKE_ON_STOP=1`，两根 IN 同高 = 绕组短接）而不是 coast，
  闭环运行时两者差别在采样噪声内（±30 rpm → 静止 0.47 s vs 0.44 s，速度环本来就用反向占空比在刹），
  但 IDLE / 急停这类不经速度环的停车，制动比滑行收得干净。
- **任务层 mission.c**：`s` 起循迹、`r DEG` 陀螺闭环原地转、`x` 中止；DONE/TIMEOUT/ABORTED 由 `o` 查看。
  `LAP_STOP_ENABLE`=0 → 达圈数不自动停（继续跑，避免比赛误停）。

## 已知待标定项（实车才能确认）

- `TRACK_KP=0.80 / TRACK_KD=0.02 / TRACK_GYRO_KP=0.0010` 的符号与大小：先用 `r 90` 验证陀螺轴向
  （车应顺时针转约 90°，误差 < 5°），再上赛道调 KP。
- `TRACK_GYRO_SIGN=-1`：车体 +Z 向上，右转为负角速度；原地转时越转越快就取反。
- `SPEED_PID_KP=0.45 / KI=0.12`：看 OLED 第 4 行 A 值（左/右实际）跟随 T 值（目标）的快慢与静差。
- `SPEED_DEADZONE_PCT=0`：用 `m 1 1` 实测能起转的最小占空比后填进去。
- `LAP_CORNER_ERROR=0.55` 的角点判定阈值；`TRACK_DIFF_LIMIT/OUTER` 决定弯道半径。
- 无堵转判定、无自动终点识别（这两项旧工程也没有真正实现）。

## 文档

`功能概览.md` 是落脚页；`README.md`、`代码阅读指南.md`、`DEBUG_LOG.md` 描述的是重写前的
旧架构，**冲突时一律以代码为准**，不要把里面的五模式/39 命令当真。

## 提交信息约定

中文 `类型：描述`，例如 `重构：按职责拆为 app/periph/sensor/comm/algorithm/task/ui 七类`。

## Claude Code 相关

- **权限白名单**：`MDK-ARM/.claude/settings.json` 与 `settings.local.json`（都被 git 跟踪）
- **clangd**：配置在**仓库根** `.clangd`，`Diagnostics` 有意设为 `Suppress: "*"`
  （clangd 会误解析 ARMCC 的 `__weak`/`__packed`），只用于导航；错误检查用上面的 `-fsyntax-only`
- **VSCode IntelliSense** 由 EIDE 提供（`C_Cpp.default.configurationProvider: "cl.eide"`）

### 任务 → 技能路由

| 任务 | 技能 |
| --- | --- |
| EIDE / ARMCC 构建失败 | `stm32-eide-build-debug` |
| Keil 工程、烧录 | `keil`、`openocd` |
| 串口日志、VOFA+ 数据 | `serial` |
| 改动审查 | `embedded-review` |
| 注释规范 | `embedded-code-commenting` |
| HAL/LL 用法 | `stm32-hal-ll-integration` |

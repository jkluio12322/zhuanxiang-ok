# BlueCar Debug Log

> **注意**：2026-10-02 的应用层重写改变了控制架构和循迹算法。
> 这条日期之前的记录描述的是**旧架构**（平均速度环 + PWM 空间差速、
> 权重和作为 PID 误差、前馈 `kf`、`skf/skfl/skfr` 命令等），
> 那些实现和命令现在都不存在了。重写后的说明以 `README.md` 为准。

## 2026-10-11: 代码文风简洁化（阶段一：压注释）

### 背景

对照逐飞风格的开源工程，`Core/Application` + `Core/Hardware` 的注释密度偏高：
8485 行里 3355 行是纯注释（39.5%），`@note` 354 处、`@brief` 444 处、分隔线 330+ 处。
本次把每个函数头的 Doxygen 块压成一行、删掉分隔线与复述代码的注释，
**把叙事性的设计史从代码里搬到这里** —— 代码里只留一行结论。

### 从代码里搬出的设计史

**状态与并发**

- 旧实现把 11 个全局变量外放在一个头文件里，被 7 个模块直接改写，出了问题无法判断是谁写的。
  现在写入口只有 `AppState_SetXxx()`。
- `AppState_Init()` 原实现漏了 `stopRequested` 清零，若在上电流程中再次调用会残留上一次的停车请求。
- `app_cmd.c` 里曾有一条被推翻的注释：「中断里 PRIMASK 本来就是 1」。Cortex-M3 上进 ISR 改的是
  **异常优先级掩码**，`__get_PRIMASK()` 在 ISR 里返回 0。临界区之所以仍然安全，靠的是保存-恢复
  本身，不是「中断里它已经是 1」。
- `AppConsole_Flush()` 早先套了一层 `__disable_irq()/__enable_irq()`：既用了盲 `enable`
  （从已关中断的上下文调过来会把中断错误地打开），又没有保护到任何东西，已删。
  `app_console.h` 里那句「响应发送期间短暂关闭中断」是同一处的过时描述。
- `app_telemetry.c` 早先的写法是**先清 fresh 再逐个读字段**，那样标志什么也没保护住 ——
  中断会在格式化期间继续改结构体，发出去的是新旧混排的一行。当时注释把竞争窗口估成 0.0005%，
  按「字段读取窗口 / 50 ms」重算是 0.2~0.6%，**差了约两个数量级**。现在是整体拷贝。
- USART1 的接收缓存与中断装填在早前的一次重构里从 `app_console.c` 移到了 `app_serial.c`
  （那里才有 `HAL_UART_RxCpltCallback`）；`app_console` 只负责拼行与解析。

**循迹误差的量纲**

- 误差信号从「八路权重和」换成归一化质心 `position`：权重和压 1 路最大 ±25、压 5 路能到 ±65，
  同一个增益要面对 2.6 倍的量纲漂移；`position` 已除以压黑路数，量纲恒定 ±7。
  单路压黑时两者比值约 25 : 7 ≈ 3.6，所以 `TRACK_PID_KP` 初值由 1.2 放大到 4.3（`Kd` 0.2 → 0.7）。
- `app_tracking.c` 每次模式复位重读 PID 配置是**兜底**，别删：限幅一旦被谁改过就得靠它恢复。
- 不要为测试另设一套限幅。早先那个做法会**原地改写共享 PID 的配置**，一旦某次模式切换
  没走到复位，真实循迹的差速上限就被永久改掉了。

**速度环与级联**

- 为什么是级联而不是「平均环 + PWM 差速」：差速放在**目标转速空间**是闭环的（某轮跟不上时
  它自己的速度环会把 PWM 顶上去补），直接加到 PWM 上是开环，轮子被卡住差速就失真。
- 为什么「禁止反转」要用 PID 的 `out_min` 而不是事后把负数夹成 0：增量式 PID 的「积分」
  就是输出本身，在外面改成 0 时内部输出仍会停在 -100，出弯要一路爬回来，表现为内轮反应迟钝。
- 增量式 PID 的累加器**必须钳位**：否则输出饱和期间累加器继续增长，等误差反向时要爬很久
  才回来 —— 这就是积分饱和。钳位之后，`out_min = 0` 才能让「禁止反转」真正生效。

**编码器滤波**

- 只用中值，不用一阶低通 EMA。原来的「中值-3 + EMA(0.75/0.25)」群延迟约 200 ms
  （4 个控制周期），是起步 judder 的成因之一；而原始信号本身只有 1~2% 量化噪声
  （2464 计数/圈，50 ms 窗口下 30 RPM 有 61 个计数）。现在延迟降到约 50 ms。
  那两个 EMA 权重宏随之删除 —— 它们零引用，留着只会让人以为滤波还是 EMA。

**其他已删除的旧方案**

- `app_pid.c` 取代原 `pid.c`：那里 `PID_t` 和 `TrackPid_t` 两套并行的
  init / set / reset / update，现在塌缩成一套。
- 增益不再支持在线修改（参数在 `app_config.h` 里改后重新编译），所以不存在
  「主循环改增益、中断读增益」的竞争；那套命令连带一串 `SetGain` 接口一并删除。
- PWM-TRACK 的模拟输入不再是「角度 + 量纲映射」：改为把八路图案直接注入驱动层，
  由驱动层算归一化质心当误差，所以不需要任何手工标定的满量程系数。

**保留在代码里的约定（这里只做索引，正文见对应文件）**

- `board.h`：八个引脚别名从原配置头文件逐字搬来。电机 A 用 `pwm1_0/pwm1_1`、
  电机 B 用 `pwm2_1/pwm2_0`（与 A 相反，B 轮正反转对调发生在**这一层**，不在 `motor.c` 里）。
  这套接线是实车调通的，改一个字轮子就可能反转。
- `app_motion.c`：`encoderRight` 取 TIM3、`encoderLeft` 取 TIM2（与 README 引脚表**相反**，
  以代码为准）；电机与编码器的交叉映射照抄自原 `yingYongKongZhi.c:353/443`，不要「理顺」。
- `app_types.h`：删命令时**不能对号入座地删枚举** —— 有些枚举值命令行已经不用了，
  但按键路径还在发。
- `app_testmode.c`：`AppTestMode_SetPattern()` 当前没有调用者（图案由 KEY2 经
  `AppTestMode_NextPattern()` 轮换），保留是为了以后要「直接指定某个图案」时有个现成入口。

### 验证

- ARMCC 全量重建：`0 Error(s), 0 Warning(s)`。
- `Program Size: Code=50708 RO-data=4032 RW-data=240 ZI-data=6920`，与改动前**完全一致**
  （注释不参与编译，这正是本阶段的安全网）。
- `app_config.h` 单行宏仍为 **83** 个（`tools/simulate_*.py` 靠正则解析它们，不能被破坏）。

### 同日续：目录拍平 + 去 APP 前缀（阶段二）

**背景**：`Core/Application/` 下 7 个子目录 + `Core/Hardware/Peripherals/{Display,IO,Motion,Sensors}`
共 14 个叶子目录、最深三层；函数名一律带 `App` 前缀、文件名一律带 `app_`。

**改动**：拍成五层平铺，文件名/函数名/宏名一起去前缀。

```
Core/user/         main.c  main.h  isr.c  isr.h
Core/system/       config.h  types.h  board.h  sched  state  timer  filter  ringbuf  bytestream
Core/interaction/  console  cmd  serial  telemetry  display  key
Core/logic/        pid  track  testmode  speed_loop  motion
Core/hardware/     motor  encoder  irtrack  mpu6050  imu  oled(+字模)  buzzer
Core/Src + Core/Inc  CubeMX 生成，保持原样
```

命名规则：`AppScheduler_OnTick` → `sched_tick`、`AppMotion_Apply` → `motion_apply`、
`Motor_SetSpeedA` → `motor_a_set`、`OLED_ShowString` → `oled_show_string`、
`APP_CONTROL_PERIOD_MS` → `CONTROL_PERIOD_MS`。寄存器宏（`MPU6050_WHO_AM_I`、
`OLED_ADDR` 等）和 CubeMX 引脚宏不动。

**踩到的坑，记下来免得下次再犯**：

- `bluecar.uvprojx` 与 `.eide/eide.yml` 都改了；但**命令行 `unify_builder` 不读 eide.yml**，
  它读的是 VSCode 扩展生成的 `build/bluecar/builder.params`，命令行构建时必须手工同步。
- `Core/Hardware/` → `Core/hardware/` 是仅大小写的重命名，Windows 文件系统不区分，
  必须 `git mv Core/Hardware Core/hw_tmp` 再 `git mv Core/hw_tmp Core/hardware` 两步走。
- `Core/Src/{gpio,tim,usart}.c` 也是 `#include "app_config.h"`，改名时容易漏。 
- `MX_TIM5_Init` 与要改的 `TIM5_Init` 只差一个 `MX_` 前缀，替换规则必须带词边界。

**验证**：`Code=50700 RO-data=4032 RW-data=236 ZI-data=6916`，`0 Error(s), 0 Warning(s)`。
与改动前的 `50708/4032/240/6920` 相差几字节，**这不是语义变化** ——
对比两次构建的 map，每个 `.o` 的 Code/RO/RW/ZI 完全相同，变的是
`(incl. Padding)` 行（RW 14→10、ZI 5→1），即段名变化导致链接器段序改变、
段间对齐填充随之变化。两个主机侧仿真（`simulate_tracking.py` / `simulate_pwm_tests.py`）
输出与改动前一致。

## 2026-10-02: 应用层重写

### 背景

应用层是逐次叠加出来的：一个 1163 行的循迹状态机、28 个文件级 `static`、
一个被 7 个模块直接改写的全局状态头文件，以及跑在最高优先级中断里的
遥测 `snprintf` 和 MPU6050 姿态解算。文档也与代码脱节（描述过航向环、
速度环死区/限速率/低速补偿、堵转保护，代码里都没有）。

### 改动

**保留驱动层不动**（只改了 `timer.c` 的中断体和 `motor.c` 的一行 include）。

1. **中断瘦身**：`HAL_TIM_PeriodElapsedCallback` 从 40 行减到 2 行，只剩
   毫秒计数和 `AppScheduler_OnTick()`。遥测 `snprintf`（100–300 µs）和
   OLED 的 I2C 移出中断，改由主循环的延后任务执行。
   依据是 USART1 一个字节只要 86.8 µs，中断里跑这么长的活会压住串口接收。

2. **状态唯一化**：`yingYongZhuangTai.c/.h` 那套外放全局变量被
   `State/app_state.c/.h` 取代——只读走 `AppState()` 视图，写走
   `AppState_SetXxx()`，没有 `extern` 变量。状态模块只持有数据，
   启停编排留在控制层，避免 `app_state` 和 `app_testmode` 互相依赖成环。

3. **循迹算法重写**（对齐参考实现 `32_XJPIDJX`）：
   - PID 误差从**未归一化**的权重和改为归一化质心 `e_y`。权重和压 1 路最大
     ±25、压 5 路到 ±65，同一增益要面对 2.6 倍量纲漂移；`e_y` 恒定 ±7。
   - 控制链改为**级联**：循迹环出目标转速，左右速度环各自闭环到 PWM。
     旧代码在寻迹模式下只用平均速度环，差速是开环加在 PWM 上的。
   - 循迹环改为 **PD**（`Ki = 0`），速度环保持 **PI**（`Kd = 0`）。
   - 增益按新量纲折算：`Kp` 1.2 → 4.3，`Kd` 0.2 → 0.7。
   - 状态机从 7 个状态减到 4 个，静态状态变量从 28 个减到 11 个。
   - 计圈改为数「`e_y` 偏到一侧」的事件；起点横线因 `e_y ≈ 0` 天然不计。

4. **编码器滤波去掉 EMA**：原来在编码器速度上手写了中值-3 + EMA(0.75/0.25)，
   按 50 ms 周期算群延迟约 200 ms（中值 50 ms + EMA 150 ms），等于 4 个控制
   周期，很可能就是旧记录里那个「起步一格一格 judder」的成因之一。
   而原始信号量化噪声只有 1~2%（2464 计数/圈，50 ms 窗口下 30 RPM 有 61 个
   计数）。现在只保留中值-3，延迟降到约 50 ms。

5. **删蓝牙**：HC-05 那条链路（`blueserial.c/h`、帧解析、空壳函数、
   `main.c` 调用）整条删除。原在 `blueserial.c` 里的
   `HAL_UART_RxCpltCallback` / `HAL_UART_TxCpltCallback` 已搬进
   `app_serial.c`——**这是删除时最需要注意的一步**，不搬走的话 USART1
   发满 256 字节后会停。

6. **通信层重命名**：`serial`→`app_serial`、`pid_tune`→`app_console`、
   `yingYongMingLing`→`app_cmd`、`yingYongYaoCe`→`app_telemetry`；
   `yingYongXunJi` 并入 `app_tracking`（1163 行 → 约 290 行）。

7. **文档修正**：
   - §2.1 编码器引脚表原来写「左轮 TIM3、右轮 TIM2」，实际代码是
     `encoderRight` 取 TIM3、`encoderLeft` 取 TIM2，**写反了**，已按代码更正。
   - §8 遥测原来写 14 字段，实际代码发 **16 字段且顺序不同**，已更正；
     表头改为由 `AppTelemetry_BootHeader()` 统一发出。
   - 删掉蓝牙章节；航向环、速度环死区/限速率/低速补偿、堵转保护改记为
     「未实现」。

### 验证

- Keil 全量重建 `0 Error(s), 0 Warning(s)`。
- 编译产物从 Code 52780 / ZI 7196 降到 Code 50668 / ZI 6920。
- `tools/simulate_tracking.py` 重写为新算法，22 个场景全过，含
  「斜穿起点横线不计圈」「连续 4 圈报 4 圈而非 5 圈」。
- **实车回归尚未执行**，见 README §9.5 的调参顺序。

## 2026-09-23: Speed PID Reverse Saturation at Zero Target

### Symptom

- VOFA waveform showed the left and right actual speeds falling from a
  positive target to about `-140 RPM`.
- The reverse motion happened on both wheels and was sustained, so it was not
  a single encoder spike.
- The fault became obvious when the speed target returned to zero.

### Root Cause

The speed loop uses an incremental PID:

```text
deltaOut = Kp * (e0 - e1) + Ki * e0 + Kd * (e0 - 2*e1 + e2)
Out      = Out + deltaOut
```

When the target changed from `+70 RPM` to `0 RPM`, the error became about
`-70 RPM`. The loop generated a negative output increment and accumulated
negative PWM while trying to reduce the measured speed.

When the measured speed entered the `1.8 RPM` PID deadband, the output
increment stopped, but the existing negative `Out` remained. The negative PWM
continued to drive the motor in reverse until the measured speed reached the
reverse maximum.

The speed PID has no feedforward term in the current implementation:

- `Kf` is not used by `PID_IncrementalUpdate()`.
- Both speed PID objects initialize `Kf` to `0`.
- The historical `Kf=0.69/0.65` implementation is not the cause of this
  reverse saturation.

### Fix

File: `Core/Application/yingYongKongZhi.c`

Added `SPEED_PID_STOP_HOLD_RPM = 0.5f` and
`appControlConstrainSpeedPidOutput()`.

The new constraint is applied after each incremental PID update:

- A target whose absolute value is at most `0.5 RPM` forces PWM to zero.
- A positive target cannot output negative PWM.
- A negative target cannot output positive PWM.
- PID state reset on target direction changes remains enabled.

This prevents the speed loop from driving opposite to its target. The tradeoff
is that a zero target now coasts instead of using reverse PWM for active
braking.

### Verification

- Keil build: `0 Error(s), 0 Warning(s)`.
- Firmware: `MDK-ARM/bluecar/bluecar.hex`.
- Real-car test: passed; the previous reverse saturation no longer occurs.

### Regression Check

If a wheel still reaches a sustained reverse maximum while the corresponding
target is zero:

1. Confirm the flashed firmware is the build containing
   `appControlConstrainSpeedPidOutput()`.
2. Add left and right PWM channels to VOFA telemetry.
3. If PWM is zero but measured speed is still a sustained negative maximum,
   check motor driver polarity and encoder sign.

### Follow-Up: Pivot Turn With One Positive And One Negative Target

The first zero-target guard prevented all PWM opposite to a nonzero target.
That was too restrictive for a pivot turn using `+40 RPM` on the outer wheel
and `-40 RPM` on the inner wheel. If the inner wheel overshot its negative
target, the PID could not apply positive braking PWM to bring it back.

The guard was temporarily revised to allow opposite-direction PWM up to 25%.
Hardware testing showed that this hard 25% brake clamp caused visible wheel
judder, so the clamp was removed. The current rules are:

- A target at or near zero still forces PWM to zero.
- For a nonzero target, the full normal `-100%...+100%` PID output range is
  available, so braking through zero stays continuous.

### Current Issue: Actual Wheels Rotate With A Start-Stop Judder

The incremental speed loop had `Kf = 0`. It therefore had to accumulate PWM
from zero through the motor and gearbox stiction region whenever a target
started. During that region the wheel could catch, move briefly, stall again,
and catch again, which appeared as a one-step-at-a-time judder.

The old position PID used `Kf = 0.69` for the left wheel and `Kf = 0.65` for
the right wheel. Incremental feedforward was first added with those values, but
the defaults are now temporarily set to zero for an A/B hardware test:

```text
SPEED_PID_LEFT_KF  = 0.0f
SPEED_PID_RIGHT_KF = 0.0f
```

The feedforward calculation and runtime commands remain in place. Setting the
defaults to zero isolates whether the actual-wheel judder comes from
feedforward or from the incremental PI state.

Runtime tuning is available through USART1:

```text
skf 0.65
skfl 0.69
skfr 0.65
```

Build result after temporarily disabling feedforward:
`0 Error(s), 0 Warning(s)`.

Hardware re-test with straight motion, zero-target braking, and pivot turning
is pending. Restore feedforward online with:

```text
skfl 0.69
skfr 0.65
```

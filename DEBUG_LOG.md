# BlueCar Debug Log

> **注意**：2026-10-02 的应用层重写改变了控制架构和循迹算法。
> 这条日期之前的记录描述的是**旧架构**（平均速度环 + PWM 空间差速、
> 权重和作为 PID 误差、前馈 `kf`、`skf/skfl/skfr` 命令等），
> 那些实现和命令现在都不存在了。重写后的说明以 `README.md` 为准。

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

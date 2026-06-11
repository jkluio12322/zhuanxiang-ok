# BlueCar PID 调参指南

## 硬件概览

| 项目 | 参数 |
|------|------|
| MCU | STM32F103ZET6, 72MHz |
| 电机 | JGB37-520 (30:1 减速箱, 11 PPR 编码器) |
| 编码器 | TIM2/TIM3 编码器模式, 4 倍频, 有效分辨率 1320 CPR |
| 驱动 | TB6612, TIM1 PWM 244Hz (ARR=4095, 乘 40 映射) |
| IMU | MPU6050, I2C2, 陀螺仪 ±2000dps, 125Hz 采样 |
| 控制周期 | 内环 50ms, 外环 100ms |
| 轮径 | 65mm |

---

## 当前状态 (2026-06-10)

### SpeedPID (内环, 50ms) — 运行中

```
Kp=0.2   Ki=0.06   Kd=0.1   Kf=0.30
OutMax=+100        OutMin=-100
OutOffset=15.0     IntegralLimit=35.0
Deadband=3.0       OutputRateLimit=12.0
```

**公式**: `Out = Kp*e + Ki*Σe - Kd*ΔActual + Kf*Target + OutOffset`

**注意**: main.c 中结构体初始化写的是 Kp=0.8/Kd=0.6/Kf=0，但 `PID_Init()` 之后的赋值覆盖为上述值。**实际生效的是上面这组**。

### YawPID (外环, 100ms) — 已禁用

```
Kp=0.0   Ki=0.0   Kd=0.0   (全部归零)
OutMax=+20         OutMin=-20
OutOffset=0        IntegralLimit=40.0
Deadband=1.0       OutputRateLimit=10.0
```

`main.c:466` 行 `float diff_yaw = 0; //YawPID.Out;` — 输出硬编码为 0，先调 SpeedPID。

---

## 串口调参命令 (USART3, 115200bps)

### SpeedPID 参数

| 命令 | 说明 | 示例 |
|------|------|------|
| `skp N` | Kp 比例增益 | `skp 0.3` |
| `ski N` | Ki 积分增益 | `ski 0.04` |
| `skd N` | Kd 微分增益 | `skd 0.15` |
| `skf N` | Kf 前馈增益 | `skf 0.30` |

### YawPID 参数

| 命令 | 说明 | 示例 |
|------|------|------|
| `ykp N` | Kp 比例增益 | `ykp 0.3` |
| `yki N` | Ki 积分增益 | `yki 0.02` |
| `ykd N` | Kd 微分增益 | `ykd 0` |
| `ykf N` | Kf 前馈增益 | `ykf 0` |


### 电机控制

| 命令 | 说明 | 示例 |
|------|------|------|
| `t N` | 速度目标 (RPM) | `t 50` |
| `yaw N` | 航向目标 (°) | `yaw 0` |
| `turn N` | 手动差速 (RPM) | `turn 10` |
| `start` / `stop` | 启停 | |
| `duty N` | 开环 PWM (-100~100) | `duty 50` |
| `show` | 查看所有参数 | |
| `help` | 帮助 | |

### 调参后须修改代码 (持久化)

串口调参只在内存生效，断电丢失。确定最优参数后，须手动更新 `main.c` 中 `PID_Init()` 后面的赋值，然后重新烧录以持久化。

---

## VOFA+ 数据格式

USART3 每 50ms 发送一行 CSV (FireWater 协议):

```
Target%, RightRPM%, LeftRPM%, PWM%, YawOut, TurnTarget, WheelDiff, YawAngle, BT_LV, BT_RH, RunFlag
```

| 列号 | 字段 | 含义 |
|------|------|------|
| 1 | Target% | 目标 RPM 占当前档位百分比 |
| 2 | RightRPM% | 右轮 RPM 占当前档位百分比 |
| 3 | LeftRPM% | 左轮 RPM 占当前档位百分比 |
| 4 | PWM% | SpeedPID 实际输出 |
| 5 | YawOut | YawPID 输出 (当前恒为 0) |
| 6 | TurnTarget | 手动差速目标 |
| 7 | WheelDiff | 左右轮 RPM 差 |
| 8 | YawAngle | 偏航角 (°) |
| 9 | BT_LV | 蓝牙摇杆前后值 |
| 10 | BT_RH | 蓝牙摇杆转向值 |
| 11 | RunFlag | 启停标志 |

VOFA+ 连接 USART3 串口后，拖入波形控件，重点观察 **Target% (目标)** 与 **RightRPM%/LeftRPM% (实际)** 的跟踪关系，以及 **PWM% (输出)** 的稳定性。

---

## 调参顺序

### 阶段一：SpeedPID (当前)

```
目标: 空载/负载下 RPM 跟踪快速、无超调、稳态无静差
方法: 串口发 t 命令给阶跃目标，VOFA+ 观察波形
```

#### 步骤 1.1 — 测开环特性 (已完成)

用 `duty N` 命令测不同占空比对应的稳态 RPM，估算 Kf 基准线:

| PWM% | 稳态 RPM (估) | Kf = PWM / RPM |
|------|---------------|-----------------|
| 30 | ~100 | ~0.30 |
| 50 | ~165 | ~0.30 |
| 70 | ~230 | ~0.30 |

当前 Kf=0.30 偏保守，稳态约补 30%，余量靠 I 项补齐。

#### 步骤 1.2 — Kp 粗调

```
duty 0         # 先停车
t 50           # 给 50 RPM 阶跃
```

从当前 Kp=0.2 开始，每次加 0.1:

| Kp | Ki | Kd | 现象 |
|----|----|----|------|
| 0.2 | 0.06 | 0.1 | **当前值** |
| 0.3 | 0.06 | 0.1 | |
| 0.4 | 0.06 | 0.1 | |
| 0.5 | 0.06 | 0.1 | 注意震荡临界点 |

**判断标准**:
- 上升时间 < 500ms (10 个控制周期) 达标
- 有轻微过冲 (< 5 RPM) 可接受，Kd 会抑制
- 出现持续等幅震荡 → Kp 过大，降 20%

#### 步骤 1.3 — Kd 微调

在 Kp 确定后，调 Kd 抑制过冲:

```
skd 0.15       # 从 0.1 开始加
skd 0.2
skd 0.3
```

| Kp | Ki | Kd | 现象 |
|----|----|----|------|
| ? | 0.06 | 0.1 | 当前 |
| ? | 0.06 | 0.2 | |
| ? | 0.06 | 0.3 | |

**判断标准**:
- 过冲 < 3 RPM
- 无二次震荡
- 上升时间未显著变慢 (Kd 过大会让响应变迟钝)

#### 步骤 1.4 — Ki 校准

在 P/D 稳定后，调整 Ki 消除静差:

```
ski 0.02        # 降低积分速度
ski 0.04
ski 0.08        # 当前 0.06
```

**判断标准**:
- 稳态误差 < 2 RPM
- 目标归零后不反向冲
- 无低频摆动 (Ki 过大会产生 1~2Hz 慢摆)

#### 步骤 1.5 — 约束参数调整

| 参数 | 当前 | 调法 |
|------|------|------|
| OutOffset | 15 | 开环测最小转动 PWM，设为该值 + 2 |
| Deadband | 3 | 设为编码器噪声幅度的 1.5 倍 |
| IntegralLimit | 35 | 设为 (100 - Kf*MaxRPM) * 0.8 |
| OutputRateLimit | 12 | 设为你接受的每 50ms 最大 PWM 变化 |

---

### 阶段二：YawPID (SpeedPID 锁定后)

```
前提: SpeedPID 调好，车能稳速直行
方法: 先恢复 diff_yaw = YawPID.Out 那行代码，再调参
```

#### 步骤 2.1 — 恢复代码

在 `main.c:466` 行，把:
```c
float diff_yaw = 0;  //YawPID.Out;
```
改为:
```c
float diff_yaw = YawPID.Out;
```

#### 步骤 2.2 — YawPID 粗调

```
t 30           # 低速巡航
ykp 0.2        # 从很小开始
yki 0.01
```

观察 YawAngle 通道:
- 直行时偏航角漂移速度
- YawPID 介入后拉回速度
- 是否出现左右摇摆 (震荡)

#### 步骤 2.3 — 转向叠加测试

```
t 30
turn 15        # 手动差速转弯
# 松手后: stop → t 30 → 观察 YawPID 是否拉回航向
```

---

## 调参日志

### 第 N 轮 — 日期

| Kp | Ki | Kd | Kf | 改动说明 |
|----|----|----|-----|----------|
| | | | | |

**VOFA 观察**:

```
(粘贴 VOFA 关键数据段)
```

**结论**:

---

## 常见问题

### 1. RPM 摆幅大 (>50 RPM 来回跳)
→ Kp 过大，或 Kd 不够。先降 Kp 20%，再加 Kd。

### 2. 稳态有持续静差 (>5 RPM)
→ Ki 不够大或 IntegralLimit 太小。检查 `show` 命令的 Intgl 值是否顶到限幅。

### 3. 输出在 0 附近来回跳，车不动
→ OutOffset 偏小，PWM 没突破 TB6612 死区。加大 `soff` 1~2 个单位。

### 4. 刹车时车反向冲一下
→ Target=0 时积分清零 (已实现)，检查 IOut 是否过大。必要时减小 IntegralLimit。

### 5. 加速太猛/太肉
→ 太猛: 加大 `srl` 或提高 Kp；太肉: 减小 `srl` 或提高 Kp。

### 6. 左右轮 RPM 不一致
→ 编码器取反正确性检查: 前进时 L>0, R>0。若符号不对，检查 TIM3 取反逻辑。

---

## 文件索引

| 文件 | 作用 |
|------|------|
| `Core/Hardware/pid.c` | PID 算法 (8 步处理) |
| `Core/Hardware/pid.h` | PID 结构体定义 |
| `Core/Hardware/pid_tune.c` | 串口调参命令 |
| `Core/Hardware/encoder.c` | JGB37-520 编码器驱动 |
| `Core/Hardware/motor.c` | TB6612 PWM 驱动 |
| `Core/Hardware/timer.c` | TIM5/TIM6 时基 |
| `Core/Src/main.c` | 串级 PID 控制循环 (核心) |
| `设计文档.md` | 整体设计文档 |
| `串级PID方案.md` | 串级控制架构方案 |
| `PID改进方案.md` | PID 算法改进记录 |
| `PID_TUNING_LOG.md` | 历史调参记录 |

# BlueCar PID 参数详解

## 架构总览

```
串级 PID (3 个控制器):

PulsePID (10ms)          SpeedPID (50ms)          YawPID (100ms)
脉冲率 → 基础PWM         平均RPM → PWM修正        偏航角 → 差速纠偏
[已停用, 见备注]          [正在使用]               [已关闭, Kp/Ki/Kd=0]
```

## 控制周期

| 控制器 | 调用频率 | 调用位置 | 物理含义 |
|--------|---------|---------|---------|
| PulsePID | 10ms | TIM6 ISR → Task_Pulse_10ms | 编码器脉冲率高速闭环 |
| SpeedPID | 50ms | TIM6 ISR → Task_Control_50ms | 平均转速闭环 |
| YawPID | 100ms | Task_Control_50ms 内部分频 | 航向角闭环(2×分频) |

---

## PID 结构体完整字段 (pid.h)

```c
typedef struct {
    // ── 输入/输出 ──
    float Target;           // 设定目标
    float Actual;           // 当前实际值
    float Actual1;          // 上次实际值(供 D 项)
    float Out;              // 最终输出

    // ── 增益系数 ──
    float Kp, Ki, Kd, Kf;   // P/I/D/前馈

    // ── 误差 ──
    float Error0;           // 本次误差 = Target - Actual
    float Error1;           // 上次误差

    // ── 各分量(调试用) ──
    float POut, IOut, DOut; // P/I/D 三项输出

    // ── 限制参数 ──
    float OutMax;           // 输出上限
    float OutMin;           // 输出下限
    float OutOffset;        // 死区补偿值
    float IntegralLimit;    // 积分限幅
    float Deadband;         // 积分死区
    float OutputRateLimit;  // 输出变化率限制
    float LastOutput;       // 上次输出(供变化率计算)
} PID_t;
```

---

## SpeedPID (内环, 50ms) — 平均转速 → PWM 占空比

物理链路: `RPM误差 → PWM%输出 → TB6612 → 直流电机`

| 字段 | 当前值 | 最小值 | 最大值 | 物理含义 |
|------|--------|--------|--------|---------|
| **Kp** | 0.5 | 0.0 | 2.0 | 比例增益。每 1RPM 误差产生 `Kp` 单位 PWM 输出。>0.5 易震荡 |
| **Ki** | 0.1 | 0.0 | 0.5 | 积分增益。每周期累积 `Ki×误差`, 消静差。>0.3 积分抢戏振荡 |
| **Kd** | 0.3 | 0.0 | 1.0 | 微分增益。抑制 RPM 突变，`-Kd×ΔRPM`。0.3 已经较大(微分先行) |
| **Kf** | 0.0 | 0.0 | 1.0 | 前馈增益。`Kf×Target` 直接叠加。当前=0 已关闭 |
| **Target** | 运行时变 | -350 | +350 | 目标 RPM（档位6满速=350RPM）|
| **Actual** | 运行时变 | -400 | +400 | 实际平均 RPM（编码器读数，含超调）|
| **Out** | 运行时变 | -80 | +80 | 最终输出 PWM%（=实际占空比）|
| **POut** | 运行时变 | -100 | +100 | P 项输出 = Kp×Error (可超输出限幅，会后续钳位) |
| **IOut** | 运行时变 | -50 | +50 | I 项输出（受 IntegralLimit 限制）|
| **DOut** | 运行时变 | -400 | +400 | D 项输出 = -Kd×ΔRPM (50ms 内 RPM 可跳 200，D 项峰值=0.3×400=120) |
| **OutMax** | **+80** | 0 | +100 | 正向输出上限 PWM%（物理边界±100）|
| **OutMin** | **-80** | -100 | 0 | 反向输出下限 PWM% |
| **OutOffset** | **0** | 0 | 30 | 死区补偿：Out>0 时 +Offset, Out<0 时 -Offset。当前=0 关闭 |
| **IntegralLimit** | **50** | 0 | 100 | 积分限幅绝对值。IOut 夹在 ±50。50×0.1=5%PWM 积分最多贡献 5% |
| **Deadband** | **2** | 0 | 20 | 积分死区（RPM）。\|误差\|≤2 时不积分，防编码器抖动 |
| **OutputRateLimit** | **0** | 0 | 100 | 输出变化率限制。0=关闭。每 50ms 最多变 ±N PWM% |

### SpeedPID 计算公式

```
Error0 = Target - Actual
POut   = Kp × Error0
IOut   += Ki × Error0  (|误差|>Deadband 时, 且夹在 ±IntegralLimit)
IOut   = 0  (Target==0 或 Ki==0 时)
DOut   = -Kd × (Actual - Actual1)   ← 微分先行
Out    = POut + IOut + DOut + Kf×Target
Out    += ±OutOffset (死区补偿, Out>0 则 +OutOffset)
Out    夹在 [OutMin, OutMax]
Out 变化量夹在 ±OutputRateLimit
```

---

## YawPID (外环, 100ms) — 偏航角 → 差速 RPM

物理链路: `角度误差 → 差速RPM → 叠加到左右轮`

| 字段 | 当前值 | 最小值 | 最大值 | 物理含义 |
|------|--------|--------|--------|---------|
| Kp | **0.0** | 0.0 | 2.0 | 角度误差增益 (°/RPM) |
| Ki | **0.0** | 0.0 | 0.5 | 角度积分 |
| Kd | **0.0** | 0.0 | 1.0 | 角速率微分 |
| Kf | 0.0 | 0.0 | 1.0 | 前馈 |
| OutMax | **+20** | 0 | +50 | 差速上限 RPM |
| OutMin | **-20** | -50 | 0 | 差速下限 RPM |
| OutOffset | **0** | 0 | 10 | 死区补偿（差速不需要）|
| IntegralLimit | **40** | 0 | 100 | 积分限幅 |
| Deadband | **1** | 0 | 10 | 积分死区（°）|
| OutputRateLimit | **10** | 0 | 50 | 变化率限制 |

> **当前状态: 已关闭 (Kp=Ki=Kd=0)**

---

## PulsePID (内环, 10ms) — 编码器脉冲率 → 基础 PWM

物理链路: `编码器脉冲率 → 高速 PWM 基底 → SpeedPID 在此基础上修正`

| 字段 | 当前值 | 最小值 | 最大值 | 物理含义 |
|------|--------|--------|--------|---------|
| Kp | 2.0 | 0.0 | 5.0 | 脉冲率比例增益 |
| Ki | 0.1 | 0.0 | 2.0 | 脉冲率积分 |
| Kd | 0.3 | 0.0 | 2.0 | 脉冲率微分 |
| Kf | **2.5** | 0.0 | 5.0 | 前馈(目标脉冲→PWM 直接映射) |
| OutMax | **+100** | 0 | +100 | 输出上限 |
| OutMin | **-100** | -100 | 0 | 输出下限 |
| OutOffset | **15** | 0 | 30 | 死区补偿(TB6612 硬件死区 ~15%) |
| IntegralLimit | **15** | 0 | 50 | 积分限幅 |
| Deadband | **1** | 0 | 10 | 脉冲率积分死区 |
| OutputRateLimit | **20** | 0 | 100 | 变化率: 每 10ms 最多变 ±20% |

> **当前状态: 代码已定义但主控制链路中未使用**
> PulsePID 在 TIM6 ISR 的 `Task_Pulse_10ms()` 中调用, 但当前 SpeedPID 直接输出到电机, PulsePID 的输出未被叠加。

---

## 档位速度映射

| 档位 | 目标 RPM | BT_LV=100 时 PWM≈% |
|------|---------|-------------------|
| 1 | 20 | ~15-20 |
| 2 | 60 | ~40-50 |
| 3 | 120 | ~60-75 |
| 4 | 180 | ~75-90 |
| 5 | 250 | ~85-100 |
| 6 | 350 | ~100(满速) |

### 电机死区实测

TB6612 驱动 JGB37-520 (30:1 减速箱): 占空比 <15% 时电机不转。

OutOffset 设为 15 意味着: PID 算出 Out=5 → 实际发送 5+15=20 PWM，刚好突破死区。

---

## 系统输出限制

| 限制 | 值 | 位置 | 说明 |
|------|-----|------|------|
| SpeedPID.OutMax/Min | ±80 | PID 第8步 | PID 内部限幅 |
| L/R 钳位 | ±100 | Process_Control | 最终 PWM 绝不超 ±100 |
| 编码器 RPM 范围 | ~±400 | 物理 | 电机空载最高 ~400 RPM |
| PWM 实际范围 | 0~4000 | TIM1 CCR | ARR=4096, Speed×40 |

# BlueCar V2.1 — 蓝牙遥控四轮小车 代码总结

## 1. 硬件总览

| 模块 | 型号/参数 | STM32 接口 |
|------|-----------|------------|
| MCU | STM32F103ZET6, 72MHz | — |
| 左电机 | TB6612 A通道 | PWM: PE11(TIM1_CH2), 方向: PF1/PF2 |
| 右电机 | TB6612 B通道 | PWM: PE9(TIM1_CH1), 方向: PF3/PF4 |
| 左编码器 | JGB37-520, 30:1 减速比 | TIM3 编码器模式: PA6(CH1), PA7(CH2) |
| 右编码器 | JGB37-520, 30:1 减速比 | TIM2 编码器模式: PA0(CH1), PA1(CH2) |
| 陀螺仪 | MPU6050, ±2000°/s | I2C1: PB6(SCL), PB7(SDA) |
| 蓝牙 | HC-05 | USART2: PA2(TX), PA3(RX), 9600bps |
| 调参串口 | USB-TTL | USART3: PD8(TX), PD9(RX), 115200bps, DMA已配 |
| OLED | 128×64, SSD1306 | I2C2: PB10(SCL), PB11(SDA) |
| 按键 | 4键, 上拉输入 | PC6/PC7/PC8/PC9 |
| 蜂鸣器 | 有源 | PG6 |

### 按键映射

| 逻辑名 | 引脚 | 功能 |
|--------|------|------|
| KEY_1 | PC8 | 单击: 切换 RUN/PWM_SET 模式 |
| KEY_2 | PC7 | PWM模式: +10% / RUN模式: 按住前进(百分比递增) |
| KEY_3 | PC6 | PWM模式: -10% / RUN模式: 按住后退(百分比递增) |
| KEY_4 | PC9 | 单击: 循环升档 1→6, 复位百分比=50 |

### 速度档位

| 档位 | 1 | 2 | 3 | 4 | 5 | 6 |
|------|---|---|---|---|---|---|
| RPM | 20 | 60 | 120 | 180 | 250 | 350 |

### PWM 频率

TIM1: `72MHz / 72 / 4096 ≈ 244Hz` (ARR=4095, PSC=71)

---

## 2. 控制架构 — 串级 PID

```
SpeedTarget ─→ SpeedPID(内环 50ms) ─→ base PWM (-100~+100)
                    ↑
                 平均转速 = (左RPM + 右RPM) / 2

YawTarget ─→ YawPID(外环 100ms) ─→ diff_yaw (暂关=0)
                    ↑
                 MPU6050 偏航角

BT摇杆/按键 ─→ TurnTarget ─→ diff_manual (直接叠加, 不走PID)

合成:  L = clamp(base + diff, ±100)
       R = clamp(base - diff, ±100)
       diff = diff_manual + diff_yaw
```

- **内环 SpeedPID (50ms)**：左右轮平均转速闭环，输出 PWM 占空比 -100~+100
- **外环 YawPID (100ms)**：航向角闭环纠偏，输出差速 RPM ±20。当前暂关 (`diff_yaw=0`，注释掉 YawPID.Out)
- **手动差速**：TurnTarget 由蓝牙 RH / 按键直接设定，不走 PID
- **RunFlag=0**：电机直接停转，PID 输出清零（`Process_Control` 入口清 Out/IOut/POut/DOut/LastOutput）
- **RunFlag=1 且 Target=0**：PID 仍在跑，通过 P+I+D 制动

---

## 3. PID 算法 (pid.c)

**公式**: `Out = Kp·e(k) + Ki·Σe + Kd·[Actual(k-1)-Actual(k)] + Kf·Target`

**8 步处理流程** (每控制周期):

| 步骤 | 操作 | 说明 |
|------|------|------|
| 1 | `Error0 = Target - Actual` | 误差传递 |
| 2 | `POut = Kp × Error0` | 比例项 |
| 3 | `IOut += Ki × Error0` | 积分累积。\|Error0\|≤Deadband 停积; Target=0 清零 |
| 4 | `DOut = -Kd × (Actual - Actual1)` | 微分先行: 对测量值微分, 目标突变不冲击 |
| 5 | `Out = POut + IOut + DOut + Kf×Target` | 四项合成 |
| 6 | `Out>0 → +OutOffset; Out<0 → -OutOffset` | 死区补偿, 克服 TB6612 低占空比不转 |
| 7 | `delta ≤ OutputRateLimit` | 变化率限制, 防急加速/急刹车 |
| 8 | `OutMax ≥ Out ≥ OutMin` | 最终限幅 |

### SpeedPID 参数

```c
// 代码位置: main.c 第201-209行 (运行时实际值)
Kp=0.2   Ki=0.06   Kd=0.1   Kf=0.30
OutMax=100        OutMin=-100
OutOffset=15      IntegralLimit=35
Deadband=3        OutputRateLimit=12
```

> 注意: 结构体初始化器 (87-93行) 中 Kp=0.8/Ki=0.01/Kd=0.6/Kf=0.0 是旧值, 被 PID_Init+PID_SetParam+Kf赋值覆盖。实际运行时参数以 202-209 行为准。

### YawPID 参数

```c
Kp=0.0   Ki=0.0   Kd=0.0   Kf=未设置     // 暂关
OutMax=20          OutMin=-20
OutOffset=0        IntegralLimit=40
Deadband=1         OutputRateLimit=10
```

---

## 4. 编码器解算 (encoder.c)

**参数**: JGB37-520, 4倍频, 30:1减速比, PPR=11, 轮半径=0.065m

**速度公式**:
```
RPM = count_increment / (4 × 30 × 11) × (1000/loop_period) × 60
m/s = RPM / 60 × π × 2 × 0.065
```

- 左编码器 (TIM3): 正转脉冲为负, `count_now` 取反统一方向
- 右编码器 (TIM2): 不取反
- TIM2/TIM3 编码器模式 TI12, 滤波器=15, ARR=65535
- 50ms 周期内脉冲数不会溢出, Update 中断仅预留

---

## 5. 电机驱动 (motor.c)

```c
Motor_SetSpeedA(Speed)   // 左轮: TIM1_CH2(PE11) + PF1/PF2方向
Motor_SetSpeedB(Speed)   // 右轮: TIM1_CH1(PE9)  + PF3/PF4方向
```

- `Speed >= 0`: 方向1=H/方向2=L, PWM比较值 = Speed×40
- `Speed < 0`: 方向1=L/方向2=H, PWM比较值 = (-Speed)×40

---

## 6. MPU6050 姿态解算

- **调用频率**: 10ms (TIM6)
- **GZ 零偏**: 停车时 EMA 追踪 (α=0.1, 每秒采集100点)
- **偏航角**: `YawAngle += gz(°/s) × 0.01s` (积分)
- **倾角**: `atan2(ax_cali, az_cali)` → 一阶低通 (α=0.8)

---

## 7. 串口数据格式 (USART3, 每 50ms)

```
目标%, 右轮RPM%, 左轮RPM%, SpeedPID.Out, YawPID.Out, TurnTarget, 速差, YawAngle, BT_LV, BT_RH, RunFlag
```

| 列 | 字段 | 公式/单位 | 说明 |
|----|------|-----------|------|
| 1 | Target% | `Target ÷ spd_max × 100` | 目标占档位满速百分比 |
| 2 | 右轮RPM% | `ecd_right.RPM ÷ spd_max × 100` | 右轮实际转速百分比 |
| 3 | 左轮RPM% | `ecd_left.RPM ÷ spd_max × 100` | 左轮实际转速百分比 |
| 4 | SpeedPID.Out | PWM % | -100~+100, 稳态时应接近 Target% |
| 5 | YawPID.Out | RPM | 差速输出, 当前=0 |
| 6 | TurnTarget | RPM | 手动转向目标 |
| 7 | L-R 速差 | RPM | `ecd_left.RPM - ecd_right.RPM` |
| 8 | YawAngle | ° | MPU6050 偏航角积分 |
| 9 | BT_LV | -100~100 | 蓝牙摇杆前后 |
| 10 | BT_RH | -100~100 | 蓝牙摇杆转向 |
| 11 | RunFlag | 0/1 | 停车/运行 |

> 列 1~3 按档位满速映射为百分比, Target% 和 RPM% 同尺度, 稳态时 Target% ≈ 左/右RPM%

---

## 8. 串口命令 (USART3, 115200bps)

### PID 调参

| 命令 | 作用 | 反馈 |
|------|------|------|
| `skp N` | SpeedPID.Kp = N | `[SpeedPID] Kp=... Ki=... Kd=... Kf=...` |
| `ski N` | SpeedPID.Ki = N | 同上 |
| `skd N` | SpeedPID.Kd = N | 同上 |
| `skf N` | SpeedPID.Kf = N | 同上 |
| `ykp/yki/ykd/ykf N` | YawPID 对应参数 | `[YawPID] Kp=...` |

### 电机控制

| 命令 | 作用 |
|------|------|
| `t N` | Target = N RPM, 启动 |
| `start` | RunFlag=1 |
| `stop` | RunFlag=0, 电机归零, Target=0 |
| `duty N` | 开环 PWM (-100~100) |
| `yaw N` | YawTarget = N ° |
| `turn N` | TurnTarget = N RPM |

### 数据流

| 命令 | 作用 |
|------|------|
| `quiet` | 停止 VOFA CSV 输出, 方便输命令 |
| `log` | 恢复 VOFA CSV 输出 |

### 查询

| 命令 | 作用 |
|------|------|
| `show` | 显示全部 PID 参数和当前状态 |
| `help` / `?` | 命令列表 |

---

## 9. 蓝牙协议 (USART2, 9600bps, 环形缓冲区)

```
[joystick, LH, LV, RH, RV]  → LV=前后(-100~100), RH=转向(-100~100)
[key, 1, up]                 → 启停切换
[key, 2, up]                 → 档位-1
[key, 3, up]                 → 档位+1
```

- LV≠0 或 RH≠0 → Target = LV/100 × SpeedTable[档位], RunFlag=1
- LV=0 且 RH=0 → Target=0, TurnTarget=0 (RunFlag 保持 1, PID 制动)
- 300ms 无蓝牙数据 + 按键未按下 → 超时停车 (RunFlag=0)

---

## 10. 串口 TX 非阻塞架构

```
Serial_SendByte() → 入队环形缓冲区(256B) → TXEIE=1
                                              ↓
USART3_IRQHandler:
  1. Serial_TXE_IRQ()        先: TXE=1 → 取队首→写DR, 队空则关TXEIE
  2. HAL_UART_IRQHandler()   后: 处理 RXNE → PIDTune_RxCallback → 重启 HAL_UART_Receive_IT
```

- TX 直接写 USART3->DR, **不调用 HAL_UART_Transmit**, 不占 HAL Lock/状态机
- RX 中断 `HAL_UART_Receive_IT` 重启不受 TX 干扰
- `PIDTune_Init()` 在 main() 初始化序列中调用, 启动 USART3 RX 中断链

---

## 11. 关键修复记录

| # | 问题 | 根因 | 修复 |
|---|------|------|------|
| 1 | SpeedPID.Out 停车不归零 | RunFlag=0 时 Process_Control 直接 return, PID_Update 跳过 | `!RunFlag` 分支清零 Out/IOut/POut/DOut/LastOutput |
| 2 | quiet/log/skp 等串口命令全部无效 | `PIDTune_Init()` 从未调用, USART3 RX 中断未启动 | main() 初始化加入 `PIDTune_Init()` |
| 3 | 命令字节在 VOFA 数据流中丢失 | `HAL_UART_Transmit` 占用 HAL Lock, RX 中断重启失败 | 改用 TXE 中断+环形缓冲区, 不碰 HAL 状态机 |
| 4 | TX 数据发不出去 | HAL_UART_IRQHandler 先看到 TXE, TxXferCount=0 就关 TXEIE | `Serial_TXE_IRQ()` 移到 HAL 之前执行, 发完立即自关 |

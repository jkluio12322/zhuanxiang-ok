# 串级 PID 控制方案

## 1. 当前（并联） vs 目标（串级）

```
【并联 - 当前】                          【串级 - 目标】
                                       
SpeedTarget ──→ SpeedPID ──→ base       SpeedTarget ──→ SpeedPID(内环) ──→ base
                                ↓          ↑ 50ms                           ↓
TurnTarget  ──→ TurnPID  ──→ diff         ave_speed ← encoder               L/R = base ± diff
                                ↓                                          ↑
                           L/R → PWM       YawAngle ──→ YawPID(外环) ──→ diff₂
                                                            ↑ 100ms
                                           手动转向(BT) ──→ diff₁ ──→ diff = diff₁ + diff₂
```

并联问题：SpeedPID 和 TurnPID 各自独立输出，互相耦合，调参时改一个影响另一个。

串级思路：
- **内环 SpeedPID**：只管平均转速跟踪，响应快（50ms），不受转向干扰
- **外环 YawPID**：只管航向纠偏，响应慢（100ms），输出限幅小（±20 RPM），只做微调
- **手动转向 diff₁**：蓝牙/按键的转向指令直接叠加，不走 PID，响应即时

## 2. 控制逻辑改动

### Process_Control() 新流程

```c
// 每 50ms：
float ave = (L_RPM + R_RPM) / 2.0f;
updatePID(&SpeedPID, ave);                    // 内环：速度闭环
float base = SpeedPID.output;

// 每 100ms（外环半速）：
static uint8_t outer_cycle = 0;
if (++outer_cycle >= 2) {
    outer_cycle = 0;
    updatePID(&YawPID, YawAngle);             // 外环：航向闭环
}
float diff_yaw = YawPID.output;               // 外环纠偏分量（限幅 ±20）

// 手动转向（来自 BT RH 参数 / 串口 turn 命令）：
float diff_manual = TurnTarget;               // 直接叠加，不走 PID

float diff = diff_manual + diff_yaw;
int16_t L = (int16_t)(base + diff);
int16_t R = (int16_t)(base - diff);
Motor_SetSpeedA(L);  Motor_SetSpeedB(R);
```

关键点：
- YawPID 输入是 `YawAngle`（MPU6050 积分偏航角），不再是轮速差
- YawPID 目标默认 0（保持直行），蓝牙转向时跟随更新
- 手动 diff 和 YawPID 输出分别限幅，YawPID 输出限 ±20 RPM（只做微调）

## 3. 串口命令调整

| 命令 | 含义 | 说明 |
|------|------|------|
| `skp/ski/skd N` | 内环 SpeedPID 参数 | 同之前 |
| `ykp/yki/ykd N` | 外环 YawPID 参数 | 原 tkp/tki/tkd 改名 |
| `t N` | 速度目标 (RPM) | 同之前 |
| `yaw N` | 航向目标 (°) | 设 YawPID 目标角度 |
| `turn N` | 手动差速 (RPM) | 直接叠加到电机差速 |
| `start/stop/duty N` | 同之前 | |

## 4. 调参顺序

```
第一步 调内环（关外环）：
  ykp 0          ← 外环全部置 0
  yki 0
  ykd 0
  t 50           ← 给阶跃目标，看 VOFA 波形左/右轮 RPM
  skp 3.0        ← 逐步加大 Kp，有轻微超调为止
  ski 0.5        ← Ki 消除静差
  skd 0.2        ← Kd 抑制振荡

第二步 调外环（内环锁定后）：
  t 30           ← 给一个巡航速度
  ykp 0.3        ← 外环从很小 Kp 开始
  yki 0.02       ← 极小 Ki
  ykd 0          ← 一般不需要 Kd
  观察 YawAngle 通道：航向漂移被拉回的速度
  
第三步 叠加测试：
  t 30
  turn 20        ← 手动给差速转弯，看 YawPID 是否在松手后拉回航向
```

## 5. 文件改动清单

| 文件 | 改动 |
|------|------|
| `main.c` | TurnPID → YawPID；YawPID 输入改为 YawAngle；外环 100ms 分频；手动 diff 独立叠加 |
| `pid_tune.c` | tkp/tki/tkd → ykp/yki/ykd；新增 yaw 命令；turn 改为手动差速 |
| 其他文件 | 不动 |

---

确认后改代码。

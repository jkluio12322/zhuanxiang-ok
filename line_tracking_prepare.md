# BlueCar V3.0 — 8路循迹功能接入准备说明

> **目标项目**: `E:\32\32\STM32HAL\f103zet6\bluecar` (STM32F103ZET6 + HAL 库)
> **参考项目**: `8路循迹STM32程序参考实验` (SPL 标准库) / `tradecar` (HAL 库已有 gray_track)
> **日期**: 2026-06-23

---

## 一、当前项目架构回顾

```
BlueCar V3.0 — 协程架构 + 环形缓冲解耦

  ┌──────────┐  CmdMsg   ┌──────────┐  VofaMsg  ┌──────────┐
  │ BT/Key/  │───→ Q ──→│ Control  │───→ Q ──→│  VOFA    │
  │ PIDTune  │           │ (50ms)   │           │  Output  │
  └──────────┘           └──────────┘           └──────────┘
                                ↓
                          Motor + Encoder
```

**现有硬件资源占用**:
| 外设 | 引脚 | 用途 |
|------|------|------|
| TIM1 CH1/CH2 | PE9, PE11 | 电机 A/B PWM (TB6612) |
| PF1~PF4 | — | 电机方向控制 |
| TIM2, TIM3 | PA0/PA1, PA6/PA7 | 编码器输入捕获 |
| I2C1 | PB6 (SCL), PB7 (SDA) | OLED (SSD1306) + MPU6050 |
| I2C2 | PB10 (SCL), PB11 (SDA) | **空闲可用** |
| USART3 | PB10/PB11 → PD8/PD9? | VOFA 遥测 + 串口调参 |
| GPIO PE8~PE15 | — | **空闲可用** ← 推荐用于循迹 |
| GPIO PD8~PD15 | — | **部分空闲** |

---

## 二、循迹传感器硬件准备

### 2.1 传感器选型

支持两种常见 8 路循迹模块，代码兼容：

| 类型 | 输出信号 | 接口 | 说明 |
|------|---------|------|------|
| **数字型** (推荐先上) | 0/1 高低电平 × 8 | 8 根 GPIO | 简单可靠，适合黑白线 |
| **灰度型** | 8 路模拟量 → I2C 输出 | I2C (SDA+SCL) | 可读取 0~255 灰度值，适合复杂赛道 |

### 2.2 引脚分配方案

**方案 A — 数字型传感器 (推荐首选)**：

```
传感器 D1 → PE8   (GPIO Input, Pull-Up)
传感器 D2 → PE9   ← 注意: PE9 已被 TIM1_CH1 占用！改用 PD8
传感器 D3 → PE10
传感器 D4 → PE11  ← 注意: PE11 已被 TIM1_CH2 占用！改用 PD9
传感器 D5 → PE12
传感器 D6 → PE13
传感器 D7 → PE14
传感器 D8 → PE15
```

> ⚠️ **关键冲突**: PE9 和 PE11 已被 TIM1 用于电机 PWM 输出，不可用作 GPIO 输入。
> **修正方案**: D2 改用 PD8, D4 改用 PD9（或全部改用 GPIOD 端口减少碎片化）。

**推荐最终引脚分配 (方案 A-修正)**：

```
传感器 D1 → PD8   (GPIOD Pin8,  Input, Pull-Up)
传感器 D2 → PD9   (GPIOD Pin9,  Input, Pull-Up)
传感器 D3 → PD10  (GPIOD Pin10, Input, Pull-Up)
传感器 D4 → PD11  (GPIOD Pin11, Input, Pull-Up)
传感器 D5 → PD12  (GPIOD Pin12, Input, Pull-Up)
传感器 D6 → PD13  (GPIOD Pin13, Input, Pull-Up)
传感器 D7 → PD14  (GPIOD Pin14, Input, Pull-Up)
传感器 D8 → PD15  (GPIOD Pin15, Input, Pull-Up)
```

> 优点：8 路集中在同一个 GPIO 端口 (GPIOD)，读取效率高，无引脚冲突。

**方案 B — I2C 灰度型传感器**：

```
传感器模块 SDA → PB10 (I2C2_SDA, 当前空闲)
传感器模块 SCL → PB11 (I2C2_SCL, 当前空闲)
传感器模块 VCC → 3.3V / 5V (根据模块规格)
传感器模块 GND → GND
```

> I2C2 已由 CubeMX 初始化 (400kHz), 可直接使用 `hi2c2` 句柄。
> 灰度型传感器通常支持多地址 (0x01~0x07), 可通过 `Set_ID` 命令配置。

### 2.3 供电说明

- 大多数 8 路循迹模块工作电压: **3.3V ~ 5V**
- STM32F103ZET6 GPIO 兼容 5V 输入 (FT 引脚)
- GPIOD 全部引脚为 **5V 耐压 (FT)**，可直接连接 5V 传感器
- 建议使用模块自带的 **3.3V LDO 供电** 以保证信号电平匹配

---

## 三、软件模块设计

### 3.1 新增文件清单

```
Core/Hardware/
├── line_track.h          # 循迹算法头文件 (模式枚举 + 函数声明)
├── line_track.c          # 循迹算法实现 (查表法 / PID法)
├── line_sensor.h         # 传感器底层读取头文件
└── line_sensor.c         # 传感器底层读取 (GPIO直读 + I2C灰度)
```

### 3.2 `line_track.h` 设计

```c
#ifndef __BSP_LINE_TRACK_H
#define __BSP_LINE_TRACK_H
#include "main.h"

/* ── 传感器通道数 ── */
#define LINE_CH_NUM  8

/* ── 传感器宏 (位定义, D1=LSB → D8=MSB) ── */
#define LINE_D1_Pin       GPIO_PIN_8
#define LINE_D1_Port      GPIOD
#define LINE_D2_Pin       GPIO_PIN_9
#define LINE_D2_Port      GPIOD
// ... D3~D8 依次为 GPIO_PIN_10~15

/* ── 循迹模式 ── */
typedef enum {
    LINE_MODE_OFF     = 0,   /* 循迹关闭, 正常遥控模式 */
    LINE_MODE_DIGITAL = 1,   /* 数字查表法 (查表+差速) */
    LINE_MODE_PID     = 2,   /* PID 循迹 (灰度传感器) */
} LineMode_t;

/* ── 对外接口 ── */
void LineTrack_Init(void);                          /* 初始化传感器 GPIO/I2C */
uint8_t LineTrack_ReadAll(void);                    /* 读取 8 路数字值 (1字节) */
uint8_t LineTrack_ReadChannel(uint8_t ch);          /* 读取单路 (1~8) */
void LineTrack_Digital(uint8_t base_speed);          /* 数字查表法循迹 */
void LineTrack_PID(uint8_t base_speed, float Kp);    /* PID 循迹 (灰度) */

/* ── 全局状态 ── */
extern LineMode_t line_mode;
#endif
```

### 3.3 循迹算法核心逻辑 (参考 `tradecar/gray_track.c`)

**3.3.1 数字查表法 (line_track_digital)**

来自参考项目 `track_zhixian1()` 的精简 HAL 版本:

```
8路传感器状态 → 查表 → 左右轮差速输出
  0001 1000  (D4/D5 踩线) → 直行  base/base
  0000 1000  (D4 踩线)    → 偏左, 右转修正
  0001 0000  (D5 踩线)    → 偏右, 左转修正
  0000 0100  (D3 踩线)    → 更偏左, 加大右转
  ...
  0000 0000  (全白/丢线)  → 保持上次方向或停车
  1111 1111  (全黑/路口)   → 路口计数 → 停车
```

**3.3.2 PID 灰度循迹 (line_track_PID)**

来自参考项目 `track_PID2()`:
- 读取 8 路灰度值 (0~255)
- 计算加权质心位置 → 实际位置值
- PID 计算位置误差 → 左右轮差速
- 使用蓝卡现有的 `PID_t` 结构体

### 3.4 命令系统扩展 (main.h 修改)

在现有 `CmdType` 枚举中新增:

```c
// main.h CmdType 枚举新增:
CMD_LINE_MODE,       /* 切换循迹模式 (OFF/DIGITAL/PID) */
CMD_LINE_SPEED,      /* 设置循迹基础速度 */
CMD_LINE_START,      /* 启动循迹 */
CMD_LINE_STOP,       /* 停止循迹, 切回手动 */
```

### 3.5 控制任务集成 (main.c 修改点)

```
Task_Control_50ms() 中新增分支:
  if (line_mode != LINE_MODE_OFF) {
      读取传感器;
      if (line_mode == LINE_MODE_DIGITAL)
          LineTrack_Digital(base_speed);
      else
          LineTrack_PID(base_speed, Kp);
      直接设置 Motor PWM;         // 旁路正常 PID 级联
      return;                     // 提前退出, 不执行遥控逻辑
  }
  // ... 原有遥控逻辑不变
```

---

## 四、实施步骤

### 第 1 步: CubeMX 配置 GPIO

在 CubeMX 中打开 `bluecar.ioc`:

1. 配置 **GPIOD Pin8~Pin15** 为 `GPIO_Input`, Pull-up
2. (可选) 确认 **I2C2** (PB10, PB11) 已使能
3. 重新生成代码 → 更新 `gpio.c` / `gpio.h`

> ⚠️ 重新生成会覆盖 `main.c` 中 USER CODE 区域外的内容。请务必备份后再操作。

### 第 2 步: 创建 sensor 驱动层

在 `Core/Hardware/` 下新建:
- `line_sensor.h` — 传感器宏定义 + 函数声明
- `line_sensor.c` — GPIO 初始化 + digital() 读取函数

参考代码: `tradecar/Core/Hardware/gray_track.c` 的 `digtal()` 函数 (已适配 HAL)。

### 第 3 步: 创建循迹算法层

在 `Core/Hardware/` 下新建:
- `line_track.h` — 模式枚举 + 函数声明
- `line_track.c` — 查表法 + PID 法实现

核心参考:
- 查表法: `参考实验/Hardware/LINE/line.c` 的 `track_zhixian1()`
- PID法: `参考实验/Hardware/LINE/line.c` 的 `track_PID1()` / `track_PID2()`
- HAL适配: `tradecar/Core/Hardware/gray_track.c` 的 `track()`

### 第 4 步: 修改 main.h

在 `CmdType` 枚举末尾 (CMD_LOG 之后) 新增循迹命令类型:

```c
CMD_LINE_MODE,       /**< 循迹模式切换 (0=OFF, 1=查表, 2=PID) */
CMD_LINE_SPEED,      /**< 设置循迹基础速度 */
CMD_LINE_START,      /**< 启动循迹 */
CMD_LINE_STOP,       /**< 停止循迹 */
```

### 第 5 步: 修改 main.c

1. **Include 新增头文件**:
   ```c
   #include "line_track.h"
   ```

2. **新增全局变量**:
   ```c
   LineMode_t line_mode   = LINE_MODE_OFF;
   uint8_t    line_speed  = 40;     /* 循迹基础速度 (PWM占空比%, 0~100) */
   float      line_Kp     = 0.05f;  /* PID 比例系数 */
   ```

3. **在 `Cmd_ProcessKeys` 或 `Cmd_ProcessBluetooth` 中添加模式切换入口**:
   - 长按 KEY4 2s → 切换循迹模式 (OFF → DIGITAL → PID → OFF)
   - 或者蓝牙命令 `[line,mode,N]` 切换

4. **在 `Task_Control_50ms` 开头加入循迹分支**:
   ```c
   if (line_mode != LINE_MODE_OFF && RunFlag) {
       /* 循迹模式: 旁路遥控, 直接输出 */
       if (line_mode == LINE_MODE_DIGITAL)
           LineTrack_Digital(line_speed);
       else if (line_mode == LINE_MODE_PID)
           LineTrack_PID(line_speed, line_Kp);
       // 填充遥测数据
       vofa_data.fresh = 1;
       return;  // 跳过遥控逻辑
   }
   ```

5. **在 `main()` 初始化区加入**:
   ```c
   LineTrack_Init();    /* 传感器 GPIO 初始化 */
   ```

### 第 6 步: 编译、烧录、测试

```bash
# 编译
cd E:\32\32\STM32HAL\f103zet6\bluecar
make -j8  # 或 CubeIDE 内 Build

# 烧录 (ST-Link / J-Link)
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program build/bluecar.elf verify reset exit"
```

---

## 五、循迹算法详解

### 5.1 查表法 (LINE_MODE_DIGITAL)

**原理**: 预定义 8 路传感器所有可能状态 (2^8 = 256 种)，每种状态对应一组左右轮 PWM 值。

**来自参考项目的查表 (PWM 0~100 映射)**：

| 传感器状态 (D1~D8) | 含义 | 左轮 | 右轮 |
|---------------------|------|------|------|
| 0001 1000 | 居中 (D4/D5踩线) | base | base |
| 0000 1000 | D4 踩线 (微偏左) | base×0.875 | base |
| 0001 0000 | D5 踩线 (微偏右) | base | base×0.875 |
| 0000 0100 | D3 踩线 (偏左) | base×0.625 | base |
| 0010 0000 | D6 踩线 (偏右) | base | base×0.625 |
| ... | 越偏差值越大 | ... | ... |
| 1000 0000 | D8 踩线 (极右) | base | 0 (原地右转) |
| 0000 0001 | D1 踩线 (极左) | 0 (原地左转) | base |
| 0000 0000 | 全白 (丢线) | 保持/Debug | 保持/Debug |
| 1111 1111 | 全黑 (路口) | 路口计数 | → 停车 |

> **注意**: 参考项目中 `motor(L,R)` 参数范围是 0~50 (占空比%), 蓝卡中 `Motor_SetSpeedA/B` 参数范围是 ±100 (正=前进, 负=后退)。需要做映射转换。

### 5.2 PID 位置式灰度循迹 (LINE_MODE_PID)

**原理**: 
1. 读取 8 路灰度值 (通过 I2C 获取 0~255 模拟量)
2. 计算加权质心:
   ```
   加权总和 = Σ(灰度值[i] × i)    (i = 1~8)
   灰度总和 = Σ(灰度值[i])
   质心位置 = 加权总和 / 灰度总和 × 1000   (放大 1000 倍避免浮点)
   ```
3. 设定目标位置 (如 4500 = 4.5，即线在中间)
4. PID 计算: `误差 = 目标位置 - 实际位置`
5. 左右差速: `L_PWM = base + 误差×Kp`, `R_PWM = base - 误差×Kp`

**与蓝卡现有 PID 的复用**:
- 可以直接新建一个 `PID_t LinePID` 实例
- 或者独立实现轻量 P 控制器 (只需 Kp，无需完整 PID)

---

## 六、关键注意事项

### 6.1 引脚冲突处理

| 引脚 | 当前用途 | 能否挪用 | 替代方案 |
|------|---------|---------|---------|
| PE9 | TIM1_CH1 (电机A PWM) | ❌ 不可 | 避开, 用 PD9 替代 |
| PE11 | TIM1_CH2 (电机B PWM) | ❌ 不可 | 避开, 用 PD11 替代 |
| PB10 | I2C2 SCL | ✅ 空闲 | 灰度传感器 SCL |
| PB11 | I2C2 SDA | ✅ 空闲 | 灰度传感器 SDA |

### 6.2 实时性考虑

- 当前控制周期: **50ms** (Task_Control_50ms) + **10ms** (Task_Pulse_10ms)
- 循迹传感器读取: < 1ms (GPIO 直读) 或 < 2ms (I2C 读取)
- 可以在 10ms 内环中跑循迹, 也可以挂在 50ms 外环
- **推荐**: 循迹初期挂在 50ms 外环中测试, 成熟后可提升到 10ms

### 6.3 与现有遥控的切换

```
RunFlag=1 + line_mode=OFF     → 蓝牙/按键遥控 (原有逻辑)
RunFlag=1 + line_mode=DIGITAL → 循迹查表法
RunFlag=1 + line_mode=PID     → 循迹 PID 法
RunFlag=0                     → 停车 (最高优先级)
```

### 6.4 OLED 显示集成

循迹模式下 OLED 可显示:
```
Line 1: [LINE TRACK] DGT/PID
Line 2: Speed:040 Enc:1234
Line 3: Sensor: 00011000
Line 4: Yaw:+005.0 Lv:3
```

---

## 七、测试步骤

### 7.1 硬件验证

1. 用万用表确认传感器模块供电正常 (3.3V/5V)
2. 用示波器/逻辑分析仪确认 8 路信号有电平变化
3. 串口打印传感器原始值 (8 位二进制)

### 7.2 桌面测试

1. 烧录固件，上电
2. 按键切换到循迹模式
3. 在 OLED 上观察传感器状态变化
4. 用手在白纸/黑线上方模拟移动, 确认电机差速方向正确

### 7.3 赛道测试

1. 先用低速 (PWM=20~30) 测试转弯
2. 逐步调高速度
3. 根据转弯过冲/不足调整 Kp 参数
4. 记录路口检测计数逻辑是否正常

---

## 八、参考代码溯源

| 文件 | 来源 | 功能 |
|------|------|------|
| `参考实验/Hardware/LINE/line.c` | SPL 8路循迹 | track_zhixian1/2, track_PID1/2/3 |
| `参考实验/Hardware/SENSOR/sensor.c` | SPL GPIO 读取 | digital() 基本函数 |
| `参考实验/System/IIC/iic.c` | SPL 软件 I2C | I2C 读写传感器寄存器 |
| `tradecar/Core/Hardware/gray_track.c` | HAL 循迹 | HAL 版 digital() + track() |
| `bluecar/Core/Hardware/motor.c` | 当前电机驱动 | Motor_SetSpeedA/B (±100) |
| `bluecar/Core/Hardware/pid.c` | 当前 PID 库 | PID_Init/Update (可复用) |

---

## 九、快速启动检查清单

- [ ] 确认 8 路循迹传感器模块型号 (数字型 / 灰度型)
- [ ] 确认传感器模块引脚顺序 (D1~D8 对应)
- [ ] CubeMX 配置 GPIOD Pin8~Pin15 为上拉输入
- [ ] (灰度型) CubeMX 确认 I2C2 已使能
- [ ] 创建 `line_sensor.h` / `line_sensor.c` — 传感器读取层
- [ ] 创建 `line_track.h` / `line_track.c` — 循迹算法层
- [ ] 修改 `main.h` — 新增 CmdType 枚举值
- [ ] 修改 `main.c` — Include → 初始化 → 按键切换 → 控制分支
- [ ] 编译通过 (0 errors, 0 warnings)
- [ ] 烧录测试 — 桌面验证传感器读数
- [ ] 赛道实测 — 低速 → 中速 → 调参

---

> **总结**: 基于蓝卡现有的协程架构和 PID 基础设施，加入循迹功能主要是 **3 个新文件 + 2 个文件修改**，核心工作量在算法移植和参数调优。建议先用 GPIOD 数字型传感器上查表法，跑通后再上 I2C 灰度型+PID 精调。

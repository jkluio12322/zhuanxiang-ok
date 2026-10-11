# BlueCar — STM32F103ZET6 循迹小车

STM32F103ZET6 + STM32 HAL 裸机工程（无 RTOS）：八路数字循迹、级联双环速度闭环、
SSD1306 OLED 状态显示、VOFA+ 遥测。

## 文档索引与权威顺序

| 文件 | 内容 |
| --- | --- |
| `功能概览.md` | **落脚页**：能干什么、时序模型、三个模式与输出路径、算法要点、接口一览（参数细节指向 README） |
| `README.md` | 硬件、引脚、参数、控制架构、串口命令、调参流程 |
| `代码阅读指南.md` | 阅读顺序、嵌入式概念扫盲、调试技巧、术语表 |
| `DEBUG_LOG.md` | 构建与调试历史 |

**冲突时以代码为准。** 改动行为后同步更新对应文档。

⚠️ `DEBUG_LOG.md` 中 2026-10-02 之前的记录描述的是**已被删除的旧架构**，
其中的 `skf`/`skfl`/`skfr` 等命令已不存在，不要据此推断现状。

## 构建 / 烧录 / 校验

```bash
# 重建（EIDE → ARMCC AC5）
unify_builder -p "e:/32/32/STM32HAL/f103zet6/bluecar/MDK-ARM/build/bluecar/builder.params" --rebuild

# Keil 命令行
UV4.exe -r MDK-ARM/bluecar.uvprojx -j0 -o build.log

# 烧录
"C:/Users/Lenovo/.eide/tools/st_cube_programer/bin/STM32_Programmer_CLI.exe" \
  -c port=SWD freq=4000 mode=UR reset=HWrst \
  -d "e:/32/32/STM32HAL/f103zet6/bluecar/MDK-ARM/build/bluecar/bluecar.hex" -v

# 单文件语法快检（不产出固件，最快的错误检查手段）
# 在仓库根目录执行；这条已在 MDK-ARM/.claude/settings.local.json 白名单里，照抄可免授权
arm-none-eabi-gcc -fsyntax-only -c -std=c11 -mcpu=cortex-m3 -DSTM32F103xE -DUSE_HAL_DRIVER \
  -I Core/Inc -I Core/Hardware -I Drivers/STM32F1xx_HAL_Driver/Inc \
  -I Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I Drivers/CMSIS/Device/ST/STM32F1xx/Include \
  -I Drivers/CMSIS/Include -I MDK-ARM/.cmsis/include -I MDK-ARM/RTE/_bluecar Core/Src/main.c

# 主机侧算法仿真（不需要重新编译）
python tools/simulate_tracking.py
python tools/simulate_pwm_tests.py
```

产物落在 `MDK-ARM/build/bluecar/`（`bluecar.hex`、`bluecar.axf`），该目录已 gitignore。
**`compile_commands.json` 每次构建自动刷新，不要手动编辑。**

## 三个必须知道的陷阱

### 1. 实际编译器是 ARMCC v5，不是 GCC

Keil（`bluecar.uvprojx`，`<uAC6>0</uAC6>`）和 EIDE（`.eide/eide.yml` → `toolchain: AC5`）
**都用 ARMCC v5**，位于 `F:\kilv5\ARM\ARMCC`。固件必须在 AC5 下编译通过。

根级 `.clangd` 里指的 `arm-none-eabi-gcc 13.3.1`（来自 `E:\32\STM32CubeCLT_1.20.0`）
**只用于 clangd 解析和上面的语法快检，不参与实际构建**。不要写只在 GCC 下成立的扩展。

### 2. 增删改名任何源文件，两个工程文件都要改

`MDK-ARM/bluecar.uvprojx` 和 `MDK-ARM/.eide/eide.yml` **都要同步修改**，包括 `IncludePath`/`incList`。
EIDE 此前出现过回退文件的情况，两个文件都视为 source of truth。

### 3. `app_config.h` 的宏必须是单行

`tools/simulate_*.py` 用正则直接解析该文件：

```python
DEFINE_RE = re.compile(r"^\s*#\s*define\s+([A-Za-z_]\w*)\s+([^\s/]+)")
```

写成多行或加复杂表达式会让仿真脚本静默取不到值。当前该文件有 83 个符合条件的宏定义。

```bash
grep -cE "^\s*#\s*define\s+([A-Za-z_]\w*)\s+([^\s/]+)" Core/Application/Config/app_config.h
```

## 分层架构不变量

依赖**只能向下**，这是全工程最重要的结构约束：

```
Display / Comm  →  State  →  Control / Motion  →  Scheduler  →  Hardware
```

| 目录 | 模块 |
| --- | --- |
| `Config/` | `app_config.h`（全部可调参数）、`app_types.h`、`board.h`（引脚别名） |
| `Scheduler/` | `app_scheduler`（中断分发、延后任务） |
| `State/` | `app_state`（唯一可变状态）、`app_imu` |
| `Motion/` | `app_motion`（编码器、滤波、电机输出）、`app_speedloop`（50 ms 控制） |
| `Control/` | `app_pid`、`app_tracking`、`app_testmode` |
| `Comm/` | `app_serial`、`app_console`、`app_cmd`、`app_telemetry` |
| `Display/` | `app_display` |

**谁只能碰什么**（越界读写基本就是 bug）：

- **写电机分两层**：`app_speedloop.c` 是唯一决定 PWM 的模块，它调
  `AppMotion_Apply()` / `AppMotion_Stop()`；`app_motion.c` 是唯一直接调用
  `Motor_SetSpeedA/B` 的模块。别把这两层混为一谈
- 只有 `app_state` 拥有可变状态：读用 `AppState()->字段`，写用 `AppState_SetXxx()`
- 不引入 `extern` 可变全局

两条可自动验证的规律：

```bash
# 除驱动层自身定义外，应该只有 app_motion.c 调用
grep -rn "Motor_SetSpeedA" Core/
# → app_motion.c:107,186 + motor.c/motor.h 的定义与声明

# 除驱动层 irtrack.c 外，应该只有 app_tracking.c 读写
grep -rn "ir_data" Core/ --include=*.c
# → app_tracking.c + irtrack.c（定义处）
```

结果多出来几行，说明有人破坏了分层。

## 时序与中断模型

TIM5 提供 1 ms 时间戳；TIM6 以 1 kHz 触发，由 `app_scheduler.c` 分频。

| 周期 | 任务 | 位置 |
| --- | --- | --- |
| 10 ms | MPU6050 采样与姿态解算 | **TIM6 中断内** |
| 20 ms | 按键状态机 + 按键转命令 | **TIM6 中断内** |
| 20 ms | 循迹（或 PWM 循迹测试） | **TIM6 中断内** |
| 50 ms | 编码器测速、速度环、命令执行、遥测快照 | **TIM6 中断内** |
| 50 ms | 遥测格式化与发送 | 主循环（延后） |
| 100 ms | OLED 状态页刷新 | 主循环（延后） |
| — | 串口命令行解析、OLED 缓冲写入 | 主循环 |

**为什么这样分**：USART1 一个字节只要 86.8 µs，而遥测 `snprintf` 16 个浮点要 100–300 µs、
OLED 刷屏可能上 ms —— 放进最高优先级中断会压住串口接收导致丢字节。而速度环/循迹环按
**固定周期常数**（50 ms、`dt = 0.02`）计算，挪到主循环产生抖动就会算错。

**规则：中断里绝不能放 `snprintf` 或 I2C I/O。** 主循环只有 `AppDeferred_Run()` +
串口命令行 + `OLED_Flush()`。

中断在 `AppScheduler_Init()` 之前不放行任何任务。

## 命令队列

按键产生的启停/模式/换档命令写入 8 项环形队列，由 50 ms 控制任务集中执行，队列满时
**丢弃新命令**而非覆盖未执行的动作。

USART1 命令行走**同一个队列**（`l`/`r`/`t`/`stop`/`quiet`/`log`），`show` 只读。
不要在命令行解析处直接改轮速目标——那是与中断里速度环的竞争点。

**在线改 PID 的路径已删除**：参数只在 `app_config.h` 定义处改，改完重新编译烧录。

## 提交信息约定

中文 `类型：描述`，例如：

```
33be6e2 代码阅读指南：补一节术语表（附录 B）
55fc834 仓库卫生：取消跟踪 EIDE 的 CMSIS 包缓存
0830931 阶段8：显示重构与文档同步
7261f7b 重构：应用层模块拆分与命名规范化
```

## 已知未实现项（不要"修复"这些）

早期文档描述过但**当前代码没有实现**：

- **直行航向闭环（陀螺仪）**：MPU6050 只采样并送遥测的 `yawDeg` 字段，不参与控制
- **速度环死区、单周期限速率、前馈 `kf`**：未实现
- **电机堵转保护**：`app_config.h` 保留 `MOTOR_STALL_*` 宏，但无任何代码使用

## Claude Code 相关

- **权限白名单**：`MDK-ARM/.claude/settings.json` 与 `settings.local.json`（**两者都被 git 跟踪**，
  注意改动会进版本库）
- **clangd**：配置在**仓库根** `.clangd`（不是 `MDK-ARM/.clangd` —— clangd 从源文件目录
  逐级向上查找，`MDK-ARM/` 是 `Core/` 的兄弟目录，放那里永远命中不了）。已实测
  `clangd --check` 对该工程返回 **0 errors**。
  `Diagnostics` 被**有意**设为 `Suppress: "*"`——clangd 会误解析 ARMCC 的
  `__weak`/`__packed`/`#pragma arm` 等扩展，放开会产生大量假错误。
  **因此 clangd 只用于导航（跳定义/查引用），错误检查请用上面的 `-fsyntax-only` 命令**
- **clangd 本体**已装在 `~/.local/bin/clangd.exe`（v23.1.0），资源目录在 `~/.local/lib/clang/23`。
  两者必须保持这个相对布局，clangd 按 `<exe目录>/../lib/clang/<版本>` 找内建头
- **VSCode IntelliSense** 由 EIDE 提供（`.vscode/settings.json` 里
  `C_Cpp.default.configurationProvider: "cl.eide"`），不是 clangd

### 任务 → 技能路由

| 任务 | 技能 |
| --- | --- |
| EIDE / ARMCC 构建失败 | `stm32-eide-build-debug` |
| Keil 工程、烧录 | `keil`、`openocd` |
| 串口日志、VOFA+ 数据 | `serial` |
| 改动审查 | `embedded-review` |
| 注释规范 | `embedded-code-commenting` |
| HAL/LL 用法 | `stm32-hal-ll-integration` |

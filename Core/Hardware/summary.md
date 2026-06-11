---
name: bluecar-v3-refactor
description: BlueCar 项目 V2.1→V3.0 重构记录：6档卡死根因、DMA TX改造、协程架构、环形缓冲
metadata:
  type: project
---

# BlueCar V2.1 → V3.0 重构总结

**日期**: 2026-06-11
**硬件**: STM32F103ZET6 + TB6612 + JGB37-520 + HC-05 + MPU6050 + OLED
**编译工具**: ARM Compiler 5.06 (Keil MDK)

## 6档卡死根因

| # | 问题 | 位置 | 严重性 |
|---|------|------|--------|
| 1 | **USART3 TXE 忙等待死锁** | `serial.c:54` `while (next == tx_tail);` | 🔴 致命 |
| 2 | **USART3 TX/RX 共享中断冲突** | `stm32f1xx_it.c` USART3_IRQHandler 内同时处理 TXE 和 RX | 🔴 致命 |
| 3 | **HAL RX 与环形缓冲共用 buffer[0]** | `blueserial.c` `HAL_UART_Receive_IT(&huart2, BlueSerial_Buffer, 1)` | 🟡 数据冲突 |
| 4 | **PIDTune_Parse 在 ISR 中执行** | `pid_tune.c` `PIDTune_RxCallback` → `PIDTune_Parse` (浮点运算) | 🟡 ISR膨胀 |
| 5 | **环形缓冲索引无 volatile** | `blueserial.c` BlueSerial_PutIndex/GetIndex | 🟡 编译器优化风险 |
| 6 | **Serial_Printf 100字节栈缓冲** | `serial.c` `char String[100]` vsnprintf 可能溢出 | 🟡 边界风险 |

**死锁链路 (6档 350RPM):**
```
VOFA_Send() → Serial_Printf() → Serial_SendByte() 循环
  └→ while(next==tx_tail); 忙等待
       └→ TXE ISR 释放1字节后立即关 TXEIE
            └→ USART3 RX 字节到达 → HAL_UART_IRQHandler
                 └→ 状态机冲突 → 发送停顿 → 主循环卡死
```

## 修改文件清单

### 新增 (2)
| 文件 | 说明 |
|------|------|
| `Core/Hardware/ringbuf.h` | 无锁 SPSC 环形缓冲 (volatile 安全) |
| `Core/Hardware/ringbuf.c` | `RingBuf_Init/Put/PutNB/Get/Available/Free/Flush` |

### 重写 (2)
| 文件 | 变更 |
|------|------|
| `Core/Hardware/serial.c` | TXE 忙等待 → DMA TX + 1024B 环形缓冲, `vsnprintf` 防溢出 |
| `Core/Src/main.c` | 协程式任务调度 + CmdMsg 命令队列 |

### 修改 (6)
| 文件 | 变更 |
|------|------|
| `Core/Hardware/serial.h` | 新增 `Serial_Write/Serial_DMA_TX_Complete`, 移除 `Serial_TXE_IRQ` |
| `Core/Hardware/pid_tune.c` | `PIDTune_RxCallback` 仅缓冲, `PIDTune_Process` 主循环解析 |
| `Core/Hardware/pid_tune.h` | 新增 `PIDTune_Process`, 导出 `tune_line_ready/tune_line` |
| `Core/Hardware/blueserial.c` | 索引改 volatile, HAL RX 独立 `bt_rx_byte` |
| `Core/Hardware/key.h` | 补充 `Key_GetNum` 声明 |
| `Core/Src/stm32f1xx_it.c` | USART3 移除 `Serial_TXE_IRQ`, DMA1_Ch2 新增 `Serial_DMA_TX_Complete` |
| `Core/Inc/main.h` | 新增 `CmdType/CmdMsg/Mode_t` 类型, `CmdQueue_Put` 接口 |
| `MDK-ARM/bluecar.uvprojx` | 添加 `ringbuf.c` 到工程 |

## 新架构

```
┌─────────────────────────────────────────────────────────────┐
│ 主循环 (协程式调度)                                          │
│                                                             │
│  Cmd_ProcessBluetooth()  ──→ cmd_queue[8] ──┐               │
│  Cmd_ProcessKeys()       ──→ cmd_queue[8] ──┤               │
│  Cmd_ProcessPIDTune()    ──→ cmd_queue[8] ──┤               │
│                                              ↓               │
│  [10ms] Task_Sensor_10ms()   MPU6050 → YawAngle             │
│  [50ms] Task_Control_50ms()  读cmd_queue→PID→Motor→vofa_data│
│  [50ms] Task_VOFA_Send()     vofa_data→Serial_Write(DMA TX) │
│  [200ms] Task_OLED_Update()                                  │
│                                                              │
│  DMA1_Ch2 ISR → Serial_DMA_TX_Complete() → 接力发送         │
│  USART2 ISR  → bt_rx_byte → BlueSerial_Put()                │
│  USART3 ISR  → pid_tune_rx_byte → 仅缓冲 (不解析)           │
└─────────────────────────────────────────────────────────────┘
```

## 三个关键原则

1. **ISR 只做事, 不做决策** — 字节缓冲在 ISR, 解析在主循环
2. **数据流单向解耦** — BT/Key/Serial → CmdQueue → Control → VofaMsg → VOFA
3. **发送永不阻塞** — DMA TX + ringbuf, 满则丢弃, 保证控制循环不卡

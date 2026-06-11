# BlueCar 开发会话日志

### 2026-06-11 09:30 速度开6档电脑卡死
- **问**: 速度开6档后电脑卡死了，程序改多线程+环形缓冲
- **答**: 根因是 USART3 TXE 忙等待 `while(next==tx_tail)` + TX/RX 共享中断导致的死锁。改为 DMA TX 非阻塞 + CmdMsg 命令队列解耦，10 文件重构为协程架构 V3.0
- **文件**: serial.c/h(重写), main.c(重构), ringbuf.c/h(新增), pid_tune.c/h, blueserial.c, stm32f1xx_it.c, main.h, key.h, bluecar.uvprojx

### 2026-06-11 09:57 编译错误修复
- **问**: ARM Compiler 5 编译报错
- **答**: 3 个错误 — (1) `Mode_t` 类型冲突，typedef 移到 main.h 统一 (2) `Key_GetNum` 未声明，补到 key.h (3) `TiltAngle` 未使用，删除 + ringbuf.c 加入 .uvprojx
- **文件**: main.h, key.h, main.c, bluecar.uvprojx

### 2026-06-11 10:04 创建嵌入式总结 Skill
- **问**: 将这个 md 改成嵌入式总结 skill，以后说"总结"自动生成 .md 积累经验
- **答**: 创建 `embedded-summary` skill，触发词"总结/复盘/记录经验"，生成 YAML frontmatter + 结构化 Markdown 追加到 summary.md。后续新增自动记录 session-log.md
- **文件**: .claude/skills/embedded-summary/SKILL.md

### 2026-06-11 10:09 工作方式反馈
- **问**: 将先建立流程再工作和边想边工作两种方式混合使用
- **答**: 多文件重构先说思路再动手，简单修改直接做，探索代码边找边呈现
- **文件**: memory/work-style-balanced.md

# 转向环采集上位机

配套固件：`Core/comm/pidtrace.c`（串口命令 `P` 开关采集，`C` 开关注入测试）。

## 帧格式
29 字节小端：`AA 55 | len=24 | 12×int16 | CRC16-CCITT(len+payload)`。
字段与缩放见 `pidtrace_host.py` 顶部注释，与固件 `pidtrace.c` 一一对应。

## 用法

```bash
# 列串口
python pidtrace_host.py --list

# 采集 10 s，落 CSV + 曲线（默认 115200，工程 USART1 配置）
python pidtrace_host.py --seconds 10 --csv run.csv --plot run.png

# 先开注入测试再采（台架验证增益×输入→输出，不需要车在线上跑）
python pidtrace_host.py --send C --seconds 12 --csv inject.csv --plot inject.png
```

`--parse-gains` 默认会向固件发 `o`，把回显里的 `track kp=.. kd=.. kg=..` 拿来复核公式；
读不到时退回 `config.h` 默认值并打印说明。

## 依赖
pyserial（必需）、matplotlib（可选，只用于画图）。
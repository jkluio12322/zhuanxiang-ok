#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BlueCar 转向环采集上位机（配套固件 Core/comm/pidtrace.c）。

帧格式（29 字节，小端）：
    AA 55 | len=24 | 12 x int16 payload | crc16(覆盖 len+payload, CCITT-FALSE)

payload 顺序与缩放：
    0 seq      x1        帧序号
    1 err      x10000    转向环输入 normalized_position (-1..+1)
    2 deriv    x100      低通后的 d(e)/dt (1/s)
    3 wz       x10       陀螺 +Z (dps)
    4 steer    x10000    限幅后转向量
    5 kv       x1000     车速解耦系数
    6 tgtL     x10       左轮目标 RPM
    7 tgtR     x10
    8 actL     x10       左轮实测 RPM
    9 actR     x10
   10 dt_us    x1        控制拍实测间隔
   11 flags   x1        bit0 line_ok | bit1-2 策略 | bit3-5 停车 | bit6-7 模式
                        | bit8 注入测试 | bit9 限幅前越界 | bit10 err 取传感器(环未跑)

用法：
    python pidtrace_host.py --list
    python pidtrace_host.py --seconds 10                      # 只收
    python pidtrace_host.py --seconds 12 --send C --tag inject # 开注入测试再收
    python pidtrace_host.py --seconds 20 --csv run.csv --plot run.png
"""
import argparse
import csv
import struct
import sys
import time

HDR = b"\xaa\x55"
PAYLOAD_LEN = 24
FRAME_LEN = PAYLOAD_LEN + 5
SCALE = (1.0, 10000.0, 100.0, 10.0, 10000.0, 1000.0,
         10.0, 10.0, 10.0, 10.0, 1.0, 1.0)
NAMES = ("seq", "err", "deriv", "wz", "steer", "kv",
         "tgtL", "tgtR", "actL", "actR", "dt_us", "flags")

MODE_TXT = {0: "IDLE", 1: "TRACK", 2: "MANUAL", 3: "?"}


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def decode_flags(flags):
    return {
        "line_ok": flags & 0x1,
        "policy": (flags >> 1) & 0x3,
        "stop": (flags >> 3) & 0x7,
        "mode": (flags >> 6) & 0x3,
        "inject": (flags >> 8) & 0x1,
        "sat": (flags >> 9) & 0x1,
        "sensor_err": (flags >> 10) & 0x1,
    }


def extract(buf, stats):
    """从字节流里取出所有完整帧，返回 (records, 剩余字节)。"""
    records = []
    i = 0
    n = len(buf)
    while True:
        j = buf.find(HDR, i)
        if j < 0:
            keep = buf[n - 1:] if (n - i) > 0 else b""
            return records, keep
        if n - j < FRAME_LEN:
            return records, buf[j:]
        if buf[j + 2] != PAYLOAD_LEN:
            stats["bad_len"] += 1
            i = j + 1
            continue
        body = buf[j + 2:j + 2 + PAYLOAD_LEN + 1]
        crc_rx = buf[j + FRAME_LEN - 2] | (buf[j + FRAME_LEN - 1] << 8)
        if crc16_ccitt(body) != crc_rx:
            stats["bad_crc"] += 1
            i = j + 1
            continue
        raw = struct.unpack_from("<12h", buf, j + 3)
        rec = {NAMES[k]: raw[k] / SCALE[k] for k in range(12)}
        rec["seq"] = raw[0]
        rec["flags"] = raw[11]
        rec["dt_us"] = raw[10]
        rec.update(decode_flags(raw[11]))
        rec["rx_t"] = time.time()
        records.append(rec)
        i = j + FRAME_LEN


def open_port(name, baud):
    try:
        import serial
    except ImportError:
        sys.exit("缺少 pyserial：pip install pyserial")
    port = serial.Serial(name, baud, timeout=0.05)
    port.reset_input_buffer()
    return port


def list_ports():
    from serial.tools import list_ports as lp
    items = list(lp.comports())
    if not items:
        print("没有发现串口")
    for p in items:
        print("%-8s %s" % (p.device, p.description))


def guess_port(prefer="COM19"):
    from serial.tools import list_ports as lp
    names = [p.device for p in lp.comports()]
    if prefer in names:
        return prefer
    return names[0] if names else None


def summarize(records, gains, gyro_sign, steer_limit):
    s = {}
    s["frames"] = len(records)
    if not records:
        return s
    span = records[-1]["rx_t"] - records[0]["rx_t"]
    s["rate"] = (len(records) - 1) / span if span > 0 else 0.0
    seq = [r["seq"] for r in records]
    lost = 0
    for a, b in zip(seq, seq[1:]):
        lost += (b - a - 1) & 0xFFFF
    s["seq_lost"] = lost
    s["seq_lost_pct"] = 100.0 * lost / max(1, len(records) + lost)
    for key in ("err", "deriv", "wz", "steer", "kv", "tgtL", "actL", "dt_us"):
        vals = [r[key] for r in records]
        s[key] = (min(vals), sum(vals) / len(vals), max(vals))
    dt = [r["dt_us"] for r in records]
    s["dt_dev_max"] = max(abs(d - 5000.0) for d in dt)
    s["sat_pct"] = 100.0 * sum(1 for r in records if r["sat"]) / len(records)
    s["inject_pct"] = 100.0 * sum(1 for r in records if r["inject"]) / len(records)
    s["line_ok_pct"] = 100.0 * sum(1 for r in records if r["line_ok"]) / len(records)
    kp, kd, kg = gains
    worst = 0.0
    acc = 0.0
    checked = 0
    for r in records:
        if r["sensor_err"]:
            continue            # 环未运行，err 是传感器原始值、steer 恒 0，不能拿来复核公式
        u = (kp * r["err"] + kd * r["deriv"] + gyro_sign * kg * r["wz"]) * r["kv"]
        u = max(-steer_limit, min(steer_limit, u))
        d = abs(u - r["steer"])
        worst = max(worst, d)
        acc += d
        checked += 1
    s["formula_n"] = checked
    s["formula_max"] = worst
    s["formula_mean"] = (acc / checked) if checked else 0.0
    return s


def print_summary(s, gains, steer_limit, gyro_sign):
    if not s.get("frames"):
        print("没有任何有效帧")
        return
    print("")
    print("== 采集统计 ==")
    print("有效帧 %d，序号缺口 %d 帧（丢帧率 %.2f%%），实测帧率 %.2f Hz"
          % (s["frames"], s["seq_lost"], s["seq_lost_pct"], s["rate"]))
    print("err      min %+0.4f  mean %+0.4f  max %+0.4f"
          % s["err"])
    print("deriv    min %+0.2f  mean %+0.2f  max %+0.2f  (1/s)" % s["deriv"])
    print("wz       min %+0.2f  mean %+0.2f  max %+0.2f  (dps)" % s["wz"])
    print("steer    min %+0.4f  mean %+0.4f  max %+0.4f  (限幅 ±%.2f)"
          % (s["steer"][0], s["steer"][1], s["steer"][2], steer_limit))
    print("kv       min %0.3f  mean %0.3f  max %0.3f" % s["kv"])
    print("tgtL     min %0.1f  mean %0.1f  max %0.1f  (RPM)" % s["tgtL"])
    print("actL     min %0.1f  mean %0.1f  max %0.1f  (RPM)" % s["actL"])
    print("dt_us    min %0.0f  mean %0.0f  max %0.0f，最大偏离标称 %0.0f us"
          % (s["dt_us"][0], s["dt_us"][1], s["dt_us"][2], s["dt_dev_max"]))
    print("限幅帧占比 %.2f%%，注入测试帧 %0.1f%%，有线帧 %0.1f%%"
          % (s["sat_pct"], s["inject_pct"], s["line_ok_pct"]))
    print("公式复核（kp=%.3f kd=%.3f kg=%.4f gyro_sign=%+d，参与 %d 帧）最大偏差 %.5f，平均 %.6f"
          % (gains[0], gains[1], gains[2], gyro_sign, s["formula_n"],
             s["formula_max"], s["formula_mean"]))


def write_csv(path, records):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["seq", "err", "deriv", "wz", "steer", "kv", "tgtL", "tgtR",
                         "actL", "actR", "dt_us", "line_ok", "policy", "stop",
                         "mode", "inject", "sat"])
        for r in records:
            writer.writerow([r["seq"], "%.4f" % r["err"], "%.2f" % r["deriv"],
                             "%.2f" % r["wz"], "%.4f" % r["steer"], "%.3f" % r["kv"],
                             "%.1f" % r["tgtL"], "%.1f" % r["tgtR"],
                             "%.1f" % r["actL"], "%.1f" % r["actR"],
                             r["dt_us"], r["line_ok"], r["policy"], r["stop"],
                             r["mode"], r["inject"], r["sat"]])


def draw_plot(path, records):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("（未安装 matplotlib，跳过曲线：pip install matplotlib）")
        return
    t = [i * 0.02 for i in range(len(records))]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t, [r["err"] for r in records], label="err")
    axes[0].plot(t, [r["steer"] for r in records], label="steer")
    axes[0].set_ylabel("err / steer")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)
    axes[1].plot(t, [r["deriv"] for r in records], label="deriv (1/s)")
    ax2 = axes[1].twinx()
    ax2.plot(t, [r["wz"] for r in records], "r-", label="wz (dps)")
    axes[1].set_ylabel("deriv")
    ax2.set_ylabel("wz")
    axes[1].grid(True)
    axes[2].plot(t, [r["tgtL"] for r in records], label="tgtL")
    axes[2].plot(t, [r["actL"] for r in records], label="actL")
    axes[2].plot(t, [r["tgtR"] for r in records], "--", label="tgtR")
    axes[2].plot(t, [r["actR"] for r in records], "--", label="actR")
    axes[2].set_ylabel("RPM")
    axes[2].set_xlabel("t (s, 50 Hz)")
    axes[2].legend(loc="upper right", ncol=2)
    axes[2].grid(True)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print("曲线已保存：%s" % path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="串口，默认自动找（优先 COM19）")
    ap.add_argument("--baud", type=int, default=115200,
                    help="波特率，默认 115200（Core/Src/usart.c 的 USART1 配置，config.h SERIAL_DEBUG_BAUD_RATE）")
    ap.add_argument("--seconds", type=float, default=8.0, help="采集时长秒")
    ap.add_argument("--send", default=None, help="开始前先发这条命令（如 C 开注入、P 关采集）")
    ap.add_argument("--csv", default=None, help="落盘 CSV 路径")
    ap.add_argument("--plot", default=None, help="曲线 PNG 路径")
    ap.add_argument("--kp", type=float, default=None, help="公式复核用 KP，默认从串口 'o' 回显读")
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--kg", type=float, default=None)
    ap.add_argument("--gyro-sign", type=int, default=-1, help="TRACK_GYRO_SIGN，默认 -1")
    ap.add_argument("--steer-limit", type=float, default=0.85, help="TRACK_STEER_LIMIT，默认 0.85")
    ap.add_argument("--list", action="store_true", help="列出串口")
    args = ap.parse_args()

    if args.list:
        list_ports()
        return

    port_name = args.port or guess_port()
    if not port_name:
        sys.exit("找不到串口：先用 --list 看设备，或 --port 指定")
    print("串口 %s @ %d（来源：工程 USART1 配置 / config.h SERIAL_DEBUG_BAUD_RATE）"
          % (port_name, args.baud))

    port = open_port(port_name, args.baud)
    stats = {"bad_len": 0, "bad_crc": 0}
    gains = (args.kp, args.kd, args.kg)
    textbuf = bytearray()
    records = []
    leftover = b""

    if args.send:
        port.write(("%s\r\n" % args.send).encode("ascii"))
        print("已发送命令：%s" % args.send)
        time.sleep(0.2)
    # 让固件回一行当前整定值，用来做公式复核（串口 'o'）
    port.write(b"o\r\n")
    time.sleep(0.25)

    # 排空串口/DAP 缓冲里的旧帧：不排空会让序号缺口统计失真（实测曾误报 68% 丢帧）
    t_drain = time.time() + 1.5
    while time.time() < t_drain:
        chunk = port.read(4096)
        if chunk:
            leftover = extract(chunk, {"bad_len": 0, "bad_crc": 0})[1]

    t_end = time.time() + args.seconds
    while time.time() < t_end:
        chunk = port.read(4096)
        if not chunk:
            continue
        data = leftover + chunk
        recs, leftover = extract(data, stats)
        for r in recs:
            records.append(r)
        # 非帧字节当中文/文本诊断看，顺手抓 "track kp=" 一行
        if b"track kp=" not in textbuf[-4096:]:
            textbuf.extend(chunk)
            if len(textbuf) > 8192:
                del textbuf[:-4096]
    port.close()

    txt = bytes(textbuf).decode("utf-8", "ignore")
    if None in gains or gains[0] is None:
        import re
        m = re.search(r"track kp=([-0-9.]+) kd=([-0-9.]+) kg=([-0-9.]+)", txt)
        if m:
            gains = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
            print("从串口读到整定值：kp=%.2f kd=%.2f kg=%.4f" % gains)
        else:
            gains = (0.80, 0.02, 0.0010)
            print("没读到整定值回显，按 config.h 默认 kp=0.80 kd=0.02 kg=0.0010 复核")

    print("原始字节流片段（前 3 帧）：")
    for r in records[:3]:
        print("  %s" % " ".join("%s=%.3f" % (k, r[k]) for k in
                                ("seq", "err", "deriv", "wz", "steer", "kv", "dt_us")))
    print("CRC 失败 %d 次，长度字段异常 %d 次" % (stats["bad_crc"], stats["bad_len"]))

    s = summarize(records, gains, args.gyro_sign, args.steer_limit)
    print_summary(s, gains, args.steer_limit, args.gyro_sign)
    if args.csv:
        write_csv(args.csv, records)
        print("CSV 已保存：%s（%d 行）" % (args.csv, len(records)))
    if args.plot:
        draw_plot(args.plot, records)


if __name__ == "__main__":
    main()

# -*- coding: utf-8 -*-
"""转角自检连续采样：全程 50 Hz 记录，事后按步切片分析。

用法: python turntest_record.py 45 3
数据帧（第二字节 0x56）字段见 Core/selftest/turntest.c 头注释。
输出: 同名 .csv + 终端分析表。
"""
import csv, struct, sys, time
import serial

FIELDS = 14
PAYLOAD = FIELDS * 2
FRAME = PAYLOAD + 5
SCALE = (1.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 0.1, 1e-4, 1.0)
CENTER = 0x18


def decode(buf):
    rows, i = [], 0
    while True:
        j = buf.find(b"\xaa\x56", i)
        if j < 0 or len(buf) - j < FRAME:
            break
        if buf[j + 2] != PAYLOAD:
            i = j + 1
            continue
        v = struct.unpack("<%dh" % FIELDS, buf[j + 3:j + 3 + PAYLOAD])
        crc = struct.unpack("<H", buf[j + 3 + PAYLOAD:j + FRAME])[0]
        c = 0xFFFF
        for b in buf[j + 2:j + 3 + PAYLOAD]:
            c ^= b << 8
            for _ in range(8):
                c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        if c != crc:
            i = j + 1
            continue
        rows.append([x * s for x, s in zip(v, SCALE)])
        i = j + FRAME
    return rows, buf[i:]


def main():
    step = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    laps = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    ser = serial.Serial("COM19", 115200, timeout=0.05)
    time.sleep(0.8)
    ser.read(8192)
    ser.write(b"Z\r\n")          # 关 pidtrace，留带宽给自检帧
    time.sleep(0.4)
    ser.write(b"q\r\n")          # 关 1 s 中文诊断
    time.sleep(0.4)
    ser.read(8192)

    cmd = ("B %g %d\r\n" % (step, laps)).encode()
    for _ in range(12):            # 陀螺没标定好会回 ERR，等它
        ser.write(cmd)
        t0 = time.time()
        buf = b""
        while time.time() - t0 < 1.0:
            c = ser.read(256)
            if c:
                buf += c
        txt = buf.decode("utf-8", "replace")
        if "OK selftest" in txt:
            break
        if "not ready" not in txt and "ERR" not in txt:
            break
    rows, lines, pend = [], [], b""
    t_end = time.time() + 240
    while time.time() < t_end:
        c = ser.read(512)
        if not c:
            continue
        pend += c
        got, pend = decode(pend)
        rows.extend(got)
        txt = pend.decode("utf-8", "replace")
        if "\n" in txt:
            parts = txt.split("\n")
            pend = parts[-1].encode()
            for ln in parts[:-1]:
                ln = ln.strip()
                if ln.startswith("[自检]") or ln.startswith("ERR") or ln.startswith("OK"):
                    lines.append(ln)
                    print(ln)
        if any("汇总" in l for l in lines[-3:]):
            break
    ser.write(b"m 0 0\r\n")

    with open("turntest_%gx%d.csv" % (step, laps), "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t_ms", "yaw", "cum", "spin_err", "actL", "actR", "tgtL", "tgtR",
                    "mask", "black", "phase", "idx", "resid", "pos", "wsum"])
        for r in rows:
            w.writerow(["%.1f" % r[0], "%.1f" % r[1], "%.1f" % r[2], "%.2f" % r[3],
                        "%.1f" % r[4], "%.1f" % r[5], "%.1f" % r[6], "%.1f" % r[7],
                        int(r[8]), int(r[9]), int(r[10]) >> 8, int(r[10]) & 0xFF,
                        "%.1f" % r[11], "%.4f" % r[12], int(r[13])])
    print("\n记录 %d 帧 -> turntest_%gx%d.csv" % (len(rows), step, laps))
    analyze(rows)


def pat(m):
    m = int(m)
    return "".join("1" if m >> i & 1 else "0" for i in range(8))


def analyze(rows):
    # 相邻帧 t_ms 展开回绕
    t, last = [], None
    for r in rows:
        v = r[0]
        if last is not None:
            while v < last - 1000:
                v += 65536.0
        t.append(v)
        last = v
    settle = [r for r in rows if (int(r[10]) >> 8) == 2]
    if not settle:
        print("没有停车窗口样本"); return
    groups = {}
    for i, r in enumerate(rows):
        if (int(r[10]) >> 8) == 2:
            k = int(r[10]) & 0xFF
            groups.setdefault(k, []).append((t[i], r))
    print("\n%-4s %-8s %-7s %-9s %-9s %-6s %-9s" %
          ("步", "累计°", "窗口帧", "残差末端°", "偏移均值", "偏移σ", "90°达标率"))
    resids, offsets, hits = [], [], 0
    for k in sorted(groups):
        g = groups[k]
        off = [x[1][12] for x in g]
        mean = sum(off) / len(off)
        var = sum((o - mean) ** 2 for o in off) / len(off)
        cum = g[-1][1][2]
        align = abs(((cum + 45.0) % 90.0) - 45.0) < 1.0
        ok = sum(1 for x in g if int(x[1][8]) == CENTER) / len(g) * 100.0
        if align:
            hits += 1 if ok > 90.0 else 0
        resids.append(g[-1][1][11])
        offsets.append(mean)
        print("%-4d %-8.1f %-7d %-9.1f %+9.4f %7.4f %7.0f%%%s" %
              (k, cum, len(g), g[-1][1][11], mean, var ** 0.5, ok,
               "  <-对臂" if align else ""))
    n = len(resids)
    m = sum(resids) / n
    print("\n残差: 均值%+.2f° σ%.2f° 最大%.2f°   |残差|>1°: %d/%d" %
          (m, (sum((x - m) ** 2 for x in resids) / n) ** 0.5,
           max(abs(x) for x in resids), sum(1 for x in resids if abs(x) > 1.0), n))
    mo = sum(offsets) / len(offsets)
    print("停车横向偏移: 均值%+.4f σ%.4f（>0 = 线偏右，单位=归一化位置，1.0≈最外侧传感器）" %
          (mo, (sum((x - mo) ** 2 for x in offsets) / len(offsets)) ** 0.5))
    print("90° 停点中间两路达标: %d/%d 步" % (hits, sum(
        1 for k in sorted(groups)
        if abs(((groups[k][-1][1][2] + 45.0) % 90.0) - 45.0) < 1.0)))
    st = [r[1] for r in settle]
    print("停车窗口偏移绝对值均值 %.4f（越小越居中）" % (sum(abs(x) for x in st) / len(st)))
    dur = [t[i + 1] - t[i] for i in range(len(t) - 1)]
    dur.sort()
    print("帧间隔 中位%.1fms 90分位%.1fms 最大%.1fms（50 Hz 标称 20 ms）" %
          (dur[len(dur) // 2], dur[int(len(dur) * 0.9)], dur[-1]))


if __name__ == "__main__":
    main()

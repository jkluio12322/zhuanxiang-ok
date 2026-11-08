# -*- coding: utf-8 -*-
"""转角环扫参：一次烧录，串口 V 命令逐组改参数，每组跑 B 45 1（8 步），
统一用 50 Hz 自检帧算「单步运动时长 / 残差 / 过冲深度」。

用法: python sweep.py            # 跑内置网格
      python sweep.py "4 3 .3 180 .8 4"   # 只跑一组
"""
import csv, struct, sys, time
import serial

FIELDS, PAYLOAD = 14, 28
FRAME = PAYLOAD + 5
SCALE = (1.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 0.1, 1e-4, 1.0)

GRID = [
    (6.2, 3, 0.35, 180, 0.8, 4),
    (6.2, 3, 0.35, 120, 0.8, 4),
    (6.2, 3, 0.35, 90, 0.8, 4),
]


def decode(buf):
    rows, i = [], 0
    while True:
        j = buf.find(b"\xaa\x56", i)
        if j < 0 or len(buf) - j < FRAME:
            break
        if buf[j + 2] != PAYLOAD:
            i = j + 1; continue
        v = struct.unpack("<%dh" % FIELDS, buf[j + 3:j + 3 + PAYLOAD])
        crc = struct.unpack("<H", buf[j + 3 + PAYLOAD:j + FRAME])[0]
        c = 0xFFFF
        for b in buf[j + 2:j + 3 + PAYLOAD]:
            c ^= b << 8
            for _ in range(8):
                c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        if c != crc:
            i = j + 1; continue
        rows.append([x * s for x, s in zip(v, SCALE)])
        i = j + FRAME
    return rows, buf[i:]


def stats(rows):
    """按 turntest 帧里的 phase 字段切片（与 turntest_record.py 的 analyze 同一套）：
    phase = (r[10] >> 8)，0=空闲 1=转角中 2=停车窗口 3=结束；idx = r[10] & 0xFF。"""
    if len(rows) < 40:
        return None
    t, last = [], None
    for r in rows:
        v = r[0]
        if last is not None:
            while v < last - 1000:
                v += 65536.0
        t.append(v)
        last = v

    spin, settle = {}, {}
    act = {}
    for i, r in enumerate(rows):
        ph = int(r[10]) >> 8
        k = int(r[10]) & 0xFF
        if ph == 1:
            spin.setdefault(k, []).append(t[i])
        elif ph == 2:
            settle.setdefault(k, []).append((t[i], r[11]))
            act.setdefault(k, []).append(abs(r[4]))

    dur, resid = [], []
    for k in sorted(settle):
        if k in spin and len(spin[k]) > 1:
            dur.append(spin[k][-1] - spin[k][0])
        resid.append(settle[k][-1][1])
    if not resid:
        return None
    m = sum(resid) / len(resid)
    sd = (sum((x - m) ** 2 for x in resid) / (len(resid) - 1)) ** 0.5 if len(resid) > 1 else 0.0
    chat = [max(v) for v in act.values() if v]
    return dict(n=len(resid),
                chat=(sum(chat) / len(chat)) if chat else 0.0,
                dur=(sum(dur) / len(dur)) if dur else 0.0,
                resid=m,
                resid_sd=sd,
                resid_max=max(abs(x) for x in resid),
                frames=len(rows))


def run(ser, cfg, laps=1):
    kp, ki, kd, rpm, tol, db = cfg
    ser.write(("V %.3f %.3f %.3f %.2f %.2f %.2f\r\n" % cfg).encode())
    time.sleep(0.3)
    ser.read(4096)
    for _ in range(12):
        ser.write(("B 45 %d\r\n" % laps).encode())
        t0 = time.time(); buf = b""
        while time.time() - t0 < 1.0:
            buf += ser.read(256)
        if b"OK selftest" in buf:
            break
        time.sleep(1.0)
    time.sleep(5.0)          # 每组之间静置，让车/电机回常温、也给现场留复位时间
    t0 = time.time(); buf = b""; rows = []
    limit = laps * 8 * 4.2 + 12.0
    while time.time() - t0 < limit:
        chunk = ser.read(1024)
        buf += chunk
        got, buf = decode(buf)
        rows += got
        if b"\xe6\xb1\x87\xe6\x80\xbb" in chunk:          # "[自检] 汇总"
            break
    return stats(rows)


def main():
    grid = GRID
    if len(sys.argv) > 1:
        grid = [tuple(float(x) for x in sys.argv[1].split())]
    ser = serial.Serial("COM19", 115200, timeout=0.05)
    time.sleep(0.8); ser.read(8192)
    ser.write(b"Z\r\n"); time.sleep(0.3)
    ser.write(b"q\r\n"); time.sleep(0.3)
    ser.read(8192)
    print("kp    ki    kd    rpm   tol   db   | 步数 单步s  残差°   σ°   最大|残|° 停车窗|轮速|峰")
    out = []
    for cfg in grid:
        st = run(ser, cfg)
        if st is None:
            print(" ".join("%5.2f" % c for c in cfg), "| 无有效帧")
            continue
        print("%5.2f %5.2f %5.2f %5.0f %5.2f %5.2f | %3d  %5.2f  %+6.2f %5.2f  %6.2f   %5.2f"
              % (cfg + (st["n"], st["dur"] / 1000.0, st["resid"], st["resid_sd"], st["resid_max"], st["chat"])))
        out.append((cfg, st))
    if out:
        best = min(out, key=lambda x: (abs(x[1]["resid_max"]), x[1]["dur"]))
        print("\n最稳的一组:", best[0])


main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""原地旋转时用编码器轮速反推车体角速度，跟陀螺比对，判陀螺标度是否可信。
   omega_enc = (v_R - v_L) / L  ；斜率 1/L 由 (actR-actL) 对 wz 的最小二乘得到。
   用法: python spin_probe.py --deg 90 [--port COM19]"""
import argparse, struct, sys, time
import serial

SC = (1.0, 1e-4, 0.01, 0.1, 1e-4, 1e-3, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0)
NAMES = ("seq", "err", "deriv", "wz", "steer", "kv", "tgtL", "tgtR",
         "actL", "actR", "dt_us", "flags")
WHEEL_R = 0.065          # ENCODER_WHEEL_RADIUS_M


def frames(buf):
    out = []
    i = 0
    while True:
        j = buf.find(b"\xaa\x55", i)
        if j < 0 or len(buf) - j < 29:
            break
        if buf[j + 2] != 24:
            i = j + 1
            continue
        p = buf[j + 3:j + 27]
        vals = struct.unpack("<12h", p)
        out.append([v * s for v, s in zip(vals, SC)])
        i = j + 29
    return out, buf[i:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM19")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--deg", type=float, default=90.0)
    ap.add_argument("--secs", type=float, default=8.0)
    a = ap.parse_args()

    ser = serial.Serial(a.port, a.baud, timeout=0.2)
    time.sleep(1.0)
    ser.reset_input_buffer()
    ser.write(b"P\r\n")            # trace 翻转
    time.sleep(0.4)
    txt = ser.read(4096)
    print("P 回显:", txt.decode("utf-8", "replace").strip().replace("\n", " | "))
    ser.reset_input_buffer()
    ser.write(("r %g\r\n" % a.deg).encode())
    print("发送: r %g" % a.deg, flush=True)

    txt = b""
    buf = b""
    rows = []
    t0 = time.time()
    while time.time() - t0 < a.secs:
        chunk = ser.read(512)
        if not chunk:
            continue
        txt += chunk
        buf += chunk
        got, buf = frames(buf)
        rows.extend(got)
    ser.write(b"Z\r\n")

    if not rows:
        print("没收到采集帧")
        return
    # 只取真正在转的帧
    print("r 回显:", txt.decode("utf-8", "replace").strip().replace("\n", " | ")[:200])
    print("max|wz|=%.1f  max|actL|=%.1f" % (max(abs(r[3]) for r in rows),
                                             max(abs(r[8]) for r in rows)))
    moving = [r for r in rows if abs(r[3]) > 20.0]
    print("帧数 %d, 旋转中 %d" % (len(rows), len(moving)))
    if not moving:
        return
    n = float(len(moving))
    wz = sum(r[3] for r in moving) / n
    actl = sum(r[8] for r in moving) / n
    actr = sum(r[9] for r in moving) / n
    print("均值 wz=%.1f dps  actL=%.1f rpm  actR=%.1f rpm" % (wz, actl, actr))
    dw = actr - actl
    vdiff = dw * 2.0 * 3.14159265 * WHEEL_R / 60.0
    omega = wz * 3.14159265 / 180.0
    print("轮速差 %.1f rpm = %.4f m/s ；车身角速度 %.3f rad/s" % (dw, vdiff, omega))
    if abs(omega) > 1e-3:
        print("反推轮距 L = %.4f m  (整车轮距通常 0.14~0.18 m)" % (vdiff / omega))


if __name__ == "__main__":
    main()

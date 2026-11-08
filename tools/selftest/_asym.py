# -*- coding: utf-8 -*-
"""给定若干组常数左右目标，测两轮实际转速，量化左右不对称。"""
import struct, time, serial
SC = (1.0, 1e-4, 0.01, 0.1, 1e-4, 1e-3, 0.1, 0.1, 0.1, 0.1, 1.0, 1.0)
def frames(buf):
    out, i = [], 0
    while True:
        j = buf.find(b"\xaa\x55", i)
        if j < 0 or len(buf) - j < 29:
            break
        if buf[j+2] != 24:
            i = j + 1; continue
        vals = struct.unpack("<12h", buf[j+3:j+27])
        out.append([v*s for v, s in zip(vals, SC)]); i = j + 29
    return out, buf[i:]
ser = serial.Serial("COM19", 115200, timeout=0.2)
time.sleep(0.6)
ser.reset_input_buffer()
for c in (b"Z", b"P"):          # Z 关、P 翻转 -> 确定是开
    ser.write(c + b"\r\n"); time.sleep(0.4)
print("trace:", ser.read(4096).decode("utf-8","replace").split("OK")[-1][:12])
buf = b""
def collect(secs):
    global buf
    rows = []
    t0 = time.time()
    while time.time()-t0 < secs:
        c = ser.read(256)
        if not c: continue
        buf += c
        g, buf = frames(buf); rows.extend(g)
    return rows
collect(0.5)
for l, r in ((30, -30), (20, -20), (10, -10), (5, -5), (2, -2), (1, -1)):
    ser.write(("m %d %d\r\n" % (l, r)).encode())
    collect(0.8)              # 过渡
    rows = collect(1.6)       # 稳态
    n = float(len(rows)) or 1.0
    if not rows:
        print("cmd %+4d/%+4d -> 无帧" % (l, r)); continue
    al = sum(x[8] for x in rows)/n
    ar = sum(x[9] for x in rows)/n
    print("cmd %+4d/%+4d -> actL %+6.2f  actR %+6.2f   右/左 %.2f" % (l, r, al, ar, (abs(ar)/abs(al)) if al else 0.0))
ser.write(b"m 0 0\r\n"); time.sleep(0.3)
ser.write(b"x\r\n"); time.sleep(0.3)
ser.write(b"Z\r\n")

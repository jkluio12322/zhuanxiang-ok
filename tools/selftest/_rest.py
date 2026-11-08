# -*- coding: utf-8 -*-
"""静止时读两轮指令与实际转速，看谁在爬。"""
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
ser.reset_input_buffer(); ser.write(b"P\r\n"); time.sleep(0.4)
print(ser.read(2048).decode("utf-8","replace").split("OK")[-1][:20])
ser.reset_input_buffer()
buf, rows = b"", []
t0 = time.time()
while time.time()-t0 < 6.0:
    c = ser.read(512)
    if not c: continue
    buf += c
    g, buf = frames(buf); rows.extend(g)
for r in rows[::10]:
    print("tgtL%6.1f actL%6.1f | tgtR%6.1f actR%6.1f | wz%6.1f" % (r[6], r[8], r[7], r[9], r[3]))
n = float(len(rows))
print("均值 actL=%.2f actR=%.2f  |actL|max=%.1f |actR|max=%.1f" % (
    sum(r[8] for r in rows)/n, sum(r[9] for r in rows)/n,
    max(abs(r[8]) for r in rows), max(abs(r[9]) for r in rows)))

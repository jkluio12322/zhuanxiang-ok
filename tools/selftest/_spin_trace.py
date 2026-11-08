# -*- coding: utf-8 -*-
"""跑一次 r 45，记录全过程两轮指令/实际转速。"""
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
ser.reset_input_buffer(); ser.write(b"P\r\n"); time.sleep(0.4); ser.read(4096)
ser.reset_input_buffer()
buf, rows = b"", []
t0 = time.time()
ser.write(b"r 45\r\n")
while time.time()-t0 < 7.0:
    c = ser.read(512)
    if not c: continue
    buf += c
    g, buf = frames(buf); rows.extend(g)
    for r in g:
        print("%5.2fs tgt %6.1f/%6.1f  act %6.1f/%6.1f  wz %6.1f  sterr %5.1f" % (
            time.time()-t0, r[6], r[7], r[8], r[9], r[3], 0.0))

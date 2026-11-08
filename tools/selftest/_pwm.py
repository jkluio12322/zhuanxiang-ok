import struct, time, serial
SC = (1.0,1e-4,0.01,0.1,1e-4,1e-3,0.1,0.1,0.1,0.1,1.0,1.0,0.1,0.1,0.01,0.01)
LEN = 32
def frames(buf):
    out, i = [], 0
    while True:
        j = buf.find(b"\xaa\x55", i)
        if j < 0 or len(buf)-j < LEN+5: break
        if buf[j+2] != LEN: i=j+1; continue
        v = struct.unpack("<16h", buf[j+3:j+35])
        out.append([a*s for a,s in zip(v,SC)]); i = j+LEN+5
    return out, buf[i:]
ser = serial.Serial("COM19",115200,timeout=0.2)
time.sleep(0.8); ser.read(8192); ser.reset_input_buffer()
buf=b""
def win(sec):
    global buf
    rows=[]; t0=time.time()
    while time.time()-t0<sec:
        c=ser.read(256)
        if c:
            buf+=c; g,buf=frames(buf); rows.extend(g)
    return rows
for _ in range(3):
    ser.write(b"P\r\n"); time.sleep(0.5)
    if len(win(0.6))>5: break
win(0.6)
print("fmt: tgtL tgtR actL actR cmdL cmdR pwmL pwmR")
for name,c in [("m 1 -1","m 1 -1"),("m 2 -2","m 2 -2"),("m 0 0","m 0 0")]:
    ser.write(c.encode()+b"\r\n"); time.sleep(1.5)
    rows=win(2.0); n=float(len(rows)) or 1.0
    print("%-8s tgt %6.2f/%6.2f  act %6.2f/%6.2f  cmd %6.2f/%6.2f  pwm %6.2f/%6.2f  n=%d" % (
        name, rows[-1][6], rows[-1][7],
        sum(r[8] for r in rows)/n, sum(r[9] for r in rows)/n,
        rows[-1][12], rows[-1][13], sum(r[14] for r in rows)/n, sum(r[15] for r in rows)/n, len(rows)))
ser.write(b"m 0 0\r\n")

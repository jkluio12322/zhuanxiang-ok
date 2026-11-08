import struct, time, serial, sys
SC = (1.0,1e-4,0.01,0.1,1e-4,1e-3,0.1,0.1,0.1,0.1,1.0,1.0,0.1,0.1,0.01,0.01)
LEN=32
def frames(buf):
    out,i=[],0
    while True:
        j=buf.find(b"\xaa\x55",i)
        if j<0 or len(buf)-j<LEN+5: break
        if buf[j+2]!=LEN: i=j+1; continue
        v=struct.unpack("<16h",buf[j+3:j+35]); out.append([a*s for a,s in zip(v,SC)]); i=j+LEN+5
    return out,buf[i:]
ser=serial.Serial("COM19",115200,timeout=0.1)
time.sleep(0.8); ser.read(8192); ser.reset_input_buffer()
buf=b""
def win(sec):
    global buf
    rows=[]; t0=time.time()
    while time.time()-t0<sec:
        c=ser.read(128)
        if c:
            buf+=c; g,buf=frames(buf); rows.extend(g)
    return rows
for _ in range(3):
    ser.write(b"P\r\n"); time.sleep(0.5)
    if len(win(0.6))>5: break
win(0.8)
ser.write(b"m 30 -30\r\n"); time.sleep(2.0)
ser.write(b"m 0 0\r\n"); t0=time.time()
rows=[]; t=0.0
while t < 3.0:
    r=win(0.1); rows.extend(r); t=time.time()-t0
    if r:
        last=r[-1]
        print("t=%.2fs actL %6.2f actR %6.2f pwm %6.2f/%6.2f" % (t, last[8], last[9], last[14], last[15]))
    if abs(last[8])<0.4 and abs(last[9])<0.4 and t>0.3: break
print("done at t=%.2f" % t)

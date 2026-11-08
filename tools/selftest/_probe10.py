
import serial, time
P = serial.Serial('COM19', 115200, timeout=0.05)
def drain(sec):
    t0=time.time(); buf=b''
    while time.time()-t0<sec: buf += P.read(512)
    return buf
def d():
    P.write(b'd\n'); return [l for l in drain(0.5).decode('utf-8','replace').splitlines() if l.startswith('spd kp')]
P.reset_input_buffer(); drain(0.5)
P.write(b'Z\n'); time.sleep(0.2); drain(0.2)
P.write(b'q\n'); time.sleep(0.2); drain(0.2)
print("B 之前:", d())
for _ in range(12):
    P.write(b'B 45 1\n'); r = drain(0.6).decode('utf-8','replace')
    if 'OK selftest' in r: print("started"); break
    if 'not ready' in r: time.sleep(1.0)
t0 = time.time()
while time.time() - t0 < 26:
    drain(0.5)
print("B 之后:", d())
P.write(b'x\n')

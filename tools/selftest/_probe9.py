
import serial, time
P = serial.Serial('COM19', 115200, timeout=0.05)
def drain(sec):
    t0=time.time(); buf=b''
    while time.time()-t0<sec: buf += P.read(512)
    return buf
def d():
    P.write(b'd\n'); return drain(0.5).decode('utf-8','replace')
P.reset_input_buffer(); drain(0.5)
print("=== 上电后立刻 ===")
print("\n".join(l for l in d().splitlines() if 'spd' in l or 'smp_lat' in l))
P.write(b'm 40 -40\n'); drain(1.0)
print("=== m 40 -40 稳态 ===")
print("\n".join(l for l in d().splitlines() if 'spd' in l))
P.write(b'm 0 0\n'); drain(0.5)

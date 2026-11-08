
import serial, time
P = serial.Serial('COM19', 115200, timeout=0.05)
def drain(sec):
    t0=time.time(); buf=b''
    while time.time()-t0<sec: buf += P.read(512)
    return buf
def cmd(s, wait=0.2):
    P.write((s+'\n').encode()); time.sleep(wait); return drain(0.15)

P.reset_input_buffer(); drain(0.3)
print(cmd('d').decode('utf-8','replace'))
print(cmd('o').decode('utf-8','replace'))
print("--- m 20 -20 ---")
cmd('m 20 -20'); print(cmd('d',0.6).decode('utf-8','replace'))
print("--- r 45 ---")
cmd('m 0 0'); cmd('y'); cmd('r 45', 0.5); print(cmd('o',0.5).decode('utf-8','replace'))
print(cmd('d',0.6).decode('utf-8','replace'))

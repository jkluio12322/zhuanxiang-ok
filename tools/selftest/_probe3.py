
import serial, time, sys
P = serial.Serial('COM19', 115200, timeout=0.05)
def cmd(s):
    P.write((s + '\n').encode())
    time.sleep(0.15)
    P.reset_input_buffer()
cmd('x'); cmd('Z')
cmd('m 20 -20')
t0 = time.time(); buf = b''
while time.time() - t0 < 1.5:
    buf += P.read(256)
print("m 20 -20 期间字节数", len(buf))
P.reset_input_buffer()
cmd('m 0 0'); cmd('y')
cmd('r 45')
t0 = time.time(); buf = b''
while time.time() - t0 < 2.5:
    buf += P.read(256)
P.write(b'd\n'); time.sleep(0.4); d = P.read(2000).decode('utf-8', 'replace')
P.write(b'o\n'); time.sleep(0.4); o = P.read(2000).decode('utf-8', 'replace')
print("r45 期间字节数", len(buf))
print(d[-600:])
print(o[-400:])

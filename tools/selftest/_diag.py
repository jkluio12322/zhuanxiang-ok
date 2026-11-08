import serial, time
ser = serial.Serial("COM19", 115200, timeout=0.3)
time.sleep(0.5)
for c in (b"Z", b"o", b"d"):
    ser.write(c + b"\r\n"); time.sleep(0.5)
t0 = time.time(); buf = b""
while time.time() - t0 < 3.5:
    buf += ser.read(4096)
print(buf.decode("utf-8", "replace"))

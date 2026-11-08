import serial, time
ser = serial.Serial("COM19", 115200, timeout=0.3)
time.sleep(0.5)
ser.reset_input_buffer()
for c in (b"Z", b"m 30 -30"):
    ser.write(c + b"\r\n"); time.sleep(0.5)
    print(c, "->", ser.read(4096).decode("utf-8","replace").strip().replace("\n"," | ")[:300])
ser.reset_input_buffer()
t0=time.time(); buf=b""
while time.time()-t0 < 2.0:
    buf += ser.read(4096)
lines=[l for l in buf.decode("utf-8","replace").split("\n") if l.startswith("lt:")]
print("最后一行:", lines[-1] if lines else "无")
ser.write(b"m 0 0\r\n"); time.sleep(0.2); ser.write(b"x\r\n")

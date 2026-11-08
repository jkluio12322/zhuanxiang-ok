
import serial, time, struct
s=serial.Serial('COM19',115200,timeout=0.2)
def send(c):
    s.write((c+'\r\n').encode()); time.sleep(0.25)
time.sleep(1.0)
s.reset_input_buffer()
send('m 0 0'); send('Z'); send('P')
buf=b''; t0=time.time(); vals=[]
while time.time()-t0 < 6.0:
    buf+=s.read(4096)
    while True:
        i=buf.find(b'\xaa\x55')
        if i<0 or len(buf)-i < 37: break
        f=buf[i:i+37]; buf=buf[i+37:]
        if f[2]!=32: continue
        v=struct.unpack('<12h', f[3:27])
        vals.append((v[6]/10.0, v[7]/10.0, v[8]/10.0, v[9]/10.0))  # tgtL tgtR actL actR
print("帧数",len(vals))
if vals:
    print("max|actL|=%.2f max|actR|=%.2f max|tgt|=%.2f"%(
      max(abs(v[2]) for v in vals), max(abs(v[3]) for v in vals), max(abs(v[0]) for v in vals)))
send('P')
s.close()

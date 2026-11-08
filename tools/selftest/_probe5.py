
import serial, time, struct
P = serial.Serial('COM19', 115200, timeout=0.05)
def drain(sec):
    t0=time.time(); buf=b''
    while time.time()-t0<sec: buf += P.read(512)
    return buf
P.reset_input_buffer(); drain(0.3)
P.write(b'x\n'); time.sleep(0.2); drain(0.2)
# 确保采集开：先发 Z 关，再发 P 开
P.write(b'Z\n'); time.sleep(0.2); drain(0.2)
P.write(b'P\n'); time.sleep(0.2); drain(0.2)
P.write(b'm 20 -20\n')
buf = drain(1.2)

SC=(1.0,10000.0,100.0,10.0,10000.0,1000.0,10.0,10.0,10.0,10.0,1.0,1.0,10.0,10.0,100.0,100.0)
i=0; n=0
last=None
while i < len(buf)-36:
    if buf[i]==0xAA and buf[i+1]==0x55:
        ln=buf[i+2]
        if ln<=34 and i+3+ln+2<=len(buf):
            pay=buf[i+3:i+3+ln]
            vals=struct.unpack('<'+'h'*(ln//2), pay)
            f=[v/s for v,s in zip(vals,SC)]
            last=f; n+=1
            i+=3+ln+2; continue
    i+=1
print("frames", n)
if last:
    names=['seq','err','deriv','wz','steer','kv','tgtL','tgtR','actL','actR','dt_us','flags','cmdL','cmdR','pwmL','pwmR']
    for k,v in zip(names,last): print(" %-6s %10.2f"%(k,v))

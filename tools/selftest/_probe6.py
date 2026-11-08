
import serial, time, struct
P = serial.Serial('COM19', 115200, timeout=0.05)
def drain(sec):
    t0=time.time(); buf=b''
    while time.time()-t0<sec: buf += P.read(512)
    return buf
P.reset_input_buffer(); drain(0.3)
P.write(b'Z\n'); time.sleep(0.2); drain(0.2)
P.write(b'P\n'); time.sleep(0.2); drain(0.2)
SC=(1.0,10000.0,100.0,10.0,10000.0,1000.0,10.0,10.0,10.0,10.0,1.0,1.0,10.0,10.0,100.0,100.0)
names=['seq','err','deriv','wz','steer','kv','tgtL','tgtR','actL','actR','dt_us','flags','cmdL','cmdR','pwmL','pwmR']
for cmdtxt in ['m 100 -100', 'm 40 -40']:
    P.write((cmdtxt+'\n').encode())
    buf = drain(1.0)
    i=0; last=None; n=0
    while i < len(buf)-36:
        if buf[i]==0xAA and buf[i+1]==0x55:
            ln=buf[i+2]
            if ln<=34 and i+3+ln+2<=len(buf):
                vals=struct.unpack('<'+'h'*(ln//2), buf[i+3:i+3+ln])
                last=[v/s for v,s in zip(vals,SC)]; n+=1
                i+=3+ln+2; continue
        i+=1
    print(cmdtxt, "frames", n, " -> " + ", ".join("%s=%.1f"%(k,v) for k,v in zip(names,last) if k in ('cmdL','pwmL','actL','tgtL')))
P.write(b'm 0 0\n')

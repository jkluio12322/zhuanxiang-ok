
import serial, time, struct
P = serial.Serial('COM19', 115200, timeout=0.05)
def drain(sec):
    t0=time.time(); buf=b''
    while time.time()-t0<sec: buf += P.read(512)
    return buf
time.sleep(3.0)
P.reset_input_buffer(); drain(0.3)
P.write(b'Z\n'); time.sleep(0.2); drain(0.2)
P.write(b'P\n'); time.sleep(0.2); drain(0.2)
SC=(1.0,10000.0,100.0,10.0,10000.0,1000.0,10.0,10.0,10.0,10.0,1.0,1.0,10.0,10.0,100.0,100.0)
def snap(cmdtxt, sec=1.0):
    P.write((cmdtxt+'\n').encode()); buf = drain(sec)
    i=0; last=None
    while i < len(buf)-36:
        if buf[i]==0xAA and buf[i+1]==0x55:
            ln=buf[i+2]
            if ln<=34 and i+3+ln+2<=len(buf):
                vals=struct.unpack('<'+'h'*(ln//2), buf[i+3:i+3+ln])
                last=[v/s for v,s in zip(vals,SC)]
                i+=3+ln+2; continue
        i+=1
    if last is None: print(cmdtxt, "no frame"); return
    print("%-14s cmdL=%.1f pwmL=%.2f actL=%.2f" % (cmdtxt, last[12], last[14], last[8]))
snap('m 40 -40'); snap('m 0 0'); snap('r 45', 2.2)

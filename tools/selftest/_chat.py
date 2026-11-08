
import csv
r=list(csv.DictReader(open('turntest_45x3.csv')))
act=[abs(float(x['actL'])) for x in r]
ph=[int(x['phase'])>>8 for x in r]
tm=[float(x['t_ms']) for x in r]
tot=0; mx=0; worst=0
for i in range(1,len(r)):
    if ph[i]==2 and act[i]>0.5:
        tot+=1
        if act[i]>mx: mx=act[i]
print("停车窗内 |actL|>0.5 的帧数", tot, "占停车帧 %.1f%%"%(100.0*tot/max(1,sum(1 for p in ph if p==2))))
print("停车窗内 |actL| 峰值 %.1f rpm"%mx)
# 运动段 pwm 与 act 的一场搏斗：actL 符号翻转次数
flip=sum(1 for i in range(1,len(r)) if (float(r[i]['actL'])>1 and float(r[i-1]['actL'])<-1) or (float(r[i]['actL'])<-1 and float(r[i-1]['actL'])>1))
print("actL 正负翻转次数", flip)


import csv, collections, math
rows=list(csv.DictReader(open('turntest_45x2.csv',encoding='utf-8')))
for r in rows:
    for k in ('t_ms','yaw','cum','actL','actR','resid'): r[k]=float(r[k])
# 取一步（idx=1）旋转中段做拟合
g=[r for r in rows if int(r['idx'])==1]
mv=[r for r in g if abs(r['actL'])>20]
print("步1 旋转帧数", len(mv))
v=[]
for a,b in zip(mv,mv[1:]):
    dt=(b['t_ms']-a['t_ms'])/1000.0
    if dt<=0: continue
    dyaw=(b['yaw']-a['yaw'])
    if dyaw>180: dyaw-=360
    if dyaw<-180: dyaw+=360
    om=dyaw/dt            # 度/秒（真实，来自陀螺积分）
    diff=b['actL']-b['actR']   # rpm 差速
    v.append((om,diff,b['actL'],b['actR']))
om=[x[0] for x in v]; df=[x[1] for x in v]
print("角速度 均值 %.1f 度/秒 (%.2f rad/s)" % (sum(om)/len(om), sum(om)/len(om)*math.pi/180))
print("差速 rpm 均值 %.1f  actL %.1f actR %.1f" % (sum(df)/len(df), sum(x[2] for x in v)/len(v), sum(x[3] for x in v)/len(v)))
w= sum(df)/len(df)*0.00681/ (sum(om)/len(om)*math.pi/180)
print("推出等效轮距 %.3f m" % abs(w))
print("若轮距 0.15 m，理论角速度 %.1f 度/秒 → 打滑率 %.0f%%" % (abs(sum(df)/len(df)*0.00681/0.15)*180/math.pi, 100*(1-abs(sum(om)/len(om))/ (abs(sum(df)/len(df)*0.00681/0.15)*180/math.pi))))


import csv
r=list(csv.DictReader(open('turntest_45x3.csv')))
t=[float(x['t_ms']) for x in r]; e=[float(x['spin_err']) for x in r]
durs=[]; inrun=False; t0=0
for i in range(len(e)):
    if not inrun and abs(e[i])>20:
        inrun=True; t0=t[i]
    elif inrun and abs(e[i])<1.0:
        durs.append((t[i]-t0)/1000.0); inrun=False
print("步数", len(durs))
print("单步运动时长 均值%.2fs 中位%.2fs 最小%.2f 最大%.2f"%(sum(durs)/len(durs), sorted(durs)[len(durs)//2], min(durs), max(durs)))

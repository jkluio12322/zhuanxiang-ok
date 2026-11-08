
import csv
r=list(csv.DictReader(open('turntest_45x2.csv')))
rows=[(float(x['t_ms']),float(x['actL']),float(x['actR']),float(x['tgtL'])) for x in r if x['phase']=='2']
starts=[];last=None
for i,(t,a,b,g) in enumerate(rows):
    if last is None or t-last>200: starts.append(i)
    last=t
starts.append(len(rows))
for k in range(len(starts)-1):
    seg=rows[starts[k]+15:starts[k+1]]
    hits=[(t,a,b) for t,a,b,g in seg if abs(a)>0.5]
    if hits:
        print("窗口",k,"异常点",hits[:6])

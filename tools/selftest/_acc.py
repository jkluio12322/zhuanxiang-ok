
import csv
r=list(csv.DictReader(open('turntest_45x2.csv')))
spin=[x for x in r if x['phase']=='1']
starts=[];last=None
for i,x in enumerate(spin):
    t=float(x['t_ms'])
    if last is None or abs(t-last)>200: starts.append(i)
    last=t
print("45° 每步起转时 |spin_err|：")
for k,i in enumerate(starts):
    print(" 步%2d cum=%6s spin_err起=%6s"%(k, spin[i]['cum'], spin[i]['spin_err']))


import csv, collections
rows=list(csv.DictReader(open('turntest_45x2.csv',encoding='utf-8')))
steps=collections.defaultdict(list)
for r in rows:
    steps[int(r['idx'])].append(r)
def f(r,k): return float(r[k])
tot=[]
for k in sorted(steps):
    g=steps[k]
    mv=[r for r in g if abs(f(r,'actL'))>1.0]
    dur=(f(mv[-1],'t_ms')-f(mv[0],'t_ms'))/1000.0 if len(mv)>1 else -1
    peak=max(abs(f(r,'actL')) for r in g)
    tail=[r for r in g if f(r,'t_ms')>f(g[-1],'t_ms')-300]
    sgn=[1 if f(r,'actL')>0.8 else (-1 if f(r,'actL')<-0.8 else 0) for r in tail]
    flips=sum(1 for a,b in zip(sgn,sgn[1:]) if a and b and a!=b)
    tot.append(dur)
    print("步%2d 运动%5.2fs 峰值%3.0frpm 末300ms反转%d 残差%s" % (k,dur,peak,flips,g[-1]['resid']))
print("运动时长均值 %.2f s" % (sum(tot)/len(tot)))


import csv, collections
rows=list(csv.DictReader(open('turntest_45x2.csv',encoding='utf-8')))
g=[r for r in rows if int(r['idx']) in (0,1)]
print("idx0/1 帧数", len(g))
for r in g[::4]:
    print("t=%7.0f yaw=%8.2f cum=%8.2f err=%7.2f actL=%6.1f actR=%6.1f ph=%s" % (
        float(r['t_ms']), float(r['yaw']), float(r['cum']), float(r['spin_err']), float(r['actL']), float(r['actR']), r['phase']))


import csv
r=list(csv.DictReader(open('turntest_45x3.csv')))
rows=[x for x in r if x['phase']=='2']
print("停车帧",len(rows))
for i,x in enumerate(rows):
    if i%6==0:
        print("t=%6.0f actL=%5.1f actR=%5.1f tgtL=%5.1f resid=%5.2f"%(float(x['t_ms']),float(x['actL']),float(x['actR']),float(x['tgtL']),float(x['resid'])))

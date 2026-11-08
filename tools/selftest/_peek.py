
import csv
rows = list(csv.DictReader(open('turntest_45x2.csv', encoding='utf-8')))
print("cols", rows[0].keys())
print("n", len(rows))
for r in rows[:6] + rows[95:110] + rows[200:215]:
    print(r['t_ms'], r['cum'], r['spin_err'], r['actL'], r['actR'], r['tgtL'], r['tgtR'], r['mask'], r['phase'], r['idx'])

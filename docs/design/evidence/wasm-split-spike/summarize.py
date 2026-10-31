"""Per case: pooled median per arm; noise = |median(base-a) - median(base-b)|, plus
per-process-median spread (IQR) of the base arms. Delta vs pooled base."""
import json, statistics, sys, collections
rows = [json.loads(l) for l in open(sys.argv[1])]
d = collections.defaultdict(lambda: collections.defaultdict(list))
pp = collections.defaultdict(lambda: collections.defaultdict(lambda: collections.defaultdict(list)))
for r in rows:
    d[r['case']][r['arm']].append(r['ns']); pp[r['case']][r['arm']][r['process']].append(r['ns'])
arms = ['split-all', 'split-smoke', 'split-perf']
print('| Case | base median ms | base-a vs base-b | ' + ' | '.join(arms) + ' |')
print('| --- | ---: | ---: | ' + ' | '.join(['---:'] * len(arms)) + ' |')
flags = []
for case, a in d.items():
    base = statistics.median(a['base-a'] + a['base-b'])
    noise = abs(statistics.median(a['base-a']) - statistics.median(a['base-b'])) / base * 100
    # process-level spread: range of per-process medians across both base arms, relative
    pm = [statistics.median(v) for arm in ('base-a', 'base-b') for v in pp[case][arm].values()]
    q = statistics.quantiles(pm, n=4); iqr = (q[2] - q[0]) / base * 100
    cells = []
    for arm in arms:
        dl = (statistics.median(a[arm]) - base) / base * 100
        mark = ' **' if abs(dl) > max(noise, iqr / 2) and dl > 0 else ''
        cells.append(f'{dl:+.1f}%{mark}')
        if mark: flags.append((case, arm, round(dl, 1), round(noise, 1), round(iqr, 1)))
    print(f'| {case} | {base/1e6:.3f} | {noise:.1f}% (IQR {iqr:.1f}%) | ' + ' | '.join(cells) + ' |')
print('\nflagged (slower beyond noise):'); [print(f) for f in flags]

"""Interleaved fresh-process timing of the scripts/performance WASM workers
across package variants. Usage: bench.py OUT processes samples warmup arm=dir[:mode] ..."""
import json, os, pathlib, statistics, subprocess, sys
sys.path.insert(0, os.path.dirname(__file__)); from perf_cases import cases
out = pathlib.Path(sys.argv[1]); procs, samples, warmup = map(int, sys.argv[2:5])
arms = []
for a in sys.argv[5:]:
    name, rest = a.split('=', 1); d, _, mode = rest.partition(':')
    arms.append((name, pathlib.Path(d), mode or 'eager'))
only = os.environ.get('CASES')
out.mkdir(parents=True, exist_ok=False)
with (out / 'samples.jsonl').open('w') as f:
    case_names = [n for n, _ in cases(arms[0][1], samples, warmup)]
    for ci, name in enumerate(case_names):
        if only and not any(name.startswith(p) for p in only.split(',')):
            continue
        for p in range(procs):
            order = arms[p % len(arms):] + arms[:p % len(arms)]
            for arm, d, mode in order:
                cmd = dict(cases(d, samples, warmup))[name]
                if os.environ.get('NODE_FLAGS'):
                    cmd = cmd[:1] + os.environ['NODE_FLAGS'].split() + cmd[1:]
                env = dict(os.environ, REMUS_SPLIT_MODE=mode)
                r = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=900)
                if r.returncode != 0:
                    raise SystemExit(f'{name} {arm} failed: {r.stderr[-500:]}')
                for line in r.stdout.splitlines():
                    row = json.loads(line)
                    if row.get('validation', 'passed') != 'passed':
                        raise SystemExit(f'{name} {arm}: validation {row}')
                    if row.get('warmup'):
                        continue
                    ns = row['operation_ns'] if 'operation_ns' in row else row['ms'] * 1e6
                    f.write(json.dumps({'case': name, 'arm': arm, 'process': p, 'ns': ns}) + '\n')
            f.flush()
        print(name, flush=True)

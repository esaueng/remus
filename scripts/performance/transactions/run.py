#!/usr/bin/env python3
"""Paired single-process samples from already-built diagnostic checkouts."""
import argparse
import json
import pathlib
import subprocess

p = argparse.ArgumentParser()
p.add_argument('baseline', type=pathlib.Path)
p.add_argument('candidate', type=pathlib.Path)
p.add_argument('output', type=pathlib.Path)
p.add_argument('--samples', type=int, default=7)
a = p.parse_args()
here = pathlib.Path(__file__).resolve().parent
roots = [('baseline', a.baseline.resolve()), ('candidate', a.candidate.resolve())]
a.output.mkdir(parents=True, exist_ok=True)
with (a.output / 'samples.jsonl').open('w') as out:
    for runtime in ['native', 'wasm']:
        for gfa in [False, True]:
            for size in [0, 100, 1000]:
                variants = [(d, False) for d in [0, 1, 4, 8]] if runtime == 'native' else [(m, cp) for m in ['direct', 'batch'] for cp in [False, True]]
                for variant, cp in variants:
                    for sample in range(a.samples):
                        for label, root in roots if sample % 2 == 0 else reversed(roots):
                            if runtime == 'native':
                                cmd = [str(root / 'target/release/examples/perf-t01'), str(size), str(variant)] + (['gfa'] if gfa else [])
                            else:
                                mode = ('gfa-' if gfa else '') + variant
                                cmd = ['node', str(here / 'wasm.cjs'), str(root / 'pkg-probe/remus_wasm.js'), str(size), mode, str(cp).lower()]
                            proc = subprocess.run(cmd, check=True, capture_output=True, text=True)
                            row = json.loads(proc.stdout)
                            row.update(revision=label, runtime=runtime, sample=sample, gfa=gfa)
                            out.write(json.dumps(row) + '\n')
                            out.flush()
    for label, root in roots:
        for runtime, cmd in [('native', [str(root / 'target/release/examples/perf-t01-fault')]), ('wasm', ['node', str(here / 'faults.cjs'), str(root / 'pkg-probe/remus_wasm.js')])]:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            (a.output / f'{label}-{runtime}-faults.json').write_text(result.stdout)
print(a.output)

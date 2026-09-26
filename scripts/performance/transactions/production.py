#!/usr/bin/env python3
"""Paired timings against uninstrumented native and actual production packages."""
import argparse
import json
import pathlib
import subprocess

p = argparse.ArgumentParser()
p.add_argument('baseline_native')
p.add_argument('candidate_native')
p.add_argument('baseline_package')
p.add_argument('candidate_package')
p.add_argument('output', type=pathlib.Path)
p.add_argument('--samples', type=int, default=7)
a = p.parse_args()
here = pathlib.Path(__file__).resolve().parent
with a.output.open('w') as out:
    for runtime in ['native', 'wasm']:
        for gfa in [False, True]:
            for size in [0, 100, 1000]:
                variants = [(d, False) for d in [0, 8]] if runtime == 'native' else [(m, cp) for m in ['direct', 'batch'] for cp in [False, True]]
                for variant, cp in variants:
                    for sample in range(a.samples):
                        labels = ['baseline', 'candidate'] if sample % 2 == 0 else ['candidate', 'baseline']
                        for label in labels:
                            if runtime == 'native':
                                cmd = [getattr(a, label + '_native'), str(size), str(variant)] + (['gfa'] if gfa else [])
                            else:
                                cmd = ['node', str(here / 'wasm.cjs'), getattr(a, label + '_package'), str(size), ('gfa-' if gfa else '') + variant, str(cp).lower()]
                            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
                            row = json.loads(result.stdout)
                            row.update(revision=label, runtime=runtime, sample=sample, gfa=gfa, metrics=None)
                            out.write(json.dumps(row) + '\n')
                            out.flush()
print(a.output)

"""WASM worker commands of the scripts/performance suites, for a given package dir."""
import json, pathlib
ROOT = pathlib.Path('/home/user/remus'); P = ROOT / 'scripts/performance'

def cases(pkg, samples, warmup, sketch_samples=None, sketch_warmup=None):
    out = []
    for c in json.loads((P / 'workloads.json').read_text())['cases']:
        if c['scenario'].startswith('wasm_'):
            out.append(('main/' + c['id'], ['node', str(P / 'wasm.cjs'), str(pkg), c['scenario'], str(c['size']), str(samples), str(warmup)]))
    for c in json.loads((P / 'sketch/workloads.json').read_text())['cases']:
        if c['workload'] not in ('independent_solved', 'coupled_chain', 'drag') or c['size'] not in (10, 100, 1000):
            continue
        for mode in ['solve', 'detailed']:
            out.append((f"sketch/{c['id']}/{mode}", ['node', str(P / 'sketch/wasm.cjs'), str(pkg), c['workload'], str(c['size']), mode,
                                                    str(sketch_samples or samples), str(sketch_warmup or warmup)]))
    for gfa in [False, True]:
        for size in [0, 100, 1000]:
            for m in ['direct', 'batch']:
                for cp in [False, True]:
                    mode = ('gfa-' if gfa else '') + m
                    out.append((f'tx/{mode}/{size}/cp={str(cp).lower()}', ['node', str(P / 'transactions/wasm.cjs'), str(pkg / 'remus_wasm_node.cjs'), str(size), mode, str(cp).lower()]))
    return out

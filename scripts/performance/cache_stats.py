#!/usr/bin/env python3
"""Compare the cache counters of two `run.py` results (RFC 0009 stage 0).

A change that only moves where caches live must leave every warm-cache
workload doing the same work: the same hits, misses and evictions per cache
and the same entries left held. Timings cannot show that; this check can.

    python3 scripts/performance/cache_stats.py BASELINE_DIR CANDIDATE_DIR

Exits 1 and lists every differing counter, or a case only one run measured.
"""
import argparse
import json
import sys
from pathlib import Path


def load(directory):
    path = Path(directory) / "cache_stats.json"
    data = json.loads(path.read_text())
    if data.get("schema") != "remus-performance-cache-stats-v1":
        raise ValueError(f"{path}: unknown schema")
    return data["cases"]


def flatten(value, prefix=""):
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            out.update(flatten(item, f"{prefix}.{key}" if prefix else key))
        return out
    return {prefix: value}


def differences(baseline, candidate):
    out = []
    for case in sorted(set(baseline) | set(candidate)):
        if case not in candidate or case not in baseline:
            out.append(f"{case}: only in {'baseline' if case in baseline else 'candidate'}")
            continue
        a, b = flatten(baseline[case]), flatten(candidate[case])
        for key in sorted(set(a) | set(b)):
            if a.get(key) != b.get(key):
                out.append(f"{case}: {key}: {a.get(key)} -> {b.get(key)}")
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    args = parser.parse_args()
    try:
        found = differences(load(args.baseline), load(args.candidate))
    except (OSError, ValueError, KeyError) as error:
        print(f"FAILED: {error}", file=sys.stderr)
        return 1
    for line in found:
        print(line)
    if found:
        print(f"{len(found)} cache counter difference(s)", file=sys.stderr)
        return 1
    print("Cache counters identical.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

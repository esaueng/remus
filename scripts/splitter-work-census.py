#!/usr/bin/env python3
"""Aggregate the face-splitter work probe (Open Kernel O2.3d budget policy).

The probe lives in `crates/algo/src/builder/face_splitter/shadow/work.rs`.
It exists only in `perf-counters` builds and appends one line per probed
face to `REMUS_SPLITTER_WORK_LOG`: wherever a legacy special case fires (the
shadow differential's sites) and at the two production adapter call sites
(early cylinder arrangement, last-resort planar arrangement). This script
reads one or more such logs and prints, per site and adapter. New logs
append an input content fingerprint, which folds exact repeated inputs;
old logs are retained without deduplication because local IDs are unsafe.
The report includes:

- how often the site was reached, and the adapter's outcome under raised
  budgets and under the operation's own budgets;
- which budget field the operation's budgets trip;
- the distribution (max, p99, p90, median) of the least march steps, queue
  size and segments that reach the raised-budget answer;
- how well the least march steps are predicted by cheap input sizes;
- adapter versus legacy wall time on the same faces.

    python scripts/splitter-work-census.py LOG [LOG ...] [--faces]

Collecting the logs (slow: every probed face is re-run about seventy times):

    REMUS_SPLITTER_SHADOW=raised REMUS_SPLITTER_WORK_LOG=work.tsv \
      cargo test --profile ci-test -p remus-operations --features perf-counters --tests
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import math
import sys
from pathlib import Path

COLUMNS = (
    "site", "test", "face", "surface", "sections", "loop_edges", "uses",
    "boundary_uses", "circle_uses", "raised", "default", "trips", "march",
    "queue", "segments", "vertices", "edges", "regions", "max_degree",
    "sum_degree_sq", "adapter_ns", "legacy_ns", "legacy_regions", "adopt",
    "context_default", "phases", "input",
)
INTS = (
    "face", "sections", "loop_edges", "uses", "boundary_uses", "circle_uses",
    "march", "queue", "segments", "vertices", "edges", "regions", "max_degree",
    "sum_degree_sq", "adapter_ns", "legacy_ns", "legacy_regions",
)


def parse(paths: list[Path]) -> list[dict]:
    rows = []
    for path in paths:
        for line in path.read_text(encoding="utf-8").splitlines():
            fields = line.split("\t")
            if len(fields) == len(COLUMNS) - 1:
                fields.append(None)  # Old logs cannot safely deduplicate inputs.
            if len(fields) != len(COLUMNS):
                raise ValueError(f"{path}: malformed work line: {line!r}")
            row = dict(zip(COLUMNS, fields))
            for key in INTS:
                row[key] = None if row[key] == "-" else int(row[key])
            row["phases"] = [
                (name, int(value))
                for name, value in (p.split("=") for p in row["phases"].split(";") if p != "-")
            ]
            row["log"] = path.name
            rows.append(row)
    return rows


def quantile(values: list[int], q: float) -> int:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, math.ceil(q * len(ordered)) - 1))]


def dist(values: list[int]) -> str:
    if not values:
        return "-"
    return (
        f"max {max(values):,} / p99 {quantile(values, 0.99):,} / p90 {quantile(values, 0.90):,}"
        f" / median {quantile(values, 0.5):,} / min {min(values):,} (n={len(values)})"
    )


def unique_faces(rows: list[dict]) -> list[dict]:
    """Fold repeated content identities, never arena-local face IDs alone.

    Old logs lack content identity: retain every row instead of silently
    discarding distinct operations with equal local IDs and input sizes.
    """
    seen = {}
    unique = []
    for row in rows:
        if row.get("input") is None:
            unique.append(row)
            continue
        key = (row["site"], row["test"], row["input"])
        if key not in seen:
            seen[key] = row
            unique.append(row)
    return unique


def fit(rows: list[dict]) -> str:
    """Least-squares ratio of the least march steps to candidate sizes."""
    out = []
    pts = [r for r in rows if r["march"] is not None and r["uses"]]
    if not pts:
        return "no data"
    for label, size in (
        ("uses^2", lambda r: r["uses"] ** 2),
        ("uses^2 + sum deg^2", lambda r: r["uses"] ** 2 + r["sum_degree_sq"]),
        ("(loop_edges + sections)^2", lambda r: (r["loop_edges"] + r["sections"]) ** 2),
    ):
        ratios = [r["march"] / size(r) for r in pts if size(r)]
        out.append(
            f"- march / {label}: max {max(ratios):.2f}, median {sorted(ratios)[len(ratios) // 2]:.2f},"
            f" min {min(ratios):.2f}"
        )
    return "\n".join(out)


def phase_shares(rows: list[dict]) -> str:
    """Median share of the least march steps per phase, over split faces."""
    order = []
    shares = defaultdict(list)
    for row in rows:
        if not row["raised"].startswith("split") or not row["phases"]:
            continue
        previous = 0
        total = None
        steps = [(n, v) for n, v in row["phases"] if not n.startswith("#")]
        if not steps:
            continue
        total = steps[-1][1] or 1
        for name, value in steps:
            if name not in order:
                order.append(name)
            shares[name].append((value - previous) / total)
            previous = value
    return " | ".join(
        f"{name} {100 * sorted(shares[name])[len(shares[name]) // 2]:.0f}%" for name in order
    )


def report(rows: list[dict], faces: bool) -> None:
    unidentified = sum(r.get("input") is None for r in rows)
    if unidentified:
        print(f"Warning: {unidentified} legacy rows lack input identity; retained without deduplication.",
              file=sys.stderr)
    rows = unique_faces(rows)
    by_site = defaultdict(list)
    for row in rows:
        by_site[(row["site"], row["surface"])].append(row)
    for (site, surface), group in sorted(by_site.items()):
        print(f"\n## {site} ({surface} adapter): {len(group)} probed faces in "
              f"{len({r['test'] for r in group})} tests\n")
        raised = Counter(r["raised"].split(":")[0] for r in group)
        default = Counter(r["default"].split(":")[0] for r in group)
        print(f"- raised-budget outcome: {dict(raised)}")
        print(f"- operation-budget outcome: {dict(default)}")
        print(f"- operation budgets trip: {dict(Counter(r['trips'] for r in group))}")
        if any(r["adopt"] != "-" for r in group):
            print(f"- adopted under raised budgets: {dict(Counter(r['adopt'] for r in group))}")
        split = [r for r in group if r["raised"].startswith("split")]
        for key in ("march", "queue", "segments", "uses", "loop_edges", "sections", "vertices", "edges", "max_degree"):
            values = [r[key] for r in split if r[key] is not None]
            print(f"- {key} (split faces): {dist(values)}")
        print(f"- phase shares of march (median): {phase_shares(split)}")
        print(fit(split))
        times = [(r["adapter_ns"], r["legacy_ns"]) for r in split if r["adapter_ns"] and r["legacy_ns"]]
        if times:
            ratio = sorted(a / l for a, l in times)
            print(f"- adapter ns: {dist([a for a, _ in times])}")
            print(f"- legacy ns: {dist([l for _, l in times])}")
            print(f"- adapter/legacy: median {ratio[len(ratio) // 2]:.2f}, max {ratio[-1]:.2f}, min {ratio[0]:.2f}")
        elif split:
            print(f"- adapter ns: {dist([r['adapter_ns'] for r in split if r['adapter_ns']])}")
        refused = [r for r in group if r["raised"].startswith(("refused", "panic"))]
        for r in refused:
            print(f"  - refused: {r['test']} face {r['face']}: {r['raised']}")
        if faces:
            for r in sorted(group, key=lambda r: -(r["march"] or 0)):
                print(
                    f"  - {r['test']} face {r['face']} input {r.get('input') or 'unavailable'}: sections {r['sections']}, loop edges {r['loop_edges']},"
                    f" uses {r['uses']}, raised {r['raised']}, default {r['default']}, trips {r['trips']},"
                    f" march {r['march']}, queue {r['queue']}, segments {r['segments']}, V {r['vertices']},"
                    f" E {r['edges']}, adopt {r['adopt']}, adapter {r['adapter_ns']} ns, legacy {r['legacy_ns']} ns"
                )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("logs", type=Path, nargs="+")
    parser.add_argument("--faces", action="store_true", help="List every probed face")
    args = parser.parse_args()
    report(parse(args.logs), args.faces)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

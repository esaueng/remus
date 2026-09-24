#!/usr/bin/env python3
"""Size the weekly mutation matrix so every listed mutant is examined (B19).

Input: the JSON array printed by `cargo mutants --list --json --in-diff ...`
(an empty file when the diff selects nothing). Output: GitHub step outputs
`mutants`, `shards` and `matrix`, plus a Markdown estimate for the run summary.

The cost model is per-package wall seconds per mutant on the hosted 4-vCPU
runner with `--jobs 2`, measured from the weekly job's own outcomes. Each
shard's verdict (`scripts/mutants-verdict.py`) prints the medians it observed,
so these constants can be re-tuned from any week's summary. They live here,
read from the checkout at run time, so tuning them needs no workflow pin bump.
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys

# Mean wall seconds each mutant occupies one of the two `--jobs 2` slots,
# build plus test, under the `.cargo/mutants.toml` settings (ci-test profile,
# first-failure stop, long-tail pins excluded), unviable mutants included.
# Measured on the hosted runner, 2026-09-24:
# - remus-operations: 305 s mean over 30 mutants (dispatch run 36055955228):
#   caught 211 s build + 74 s test (medians), missed 215 s + 272 s, unviable
#   12 s. That run still built the 17 examples `--tests` now skips.
# - remus-algo: 57 s over 3 (same run; builds contend with the other slot's
#   operations build).
# - remus-math / remus-blend: 24 s / 31 s over 113 / 7 in math-and-blend-only
#   shards (probe run 36059529895); 84 s for the one math mutant next to
#   operations builds, so mixed shards are costed higher.
# - remus-offset: 80 s over 1.
SECONDS_PER_MUTANT = {
    "remus-operations": 300.0,
    "remus-algo": 60.0,
    "remus-math": 50.0,
    "remus-blend": 40.0,
    "remus-offset": 80.0,
}
# Per shard: toolchain and cache restore (~1 min), the cold ci-test baseline
# (229 s build + 133 s test on the runner) and the second scratch directory's
# cold build.
SHARD_OVERHEAD_SECONDS = 12 * 60
JOBS_PER_SHARD = 2
# Plan each shard to 85% of its slot time: round-robin evens the package mix,
# but not which mutants survive and run their package's whole suite.
TARGET_UTILIZATION = 0.85
# 24 shards at 300 minutes cover every 2026-09 week except the one that added
# the operations/blend/offset scope; with `max-parallel: 8` that is at most
# three waves. A larger week fails its shards' verdicts as incomplete.
MAX_SHARDS = 24


def plan(mutants, budget_minutes, max_shards=MAX_SHARDS):
    counts = {}
    for mutant in mutants:
        counts[mutant["package"]] = counts.get(mutant["package"], 0) + 1
    fallback = max(SECONDS_PER_MUTANT.values())
    estimate = sum(n * SECONDS_PER_MUTANT.get(p, fallback) for p, n in counts.items())
    capacity = (budget_minutes * 60 - SHARD_OVERHEAD_SECONDS) * JOBS_PER_SHARD * TARGET_UTILIZATION
    if capacity <= 0:
        raise ValueError("the budget does not cover the per-shard overhead")
    if max_shards < 1:
        raise ValueError("the shard cap must be at least one")
    needed = math.ceil(estimate / capacity) if mutants else 0
    shards = min(max(needed, 1), max_shards) if mutants else 0
    return counts, estimate, capacity, needed, shards


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("listing", type=Path, help="output of cargo mutants --list --json")
    parser.add_argument("--budget-minutes", type=int, required=True)
    parser.add_argument("--max-shards", type=int, default=MAX_SHARDS,
                        help="cap on the matrix size (a smaller cap only for bounded probes)")
    parser.add_argument("--github-output", type=Path, default=os.environ.get("GITHUB_OUTPUT"))
    parser.add_argument("--summary", type=Path, default=os.environ.get("GITHUB_STEP_SUMMARY"))
    args = parser.parse_args(argv)

    text = args.listing.read_text().strip()
    mutants = json.loads(text) if text else []
    if not isinstance(mutants, list):
        raise ValueError("the listing is not a JSON array of mutants")
    counts, estimate, capacity, needed, shards = plan(mutants, args.budget_minutes, args.max_shards)

    lines = ["## Mutation shard plan", "", "| Package | Mutants | Est. slot-hours |", "| --- | ---: | ---: |"]
    fallback = max(SECONDS_PER_MUTANT.values())
    for package in sorted(counts):
        cost = counts[package] * SECONDS_PER_MUTANT.get(package, fallback) / 3600
        lines.append(f"| {package} | {counts[package]} | {cost:.1f} |")
    lines += [
        f"| **total** | **{len(mutants)}** | **{estimate / 3600:.1f}** |",
        "",
        f"Capacity per shard: {capacity / 3600:.1f} slot-hours "
        f"({args.budget_minutes} min budget, {JOBS_PER_SHARD} jobs, {SHARD_OVERHEAD_SECONDS // 60} min overhead, "
        f"{TARGET_UTILIZATION:.0%} planned utilization).",
        f"Shards: {shards} (needed {needed}, cap {args.max_shards}).",
        "",
    ]
    warning = None
    if needed > args.max_shards:
        warning = (f"the week's estimated {estimate / 3600:.0f} slot-hours need {needed} shards but the cap is "
                   f"{args.max_shards}: expect unexamined mutants and an incomplete verdict")
        lines.append(f"**Warning:** {warning}.")
    text = "\n".join(lines) + "\n"
    print(text)
    if warning:
        print(f"::warning::{warning}")
    if args.summary:
        with open(args.summary, "a") as handle:
            handle.write(text)
    if args.github_output:
        with open(args.github_output, "a") as handle:
            handle.write(f"mutants={len(mutants)}\n")
            handle.write(f"shards={shards}\n")
            handle.write(f"matrix={json.dumps(list(range(shards)))}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Size the weekly mutation matrix within its runner budget (B19).

Input: the JSON array printed by `cargo mutants --list --json --in-diff ...`
(an empty file when the diff selects nothing). Output: GitHub step outputs
`mutants`, `shards` and `matrix`, plus a Markdown estimate for the run summary.

The cost model is per-package occupied-slot seconds per mutant with `--jobs 2`,
measured from the weekly job's own outcomes. A week that needs more than
`MAX_SHARDS` shards is sampled, not completed: each shard runs a shuffled
order until its budget expires, and the verdict reports the rest as not
sampled. Each shard's verdict
(`scripts/mutants-verdict.py`) prints the means and medians it observed,
so these constants can be re-tuned from a run's summary. They live here,
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
# Operations/algo/math remeasured from scheduled run 38123404817 (2026-10-11,
# source 30206c06d03fd14bb0ee88c24c4fe8b30f509105), inspected shards 0 and 5.
# Round each package's larger observed shard mean upward: 637.59 / 84.62 /
# 102.81 s. Includes missed, timed-out and unviable outcomes, not only catches.
# These are planning estimates, not upper bounds; other shards can be slower.
# Blend/offset retain the 2026-09-24 estimates (no samples in these two shards).
# Source-pinned counts/timings: docs/kernel-maturity/mutation-plan-2026-10-13.md.
SECONDS_PER_MUTANT = {
    "remus-operations": 650.0,
    "remus-algo": 90.0,
    "remus-math": 110.0,
    "remus-blend": 40.0,
    "remus-offset": 80.0,
}
# Baseline alone reached 285 s build + 503 s test in shard 0. Reserve another
# ~6 min for setup and the second scratch directory's cold operations build.
SHARD_OVERHEAD_SECONDS = 20 * 60
JOBS_PER_SHARD = 2
# Plan each shard to 85% of its slot time. Test the actual round-robin package
# assignment: equal mutant counts do not imply equal estimated work.
TARGET_UTILIZATION = 0.85
# One wave at the callee's max-parallel 8: about 40 hosted runner-hours a week.
# The 24-shard cap (three waves, ~120 runner-hours) still covered only ~40% of
# a typical week, so the planner samples a larger week instead.
MAX_SHARDS = 8


def round_robin_costs(mutants, shards):
    """Match cargo-mutants 27: mutant i belongs to shard i % shards.

    `--shuffle` only changes execution order after this partition, so the
    listing's order is authoritative for the per-shard package mix.
    """
    if shards < 1:
        raise ValueError("the shard count must be at least one")
    costs = [0.0] * shards
    fallback = max(SECONDS_PER_MUTANT.values())
    for index, mutant in enumerate(mutants):
        costs[index % shards] += SECONDS_PER_MUTANT.get(mutant["package"], fallback)
    return costs


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
    shards = min(len(mutants), max_shards)
    # Total cost is only a lower bound. Search every eligible matrix size,
    # because round-robin imbalance is not monotone as the divisor changes.
    for candidate in range(max(needed, 1), shards + 1):
        if max(round_robin_costs(mutants, candidate)) <= capacity:
            shards = candidate
            break
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
        f"Shards: {shards} (total-cost lower bound {needed}, cap {args.max_shards}).",
        "",
    ]
    shard_costs = round_robin_costs(mutants, shards) if shards else []
    peak = max(shard_costs, default=0.0)
    if shard_costs:
        lines += [
            "| Shard | Est. slot-hours | Within capacity |",
            "| ---: | ---: | --- |",
            *[f"| {index} | {cost / 3600:.1f} | {'yes' if cost <= capacity else 'no'} |"
              for index, cost in enumerate(shard_costs)],
            "",
        ]
    notice = None
    if peak > capacity:
        sampled = sum(min(cost, capacity) for cost in shard_costs) / estimate
        notice = (f"the week needs {needed} shards at the estimated cost and the cap is "
                  f"{args.max_shards}: each shard examines a shuffled sample, about "
                  f"{sampled:.0%} of the listed mutants in all; the rest are reported as "
                  "not sampled in each shard's unexamined.txt")
        lines.append(f"**Sampled:** {notice}.")
    lines.append("Estimates are not guarantees. Missed mutants are reported; timed-out "
                 "mutants, a failed baseline or a shard that examines nothing fail the run.")
    text = "\n".join(lines) + "\n"
    print(text)
    if notice:
        print(f"::notice::{notice}")
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

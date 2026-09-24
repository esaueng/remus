#!/usr/bin/env python3
"""Turn one weekly cargo-mutants shard into an exact, fail-closed verdict (B19).

The weekly job used to count only the outcome files it found. A run that
`timeout` stopped during the unmutated baseline therefore printed zeros and
passed: the 2026-09-13 and 2026-09-16 runs were green after examining 0 of
4,620 and 0 of 3,375 listed mutants. This verdict compares what cargo-mutants
listed for the shard (`mutants.json`, written before the baseline) with what
it finished (`outcomes.json`, rewritten after every scenario), and fails when

- cargo-mutants exited with anything but 0 (clean), 2 (missed), 3 (timeout)
  or 124 (`timeout` reached the budget),
- the unmutated baseline did not finish green,
- any mutant was MISSED, TIMEOUT, or ended in an unattributed state,
- any listed mutant was never examined (the budget ran out), or
- the output is missing or inconsistent.

It writes a Markdown summary (to `$GITHUB_STEP_SUMMARY` when set) with the
per-package build and test times the shard planner's cost model is tuned from,
and `unexamined.txt` next to the outcomes when coverage is incomplete.
"""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import statistics
import sys

EXIT_MEANINGS = {
    0: "clean",
    1: "usage error",
    2: "missed mutants",
    3: "timed-out mutants",
    4: "baseline failed",
    5: "--in-diff does not match the tree",
    6: "--in-diff is invalid",
    70: "cargo-mutants internal error",
    124: "budget reached (timeout)",
    137: "killed after the budget's grace period",
}
# 2 and 3 are judged from the outcomes below (so the summary is always
# written), 124 leaves partial-but-valid outcomes whose coverage is judged
# below; anything else means the outcome files cannot be trusted.
READABLE_EXIT_CODES = {0, 2, 3, 124}
CLEAN = {"CaughtMutant", "Unviable"}


def mutant_name(outcome):
    scenario = outcome.get("scenario")
    if isinstance(scenario, dict) and "Mutant" in scenario:
        return scenario["Mutant"]["name"], scenario["Mutant"]["package"]
    return None, None


def phase_seconds(outcome, phase):
    for result in outcome.get("phase_results", []):
        if result.get("phase") == phase:
            return float(result.get("duration", 0.0))
    return None


def verdict(output_dir, exit_code, label):
    """Return (errors, summary_lines, unexamined_names)."""
    errors = []
    lines = [f"## Mutation testing: {label}", ""]
    meaning = EXIT_MEANINGS.get(exit_code, "unexpected")
    lines.append(f"cargo-mutants exit code: `{exit_code}` ({meaning})")
    lines.append("")
    if exit_code not in READABLE_EXIT_CODES:
        errors.append(f"cargo mutants exited {exit_code} ({meaning})")

    mutants_path = output_dir / "mutants.json"
    outcomes_path = output_dir / "outcomes.json"
    if not output_dir.is_dir():
        if exit_code == 0:
            # cargo-mutants returns 0 without creating an output directory
            # when the diff selects no mutants at all.
            lines.append("No mutants in scope for this shard.")
            return errors, lines, []
        errors.append(f"no cargo-mutants output at {output_dir}")
        return errors, lines, []
    try:
        listed = json.loads(mutants_path.read_text())
    except (OSError, ValueError) as err:
        errors.append(f"cannot read the listed mutants ({mutants_path}): {err}")
        return errors, lines, []
    listed_names = [m["name"] for m in listed]
    if not listed_names:
        lines.append("No mutants in scope for this shard.")
        return errors, lines, []
    try:
        outcomes = json.loads(outcomes_path.read_text())["outcomes"]
    except (OSError, ValueError, KeyError) as err:
        errors.append(f"{len(listed_names)} mutants listed but no readable outcomes ({outcomes_path}): {err}")
        return errors, lines, listed_names

    baselines = [o for o in outcomes if o.get("scenario") == "Baseline"]
    if not baselines:
        errors.append("the unmutated baseline did not finish within the budget; no mutant was examined")
    elif baselines[0].get("summary") != "Success":
        errors.append(f"the unmutated baseline ended {baselines[0].get('summary')}")
    else:
        build = phase_seconds(baselines[0], "Build") or 0.0
        test = phase_seconds(baselines[0], "Test") or 0.0
        lines.append(f"Baseline: {build:.0f} s build + {test:.0f} s test")
        lines.append("")

    # Multisets, not sets: names have been unique in every weekly listing so
    # far, but a repeated name must still be examined as often as it is listed.
    by_summary = {}
    per_package = {}
    examined = Counter()
    for outcome in outcomes:
        name, package = mutant_name(outcome)
        if name is None:
            continue
        examined[name] += 1
        summary = outcome.get("summary", "missing")
        by_summary.setdefault(summary, []).append(name)
        stats = per_package.setdefault(package, {"n": 0, "build": [], "test": []})
        stats["n"] += 1
        for phase, key in (("Build", "build"), ("Test", "test")):
            seconds = phase_seconds(outcome, phase)
            if seconds is not None:
                stats[key].append(seconds)

    listed_counts = Counter(listed_names)
    unknown = examined - listed_counts
    if unknown:
        errors.append(f"{sum(unknown.values())} outcomes do not match a listed mutant of this shard")
    remaining = listed_counts - examined
    unexamined = []
    for name in listed_names:
        if remaining[name]:
            remaining[name] -= 1
            unexamined.append(name)

    count = lambda key: len(by_summary.get(key, []))
    other = sum(len(v) for k, v in by_summary.items() if k not in CLEAN | {"MissedMutant", "Timeout"})
    lines += [
        "| Outcome | Count |",
        "| --- | ---: |",
        f"| listed | {len(listed_names)} |",
        f"| examined | {sum(examined.values())} |",
        f"| not examined | {len(unexamined)} |",
        f"| caught | {count('CaughtMutant')} |",
        f"| missed | {count('MissedMutant')} |",
        f"| timeout | {count('Timeout')} |",
        f"| unviable | {count('Unviable')} |",
        f"| other | {other} |",
        "",
    ]
    if per_package:
        lines += [
            "| Package | Examined | Median build (s) | Median test (s) |",
            "| --- | ---: | ---: | ---: |",
        ]
        for package in sorted(per_package):
            stats = per_package[package]
            median = lambda xs: f"{statistics.median(xs):.0f}" if xs else "-"
            lines.append(f"| {package} | {stats['n']} | {median(stats['build'])} | {median(stats['test'])} |")
        lines.append("")

    for key, label_text in (("MissedMutant", "missed"), ("Timeout", "timed out")):
        names = by_summary.get(key, [])
        if names:
            errors.append(f"{len(names)} mutants {label_text}")
            lines.append(f"**{label_text.capitalize()}:**")
            lines += [f"- `{n}`" for n in names]
            lines.append("")
    if other:
        errors.append(f"{other} mutants ended in an unattributed state")
    if unexamined:
        errors.append(
            f"{len(unexamined)} of {len(listed_names)} listed mutants were not examined "
            "(budget reached): this shard's verdict is incomplete"
        )
    elif exit_code == 124 and not errors:
        lines.append("The budget expired after the last mutant finished; coverage is complete.")
    return errors, lines, unexamined


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", required=True, type=Path, help="the mutants.out directory")
    parser.add_argument("--exit-code", required=True, type=int, help="exit status of the cargo mutants command")
    parser.add_argument("--label", default="weekly changes")
    parser.add_argument("--summary", type=Path, default=os.environ.get("GITHUB_STEP_SUMMARY"))
    args = parser.parse_args(argv)

    errors, lines, unexamined = verdict(args.output, args.exit_code, args.label)
    if unexamined and args.output.is_dir():
        (args.output / "unexamined.txt").write_text("".join(n + "\n" for n in unexamined))
    lines.append("Scope: mutants overlapping the last seven days of changes under `crates/`.")
    lines.append("Missed, timed-out and unexamined mutants fail this workflow; review the attached report.")
    text = "\n".join(lines) + "\n"
    if args.summary:
        with open(args.summary, "a") as handle:
            handle.write(text)
    print(text)
    for error in errors:
        print(f"::error::{args.label}: {error}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())

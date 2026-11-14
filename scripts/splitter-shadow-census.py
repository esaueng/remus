#!/usr/bin/env python3
"""Engagement census of the face-splitter shadow differential (Open Kernel O2.3d).

Runs the face-splitter foil corpus (the fixtures the roadmap skill requires
on any `face_splitter` change) with the shadow differential on, once under
the operation's own work budgets and once under the raised budgets, and
prints how often each legacy special-case emitter fired and what the
UV-arrangement adapter made of the same face.

The differential lives in `crates/algo/src/builder/face_splitter/shadow.rs`.
It exists only in `perf-counters` builds, is switched on by
`REMUS_SPLITTER_SHADOW`, and appends one line per shadowed face to
`REMUS_SPLITTER_SHADOW_LOG`. The legacy result is always the one the boolean
uses, so the foils pass or fail exactly as they do without it; a foil that
fails here is reported, never hidden.

    python3 scripts/splitter-shadow-census.py              # the whole corpus
    python3 scripts/splitter-shadow-census.py --foil d4    # one family
    python3 scripts/splitter-shadow-census.py --keep-logs DIR

The corpus is slow on a cold build (three crates rebuild with
`perf-counters`), so this is a manual measurement, not a CI check.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MODES = ("default", "raised")
ENTRIES = (
    "try_split_crossing_plane_face",
    "try_split_disk_by_chords",
    "split_periodic_face_into_sectors",
    "split_periodic_face_into_bands[cylinder]",
)
OUTCOMES = ("agreed", "disagreed", "declined", "declined-on-budget", "refused")
COLUMNS = (
    "budgets", "entry", "outcome", "reason", "test", "face", "surface",
    "sections", "legacy_regions", "shadow_regions", "locator",
)


@dataclass(frozen=True)
class Foil:
    """One foil family: the cargo test targets that exercise it."""

    family: str
    package: str
    feature: str
    target: tuple[str, ...]
    filters: tuple[str, ...] = ()

    @property
    def label(self) -> str:
        return self.target[-1] if self.target[0] == "--test" else self.filters[0]


def io_foil(family: str, test_file: str) -> Foil:
    return Foil(family, "remus-io", "remus-operations/perf-counters", ("--test", test_file))


FOILS = (
    Foil("d4", "remus-operations", "perf-counters", ("--lib",), ("d4_shelled_box_fuse_lip",)),
    Foil("d4", "remus-wasm", "remus-algo/perf-counters", ("--lib",), ("gridfinity_d4_full_1x1_bin",)),
    io_foil("honeycomb-pcut3", "gridfinity_honeycomb_cut_inmem"),
    io_foil("divider-lip", "gridfinity_lipfuse_dividers_inmem"),
    io_foil("divider-lip", "lipfuse_boundary_retrace_inmem"),
    io_foil("nub", "dovetail_a1corner_nubfuse_inmem"),
    io_foil("nub", "dovetail_dblcorner_nub_inmem"),
    io_foil("nub", "fitoffset_nub_chain_inmem"),
    io_foil("cylinder-slot", "slotted_nolip_fuse_inmem"),
    io_foil("cylinder-slot", "snapclip_slot_cut_inmem"),
    io_foil("groove-mouth", "fitoffset_groove_mouth_inmem"),
    io_foil("junction-disc", "socket_junction_disc_inmem"),
)


def parse_log(text: str, foil: str) -> list[dict[str, str]]:
    """Events of one shadow log, each tagged with the foil that produced it."""
    events = []
    for line in text.splitlines():
        fields = line.split("\t")
        if len(fields) != len(COLUMNS):
            raise ValueError(f"malformed shadow log line: {line!r}")
        events.append(dict(zip(COLUMNS, fields), foil=foil))
    return events


def tally(events: list[dict[str, str]], mode: str) -> dict[str, Counter]:
    """Outcome counts per entry point under one budget mode. A disagreement
    that only breaks a carrier elsewhere is also counted as `segmentation`."""
    table = {entry: Counter() for entry in ENTRIES}
    for event in events:
        if event["budgets"] != mode:
            continue
        counts = table.setdefault(event["entry"], Counter())
        counts["fired"] += 1
        counts[event["outcome"]] += 1
        if event["outcome"] == "disagreed" and event["reason"].startswith("segmentation-only"):
            counts["segmentation"] += 1
    return table


def render_table(events: list[dict[str, str]], mode: str) -> str:
    head = "| Legacy entry point | fired | agreed | disagreed (segmentation-only) | declined | declined on budget | refused |"
    rows = [head, "|---|---:|---:|---:|---:|---:|---:|"]
    for entry, counts in tally(events, mode).items():
        rows.append(
            f"| `{entry}` | {counts['fired']} | {counts['agreed']} | "
            f"{counts['disagreed']} ({counts['segmentation']}) | {counts['declined']} | "
            f"{counts['declined-on-budget']} | {counts['refused']} |"
        )
    return "\n".join(rows)


def render_findings(events: list[dict[str, str]], outcome: str) -> str:
    """Every event with `outcome`, one line per distinct foil, face and reason."""
    seen: Counter = Counter()
    for event in events:
        if event["outcome"] != outcome:
            continue
        seen[
            (
                event["budgets"], event["foil"], event["test"], event["entry"], event["reason"],
                event["surface"], event["face"], event["sections"], event["legacy_regions"],
                event["shadow_regions"], event["locator"],
            )
        ] += 1
    if not seen:
        return "none"
    lines = []
    for key in sorted(seen):
        budgets, foil, test, entry, reason, surface, face, sections, legacy, shadow, locator = key
        times = f" x{seen[key]}" if seen[key] > 1 else ""
        lines.append(
            f"- [{budgets}] {foil} :: {test} :: `{entry}` on {surface} face {face} "
            f"({sections} sections, first at {locator}): {reason}; "
            f"legacy {legacy} regions, arrangement {shadow}{times}"
        )
    return "\n".join(lines)


def render_foils(events: list[dict[str, str]], mode: str) -> str:
    """Shadowed faces per foil under one budget mode."""
    rows = ["| Foil | " + " | ".join(OUTCOMES) + " |", "|---|" + "---:|" * len(OUTCOMES)]
    per_foil: dict[str, Counter] = {}
    for event in events:
        if event["budgets"] == mode:
            per_foil.setdefault(event["foil"], Counter())[event["outcome"]] += 1
    for foil in sorted(per_foil):
        rows.append(f"| {foil} | " + " | ".join(str(per_foil[foil][o]) for o in OUTCOMES) + " |")
    return "\n".join(rows) if per_foil else "no shadowed face"


def run_foil(foil: Foil, mode: str, log: Path, args: argparse.Namespace) -> int:
    command = [
        "cargo", "test", "--profile", args.profile, "-j", str(args.jobs),
        "-p", foil.package, "--features", foil.feature, *foil.target, "--", *foil.filters,
    ]
    env = dict(os.environ, REMUS_SPLITTER_SHADOW=mode, REMUS_SPLITTER_SHADOW_LOG=str(log))
    print(f"> [{mode}] {' '.join(command)}", file=sys.stderr, flush=True)
    result = subprocess.run(command, cwd=ROOT, env=env, check=False, capture_output=not args.verbose, text=True)
    if result.returncode != 0 and not args.verbose:
        sys.stderr.write(result.stdout[-4000:] + result.stderr[-4000:])
    return result.returncode


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--foil", action="append", help="Limit the run to these foil families")
    parser.add_argument("--profile", default="ci-test", help="Cargo profile (default: ci-test)")
    parser.add_argument("--jobs", type=int, default=4, help="Cargo build jobs (default: 4)")
    parser.add_argument("--keep-logs", type=Path, help="Write the raw shadow logs to this directory")
    parser.add_argument("--verbose", action="store_true", help="Show cargo output")
    args = parser.parse_args(argv)

    foils = [foil for foil in FOILS if not args.foil or foil.family in args.foil]
    if not foils:
        parser.error(f"no foil family matches {args.foil}; known: {sorted({f.family for f in FOILS})}")

    events: list[dict[str, str]] = []
    failed: list[str] = []
    with tempfile.TemporaryDirectory(prefix="splitter-shadow-") as scratch:
        log_dir = args.keep_logs or Path(scratch)
        log_dir.mkdir(parents=True, exist_ok=True)
        for mode in MODES:
            for foil in foils:
                log = log_dir / f"{mode}--{foil.family}--{foil.label}.tsv"
                log.unlink(missing_ok=True)
                if run_foil(foil, mode, log, args) != 0:
                    failed.append(f"[{mode}] {foil.family} :: {foil.label}")
                if log.is_file():
                    events += parse_log(log.read_text(encoding="utf-8"), f"{foil.family}/{foil.label}")

    for mode in MODES:
        print(f"\n### Shadow engagement, {mode} budgets\n")
        print(render_table(events, mode))
        print(f"\nPer foil ({mode} budgets):\n")
        print(render_foils(events, mode))
    print("\n### Disagreements\n")
    print(render_findings(events, "disagreed"))
    print("\n### Refusals (typed errors an early dispatch would propagate)\n")
    print(render_findings(events, "refused"))
    print("\n### Foil test failures with the differential on\n")
    print("\n".join(f"- {name}" for name in failed) if failed else "none")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

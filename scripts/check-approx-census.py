#!/usr/bin/env python3
"""Ratchet the approximation-path census without pinning machine timing.

The census example is intentionally human-readable and includes wall-clock
measurements.  This check extracts only the reviewable contract: operation
identity, result face count, exact/fallback/error path, and the revolve
surface census.  Arena IDs are normalized because they are rebuild-local;
all other text remains authoritative.
"""

from __future__ import annotations

import argparse
import difflib
import os
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
BASELINE = ROOT / "crates/operations/examples/approx_census.snapshot"
COMMAND = (
    "cargo",
    "run",
    "--release",
    "--quiet",
    "-p",
    "remus-operations",
    "--example",
    "approx_census",
)

ROW = re.compile(
    r"^  (?P<family>\S+)\s+(?P<case>.+?)\s+"
    r"(?P<milliseconds>\d+(?:\.\d+)?)ms\s+"
    r"faces=(?P<faces>\d+)\s+(?P<path>.+)$"
)
SURFACES = re.compile(
    r"^\s+(?P<surface>plane=\d+ cyl=\d+ cone=\d+ sphere=\d+ "
    r"torus=\d+ NURBS=\d+)$"
)
ARENA_ID = re.compile(r"Id\(\d+\)")


def parse_output(output: str) -> list[str]:
    """Extract deterministic semantic rows from one census run."""
    rows: list[str] = []
    identities: set[tuple[str, str]] = set()

    for line in output.splitlines():
        match = ROW.match(line)
        if match:
            family = match.group("family")
            case = match.group("case").strip()
            identity = (family, case)
            if identity in identities:
                raise ValueError(f"duplicate census row: {family} / {case}")
            identities.add(identity)

            path = ARENA_ID.sub("Id(_)", match.group("path").strip())
            rows.append(
                f"{family}\t{case}\tfaces={match.group('faces')}\t{path}"
            )
            continue

        surface = SURFACES.match(line)
        if surface:
            if not rows or not rows[-1].startswith("revolve\t"):
                raise ValueError("surface census is not attached to a revolve row")
            rows[-1] += f"\tsurfaces={surface.group('surface')}"

    if not rows:
        raise ValueError("census output contained no parseable rows")
    return rows


def expected_rows() -> list[str]:
    """Read the committed snapshot, excluding comments and blank lines."""
    try:
        lines = BASELINE.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise ValueError(f"cannot read {BASELINE.relative_to(ROOT)}: {error}") from error
    return [line for line in lines if line and not line.startswith("#")]


def run_census() -> str:
    """Run the release-mode census and return stdout."""
    try:
        completed = subprocess.run(
            COMMAND,
            cwd=ROOT,
            env=os.environ.copy(),
            check=False,
            capture_output=True,
            text=True,
            timeout=15 * 60,
        )
    except subprocess.TimeoutExpired as error:
        raise ValueError("approx_census exceeded its 15-minute budget") from error

    if completed.returncode != 0:
        if completed.stdout:
            print(completed.stdout, file=sys.stderr, end="")
        if completed.stderr:
            print(completed.stderr, file=sys.stderr, end="")
        raise ValueError(f"approx_census exited with status {completed.returncode}")
    return completed.stdout


def snapshot(rows: list[str]) -> str:
    """Render rows in the committed line-oriented format."""
    return "\n".join(rows) + "\n"


def self_test() -> None:
    """Prove normalization ignores noise and rejects semantic drift."""
    reference = """\
  boolean   box / sphere ∪                          12.34ms  faces=1192  FALLBACK x1: boolean Fuse: mesh Id(4)
  revolve   frustum                                      0.01ms  faces=3     exact analytic
            plane=2 cyl=0 cone=1 sphere=0 torus=0 NURBS=0
"""
    timing_and_id_noise = """\
  boolean   box / sphere ∪                         987.65ms  faces=1192  FALLBACK x1: boolean Fuse: mesh Id(99)
  revolve   frustum                                     42.00ms  faces=3     exact analytic
            plane=2 cyl=0 cone=1 sphere=0 torus=0 NURBS=0
"""
    if parse_output(reference) != parse_output(timing_and_id_noise):
        raise AssertionError("timing and arena IDs must not affect the snapshot")

    face_drift = reference.replace("faces=1192", "faces=1193")
    if parse_output(reference) == parse_output(face_drift):
        raise AssertionError("face-count drift must affect the snapshot")

    path_drift = reference.replace("FALLBACK x1", "exact analytic")
    if parse_output(reference) == parse_output(path_drift):
        raise AssertionError("approximation-path drift must affect the snapshot")

    surface_drift = reference.replace("cone=1", "cone=0").replace(
        "NURBS=0", "NURBS=1"
    )
    if parse_output(reference) == parse_output(surface_drift):
        raise AssertionError("surface-type drift must affect the snapshot")

    duplicate = reference + reference.splitlines(keepends=True)[0]
    try:
        parse_output(duplicate)
    except ValueError as error:
        if "duplicate census row" not in str(error):
            raise
    else:
        raise AssertionError("duplicate row identities must be rejected")


def check(actual: list[str], expected: list[str]) -> int:
    """Compare one run to the committed baseline and print a review diff."""
    if actual == expected:
        print(f"Approximation census matches {len(expected)} committed rows.")
        return 0

    print(
        "Approximation census changed. Review every row; if the movement is "
        "intentional, update the snapshot in the same PR.",
        file=sys.stderr,
    )
    diff = difflib.unified_diff(
        expected,
        actual,
        fromfile=str(BASELINE.relative_to(ROOT)),
        tofile="current approx_census",
        lineterm="",
    )
    for line in diff:
        print(line, file=sys.stderr)
    return 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--emit",
        action="store_true",
        help="print the normalized current census for intentional snapshot updates",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="test the normalizer and drift detector without compiling the kernel",
    )
    args = parser.parse_args()

    try:
        if args.self_test:
            self_test()
            print("Approximation census contract self-test passed.")
            return 0

        actual = parse_output(run_census())
        if args.emit:
            print(snapshot(actual), end="")
            return 0
        return check(actual, expected_rows())
    except (AssertionError, ValueError) as error:
        print(f"VIOLATION: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

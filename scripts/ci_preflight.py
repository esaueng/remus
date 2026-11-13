#!/usr/bin/env python3
"""Run the repository policy locally and in CI, retaining every failed check."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Check:
    """A named policy command, executed without a shell in the repository root."""

    name: str
    command: tuple[str, ...]


# The Repository Policy job in ci-checks.yml runs exactly this catalog.
CHECKS = (
    Check("Apache lineage", ("./scripts/check-apache-lineage.sh",)),
    Check("Rust format", ("cargo", "fmt", "--all", "--", "--check")),
    Check("Layer boundaries", ("./scripts/check-boundaries.sh",)),
    Check("Deterministic hashing ratchet", ("./scripts/check-det-hash.sh",)),
    Check("Wildcard-arm ratchet", ("./scripts/check-wildcard-arms.sh",)),
    Check("Wildcard ratchet tests", ("python3", "scripts/test-wildcard-arms.py")),
    Check("Splitter special-case ratchet", ("python3", "scripts/check-splitter-ratchet.py")),
    Check("Splitter ratchet tests", ("python3", "scripts/test-splitter-ratchet.py")),
    Check("O4.7 twin coverage", ("python3", "scripts/check-wasm-o47-coverage.py")),
    Check("O4.7 coverage fixtures", ("python3", "scripts/test-wasm-o47-coverage.py")),
    Check("Edge-domain authority", ("./scripts/check-edge-domain-authority.py",)),
    Check("Documentation paths", ("./scripts/check-doc-paths.sh",)),
    Check("Documentation module map", ("./scripts/check-doc-module-map.py",)),
    Check("Product naming", ("./scripts/check-remus-rename.sh",)),
    Check("Publish workflow contract", ("./scripts/test-publish-credentials.sh",)),
    Check("Gauntlet workflow contract", ("python3", "scripts/test-gauntlet-workflow.py")),
    Check("CI classifier tests", ("python3", "scripts/test-classify-ci-changes.py")),
    Check("CI gate and native runner tests", ("python3", "scripts/test_ci_native.py")),
    Check("Package suite split tests", ("node", "--test", "scripts/test-packaged-suite-parts.mjs")),
    Check("Privacy regressions", ("python3", "-m", "unittest", "discover", "-s", ".github/privacy", "-p", "test_*.py")),
    Check("Unused dependencies", ("cargo", "machete")),
    Check("TOML format", ("taplo", "fmt", "--check")),
)


def workflow_command(value: str) -> str:
    """Quote the data portion of a GitHub workflow command."""
    return value.replace("%", "%25").replace("\r", "%0D").replace("\n", "%0A")


def run_checks(checks=CHECKS, *, root=ROOT, runner=None, clock=None, actions=False):
    """Run all checks, failing on missing tools as well as nonzero exit codes."""
    runner = subprocess.run if runner is None else runner
    clock = time.monotonic if clock is None else clock
    results = []
    for check in checks:
        print(f"::group::{workflow_command(check.name)}" if actions else f"\n> {check.name}", flush=True)
        started = clock()
        try:
            exit_code = runner(list(check.command), cwd=root, check=False).returncode
        except OSError as error:
            # A missing policy tool must never become a skipped/passing check.
            exit_code = 127
            print(f"Unable to start {check.name}: {error.strerror}", flush=True)
        elapsed = round(max(0.0, clock() - started), 3)
        result = {"name": check.name, "exit_code": exit_code, "seconds": elapsed}
        results.append(result)
        print(f"{'PASS' if exit_code == 0 else 'FAIL'}: {check.name} (exit {exit_code}, {elapsed:.3f}s)", flush=True)
        if actions:
            print("::endgroup::", flush=True)
            if exit_code != 0:
                message = workflow_command(f"{check.name} failed with exit {exit_code}. See its policy log group.")
                print(f"::error title=Repository policy::{message}", flush=True)
    return {"schema": 1, "passed": all(row["exit_code"] == 0 for row in results), "checks": results}


def write_summary(report, destination):
    """Append a small result table; no command output or environment is stored."""
    with destination.open("a") as stream:
        stream.write("\n### Repository policy\n\n| Check | Result | Seconds |\n| --- | --- | ---: |\n")
        for row in report["checks"]:
            label = row["name"].replace("|", "\\|").replace("\r", " ").replace("\n", " ")
            result = "pass" if row["exit_code"] == 0 else f"FAIL (exit {row['exit_code']})"
            stream.write(f"| {label} | {result} | {row['seconds']:.3f} |\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=Path, help="Write compact check outcomes and durations")
    args = parser.parse_args(argv)
    report = run_checks(actions=os.environ.get("GITHUB_ACTIONS") == "true")
    if args.json:
        args.json.write_text(json.dumps(report, indent=2) + "\n")
    if summary := os.environ.get("GITHUB_STEP_SUMMARY"):
        write_summary(report, Path(summary))
    failed = [row["name"] for row in report["checks"] if row["exit_code"] != 0]
    print("\nRepository policy passed." if not failed else "\nRepository policy failed: " + ", ".join(failed), flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

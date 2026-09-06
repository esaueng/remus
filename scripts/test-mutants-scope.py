#!/usr/bin/env python3
"""Exercise cargo-mutants' real default config discovery and CDT selection."""

import collections
import json
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / ".cargo/mutants.toml"
CDT = "crates/math/src/cdt/"


def selected(*args):
    result = subprocess.run(
        ["cargo", "mutants", "--list", "--json", "--package", "remus-math", *args],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
        timeout=120,
    )
    return json.loads(result.stdout)


def check_scope(mutants):
    files = {mutant["file"] for mutant in mutants}
    expected_cdt = {
        str(path.relative_to(ROOT))
        for path in (ROOT / CDT).glob("*.rs")
        if path.name != "tests.rs"
    }
    if not expected_cdt or {p for p in files if p.startswith(CDT)} != expected_cdt:
        raise AssertionError("CDT production modules are missing from the mutation scope")
    required = {
        "crates/math/src/predicates.rs",
        "crates/math/src/filtered.rs",
        "crates/math/src/convex_hull.rs",
    }
    if not required <= files or not any("/nurbs/" in p for p in files):
        raise AssertionError("existing numeric scope was lost")
    if any(
        not (p in required or "/nurbs/" in p or p.startswith(CDT))
        or p.endswith("/tests.rs")
        or any(part in {"tests", "benches", "examples"} for part in Path(p).parts)
        for p in files
    ):
        raise AssertionError("default config discovery or test exclusions failed")


def rejects(mutants, label):
    try:
        check_scope(mutants)
    except AssertionError:
        print(f"Rejected negative control: {label}")
    else:
        raise AssertionError(f"scope oracle accepted {label}")


def main():
    weekly = (ROOT / ".github/workflows/mutants.yml").read_text()
    versions = re.findall(r"cargo-mutants@([0-9.]+)", weekly)
    version = subprocess.run(
        ["cargo", "mutants", "--version"], cwd=ROOT, check=True,
        capture_output=True, text=True, timeout=30,
    ).stdout.strip()
    if len(versions) != 1 or version != f"cargo-mutants {versions[0]}":
        raise AssertionError("scope check must use the weekly workflow's cargo-mutants version")
    if (ROOT / "mutants.toml").exists():
        raise AssertionError("ambiguous root-level cargo-mutants config remains")
    mutants = selected()
    check_scope(mutants)
    rejects(selected("--no-config"), "config not loaded")
    config = CONFIG.read_text()
    stale = config.replace('"crates/math/src/cdt/**"', '"crates/math/src/cdt.rs"')
    if stale == config:
        raise AssertionError("stale-glob negative control was not constructed")
    with tempfile.TemporaryDirectory(prefix="remus-mutants-scope-") as directory:
        path = Path(directory) / "mutants.toml"
        path.write_text(stale)
        rejects(selected("--config", str(path)), "obsolete CDT file glob")
    counts = collections.Counter(m["file"] for m in mutants if m["file"].startswith(CDT))
    print(json.dumps(dict(sorted(counts.items())), indent=2))
    print(f"Scope passed: {sum(counts.values())} CDT mutants in {len(counts)} modules")


if __name__ == "__main__":
    main()

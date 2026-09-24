#!/usr/bin/env python3
"""Exercise cargo-mutants' real default config discovery and CDT selection.

Also guards the weekly job's budget settings (B19): every workflow that installs
cargo-mutants pins the version this check runs, each long-tail test that
`.cargo/mutants.toml` drops from the per-mutant oracle still exists under its
exact name (a renamed test would silently rejoin it), and the verdict/planner
tests in scripts/test-mutants-verdict.py pass.
"""

import collections
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / ".cargo/mutants.toml"
CDT = "crates/math/src/cdt/"
WORKFLOWS = ROOT / ".github/workflows"
EXCLUDED_TEST = re.compile(r"test\(=([A-Za-z_][A-Za-z0-9_]*)\)")


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


def test_definitions(name):
    """Count `fn NAME(` definitions under crates/ (git grep: rg is absent on runners)."""
    result = subprocess.run(
        ["git", "grep", "-c", "-E", rf"fn {name}\(", "--", "crates"],
        cwd=ROOT, capture_output=True, text=True, timeout=60,
    )
    if result.returncode == 1:
        return 0
    if result.returncode != 0:
        raise RuntimeError(f"git grep failed ({result.returncode}): {result.stderr.strip()}")
    return sum(int(line.rsplit(":", 1)[1]) for line in result.stdout.splitlines())


def check_excluded_tests(config):
    """Every exact-name exclusion must still name exactly one test."""
    names = EXCLUDED_TEST.findall(config)
    if not names:
        raise AssertionError("no long-tail test exclusions found in the mutation config")
    if 'test_tool = "nextest"' not in config:
        raise AssertionError('nextest filter arguments need test_tool = "nextest" in the config')
    stale = [name for name in names if test_definitions(name) != 1]
    if stale:
        raise AssertionError(f"excluded tests no longer defined exactly once: {stale}")
    return names


def check_versions(version):
    """Every workflow that installs cargo-mutants pins the version this check runs."""
    installs = {}
    for path in sorted(WORKFLOWS.glob("*.yml")):
        for found in re.findall(r"cargo-mutants@([0-9.]+)", path.read_text()):
            installs.setdefault(path.name, set()).add(found)
    if "fleet-mutants-sharded.yml" not in installs:
        raise AssertionError("the weekly sharded mutation workflow does not install cargo-mutants")
    caller = (WORKFLOWS / "mutants.yml").read_text()
    if not re.search(r"uses: esaueng/remus/\.github/workflows/fleet-mutants-sharded\.yml@[0-9a-f]{40}", caller):
        raise AssertionError("mutants.yml does not call the sharded mutation workflow at a pinned commit")
    for name, found in sorted(installs.items()):
        if {f"cargo-mutants {v}" for v in found} != {version}:
            raise AssertionError(f"{name} pins cargo-mutants {sorted(found)}; this check runs {version}")


def main():
    version = subprocess.run(
        ["cargo", "mutants", "--version"], cwd=ROOT, check=True,
        capture_output=True, text=True, timeout=30,
    ).stdout.strip()
    check_versions(version)
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

    excluded = check_excluded_tests(config)
    renamed = config.replace(f"test(={excluded[0]})", f"test(={excluded[0]}_renamed)")
    try:
        check_excluded_tests(renamed)
    except AssertionError:
        print("Rejected negative control: renamed long-tail test exclusion")
    else:
        raise AssertionError("exclusion oracle accepted a test name that no longer exists")
    print(f"Exclusions passed: {len(excluded)} long-tail tests still defined once each")

    verdict = subprocess.run(
        [sys.executable, str(ROOT / "scripts/test-mutants-verdict.py")],
        cwd=ROOT, capture_output=True, text=True, timeout=120,
    )
    if verdict.returncode != 0:
        raise AssertionError("verdict/planner tests failed:\n" + verdict.stdout + verdict.stderr)
    print("Verdict and planner tests passed")


if __name__ == "__main__":
    main()

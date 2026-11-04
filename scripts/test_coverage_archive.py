#!/usr/bin/env python3
"""Exercise real instrumented archives, isolated workers and merged line totals."""

import json
import os
from pathlib import Path
import subprocess
import tempfile
import tomllib

import ci_coverage

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    toolchain = tomllib.loads((ROOT / "rust-toolchain.toml").read_text())["toolchain"]["channel"]
    with tempfile.TemporaryDirectory(prefix="coverage-archive-proof-") as tmp:
        root = Path(tmp)
        (root / "src").mkdir()
        (root / "Cargo.toml").write_text('''[package]
name = "coverage-archive-proof"
version = "0.1.0"
edition = "2024"
[profile.ci-test]
inherits = "test"
opt-level = 1
debug = false
''')
        (root / "src/lib.rs").write_text('''pub fn value(n: u32) -> u32 {
    match n {
        0 => 11,
        1 => 22,
        2 => 33,
        _ => 44,
    }
}
pub fn never_called() -> u32 {
    99
}
#[cfg(test)]
mod tests {
''' + "".join(f"#[test] fn case_{i}() {{ assert_eq!(super::value({i % 4}), {(i % 4 + 1) * 11}); }}\n" for i in range(32)) + "}\n")

        def run(*args, target=None, expected=0):
            env = dict(os.environ, RUSTUP_TOOLCHAIN=toolchain)
            if target:
                env["CARGO_LLVM_COV_TARGET_DIR"] = str(target)
                target.mkdir(parents=True, exist_ok=True)
            result = subprocess.run(["cargo", *args], cwd=root, env=env, capture_output=True, text=True)
            if result.returncode != expected:
                raise AssertionError(f"cargo {' '.join(args)} exited {result.returncode}\n{result.stdout}\n{result.stderr}")
            return result

        archive = root / "tests.tar.zst"
        run("llvm-cov", "nextest-archive", "--workspace", "--cargo-profile", "ci-test", "--archive-file", str(archive))
        baseline = root / "baseline"
        run("llvm-cov", "nextest", "--archive-file", str(archive), "--workspace-remap", str(root), "--no-report", target=baseline)
        baseline_report = root / "baseline.json"
        run("llvm-cov", "report", "--nextest-archive-file", str(archive), "--json", "--summary-only", "--output-path", str(baseline_report), target=baseline)
        artifacts = root / "artifacts"
        artifacts.mkdir()
        for shard in range(1, 5):
            worker = root / f"worker-{shard}"
            run("llvm-cov", "nextest", "--archive-file", str(archive), "--workspace-remap", str(root), "--no-report", "--partition", f"hash:{shard}/4", target=worker)
            inventory = json.loads(run("nextest", "list", "--archive-file", str(archive), "--workspace-remap", str(root), "--partition", f"hash:{shard}/4", "--message-format", "json").stdout)
            ci_coverage.record(artifacts / f"shard-{shard}", worker, inventory, source, archive, shard, 4)
        merged = root / "merged"
        merged.mkdir()
        inventory = json.loads(run("nextest", "list", "--archive-file", str(archive), "--workspace-remap", str(root), "--extract-to", str(merged), "--message-format", "json").stdout)
        verified = ci_coverage.verify(sorted(artifacts.iterdir()), inventory, source, archive, 4)
        import shutil
        for shard, profile in verified:
            shutil.copyfile(profile, merged / f"{shard}-{profile.name}")
        merged_report = root / "merged.json"
        run("llvm-cov", "report", "--nextest-archive-file", str(archive), "--json", "--summary-only", "--output-path", str(merged_report), "--fail-under-lines", "60", target=merged)
        before = json.loads(baseline_report.read_text())["data"][0]["totals"]["lines"]
        after = json.loads(merged_report.read_text())["data"][0]["totals"]["lines"]
        assert before == after, (before, after)
        assert after["covered"] < after["count"], "fixture must have uncovered lines to exercise threshold failure"
        rejected = run("llvm-cov", "report", "--nextest-archive-file", str(archive), "--lcov", "--output-path", str(root / "rejected.info"), "--fail-under-lines", "100", target=merged, expected=1)
        assert "coverage" in rejected.stderr.lower(), rejected.stderr
        print(f"PASS: four isolated archive partitions reproduce the single-run line totals: {after}")
        print("PASS: incomplete coverage fails the configured floor during LCOV export")


if __name__ == "__main__":
    main()

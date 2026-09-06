#!/usr/bin/env python3
"""Check hosted fallback and required-gate wiring for the VPS migration."""

from __future__ import annotations

import json
import os
import re
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
CI = (ROOT / ".github/workflows/ci.yml").read_text()
OWNER = (ROOT / ".github/workflows/owner-pr.yml").read_text()


def job(name):
    return re.split(r"\n  [a-z][a-z-]*:\n", CI.split(f"\n  {name}:\n", 1)[1], 1)[0]


class OwnerRoutingTests(unittest.TestCase):
    def test_caller_pins_the_reviewed_workflow_content(self):
        match = re.search(r"uses: esaueng/remus/.github/workflows/owner-pr.yml@([0-9a-f]{40})", CI)
        self.assertIsNotNone(match)
        pinned = subprocess.check_output(
            ["git", "show", f"{match[1]}:.github/workflows/owner-pr.yml"],
            cwd=ROOT, text=True)
        self.assertEqual(pinned, OWNER, "Update the immutable pin after changing the callee")
        self.assertNotIn("secrets:", job("owner-pr"))

    def test_native_checks_keep_hosted_fallback(self):
        for name in ("clippy", "test", "msrv", "fuzz-check", "docs"):
            with self.subTest(name=name):
                block = job(name)
                self.assertIn("needs: [changes, owner-pr]", block)
                self.assertIn("needs.owner-pr.outputs.trusted != 'true'", block)
                self.assertIn("runs-on: ubuntu-latest", block)
                flag = "docs" if name == "docs" else "heavy"
                self.assertIn(f"needs.changes.outputs.{flag} == 'true'", block)

    def test_specialized_jobs_keep_their_runners_and_permissions(self):
        for name in ("repo-policy", "approx-census", "coverage", "wasm",
                     "render", "deny", "audit", "secrets-scan", "wasm-size"):
            with self.subTest(name=name):
                self.assertIn("runs-on: ubuntu-latest", job(name))
                self.assertNotIn("needs.owner-pr.outputs.trusted", job(name))
        self.assertIn("os: [macos-latest]", job("platform-test"))
        self.assertIn("cargo llvm-cov report --fail-under-lines 60", job("coverage"))

    def test_vps_preserves_native_commands(self):
        for command in (
            "cargo clippy --all-targets --all-features -- -D warnings",
            "cargo nextest run --workspace --cargo-profile ci-test",
            "cargo test --workspace --profile ci-test --doc",
            "cargo nextest run -p remus-operations --features perf-counters --cargo-profile ci-test",
            "cargo check --manifest-path fuzz/Cargo.toml --bins",
            "cargo doc --no-deps --all-features",
            "mdbook build book",
            "cargo +1.88.0 check --workspace --all-features",
        ):
            with self.subTest(command=command):
                self.assertIn(command, OWNER)
        self.assertIn("test(scaling_)", OWNER)
        self.assertNotIn("continue-on-error", OWNER)

    def test_ci_pass_rejects_vps_failure_or_cancellation(self):
        block = job("ci-pass")
        self.assertIn("needs: [changes, owner-pr,", block)
        self.assertIn("if: always()", block)
        command = block.split("        run: |\n", 1)[1].strip()
        for result, expected in (("success", 0), ("failure", 1), ("cancelled", 1)):
            with self.subTest(result=result):
                needs = {"changes": {"result": "success"}, "owner-pr": {"result": result},
                         "clippy": {"result": "skipped"}, "test": {"result": "skipped"}}
                run = subprocess.run(
                    ["bash", "-c", command],
                    env=dict(os.environ, NEEDS=json.dumps(needs)), capture_output=True)
                self.assertEqual(run.returncode, expected, run.stderr.decode())

    def test_policy_regressions_run_in_repository_policy(self):
        self.assertIn("python3 scripts/test-owner-pr-policy.py", job("repo-policy"))
        self.assertIn("python3 scripts/test-owner-pr-routing.py", job("repo-policy"))


if __name__ == "__main__":
    unittest.main()


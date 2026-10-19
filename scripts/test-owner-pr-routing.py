#!/usr/bin/env python3
"""Check hosted fallback and required-gate wiring for the VPS migration."""

from __future__ import annotations

import json
import os
import re
import subprocess
import unittest
from pathlib import Path
from fleet_workflows import workflow_path
from test_fleet_workflows import FleetWorkflowTests


ROOT = Path(__file__).resolve().parent.parent
CI = workflow_path("ci").read_text()
CALLER = (ROOT / ".github/workflows/ci.yml").read_text()


def job(name):
    return re.split(r"\n  [a-z][a-z-]*:\n", CI.split(f"\n  {name}:\n", 1)[1], 1)[0]


class OwnerRoutingTests(unittest.TestCase):
    def test_caller_pins_the_reviewed_workflow_content(self):
        match = re.search(r"uses: esaueng/remus/.github/workflows/(fleet-ci(?:-v[1-9][0-9]*)?\.yml)@([0-9a-f]{40})", CALLER)
        self.assertIsNotNone(match)
        pinned = subprocess.check_output(
            ["git", "show", f"{match[2]}:.github/workflows/{match[1]}"],
            cwd=ROOT, text=True)
        self.assertEqual(pinned, CI, "Update the immutable pin after changing the callee")
        self.assertNotIn("secrets:", CALLER)
        self.assertNotIn("select-runner.yml", CI)

    def test_native_checks_keep_hosted_fallback(self):
        for name, flag in (("test", "heavy"), ("msrv", "full"), ("fuzz-check", "full"),
                           ("docs", "docs")):
            with self.subTest(name=name):
                block = job(name)
                self.assertIn("needs: [changes, repo-policy, secrets-scan]", block)
                self.assertNotIn("needs.owner-pr", block)
                self.assertRegex(block, r"runs-on: [*&]fleet(?:-light)?-runner")
                self.assertIn(f"needs.changes.outputs.{flag} == 'true'", block)
        self.assertIn("cargo clippy --all-targets --all-features -- -D warnings", job("test"))

    def test_specialized_jobs_keep_their_runners_and_permissions(self):
        for name in ("repo-policy", "approx-census", "coverage", "wasm",
                     "render", "deny", "audit", "secrets-scan"):
            with self.subTest(name=name):
                self.assertRegex(job(name), r"runs-on: [*&]fleet(?:-light)?-runner")
                self.assertNotIn("needs.owner-pr.outputs.trusted", job(name))
        self.assertIn("cargo llvm-cov report --profile ci-test --fail-under-lines 60", job("coverage"))
        self.assertIn("id-token: write", job("coverage"))
        self.assertIn("id-token: write", CALLER)

    def test_ci_pass_rejects_vps_failure_or_cancellation(self):
        block = job("ci-pass")
        self.assertIn("needs: [changes, repo-policy,", block)
        self.assertIn("if: always()", block)
        command = block.rsplit("        run: |\n", 1)[1].strip()
        for result, expected in (("success", 0), ("failure", 1), ("cancelled", 1)):
            with self.subTest(result=result):
                needs = {"changes": {"result": "success"}, "repo-policy": {"result": result},
                         "secrets-scan": {"result": "success"},
                         "test": {"result": "skipped"}, "coverage": {"result": "skipped"}}
                run = subprocess.run(
                    ["bash", "-c", command],
                    env=dict(os.environ, NEEDS=json.dumps(needs), HEAVY="false", DOCS="false",
                             FULL="false", WASM="false", EVENT="pull_request"),
                    capture_output=True)
                self.assertEqual(run.returncode, expected, run.stderr.decode())

    def test_policy_regressions_run_in_repository_policy(self):
        self.assertIn("python3 scripts/test-owner-pr-policy.py", job("repo-policy"))
        self.assertIn("python3 scripts/test-owner-pr-routing.py", job("repo-policy"))


if __name__ == "__main__":
    unittest.main()


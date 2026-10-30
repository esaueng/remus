#!/usr/bin/env python3
"""Pin the staged fleet-ci-v7.yml to fleet-ci-v6.yml plus two reviewed job splits.

V7 moves two steps out of the jobs that set the PR critical path (hosted run
40112934568, 2026-10-24) into jobs of their own that start beside them:

* Complexity guards left Test, where they spent about four minutes rebuilding
  remus-operations with `perf-counters` after the workspace suite. The new
  Complexity Guards job keeps them a tier-1 (`heavy`) gate.
* W9 preflight left WASM Build & Validate, where it spent about 2.5 minutes
  after the package build. It builds its own native example and dev package
  and reads nothing the package build writes, so W9 Preflight runs under the
  same `wasm` selection beside it.

CI Pass needs both jobs and lets each skip only when its own selector is off.
Every other byte, including every guard, permission, runner expression and
command, must equal the landed v6 callee.
"""

import json
import os
import re
import subprocess
import unittest
from pathlib import Path

from fleet_workflows import workflow_text

ROOT = Path(__file__).resolve().parents[1]
V6_LANDED = "05fca2fc1029be782afe41ee6ebbeae812e5e40c"
V6_PATH = ".github/workflows/fleet-ci-v6.yml"

CHECKOUT = "actions/checkout@3d3c42e5aac5ba805825da76410c181273ba90b1 # v7.0.1"
RUST_CACHE = "Swatinem/rust-cache@f0d9c3887740aee45f6153b24b3a6b815192ec16 # v2"
INSTALL = "taiki-e/install-action@ab6895351a88b908e96a4697860d4ffea5e57dbe # v2.87.27"
SETUP_NODE = "actions/setup-node@949feb2413d6458794dcd2491c4babbbce0c15c1 # v7.1.0"
UPLOAD = "actions/upload-artifact@cf430e030ddbb5b0abf93d22962f4752f3646cd9 # v7.0.2"
GATED = ("${{ !cancelled() && needs.changes.result == 'success' && "
         "needs.repo-policy.result == 'success' && needs.secrets-scan.result == 'success' && "
         "(needs.changes.outputs.%s == 'true') }}")
GUARDS = ("cargo nextest run -p remus-operations --features perf-counters "
          "--cargo-profile ci-test -E 'test(scaling_)'")
W9_STEPS = f"""\
      - name: W9 native and WASM preflight workflow
        run: bash scripts/test-w9-preflight.sh
      - name: Preserve W9 stage report
        if: always()
        uses: {UPLOAD}
        with:
          name: w9-preflight
          path: target/w9-preflight.json
          if-no-files-found: warn
"""

# (old, new) pairs applied in order; each old text occurs exactly once.
EDITS = (
    (f"""\
      - run: cargo test --workspace --profile ci-test --doc
      # Complexity-regression guards: deterministic work-counter tests that fail
      # if a boolean hot path regresses to O(N²) (issue #987). Gated behind the
      # `perf-counters` feature, so they need an explicit run.
      - name: Complexity guards
        run: {GUARDS}
  approx-census:
""", f"""\
      - run: cargo test --workspace --profile ci-test --doc
  complexity-guards:
    # Complexity-regression guards: deterministic work-counter tests that fail
    # if a boolean hot path regresses to O(N²) (issue #987). Gated behind the
    # `perf-counters` feature, so they need their own build; this job runs it
    # beside Test instead of after the workspace suite.
    name: Complexity Guards
    needs: [queue-trust, changes, repo-policy, secrets-scan]
    if: {GATED % "heavy"}
    runs-on: *fleet-runner
    timeout-minutes: 30
    steps:
      - *fleet-source
      - *fleet-isolation
      - uses: {CHECKOUT}
        with:
          persist-credentials: false
      - *fleet-rust
      - uses: {RUST_CACHE}
        with:
          save-if: ${{{{ github.ref == 'refs/heads/main' }}}}
      - uses: {INSTALL}
        with:
          tool: nextest
      - name: Complexity guards
        run: {GUARDS}
  approx-census:
"""),
    (f"""\
            (cd "$pkg" && npm pack --dry-run)
          done
{W9_STEPS}  wasm-size-gate:
""", f"""\
            (cd "$pkg" && npm pack --dry-run)
          done
  w9-preflight:
    # W9 builds its own native example and dev WASM package with the
    # `workflow-probes` feature and reads nothing the package build writes,
    # so it runs beside WASM Build & Validate under the same selection.
    name: W9 Preflight
    needs: [queue-trust, changes, repo-policy, secrets-scan]
    if: {GATED % "wasm"}
    runs-on: *fleet-runner
    timeout-minutes: 30
    steps:
      - *fleet-source
      - *fleet-isolation
      - uses: {CHECKOUT}
        with:
          persist-credentials: false
      - *fleet-rust
      - name: Install Node for the W9 preflight
        if: runner.environment == 'self-hosted'
        uses: {SETUP_NODE}
        with:
          node-version: 24
      - uses: {RUST_CACHE}
        with:
          save-if: ${{{{ github.ref == 'refs/heads/main' }}}}
      - uses: {INSTALL}
        with:
          tool: wasm-pack
{W9_STEPS}  wasm-size-gate:
"""),
    ("    needs: [queue-trust, changes, repo-policy, test, approx-census, coverage, msrv, "
     "wasm, wasm-size-gate, wasm-no-io, fuzz-check, render, deny, audit, docs, secrets-scan]\n",
     "    needs: [queue-trust, changes, repo-policy, test, complexity-guards, approx-census, "
     "coverage, msrv, wasm, w9-preflight, wasm-size-gate, wasm-no-io, fuzz-check, render, "
     "deny, audit, docs, secrets-scan]\n"),
    ("""\
          # A skipped job passes only when the classifier deselected its
          # tier: docs by `docs`, the package build by `wasm`, the second
          # tier by `full`, and the PR tier by `heavy`. The size gate
""", """\
          # A skipped job passes only when the classifier deselected its
          # tier: docs by `docs`, the package build and W9 preflight by
          # `wasm`, the second tier by `full`, and the PR tier (with the
          # complexity guards) by `heavy`. The size gate
"""),
    ('                elif $job == "wasm" then $wasm == "false"\n',
     '                elif (["wasm", "w9-preflight"] | index($job)) != null then $wasm == "false"\n'),
)

NEW_JOBS = {"complexity-guards": "heavy", "w9-preflight": "wasm"}
ALWAYS = {"changes", "repo-policy", "secrets-scan"}
TIER1 = {"test", "complexity-guards", "approx-census", "wasm-no-io", "fuzz-check", "render"}
TIER2 = {"coverage", "msrv", "deny", "audit"}
PACKAGE = {"wasm", "w9-preflight"}
SIZE_GATE = "wasm-size-gate"


def landed_v6():
    return subprocess.check_output(["git", "show", f"{V6_LANDED}:{V6_PATH}"], cwd=ROOT, text=True)


def apply_edits(text):
    for old, new in EDITS:
        if text.count(old) != 1:
            raise AssertionError(f"Reviewed v7 edit does not match v6 exactly once:\n{old}")
        text = text.replace(old, new)
    return text


def jobs(text):
    return dict(re.findall(r"^  ([\w-]+):\n(.*?)(?=^  [\w-]+:\n|\Z)",
                           text.split("\njobs:\n", 1)[1], re.M | re.S))


TEXT = workflow_text("fleet-ci-v7.yml")
JOBS = jobs(TEXT)


class StagedCiV7Tests(unittest.TestCase):
    def complete(self, results, heavy="true", docs="true", full="false", wasm="true",
                 event="pull_request"):
        command = JOBS["ci-pass"].rsplit("        run: |\n", 1)[1]
        return subprocess.run(
            ["bash", "-c", command], capture_output=True, text=True,
            env=dict(os.environ, NEEDS=json.dumps(results), HEAVY=heavy, DOCS=docs,
                     FULL=full, WASM=wasm, EVENT=event),
        ).returncode

    def success(self):
        return {name: {"result": "success"}
                for name in TIER1 | TIER2 | PACKAGE | ALWAYS | {"docs", SIZE_GATE}}

    def test_landed_v6_is_a_main_ancestor(self):
        self.assertEqual(subprocess.run(
            ["git", "merge-base", "--is-ancestor", V6_LANDED, "origin/main"],
            cwd=ROOT, capture_output=True).returncode, 0)

    def test_v7_is_v6_plus_only_the_reviewed_edits(self):
        self.assertEqual(TEXT, apply_edits(landed_v6()),
                         "Staged v7 may change only the reviewed job splits")

    def test_a_drifted_v7_is_rejected(self):
        drifted = TEXT.replace("  contents: read", "  contents: write", 1)
        self.assertNotEqual(drifted, apply_edits(landed_v6()))
        # An intervening change to a moved step leaves the reviewed edit unmatched.
        with self.assertRaisesRegex(AssertionError, "exactly once"):
            apply_edits(landed_v6().replace("      - name: Complexity guards\n", "", 1))

    def test_moved_steps_run_only_in_their_new_jobs(self):
        self.assertEqual(TEXT.count("test(scaling_)"), 1)
        self.assertIn(GUARDS, JOBS["complexity-guards"])
        self.assertEqual(TEXT.count("bash scripts/test-w9-preflight.sh"), 1)
        self.assertIn(W9_STEPS, JOBS["w9-preflight"])
        self.assertNotIn("w9", JOBS["wasm"].lower())
        self.assertIn("cargo xtask wasm-build", JOBS["wasm"])
        self.assertIn("cargo nextest run --workspace", JOBS["test"])

    def test_new_jobs_keep_the_tier_one_admission_and_isolation(self):
        for name, flag in NEW_JOBS.items():
            with self.subTest(job=name):
                block = JOBS[name]
                header = block.split("    steps:", 1)[0]
                self.assertIn("    needs: [queue-trust, changes, repo-policy, secrets-scan]\n", header)
                self.assertIn(f"    if: {GATED % flag}\n", header)
                self.assertIn("    runs-on: *fleet-runner\n    timeout-minutes: 30\n", header)
                self.assertNotIn("permissions:", block)
                self.assertNotIn("always()", header)
                self.assertNotIn("secrets.", block)
                steps = block.split("    steps:\n", 1)[1]
                self.assertTrue(steps.startswith("      - *fleet-source\n      - *fleet-isolation\n"))
                self.assertLess(steps.index("*fleet-isolation"), steps.index("actions/checkout@"))
                self.assertIn("persist-credentials: false", steps)
                self.assertIn("save-if: ${{ github.ref == 'refs/heads/main' }}", steps)

    def test_every_job_is_in_the_completion_gate(self):
        names = set(re.search(r"needs: \[([^]]+)\]", JOBS["ci-pass"])[1].split(", "))
        self.assertEqual(names, set(JOBS) - {"ci-pass"})
        self.assertEqual(names, TIER1 | TIER2 | PACKAGE | ALWAYS | {"queue-trust", "docs", SIZE_GATE})

    def test_full_suite_passes_and_any_new_job_failure_fails(self):
        self.assertEqual(self.complete(self.success()), 0)
        for name in NEW_JOBS:
            for result in ("failure", "cancelled", "skipped"):
                with self.subTest(job=name, result=result):
                    results = self.success()
                    results[name]["result"] = result
                    self.assertNotEqual(self.complete(results), 0)

    def test_guards_skip_only_with_the_pr_tier(self):
        results = self.success()
        for name in TIER2 | PACKAGE | {SIZE_GATE}:
            results[name]["result"] = "skipped"
        self.assertEqual(self.complete(results, wasm="false"), 0)
        results["complexity-guards"]["result"] = "skipped"
        self.assertNotEqual(self.complete(results, wasm="false"), 0)
        for name in TIER1:
            results[name]["result"] = "skipped"
        self.assertEqual(self.complete(results, heavy="false", wasm="false"), 0)

    def test_w9_skips_only_with_the_package_build(self):
        results = self.success()
        for name in TIER2:
            results[name]["result"] = "skipped"
        results["w9-preflight"]["result"] = "skipped"
        self.assertNotEqual(self.complete(results, wasm="true"), 0)
        for event in ("push", "merge_group", "workflow_dispatch"):
            with self.subTest(event=event):
                self.assertNotEqual(self.complete(results, wasm="true", event=event), 0)
        for name in PACKAGE | {SIZE_GATE}:
            results[name]["result"] = "skipped"
        self.assertEqual(self.complete(results, wasm="false"), 0)

    def test_queue_timeout_leaves_headroom_for_required_jobs(self):
        config = json.loads((ROOT / ".github/merge-queue-ruleset.json").read_text())
        queue_timeout = config["rules"][0]["parameters"]["check_response_timeout_minutes"]
        timeouts = [int(re.search(r"^    timeout-minutes: ([0-9]+)$", JOBS[name], re.M)[1])
                    for name in set(JOBS) - {"queue-trust"}]
        self.assertGreaterEqual(queue_timeout, max(timeouts) + 60)


if __name__ == "__main__":
    unittest.main()

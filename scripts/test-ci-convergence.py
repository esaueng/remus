#!/usr/bin/env python3
"""Exercise admission, early-failure, and merge-group completion contracts."""

import importlib.util
import json
import os
import re
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CALLER = (ROOT / ".github/workflows/ci.yml").read_text()
FLEET = (ROOT / ".github/workflows/fleet-ci.yml").read_text()
BENCH = (ROOT / ".github/workflows/fleet-benchmark.yml").read_text()
BENCH_CALLER = (ROOT / ".github/workflows/benchmark.yml").read_text()
SPEC = importlib.util.spec_from_file_location("direct", ROOT / "scripts/test-direct-fleet.py")
DIRECT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DIRECT)


def jobs(text):
    return dict(re.findall(r"^  ([\w-]+):\n(.*?)(?=^  [\w-]+:\n|\Z)",
                           text.split("\njobs:\n", 1)[1], re.M | re.S))


JOBS = jobs(FLEET)
# Tier 1 runs for every heavy change; tier 2 only for main pushes, merge
# groups, dispatches, and PRs labelled `ci:full`; the package build when the
# diff touches WASM-affecting paths or tier 2 is selected.
TIER1 = {"test", "approx-census", "wasm-no-io"}
TIER2 = {"platform-test", "coverage", "msrv", "fuzz-check", "render", "deny", "audit"}
PACKAGE = {"wasm"}
HEAVY = TIER1 | TIER2 | PACKAGE
ALWAYS = {"changes", "repo-policy", "secrets-scan"}


class ConvergenceTests(unittest.TestCase):
    def complete(self, results, heavy="true", docs="true", full=None, wasm=None):
        command = JOBS["ci-pass"].rsplit("        run: |\n", 1)[1]
        full = heavy if full is None else full
        wasm = full if wasm is None else wasm
        return subprocess.run(
            ["bash", "-c", command], capture_output=True, text=True,
            env=dict(os.environ, NEEDS=json.dumps(results), HEAVY=heavy, DOCS=docs,
                     FULL=full, WASM=wasm),
        ).returncode

    def success(self):
        return {name: {"result": "success"} for name in HEAVY | ALWAYS | {"docs"}}

    def concurrency(self, event, ref, run_id):
        github = {"event_name": event, "ref": ref, "run_id": str(run_id)}
        group = re.search(r"^  group: (.+)$", CALLER, re.M)[1]
        group = re.sub(r"\$\{\{.*?\}\}",
                       lambda match: str(DIRECT.evaluate(match[0], github, {})), group)
        cancel = re.search(r"^  cancel-in-progress: (.+)$", CALLER, re.M)[1]
        return group, DIRECT.evaluate(cancel, github, {})

    def test_three_main_runs_cannot_replace_each_others_pending_slot(self):
        # A concurrency group has only one pending slot even when running
        # cancellation is disabled. Three pushes must therefore use three groups.
        runs = [self.concurrency("push", "refs/heads/main", run_id)
                for run_id in (100, 101, 102)]
        self.assertEqual(len({group for group, _ in runs}), 3)
        self.assertTrue(all(cancel is False for _, cancel in runs))

    def test_same_pr_is_superseded_but_other_prs_are_independent(self):
        first = self.concurrency("pull_request", "refs/pull/7/merge", 100)
        newer = self.concurrency("pull_request", "refs/pull/7/merge", 101)
        other = self.concurrency("pull_request", "refs/pull/8/merge", 102)
        self.assertEqual(first[0], newer[0])
        self.assertNotEqual(first[0], other[0])
        self.assertTrue(all(cancel is True for _, cancel in (first, newer, other)))

    def test_merge_group_runs_keep_independent_verdicts(self):
        runs = [self.concurrency("merge_group", "refs/heads/gh-readonly-queue/main/test", run_id)
                for run_id in (100, 101, 102)]
        self.assertEqual(len({group for group, _ in runs}), 3)
        self.assertTrue(all(cancel is False for _, cancel in runs))

    def test_reusable_jobs_do_not_add_another_concurrency_gate(self):
        self.assertNotIn("concurrency:", jobs(CALLER)["checks"])
        self.assertNotIn("remus-ci-suite", CALLER)
        self.assertNotIn("concurrency:", FLEET)

    def test_expensive_jobs_wait_for_both_early_gates(self):
        for name in HEAVY | {"docs"}:
            with self.subTest(name=name):
                self.assertIn("needs: [changes, repo-policy, secrets-scan]", JOBS[name])
                self.assertNotIn("always()", JOBS[name].split("    steps:", 1)[0])
        for name in ALWAYS:
            self.assertNotIn("    needs:", JOBS[name])

    def test_each_job_is_selected_by_its_tier_flag(self):
        for name, flag in [*((n, "heavy") for n in TIER1), *((n, "full") for n in TIER2),
                           ("wasm", "wasm"), ("docs", "docs")]:
            with self.subTest(name=name):
                header = JOBS[name].split("    steps:", 1)[0]
                self.assertIn(f"if: needs.changes.outputs.{flag} == 'true'", header)
        self.assertNotIn("clippy:", FLEET)
        self.assertIn("cargo clippy --all-targets --all-features -- -D warnings", JOBS["test"])
        self.assertLess(JOBS["test"].index("cargo clippy"), JOBS["test"].index("cargo nextest run"))
        self.assertIn("FORCE_FULL: ${{ github.event_name != 'pull_request' || "
                      "contains(github.event.pull_request.labels.*.name, 'ci:full') }}",
                      JOBS["changes"])
        self.assertIn("--head \"$GITHUB_SHA\" $full_flag", JOBS["changes"])

    def test_every_required_job_is_in_completion_gate(self):
        names = re.search(r"needs: \[([^]]+)\]", JOBS["ci-pass"])[1]
        self.assertEqual(set(names.split(", ")), HEAVY | ALWAYS | {"docs"})
        self.assertNotIn("continue-on-error", FLEET)

    def test_full_suite_passes(self):
        self.assertEqual(self.complete(self.success()), 0)

    def test_failure_cancellation_and_unexpected_skip_fail(self):
        for name in self.success():
            for result in ("failure", "cancelled", "skipped"):
                with self.subTest(job=name, result=result):
                    results = self.success()
                    results[name]["result"] = result
                    self.assertNotEqual(self.complete(results), 0)

    def test_pr_tier_can_skip_only_second_tier_and_package_jobs(self):
        results = self.success()
        for name in TIER2 | PACKAGE:
            results[name]["result"] = "skipped"
        self.assertEqual(self.complete(results, full="false", wasm="false"), 0)
        # The package build is still required when the classifier selected it.
        self.assertNotEqual(self.complete(results, full="false", wasm="true"), 0)
        # Tier 2 stays required whenever it was selected.
        self.assertNotEqual(self.complete(results, full="true", wasm="false"), 0)
        for name in TIER1 | ALWAYS | {"docs"}:
            altered = json.loads(json.dumps(results))
            altered[name]["result"] = "skipped"
            with self.subTest(job=name):
                self.assertNotEqual(self.complete(altered, full="false", wasm="false"), 0)
        # A wasm-affecting PR runs the package build without tier 2.
        results["wasm"]["result"] = "success"
        self.assertEqual(self.complete(results, full="false", wasm="true"), 0)
        # A tier-2 or package selection without a heavy selection is inconsistent.
        self.assertNotEqual(self.complete(self.success(), heavy="false", full="true", wasm="true"), 0)
        self.assertNotEqual(self.complete(self.success(), heavy="false", full="false", wasm="true"), 0)

    def test_docs_and_metadata_can_skip_only_unselected_jobs(self):
        results = self.success()
        for name in HEAVY:
            results[name]["result"] = "skipped"
        self.assertEqual(self.complete(results, heavy="false"), 0)
        results["docs"]["result"] = "skipped"
        self.assertNotEqual(self.complete(results, heavy="false"), 0)
        self.assertEqual(self.complete(results, heavy="false", docs="false"), 0)
        for name in ALWAYS:
            altered = json.loads(json.dumps(results))
            altered[name]["result"] = "skipped"
            self.assertNotEqual(self.complete(altered, heavy="false", docs="false"), 0)

    def test_missing_classifier_output_cannot_pass(self):
        for heavy, docs in (("", "true"), ("true", ""), ("invalid", "false")):
            self.assertNotEqual(self.complete(self.success(), heavy, docs), 0)
        for full, wasm in (("", "true"), ("true", ""), ("invalid", "true"), ("true", "maybe")):
            self.assertNotEqual(self.complete(self.success(), full=full, wasm=wasm), 0)

    def test_required_completion_runs_in_the_trusted_suite(self):
        gate = JOBS["ci-pass"]
        self.assertIn("name: CI Pass", gate)
        self.assertIn("if: always()", gate)
        self.assertIn("runs-on: *fleet-light-runner", gate)
        self.assertNotIn("ci-pass", jobs(CALLER))
        self.assertIn("checks / CI Pass", (ROOT / "docs/owner-pr-ci.md").read_text())

    def test_merge_groups_use_all_changes_against_queue_base(self):
        self.assertIn("merge_group:\n    types: [checks_requested]", CALLER)
        self.assertIn("MERGE_BASE: ${{ github.event.merge_group.base_sha }}", JOBS["changes"])
        self.assertIn('base="${MERGE_BASE:-$PR_BASE}"', JOBS["changes"])

    def test_coverage_uses_one_profile_without_weakening_threshold(self):
        commands = re.findall(r"^          (cargo llvm-cov .+)$", JOBS["coverage"], re.M)
        self.assertEqual(len(commands), 4)
        self.assertEqual(commands[0], "cargo llvm-cov clean --workspace")
        self.assertIn("--cargo-profile ci-test", commands[1])
        for command in commands[2:]:
            self.assertIn("--profile ci-test", command)
        self.assertIn("--fail-under-lines 60", JOBS["coverage"])
        self.assertNotIn("--ignore-run-fail", JOBS["coverage"])
        self.assertNotIn("--exclude", JOBS["coverage"])

    def test_optional_io_is_independent_and_still_required(self):
        self.assertNotIn("--no-default-features", JOBS["wasm"])
        self.assertIn("cargo xtask wasm-build", JOBS["wasm"])
        self.assertIn("test-w9-preflight.sh", JOBS["wasm"])
        self.assertIn("--target wasm32-unknown-unknown --no-default-features -- -D warnings",
                      JOBS["wasm-no-io"])
        self.assertIn("cargo test -p remus-wasm --no-default-features --profile ci-test",
                      JOBS["wasm-no-io"])

    def test_benchmarks_are_opt_in_for_prs(self):
        bench = jobs(BENCH)["bench"]
        self.assertIn("github.event_name != 'pull_request' ||", bench)
        self.assertIn("contains(github.event.pull_request.labels.*.name, 'ci:benchmark')", bench)
        self.assertIn("github.event.label.name == 'ci:benchmark'", bench)
        self.assertIn("workflow_dispatch:", BENCH_CALLER)
        self.assertIn("test(scaling_)", JOBS["test"])

    def test_advisory_size_report_cannot_block_required_gate(self):
        self.assertNotIn("wasm-size", JOBS)
        caller_jobs = jobs(CALLER)
        self.assertIn("needs: checks", caller_jobs["wasm-size"])
        self.assertIn("needs.checks.outputs.wasm == 'true'", caller_jobs["wasm-size"])
        self.assertNotIn("wasm-size", JOBS["ci-pass"])
        self.assertIn("pull-requests: read", caller_jobs["checks"])

    def test_queue_configuration_preserves_every_merge_validation(self):
        config = json.loads((ROOT / ".github/merge-queue-ruleset.json").read_text())
        self.assertEqual(config["enforcement"], "disabled")
        self.assertEqual(config["conditions"]["ref_name"]["include"], ["refs/heads/main"])
        self.assertEqual(config["bypass_actors"], [])
        queue = config["rules"][0]["parameters"]
        self.assertEqual(queue["grouping_strategy"], "ALLGREEN")
        self.assertEqual(queue["max_entries_to_build"], 2)


if __name__ == "__main__":
    unittest.main()

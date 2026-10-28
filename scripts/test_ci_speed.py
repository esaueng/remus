#!/usr/bin/env python3
"""Exercise the staged CI fan-in, artifact boundary, and privacy event coverage."""

import json
import os
from pathlib import Path
import re
import subprocess
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[1]
WORKFLOWS = ROOT / ".github/workflows"
STAGED = (WORKFLOWS / "fleet-ci-v9.yml").read_text()
PREVIOUS = (WORKFLOWS / "fleet-ci-v7.yml").read_text()


def jobs(text):
    return dict(re.findall(r"^  ([\w-]+):\n(.*?)(?=^  [\w-]+:\n|\Z)",
                           text.split("\njobs:\n", 1)[1], re.M | re.S))


JOBS = jobs(STAGED)
ALWAYS = {"changes", "repo-policy", "secrets-scan"}
NATIVE = {"test", "test-shards", "complexity-guards", "approx-census",
          "wasm-no-io", "fuzz-check", "render"}
PACKAGE = {"wasm", "wasm-runtime", "w9-preflight"}
SECOND = {"coverage", "msrv", "deny", "audit"}
REQUIRED = ALWAYS | NATIVE | PACKAGE | SECOND | {"docs", "wasm-size-gate"}


class SpeedContractTests(unittest.TestCase):
    def verdict(self, results, event="pull_request", heavy="true", full="true",
                wasm="true", docs="true"):
        command = textwrap.dedent(JOBS["ci-pass"].rsplit("        run: |\n", 1)[1])
        return subprocess.run(
            ["bash", "-c", command], capture_output=True, text=True,
            env=dict(os.environ, NEEDS=json.dumps(results), EVENT=event,
                     HEAVY=heavy, FULL=full, WASM=wasm, DOCS=docs),
        ).returncode

    def success(self):
        return {name: {"result": "success"} for name in REQUIRED}

    def test_no_application_job_or_guard_was_removed(self):
        self.assertEqual(set(JOBS), set(jobs(PREVIOUS)) | {"test-shards", "wasm-runtime"})
        # A new immutable revision is hosted until the fleet owner explicitly
        # registers it and opts in. Existing fleet variables alone cannot route it.
        self.assertEqual(JOBS["queue-trust"].replace(
            "      vars.CI_FLEET_CI_V9_ENABLED == 'true' &&\n", ""),
            jobs(PREVIOUS)["queue-trust"])
        for anchor in ("fleet-isolation", "fleet-source", "fleet-rust"):
            pattern = r"      - &" + anchor + r"\n(.*?)(?=^      - )"
            self.assertEqual(re.search(pattern, STAGED, re.M | re.S)[0],
                             re.search(pattern, PREVIOUS, re.M | re.S)[0])
        for name in ("test-shards", "wasm-runtime"):
            job = JOBS[name]
            self.assertIn("      fail-fast: false", job)
            self.assertIn("runs-on: *fleet-runner", job)
            self.assertLess(job.index("*fleet-source"), job.index("actions/checkout@"))
            self.assertLess(job.index("*fleet-isolation"), job.index("actions/checkout@"))
            self.assertIn("persist-credentials: false", job)
            self.assertNotIn("secrets.", job)
            self.assertNotIn(": write", job)

    def test_native_suite_builds_once_and_both_partitions_are_required(self):
        build, shards = JOBS["test"], JOBS["test-shards"]
        self.assertIn("cargo nextest archive --workspace --cargo-profile ci-test", build)
        self.assertNotIn("cargo nextest run --workspace", build)
        self.assertIn("cargo clippy --all-targets --all-features -- -D warnings", build)
        self.assertIn("cargo test --workspace --profile ci-test --doc", build)
        self.assertIn("shard: [1, 2]", shards)
        self.assertIn('--partition "hash:$SHARD/2"', shards)
        self.assertIn('--workspace-remap "$GITHUB_WORKSPACE"', shards)
        self.assertIn("--archive-file", shards)
        self.assertNotIn("--exclude", shards)
        self.assertNotIn("--no-tests", shards)
        for job in (build, shards):
            self.assertIn("tool: nextest@0.9.150", job)

    def test_both_complete_runtime_suites_consume_the_same_artifact(self):
        producer, consumer = JOBS["wasm"], JOBS["wasm-runtime"]
        self.assertIn("cargo xtask wasm-build --defer-package-tests", producer)
        self.assertIn("suite: [smoke, tarball-consumer]", consumer)
        self.assertIn('node "scripts/test-wasm-$SUITE.mjs"', consumer)
        self.assertIn("scripts/install-wasm-package-archive.py", consumer)
        for job in (producer, consumer):
            self.assertIn("name: committed-wasm-package", job)
        self.assertNotIn("run-id:", consumer)
        self.assertNotIn("github-token:", consumer)
        self.assertNotIn("tar -x", consumer)
        integrity = (WORKFLOWS / "wasm-version.yml").read_text()
        self.assertIn("cargo xtask wasm-build\n", integrity)
        self.assertNotIn("--defer-package-tests", integrity)

    def test_every_selected_failure_cancellation_or_skip_fails_ci_pass(self):
        declared = re.search(r"needs: \[([^]]+)\]", JOBS["ci-pass"])[1]
        self.assertEqual(set(declared.split(", ")), REQUIRED | {"queue-trust"})
        self.assertEqual(self.verdict(self.success()), 0)
        for job in REQUIRED:
            for outcome in ("failure", "cancelled", "skipped"):
                with self.subTest(job=job, outcome=outcome):
                    results = self.success()
                    results[job]["result"] = outcome
                    self.assertNotEqual(self.verdict(results), 0)

    def test_coverage_deferral_is_limited_to_main_pushes(self):
        results = self.success()
        results["coverage"]["result"] = "skipped"
        results["wasm-size-gate"]["result"] = "skipped"
        self.assertEqual(self.verdict(results, event="push"), 0)
        for event in ("merge_group", "workflow_dispatch", "pull_request"):
            self.assertNotEqual(self.verdict(results, event=event), 0)
        for result in ("failure", "cancelled"):
            results["coverage"]["result"] = result
            self.assertNotEqual(self.verdict(results, event="push"), 0)
        header = JOBS["coverage"].split("    steps:\n", 1)[0]
        self.assertIn("github.event_name != 'push'", header)

    def test_light_selections_cannot_hide_required_privacy_or_policy_checks(self):
        results = self.success()
        for job in NATIVE | PACKAGE | SECOND | {"wasm-size-gate"}:
            results[job]["result"] = "skipped"
        self.assertEqual(self.verdict(results, heavy="false", full="false", wasm="false"), 0)
        for job in ALWAYS:
            for result in ("failure", "cancelled", "skipped"):
                altered = json.loads(json.dumps(results))
                altered[job]["result"] = result
                self.assertNotEqual(self.verdict(altered, heavy="false", full="false", wasm="false"), 0)

    def test_nightly_coverage_preserves_all_tests_and_the_threshold(self):
        nightly = (WORKFLOWS / "coverage.yml").read_text()
        self.assertIn("schedule:", nightly)
        self.assertIn("workflow_dispatch:", nightly)
        self.assertIn("github.ref == 'refs/heads/main' && github.ref_protected", nightly)
        for command in ("cargo llvm-cov clean --workspace",
                        "cargo llvm-cov nextest --workspace --cargo-profile ci-test --no-report",
                        "cargo llvm-cov report --profile ci-test --lcov --output-path lcov.info",
                        "cargo llvm-cov report --profile ci-test --fail-under-lines 60"):
            self.assertIn(command, nightly)
        self.assertNotIn("--exclude", nightly)
        self.assertNotIn("--ignore-run-fail", nightly)

    def test_workspace_cache_is_confined_to_the_native_producer(self):
        writers = {name for name, job in JOBS.items() if "cache-workspace-crates: true" in job}
        self.assertEqual(writers, {"test"})
        self.assertIn("save-if: ${{ github.ref == 'refs/heads/main' }}", JOBS["test"])
        self.assertIn("prefix-key: v1-native-workspace", JOBS["test"])
        self.assertNotIn("Swatinem/rust-cache", JOBS["test-shards"])
        self.assertNotIn("Swatinem/rust-cache", JOBS["wasm-runtime"])

    def test_shared_publisher_preserves_the_clean_credential_boundary(self):
        publisher = (WORKFLOWS / "publish.yml").read_text()
        build, write = jobs(publisher).values()
        self.assertIn("workflow_call:", publisher)
        self.assertIn("reuse_artifact:", publisher)
        self.assertIn("!inputs.reuse_artifact", build)
        self.assertNotIn("environment:", build)
        self.assertNotIn("secrets.", build)
        self.assertIn("github.ref == 'refs/heads/main' && github.ref_protected", write)
        self.assertIn("inputs.reuse_artifact && github.event_name == 'push'", write)
        self.assertIn("environment: remus-bot", write)
        self.assertIn("scripts/install-wasm-package-archive.py", write)
        self.assertNotIn("cargo ", write)
        self.assertNotIn("run-id:", write)
        self.assertIn("name: committed-wasm-package", write)
        self.assertIn('if [ "$current_main" != "$GITHUB_SHA" ]', write)
        self.assertLess(write.index('if [ "$current_main"'),
                        write.index("secrets.REMUS_BOT_PRIVATE_KEY"))

    def test_benchmark_baseline_updates_only_on_nightly_or_main_dispatch(self):
        benchmark = (WORKFLOWS / "fleet-benchmark-v4.yml").read_text()
        push = re.search(r"auto-push: (.+)", benchmark)[1]
        self.assertIn("github.ref == 'refs/heads/main'", push)
        self.assertIn("github.event_name == 'schedule'", push)
        self.assertIn("github.event_name == 'workflow_dispatch'", push)
        self.assertNotIn("github.event_name == 'pull_request'", push)
        self.assertIn("alert-threshold: '200%'", benchmark)
        self.assertIn("fail-on-alert: false", benchmark)

    def test_every_branch_tag_and_merge_candidate_keeps_a_privileged_audit(self):
        request = (WORKFLOWS / "history-privacy-request.yml").read_text()
        events = request.split("permissions:", 1)[0]
        self.assertIn('branches: ["**"]', events)
        self.assertIn('tags: ["**"]', events)
        self.assertIn("  merge_group:", events)
        self.assertNotIn("branches-ignore", events)
        self.assertNotIn("paths", events)
        self.assertNotIn("!gh-readonly-queue", events)
        history = (WORKFLOWS / "history-privacy.yml").read_text()
        self.assertIn("pull_request_target:", history)
        for event in ("opened", "synchronize", "reopened", "edited", "ready_for_review"):
            self.assertIn(event, history.split("permissions:", 1)[0])
        self.assertIn('workflows: ["History privacy request"]', history)
        self.assertIn("environment: history-privacy", history)
        self.assertIn("python3 .github/privacy/commit_privacy.py --publish", history)
        self.assertIn("python3 .github/privacy/history_privacy.py --ci", history)
        # Regression coverage is part of the required policy gate after the
        # redundant unprivileged PR request is retired during activation.
        self.assertIn("unittest discover -s .github/privacy -p 'test_*.py'", JOBS["repo-policy"])


if __name__ == "__main__":
    unittest.main()

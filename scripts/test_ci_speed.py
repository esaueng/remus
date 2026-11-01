#!/usr/bin/env python3
"""Exercise the staged CI fan-in, artifact boundary, and privacy event coverage."""

import json
import importlib.util
import os
from pathlib import Path
import re
import subprocess
import tempfile
import textwrap
import unittest

from fleet_workflows import workflow_text

ROOT = Path(__file__).resolve().parents[1]
WORKFLOWS = ROOT / ".github/workflows"
STAGED = workflow_text("fleet-ci-v9.yml")
PREVIOUS = workflow_text("fleet-ci-v7.yml")


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
            self.assertIn("runs-on: *native-runner" if name == "test-shards" else "runs-on: *fleet-runner", job)
            self.assertLess(job.index("*fleet-source"), job.index("actions/checkout@"))
            self.assertLess(job.index("*fleet-isolation"), job.index("actions/checkout@"))
            self.assertIn("persist-credentials: false", job)
            self.assertNotIn("secrets.", job)
            self.assertNotIn(": write", job)

    def test_compressed_transport_archives_are_not_compressed_again(self):
        for producer, name in ((JOBS["test"], "native-tests"),
                               (JOBS["wasm"], "committed-wasm-package"),
                               (jobs((WORKFLOWS / "publish.yml").read_text())["build-committed-package"],
                                "committed-wasm-package")):
            self.assertIn(f"name: {name}\n          compression-level: 0\n", producer)

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

    def test_native_archives_require_the_same_source_and_compiled_fixture_root(self):
        self.assertIn("runs-on: &native-runner ubuntu-24.04", JOBS["test"])
        self.assertIn("runs-on: *native-runner", JOBS["test-shards"])
        self.assertIn("check-native-test-workspace.py --record", JOBS["test"])
        consumer = JOBS["test-shards"]
        self.assertLess(consumer.index("check-native-test-workspace.py"),
                        consumer.index("cargo-nextest nextest run"))
        spec = importlib.util.spec_from_file_location(
            "native_workspace", ROOT / "scripts/check-native-test-workspace.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        source = "a" * 40
        workspace = "/srv/ci-slot1/work/remus/remus"
        receipt = {"schema": 1, "workspace": workspace, "source_sha": source}
        module.validate(receipt, source, workspace)
        for sha, directory in (("b" * 40, workspace), ("", workspace),
                               (source, "/srv/ci-slot2/work/remus/remus")):
            with self.subTest(source=sha, workspace=directory), self.assertRaises(ValueError):
                module.validate(receipt, sha, directory)
        for changes in ({"schema": True}, {"schema": 2}, {"extra": "unexpected"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                module.validate(dict(receipt, **changes), source, workspace)

    def test_lightweight_newer_main_does_not_discard_a_pending_package(self):
        publisher = jobs((WORKFLOWS / "publish.yml").read_text())["sync-committed-package"]
        step = publisher.split("        id: package-diff\n", 1)[1].split(
            "      - name: Require publish credentials", 1)[0]
        command = textwrap.dedent(step.split("        run: |\n", 1)[1])
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, script in {
                "git": '#!/bin/sh\ncase "$1" in\nls-remote) echo "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb refs/heads/main";;\nadd) exit 0;;\ndiff) exit "$DIFF_STATUS";;\n*) exit 93;;\nesac\n',
                "python3": '#!/bin/sh\ntest "$1" = scripts/check-wasm-version.py && test "$2" = --base && test "$3" = "$GITHUB_SHA" || exit 94\ntouch "$VALIDATED"\n',
            }.items():
                path = root / name
                path.write_text(script)
                path.chmod(0o755)
            for different in (False, True):
                output, validated = root / "output", root / "validated"
                output.unlink(missing_ok=True)
                validated.unlink(missing_ok=True)
                result = subprocess.run(["bash", "-euo", "pipefail", "-c", command],
                    capture_output=True, text=True, env=dict(os.environ,
                    PATH=str(root) + os.pathsep + os.environ["PATH"],
                    GITHUB_SHA="a" * 40, GITHUB_OUTPUT=str(output),
                    DIFF_STATUS="1" if different else "0", VALIDATED=str(validated)))
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("needs_publish=" + str(different).lower(), output.read_text())
                self.assertEqual(validated.exists(), different)

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
        self.assertIn("shared-key: wasm-package", JOBS["wasm"])
        self.assertIn("tool: wasm-pack@0.15.0", JOBS["wasm"])
        manual = jobs((WORKFLOWS / "publish.yml").read_text())["build-committed-package"]
        self.assertIn("shared-key: wasm-package", manual)
        self.assertIn("RUSTFLAGS: -Dwarnings", manual)

    def test_new_revisions_stay_hosted_until_their_own_registration_switch(self):
        spec = importlib.util.spec_from_file_location("speed_direct", ROOT / "scripts/test-direct-fleet.py")
        direct = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(direct)
        fixture = direct.DirectFleetTests()
        fixture.setUp()
        ci_expressions = re.findall(
            r"^    runs-on: (?:&fleet(?:-light)?-runner )?(\$\{\{ fromJSON\(.+\) \}\})$",
            STAGED, re.M,
        )
        self.assertEqual(len(ci_expressions), 2)
        benchmark = workflow_text("fleet-benchmark-v4.yml")
        bench_expression = re.search(r"runs-on: &fleet-runner (\$\{\{.+\}\})", benchmark)[1]
        for switch, expressions in (
            ("CI_FLEET_CI_V9_ENABLED", ci_expressions),
            ("CI_FLEET_BENCHMARK_V4_ENABLED", [bench_expression]),
        ):
            for expression in expressions:
                for enabled in (None, "", "false", "true"):
                    variables = dict(fixture.variables, **{switch: enabled})
                    expected = ({"group": "ci-trusted-main", "labels": "ci-server-jane-5gb"}
                                if enabled == "true" else "ubuntu-24.04")
                    self.assertEqual(direct.evaluate(expression, fixture.github, variables), expected)
                wrong_actor = dict(fixture.github, actor="outsider")
                self.assertEqual(direct.evaluate(expression, wrong_actor, dict(fixture.variables, **{switch: "true"})),
                                 "ubuntu-24.04")

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
        self.assertNotIn("current_main", write)

    def test_publisher_requires_explicit_environment_secret_forwarding(self):
        caller_jobs = jobs((WORKFLOWS / "ci.yml").read_text())
        secret_jobs = {name for name, block in caller_jobs.items()
                       if re.search(r"^    secrets:", block, re.M)}
        self.assertEqual(secret_jobs, {"publish-package"})
        caller = caller_jobs["publish-package"]
        bindings = re.findall(r"^      ([\w-]+): ([^\n]+)$",
                              caller.split("    secrets:\n", 1)[1], re.M)
        self.assertEqual(bindings, [("REMUS_BOT_PRIVATE_KEY", "${{ secrets.REMUS_BOT_PRIVATE_KEY }}")])
        self.assertNotIn("secrets: inherit", caller)
        publisher = (WORKFLOWS / "publish.yml").read_text()
        interface = publisher.split("  workflow_call:\n", 1)[1].split("  workflow_dispatch:", 1)[0]
        declared = interface.split("    secrets:\n", 1)[1]
        self.assertEqual(re.findall(r"^      ([\w-]+):$", declared, re.M),
                         ["REMUS_BOT_PRIVATE_KEY"])
        self.assertIn("        required: true", declared)
        # The environment, rather than the build or test workers, holds the value.
        for name, block in jobs(publisher).items():
            if name == "sync-committed-package":
                self.assertIn("environment: remus-bot", block)
            else:
                self.assertNotIn("environment:", block)
                self.assertNotIn("secrets.", block)

    def test_benchmark_baseline_updates_only_on_nightly_or_main_dispatch(self):
        benchmark = workflow_text("fleet-benchmark-v4.yml")
        push = re.search(r"auto-push: (.+)", benchmark)[1]
        self.assertIn("github.ref == 'refs/heads/main'", push)
        self.assertIn("github.event_name == 'schedule'", push)
        self.assertIn("github.event_name == 'workflow_dispatch'", push)
        self.assertNotIn("github.event_name == 'pull_request'", push)
        caller = (WORKFLOWS / "benchmark.yml").read_text()
        self.assertIn('  schedule:\n    - cron: "29 6 * * *"', caller)
        pull_request = caller.split("  pull_request:\n", 1)[1].split("  workflow_dispatch:", 1)[0]
        self.assertIn("labeled", pull_request)
        self.assertNotIn("paths", pull_request)
        self.assertIn("contains(github.event.pull_request.labels.*.name, 'ci:benchmark')", benchmark)
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

    def test_automatic_publishing_requires_a_qualified_protected_main_artifact(self):
        from types import SimpleNamespace
        caller = jobs((WORKFLOWS / 'ci.yml').read_text())['publish-package']
        self.assertIn('needs: [checks, public-commit-metadata]', caller)
        self.assertIn('reuse_artifact: true', caller)
        self.assertIn('uses: ./.github/workflows/publish.yml', caller)
        self.assertNotIn('secrets: inherit', caller)
        condition = re.search(r'    if: >-\n(.*?)^    uses:', caller, re.M | re.S)[1]
        condition = ' '.join(condition.split()).replace('&&', ' and ').replace('||', ' or ')
        condition = condition.replace('needs.public-commit-metadata', 'needs.public_commit_metadata')
        def allowed(event='push', ref='refs/heads/main', protected=True,
                    result='success', wasm='true', metadata='success'):
            github = SimpleNamespace(event_name=event, ref=ref, ref_protected=protected)
            needs = SimpleNamespace(checks=SimpleNamespace(
                result=result, outputs=SimpleNamespace(wasm=wasm)),
                public_commit_metadata=SimpleNamespace(result=metadata))
            return bool(eval(condition, {'__builtins__': {}}, dict(github=github, needs=needs)))
        self.assertTrue(allowed())
        for changes in (
            {'event': 'pull_request'}, {'event': 'pull_request_target'},
            {'event': 'merge_group'}, {'event': 'workflow_dispatch'},
            {'ref': 'refs/heads/feature'}, {'protected': False},
            {'result': 'failure'}, {'result': 'cancelled'}, {'result': 'skipped'},
            {'wasm': 'false'}, {'wasm': ''},
            {'metadata': 'failure'}, {'metadata': 'cancelled'}, {'metadata': 'skipped'},
        ):
            with self.subTest(changes=changes):
                self.assertFalse(allowed(**changes))
        events = (WORKFLOWS / 'publish.yml').read_text().split('permissions:', 1)[0]
        self.assertNotIn('  push:', events)
        self.assertIn('  workflow_dispatch:', events)

    def test_every_previous_refresh_trigger_still_gets_a_main_package_build(self):
        spec = importlib.util.spec_from_file_location('speed_classifier', ROOT / 'scripts/classify-ci-changes.py')
        classifier = importlib.util.module_from_spec(spec)
        import sys
        sys.modules[spec.name] = classifier
        spec.loader.exec_module(classifier)
        previous = subprocess.check_output([
            'git', 'show', '70f675bea4aebd35b0a36d59eaec89118c278ef6:.github/workflows/publish.yml',
        ], cwd=ROOT, text=True)
        path_scope = previous.split('    paths:\n', 1)[1].split('  workflow_dispatch:', 1)[0]
        paths = re.findall(r'^      - "([^"!][^"]*)"$', path_scope, re.M)
        self.assertGreaterEqual(len(paths), 10)
        for path in paths + ['NOTICE']:
            path = path.replace('**', 'example.rs').replace('*', 'example')
            with self.subTest(path=path):
                # Main-push classification uses --full. Package-only refreshes
                # remain independently rebuilt and byte-compared by the guard.
                result = classifier.classify_paths([path], force_full=True)
                self.assertTrue(result.heavy)
                self.assertTrue(result.wasm)

    def test_privileged_privacy_policy_and_scanner_bytes_are_preserved(self):
        source = '70f675bea4aebd35b0a36d59eaec89118c278ef6'
        for path in (
            '.github/workflows/history-privacy.yml', '.github/privacy/policy.json',
            '.github/privacy/blocked-history.bin', '.github/privacy/history_privacy.py',
            '.github/privacy/commit_privacy.py',
        ):
            with self.subTest(path=path):
                expected = subprocess.check_output(['git', 'show', f'{source}:{path}'], cwd=ROOT)
                self.assertTrue((ROOT / path).read_bytes() == expected,
                                f'Privileged policy or scanner changed: {path}')

    def test_package_cache_consumers_match_the_protected_writer_inputs(self):
        publisher = jobs((WORKFLOWS / 'publish.yml').read_text())['build-committed-package']
        integrity = (WORKFLOWS / 'wasm-version.yml').read_text()
        producer_env = re.search(r'^env:\n(.*?)(?=^jobs:)', STAGED, re.M | re.S)[1]
        expected = dict(re.findall(r'^  ([A-Z_]+): (.+)$', producer_env, re.M))
        manual_env = re.search(r'^    env:\n(.*?)(?=^    runs-on:)', publisher, re.M | re.S)[1]
        self.assertEqual(dict(re.findall(r'^      ([A-Z_]+): (.+)$', manual_env, re.M)), expected)
        guard_env = re.search(r'^env:\n(.*?)(?=^jobs:)', integrity, re.M | re.S)[1]
        self.assertEqual(dict(re.findall(r'^  ([A-Z_]+): (.+)$', guard_env, re.M)), expected)
        for content in (JOBS['wasm'], publisher, integrity):
            self.assertIn('shared-key: wasm-package', content)
            self.assertIn("save-if: ${{ github.ref == 'refs/heads/main' }}", content)
            self.assertIn('tool: wasm-pack@0.15.0', content)


if __name__ == "__main__":
    unittest.main()

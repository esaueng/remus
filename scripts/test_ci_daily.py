"""Behavioral and workflow contracts for the staged fast/daily CI split."""

import copy
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import tomllib
import unittest

import ci_coverage
import ci_gate
import ci_native

CI_V12_LANDED = "6701fc2dc528514e4e841270255eacfa5ff1dcd6"
ROOT = Path(__file__).resolve().parents[1]
WORKFLOWS = ROOT / ".github/workflows"
STAGED = (WORKFLOWS / "fleet-ci-v12.yml").read_text()
spec = importlib.util.spec_from_file_location("daily_classifier", ROOT / "scripts/classify-ci-daily.py")
classifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(classifier)


def jobs(text):
    return dict(re.findall(r"^  ([\w-]+):\n(.*?)(?=^  [\w-]+:\n|\Z)", text.split("\njobs:\n", 1)[1], re.M | re.S))


class NativeSelectionTests(unittest.TestCase):
    def setUp(self):
        self.manifest = ci_native.suite_manifest()

    def test_leaf_source_adds_affected_module_and_keeps_critical_contracts(self):
        result = ci_native.plan(["crates/operations/src/fillet/rolling_ball.rs"], self.manifest)
        self.assertEqual(result["mode"], "fast")
        self.assertIn(["remus-operations", "fillet"], result["modules"])
        self.assertEqual(set(result["targets"]), set(self.manifest["critical"]))
        self.assertIn("kind(lib)", ci_native.expression(result))
        self.assertNotIn("--exclude", ci_native.build_args(result))

    def test_fast_archives_build_every_selected_package_and_full_archives_keep_the_workspace(self):
        selected = ci_native.plan(["crates/wasm/src/bindings/fillet_tests.rs"], self.manifest)
        args = ci_native.build_args(selected)
        packages = {args[i + 1] for i, arg in enumerate(args) if arg == "--package"}
        self.assertEqual(packages, {"remus-operations", "remus-io", "remus-wasm"})
        self.assertNotIn("--workspace", args)
        self.assertIn("--workspace", ci_native.build_args(dict(selected, mode="full")))

    def test_added_and_changed_integration_tests_are_selected_whole(self):
        for path in ("crates/operations/tests/new_regression.rs", "crates/io/tests/new_import.rs"):
            result = ci_native.plan([path], self.manifest)
            self.assertEqual(result["mode"], "fast")
            self.assertIn(path, result["targets"])
            self.assertIn(Path(path).stem, ci_native.build_args(result))
            self.assertIn("binary(=" + Path(path).stem + ")", ci_native.expression(result))

    def test_shared_test_support_is_not_treated_as_an_unrelated_file(self):
        result = ci_native.plan(["crates/operations/tests/a_regression/support.rs"], self.manifest)
        self.assertIn("crates/operations/tests/a_regression.rs", result["targets"])

    def test_foundational_unknown_fixture_and_manifest_changes_fall_back_to_full(self):
        for path in ("Cargo.toml", "Cargo.lock", ".cargo/config.toml", "scripts/new-policy.py",
                     "crates/math/src/new.rs", "crates/topology/src/arena.rs", "crates/algo/src/builder.rs",
                     "crates/blend/src/builder.rs", "crates/operations/src/lib.rs",
                     "crates/io/tests/data/model.step", "crates/future/src/lib.rs"):
            with self.subTest(path=path):
                result = ci_native.plan([path], self.manifest)
                self.assertEqual(result["mode"], "full")
                self.assertEqual(ci_native.expression(result), "all()")
                self.assertNotIn("--lib", ci_native.build_args(result))
        self.assertEqual(ci_native.plan([], self.manifest)["mode"], "full")

    def test_queue_smoke_keeps_changed_tests_and_foundational_fallback(self):
        result = ci_native.plan(["crates/operations/src/fillet/mod.rs", "crates/operations/tests/new.rs"], self.manifest, queue=True)
        self.assertEqual(result["modules"], [])
        self.assertIn("crates/operations/tests/new.rs", result["targets"])
        self.assertEqual(ci_native.plan(["crates/math/src/vec.rs"], self.manifest, queue=True)["mode"], "full")

    def test_explicit_full_and_path_validation(self):
        self.assertEqual(ci_native.plan(["README.md"], self.manifest, full=True)["mode"], "full")
        for path in ("../escape", "/absolute", "crates/../README.md", "a//b"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                ci_native.plan([path], self.manifest)

    def test_manifest_rejects_missing_duplicate_and_malformed_critical_paths(self):
        for change in ({"schema": True}, {"schema": 2}, {"critical": []}, {"extra": True},
                       {"critical": [self.manifest["critical"][0]] * 2},
                       {"critical": ["crates/operations/tests/missing.rs"]},
                       {"critical": ["../escape.rs"]}, {"focused_crates": ["missing"]}):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                (root / ".github").mkdir()
                (root / ci_native.MANIFEST).write_text(json.dumps(dict(self.manifest, **change)))
                with self.assertRaises(ValueError):
                    ci_native.suite_manifest(root)

    def test_actual_inventory_rejects_missing_targets_and_requests_full_for_empty_modules(self):
        selection = {"targets": ["crates/operations/tests/case.rs"], "modules": [["remus-operations", "fillet"]]}
        inventory = {"rust-suites": {"case": {"package-name": "remus-operations", "binary-name": "case", "kind": "test", "testcases": {"works": {"ignored": False}}}}}
        self.assertFalse(ci_native.verify_inventory(selection, inventory))
        inventory["rust-suites"]["lib"] = {"package-name": "remus-operations", "binary-name": "remus_operations", "kind": "lib", "testcases": {"fillet::tests::works": {"ignored": False}}}
        self.assertTrue(ci_native.verify_inventory(selection, inventory))
        inventory["rust-suites"]["lib"]["testcases"] = {"unrelated::fillet::works": {"ignored": False}}
        self.assertFalse(ci_native.verify_inventory(selection, inventory))
        self.assertIn("test(/^fillet::/)", ci_native.expression(dict(selection, mode="fast")))
        inventory["rust-suites"].pop("case")
        with self.assertRaises(ValueError):
            ci_native.verify_inventory(selection, inventory)


class ClassifierTests(unittest.TestCase):
    def test_dependency_and_ci_changes_do_not_implicitly_request_coverage(self):
        for event in ("pull_request", "merge_group", "push"):
            for path in ("Cargo.lock", ".github/workflows/ci.yml", "crates/math/src/lib.rs"):
                result = classifier.classify([path], event, force_full=event != "pull_request")
                self.assertFalse(result["coverage"])
                self.assertTrue(result["native"])

    def test_ordinary_queue_candidates_receive_native_smoke_without_other_heavy_jobs(self):
        result = classifier.classify(["crates/operations/src/fillet/mod.rs"], "merge_group", force_full=True)
        self.assertTrue(result["native"])
        self.assertFalse(result["heavy"])
        self.assertFalse(result["full"])
        self.assertFalse(result["wasm"])

    def test_main_preserves_exact_package_validation_without_daily_dependency_repetition(self):
        result = classifier.classify(["crates/operations/src/chamfer.rs"], "push", force_full=True)
        self.assertTrue(result["native"])
        self.assertTrue(result["wasm"])
        self.assertFalse(result["full"])
        self.assertTrue(classifier.classify(["Cargo.lock"], "push", force_full=True)["full"])

    def test_explicit_full_and_coverage_requests_are_not_dropped_on_light_diffs(self):
        for paths in (["README.md"], ["crates/wasm/pkg/package.json"], []):
            for event in ("pull_request", "workflow_dispatch"):
                result = classifier.classify(paths, event, force_full=True, coverage=True)
                for flag in ("heavy", "full", "wasm", "native", "coverage"):
                    self.assertTrue(result[flag], (paths, event, flag))
        self.assertFalse(classifier.classify(["README.md"], "workflow_dispatch", True)["coverage"])

    def test_suite_manifest_cannot_be_treated_as_lightweight_repository_metadata(self):
        for event in ("pull_request", "merge_group", "push"):
            result = classifier.classify([".github/ci-test-suites.json"], event, force_full=event != "pull_request")
            self.assertTrue(result["native"])
            self.assertFalse(result["coverage"])

    def test_explicit_coverage_and_full_runs_remain_available(self):
        result = classifier.classify(["crates/operations/src/chamfer.rs"], "workflow_dispatch", True, True)
        self.assertTrue(result["coverage"])
        self.assertTrue(result["full"])
        for event in ("pull_request", "merge_group", "push"):
            light = classifier.classify(["README.md"], event, coverage=True)
            self.assertFalse(light["native"])
            self.assertFalse(light["coverage"])


class GateTests(unittest.TestCase):
    def setUp(self):
        self.flags = dict.fromkeys(("heavy", "native", "full", "wasm", "coverage", "docs"), "true")
        self.results = {name: {"result": "success"} for name in set.union(*ci_gate.GROUPS.values()) | {"queue-trust"}}

    def test_every_selected_failure_cancellation_and_skip_fails(self):
        ci_gate.validate(self.results, self.flags, "pull_request")
        for name in set(self.results) - {"queue-trust"}:
            for result in ("failure", "cancelled", "skipped", "pending", None):
                altered = copy.deepcopy(self.results)
                altered[name]["result"] = result
                with self.subTest(job=name, result=result), self.assertRaises(ValueError):
                    ci_gate.validate(altered, self.flags, "pull_request")

    def test_no_missing_or_unknown_jobs_or_flags_can_pass(self):
        for name in self.results:
            altered = copy.deepcopy(self.results)
            del altered[name]
            with self.subTest(job=name), self.assertRaises(ValueError):
                ci_gate.validate(altered, self.flags, "pull_request")
        for flags in ({}, dict(self.flags, native=""), dict(self.flags, native="false")):
            with self.assertRaises(ValueError):
                ci_gate.validate(self.results, flags, "pull_request")

    def test_queue_smoke_is_required_even_when_other_heavy_jobs_are_deselected(self):
        flags = dict.fromkeys(self.flags, "false")
        flags.update(native="true", docs="true")
        for group, names in ci_gate.GROUPS.items():
            if group not in ("always", "native", "docs"):
                for name in names:
                    self.results[name]["result"] = "skipped"
        ci_gate.validate(self.results, flags, "merge_group", native_mode="fast")
        self.results["test"]["result"] = "skipped"
        with self.assertRaises(ValueError):
            ci_gate.validate(self.results, flags, "merge_group", native_mode="fast")

    def test_fast_mode_can_skip_only_the_full_partition_job(self):
        self.flags["full"] = "false"
        self.results["test-shards"]["result"] = "skipped"
        ci_gate.validate(self.results, self.flags, "pull_request", native_mode="fast")
        for mode in ("full", "none", ""):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                ci_gate.validate(self.results, self.flags, "pull_request", native_mode=mode)
        self.results["test"]["result"] = "skipped"
        with self.assertRaises(ValueError):
            ci_gate.validate(self.results, self.flags, "pull_request", native_mode="fast")

    def test_full_request_cannot_be_satisfied_by_a_fast_producer_result(self):
        self.results["test-shards"]["result"] = "skipped"
        with self.assertRaisesRegex(ValueError, "complete native partitions"):
            ci_gate.validate(self.results, self.flags, "pull_request", native_mode="fast")

    def test_coverage_can_skip_only_when_deselected_and_failures_never_pass(self):
        self.flags["coverage"] = "false"
        self.results["coverage"]["result"] = "skipped"
        ci_gate.validate(self.results, self.flags, "pull_request")
        for outcome in ("failure", "cancelled"):
            self.results["coverage"]["result"] = outcome
            with self.assertRaises(ValueError):
                ci_gate.validate(self.results, self.flags, "pull_request")


class CoverageReceiptTests(unittest.TestCase):
    def inventory(self, names):
        return {"rust-suites": {"binary": {"testcases": {name: {"ignored": False, "filter-match": {"status": "matches"}} for name in names}}}}

    def test_missing_mixed_duplicate_and_corrupted_profiles_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            archive = root / "archive"
            archive.write_bytes(b"same instrumented binaries")
            raw = root / "raw"
            raw.mkdir()
            (raw / "data.profraw").write_bytes(b"profile")
            dirs = [root / str(i) for i in (1, 2)]
            for i, directory in enumerate(dirs, 1):
                ci_coverage.record(directory, raw, self.inventory([f"case{i}"]), "a" * 40, archive, i, 2)
            complete = self.inventory(["case1", "case2"])
            self.assertEqual(len(ci_coverage.verify(dirs, complete, "a" * 40, archive, 2)), 2)
            for altered in (dirs[:1], dirs * 2, [dirs[0], dirs[0]]):
                with self.assertRaises(ValueError):
                    ci_coverage.verify(altered, complete, "a" * 40, archive, 2)
            with self.assertRaises(ValueError):
                ci_coverage.verify(dirs, complete, "b" * 40, archive, 2)
            receipt_path = dirs[1] / "receipt.json"
            original = json.loads(receipt_path.read_text())
            for change in ({"archive": "wrong"}, {"shard": 1}, {"schema": True}, {"tests": ["binary\0case1"]},
                           {"tests": ["binary\0unknown"]}, {"profiles": {}}, {"total": 3}):
                receipt_path.write_text(json.dumps(dict(original, **change)))
                with self.subTest(change=change), self.assertRaises(ValueError):
                    ci_coverage.verify(dirs, complete, "a" * 40, archive, 2)
            receipt_path.write_text(json.dumps(original))
            (dirs[1] / "data.profraw").write_bytes(b"corrupted")
            with self.assertRaises(ValueError):
                ci_coverage.verify(dirs, complete, "a" * 40, archive, 2)


class StagedWorkflowTests(unittest.TestCase):
    def test_coverage_budget_reservation_is_used_without_changing_test_selection(self):
        profiles = tomllib.loads((ROOT / ".config/nextest.toml").read_text())["profile"]
        self.assertEqual(set(profiles["coverage"]), {"overrides"})
        override, = profiles["coverage"]["overrides"]
        self.assertEqual(set(override), {"filter", "priority", "threads-required"})
        self.assertEqual(override["filter"], "binary(=op_seq_81) & test(=finding_sphere_minus_box_open_mesh)")
        self.assertEqual(override["threads-required"], "num-test-threads")
        self.assertEqual(override["priority"], 100)
        coverage = jobs((WORKFLOWS / "coverage-sharded.yml").read_text())
        self.assertIn("NEXTEST_PROFILE: coverage", coverage["partitions"])
        self.assertIn('--partition "hash:$SHARD/4"', coverage["partitions"])
        proof = (WORKFLOWS / "ci-daily-proof.yml").read_text()
        self.assertIn("python3 scripts/test_coverage_scheduling.py", proof)
        self.assertIn("NEXTEST_PROFILE: coverage", jobs(proof)["coverage-budget-replay"])
        self.assertIn("test(=finding_sphere_minus_box_open_mesh)", jobs(proof)["coverage-budget-replay"])

    def test_existing_active_workflow_and_protections_are_unchanged(self):
        active = json.loads((ROOT / ".github/fleet-workflows.json").read_text())["ci"]
        if active != "fleet-ci-v12.yml":
            self.assertEqual(active, "fleet-ci-v11.yml")
            pinned = re.search(r"fleet-ci-v11.yml@([0-9a-f]{40})", (WORKFLOWS / "ci.yml").read_text())[1]
            self.assertEqual((WORKFLOWS / active).read_bytes(), subprocess.check_output(["git", "show", f"{pinned}:.github/workflows/{active}"], cwd=ROOT))
        else:
            from fleet_workflows import validate_calls
            self.assertGreater(validate_calls(), 0)
            self.assertIn("fleet-ci-v12.yml@" + CI_V12_LANDED, (WORKFLOWS / "ci.yml").read_text())
        self.assertNotIn("continue-on-error", STAGED)
        self.assertNotIn("secrets: inherit", STAGED)

    def test_unrelated_job_commands_permissions_and_trust_guards_are_preserved(self):
        before = jobs((WORKFLOWS / "fleet-ci-v11.yml").read_text())
        after = jobs(STAGED.replace("CI_FLEET_CI_V12_ENABLED", "CI_FLEET_CI_V9_ENABLED"))
        edited = {"changes", "repo-policy", "test", "test-shards", "coverage", "ci-pass"}
        for name in set(before) - edited:
            with self.subTest(job=name):
                self.assertEqual(after[name], before[name])
        gate = after["ci-pass"]
        self.assertLess(gate.index("actions/checkout@"), gate.index("python3 scripts/ci_gate.py"))

    def test_fan_in_requires_every_staged_job_and_uses_the_behaviorally_tested_gate(self):
        selected = jobs(STAGED)
        declared = set(re.search(r"needs: \[([^]]+)\]", selected["ci-pass"])[1].split(", "))
        self.assertEqual(declared, set(selected) - {"ci-pass"})
        self.assertEqual(declared, set.union(*ci_gate.GROUPS.values()) | {"queue-trust"})
        self.assertIn("python3 scripts/ci_gate.py", selected["ci-pass"])
        self.assertIn("CI_FLEET_CI_V12_ENABLED", STAGED)
        self.assertNotIn("CI_FLEET_CI_V9_ENABLED", STAGED)

    def test_critical_test_archive_and_four_partitions_preserve_source_receipts(self):
        selected = jobs(STAGED)
        self.assertIn("fetch-depth: 0", selected["test"])
        self.assertIn("filter: blob:none", selected["test"])
        self.assertIn("ci_native.py archive", selected["test"])
        self.assertIn("Run the fast selection on its build worker", selected["test"])
        self.assertIn("--shard 1 --total 1", selected["test"])
        self.assertIn("needs.test.outputs.mode == 'full'", selected["test-shards"])
        self.assertIn("check-native-test-workspace.py --record", selected["test"])
        self.assertIn("native-plan.json", selected["test"])
        self.assertIn("shard: [1, 2, 3, 4]", selected["test-shards"])
        self.assertIn("check-native-test-workspace.py", selected["test-shards"])
        self.assertIn("cargo clippy --all-targets --all-features -- -D warnings", selected["clippy"])
        self.assertNotIn("cargo clippy", selected["test"])

    def test_coverage_is_explicit_and_oidc_remains_forbidden_to_forks(self):
        coverage = jobs(STAGED)["coverage"]
        self.assertIn("needs.changes.outputs.coverage == 'true'", coverage)
        self.assertIn("github.event.pull_request.head.repo.full_name == github.repository", coverage)
        self.assertIn("--fail-under-lines 60", coverage)
        self.assertIn("COVERAGE_REQUESTED:", jobs(STAGED)["changes"])

    def test_historical_contracts_remain_valid_after_v12_becomes_main(self):
        from unittest import mock
        import test_fleet_actions_stage
        import test_fleet_ci_v10_stage
        import test_dependabot_actions_stage
        import test_fleet_workflows
        manifest = (ROOT / ".github/fleet-workflows.json").read_text()
        original = subprocess.check_output

        def main_after_activation(command, *args, **kwargs):
            if isinstance(command, (list, tuple)) and command[-1] == "origin/main:.github/fleet-workflows.json":
                return manifest if kwargs.get("text") else manifest.encode()
            return original(command, *args, **kwargs)

        cases = (
            test_fleet_actions_stage.FleetActionsStageTests("test_active_and_current_main_callees_are_preserved"),
            test_fleet_ci_v10_stage.FleetCiV10StageTests("test_active_and_current_main_ci_callees_are_v9_bytes"),
            test_dependabot_actions_stage.DependabotActionsStageTests("test_active_and_current_main_definitions_are_preserved_until_activation"),
            test_fleet_workflows.FleetWorkflowTests("test_staged_v2_only_changes_reviewed_action_dependencies"),
        )
        with mock.patch("subprocess.check_output", side_effect=main_after_activation):
            for case in cases:
                with self.subTest(contract=case.id()):
                    result = unittest.TestResult()
                    case.run(result)
                    self.assertTrue(result.wasSuccessful(), result.errors + result.failures)

    def test_activated_caller_keeps_the_schedule_guard_permissions_and_floor(self):
        nightly = (WORKFLOWS / "coverage.yml").read_text()
        self.assertIn('cron: "13 7 * * *"', nightly)
        self.assertIn("workflow_dispatch:", nightly)
        self.assertIn("cancel-in-progress: false", nightly)
        self.assertIn("github.ref == 'refs/heads/main' && github.ref_protected", nightly)
        self.assertIn("uses: ./.github/workflows/coverage-sharded.yml", nightly)
        self.assertIn("id-token: write", nightly)
        caller = (WORKFLOWS / "ci.yml").read_text()
        self.assertIn("coverage: ${{ inputs.coverage || false }}", caller)
        self.assertIn("Also run complete instrumented coverage", caller)
        self.assertIn("--fail-under-lines 60", (WORKFLOWS / "coverage-sharded.yml").read_text())

    def test_v12_runner_opt_in_still_requires_its_own_switch(self):
        spec = importlib.util.spec_from_file_location("daily_direct", ROOT / "scripts/test-direct-fleet.py")
        direct = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(direct)
        fixture = direct.DirectFleetTests()
        fixture.setUp()
        expressions = re.findall(r"^    runs-on: (?:&fleet(?:-light)?-runner )?(\$\{\{ fromJSON\(.+\) \}\})$", STAGED, re.M)
        self.assertEqual(len(expressions), 2)
        for expression in expressions:
            for enabled in (None, "false", "true"):
                variables = dict(fixture.variables, CI_FLEET_CI_V12_ENABLED=enabled)
                expected = ({"group": "ci-trusted-main", "labels": "ci-server-jane-5gb"}
                            if enabled == "true" else "ubuntu-24.04")
                self.assertEqual(direct.evaluate(expression, fixture.github, variables), expected)

    def test_daily_and_weekly_full_suites_remain_available(self):
        daily = (WORKFLOWS / "daily-validation.yml").read_text()
        self.assertIn("schedule:", daily)
        self.assertIn("--base HEAD^ --full", daily)
        self.assertIn("cargo test --workspace --profile ci-test --doc", daily)
        self.assertIn("--all-features --locked", daily)
        self.assertIn("rustsec/audit-check@", daily)
        for filename in ("fuzz.yml", "mutants.yml", "gauntlet.yml"):
            self.assertIn("schedule:", (WORKFLOWS / filename).read_text())

    def test_coverage_merge_checks_complete_partitions_before_enforcing_threshold(self):
        coverage = (WORKFLOWS / "coverage-sharded.yml").read_text()
        selected = jobs(coverage)
        self.assertIn("shard: [1, 2, 3, 4]", selected["partitions"])
        self.assertIn("ci_coverage.py merge", selected["report"])
        self.assertIn("--nextest-archive-file", selected["report"])
        self.assertIn("--fail-under-lines 60", selected["report"])
        self.assertEqual(coverage.count("id-token: write"), 1)
        self.assertIn("id-token: write", selected["upload"])
        for name in ("archive", "partitions", "report"):
            self.assertNotIn(": write", selected[name])
        self.assertNotIn("--ignore-run-fail", coverage)
        self.assertNotIn("--exclude", coverage)


if __name__ == "__main__":
    unittest.main()

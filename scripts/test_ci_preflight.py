"""Exercise policy failures, missing tools, and the immutable v13 workflow delta."""

from contextlib import redirect_stdout
import io
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import ci_preflight

ROOT = Path(__file__).resolve().parents[1]
WORKFLOWS = ROOT / ".github/workflows"


class PolicyRunnerTests(unittest.TestCase):
    def test_every_legacy_policy_command_is_preserved_in_order(self):
        source = (WORKFLOWS / "fleet-ci-v12.yml").read_text()
        legacy = [shlex.split(line.strip()) for line in source.splitlines() if line.startswith('          check "')]
        actual = [(check.name, *check.command) for check in ci_preflight.CHECKS]
        self.assertEqual(actual, [tuple(command[1:]) for command in legacy])
        self.assertEqual(len(actual), 25)
        self.assertEqual(len(actual), len(set(check.name for check in ci_preflight.CHECKS)))

    def test_failure_retains_its_exit_code_and_later_checks_still_run(self):
        checks = [ci_preflight.Check("first", (sys.executable, "-c", "raise SystemExit(9)")),
                  ci_preflight.Check("second", (sys.executable, "-c", "raise SystemExit(0)"))]
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()) as output:
            report = ci_preflight.run_checks(checks, root=Path(temporary), actions=True)
        self.assertFalse(report["passed"])
        self.assertEqual([row["exit_code"] for row in report["checks"]], [9, 0])
        self.assertIn("::error title=Repository policy::first failed with exit 9", output.getvalue())
        self.assertIn("PASS: second", output.getvalue())
        self.assertEqual(output.getvalue().count("::endgroup::"), 2)

    def test_missing_tool_is_a_failure_and_does_not_skip_the_next_check(self):
        checks = [ci_preflight.Check("missing", ("a-policy-tool-that-does-not-exist",)),
                  ci_preflight.Check("present", (sys.executable, "-c", "pass"))]
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            report = ci_preflight.run_checks(checks, root=Path(temporary))
        self.assertFalse(report["passed"])
        self.assertEqual([row["exit_code"] for row in report["checks"]], [127, 0])

    def test_report_is_compact_and_does_not_store_environment_or_command_output(self):
        with mock.patch.dict(os.environ, PRIVATE_FIXTURE="must-not-be-recorded"), redirect_stdout(io.StringIO()):
            report = ci_preflight.run_checks([ci_preflight.Check("fixture", ("test-command",))],
                                            runner=lambda *a, **k: subprocess.CompletedProcess(a, 0),
                                            clock=iter([10.0, 11.25]).__next__)
        self.assertEqual(report, {"schema": 1, "passed": True,
                                  "checks": [{"name": "fixture", "exit_code": 0, "seconds": 1.25}]})
        self.assertNotIn("must-not-be-recorded", json.dumps(report))

    def test_main_writes_failure_evidence_and_returns_nonzero(self):
        report = {"schema": 1, "passed": False,
                  "checks": [{"name": "fixture", "exit_code": 3, "seconds": 0.5}]}
        with tempfile.TemporaryDirectory() as temporary:
            output, summary = (Path(temporary) / name for name in ("policy.json", "summary.md"))
            with mock.patch.object(ci_preflight, "run_checks", return_value=report), \
                    mock.patch.dict(os.environ, GITHUB_STEP_SUMMARY=str(summary)), redirect_stdout(io.StringIO()):
                self.assertEqual(ci_preflight.main(["--json", str(output)]), 1)
            self.assertEqual(json.loads(output.read_text()), report)
            self.assertIn("FAIL (exit 3)", summary.read_text())

    def test_workflow_command_data_cannot_inject_another_annotation(self):
        self.assertEqual(ci_preflight.workflow_command("name%\n::notice::fixture\r"),
                         "name%25%0A::notice::fixture%0D")


class StagedPolicyWorkflowTests(unittest.TestCase):
    def test_only_the_policy_command_and_its_report_upload_change(self):
        before = (WORKFLOWS / "fleet-ci-v12.yml").read_text()
        after = (WORKFLOWS / "fleet-ci-v13.yml").read_text()
        start = before.index("      - name: Run repository policy checks\n")
        end = before.index("  test:\n", start)
        new_start = after.index("      - name: Run repository policy checks\n")
        new_end = after.index("  test:\n", new_start)
        self.assertEqual(after[:new_start], before[:start])
        self.assertEqual(after[new_end:], before[end:])
        block = after[new_start:new_end]
        self.assertIn('run: python3 scripts/ci_preflight.py --json "$RUNNER_TEMP/repository-policy.json"', block)
        self.assertIn("if: ${{ !cancelled() }}", block)
        self.assertIn("retention-days: 7", block)
        self.assertIn("name: repository-policy", block)
        self.assertNotIn("continue-on-error", after)

    def test_staged_workflow_does_not_activate_or_modify_the_pinned_v12(self):
        from fleet_workflows import validate_calls, workflow_manifest
        self.assertGreater(validate_calls(), 0)
        self.assertEqual(workflow_manifest()["ci"], "fleet-ci-v12.yml")
        caller = (WORKFLOWS / "ci.yml").read_text()
        self.assertNotIn("fleet-ci-v13.yml@", caller)

    def test_required_entrypoint_discovers_these_regressions(self):
        spec = importlib.util.spec_from_file_location("policy_routing_entrypoint", ROOT / "scripts/test-owner-pr-routing.py")
        entrypoint = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entrypoint)

        def case_ids(suite):
            for case in suite:
                if isinstance(case, unittest.TestSuite):
                    yield from case_ids(case)
                else:
                    yield case.id()

        loader = unittest.TestLoader()
        expected = set(case_ids(loader.loadTestsFromModule(entrypoint.test_ci_preflight)))
        discovered = set(case_ids(loader.loadTestsFromModule(entrypoint)))
        self.assertTrue(expected)
        self.assertLessEqual(expected, discovered)


if __name__ == "__main__":
    unittest.main()

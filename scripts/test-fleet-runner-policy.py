#!/usr/bin/env python3
"""Exercise the runner identity table in the fleet guard and the trusted pilot."""
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys
import unittest

from fleet_workflows import workflow_path
import test_jane_helper_workflows

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ('.github/actions/fleet-guard/action.yml', '.github/workflows/trusted-vps.yml',
           workflow_path('ci').relative_to(ROOT).as_posix())
V6_LANDED = '05fca2fc1029be782afe41ee6ebbeae812e5e40c'


class FleetRunnerPolicy(unittest.TestCase):
    def test_required_entrypoint_discovers_helper_stage_regressions(self):
        loader = unittest.TestLoader()
        expected = {case.id() for case in loader.loadTestsFromTestCase(
            test_jane_helper_workflows.JaneHelperWorkflowTests)}

        def ids(suite):
            for case in suite:
                if isinstance(case, unittest.TestSuite):
                    yield from ids(case)
                else:
                    yield case.id()

        self.assertTrue(expected)
        self.assertLessEqual(expected, set(ids(loader.loadTestsFromModule(sys.modules[__name__]))))

    def test_only_provisioned_runner_names_pass(self):
        for workflow in SOURCES:
            text = (ROOT / workflow).read_text()
            guard = re.search(r'case "\$RUNNER_NAME" in\n.*?\besac', text, re.S).group()
            for name, allowed in [('ci-vm-1441561', True), ('ci-server-jane', True), ('ci-server-john', True),
                                  ('ci-server-jane-1', True), ('ci-server-jane-2', True), ('ci-server-jane-3', True),
                                  ('ci-server-jane-0', False), ('ci-server-jane-4', False),
                                  ('', False), ('ci-server-jane-5gb', False), ('ci-server-jane-extra', False), ('unknown', False)]:
                with self.subTest(workflow=workflow, runner=name):
                    result = subprocess.run(['bash', '-c', guard], env=dict(os.environ, RUNNER_NAME=name),
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    self.assertEqual(result.returncode == 0, allowed)

    def test_profiles_match_each_slots_actual_resources(self):
        for workflow in SOURCES:
            text = (ROOT / workflow).read_text()
            guard = re.search(r'case "\$RUNNER_NAME" in\n.*?\besac', text, re.S).group()
            for name, expected in (
                ('ci-vm-1441561', 'ci-runner /srv/ci ci.slice ci-docker 3221225472 1 1 1'),
                ('ci-server-jane-1', 'ci-slot1 /srv/ci-slot1 cislot1.slice ci-slot1-docker 5000000000 6 3 6'),
                ('ci-server-jane-2', 'ci-slot2 /srv/ci-slot2 cislot2.slice ci-slot2-docker 5000000000 6 3 6'),
                ('ci-server-jane-3', 'ci-slot3 /srv/ci-slot3 cislot3.slice ci-slot3-docker 5000000000 6 3 6'),
            ):
                with self.subTest(workflow=workflow, runner=name):
                    result = subprocess.check_output(
                        ['bash', '-eu', '-c', guard + '\necho "$CI_USER $CI_STORAGE $CI_SLICE $CI_DOCKER $CI_MEMORY $CI_CPUS $CARGO_BUILD_JOBS $RUST_TEST_THREADS"'],
                        env=dict(os.environ, RUNNER_NAME=name), text=True)
                    self.assertEqual(result.strip(), expected)

    def test_cpu_validation_rejects_wrong_or_unbounded_quota(self):
        for workflow in SOURCES:
            text = (ROOT / workflow).read_text()
            program = re.search(r"awk -v cpus=\"\$CI_CPUS\" '([^']+)'", text).group(1)
            for cpus, quota, expected in [(1, '100000 100000', 0), (6, '600000 100000', 0),
                                           (6, '100000 100000', 1), (6, 'max 100000', 1)]:
                result = subprocess.run(['awk', '-v', f'cpus={cpus}', program],
                                        input=quota, text=True)
                self.assertEqual(result.returncode, expected)

    def test_staging_preserves_every_other_workflow_byte(self):
        # fleet-ci-v5.yml at reviewed routing stage ea5480872be1cda9ccd0410e6b11c4f901dae45f.
        # A committed digest keeps this check independent of peer branch reachability.
        # fleet-ci-v7.yml retired v6; read its immutable landed bytes.
        staged = subprocess.check_output(
            ['git', 'show', f'{V6_LANDED}:.github/workflows/fleet-ci-v6.yml'], cwd=ROOT, text=True)
        baseline = staged.replace('ci-server-jane-1|ci-server-jane-2|ci-server-jane-3)',
                                  'ci-server-jane-1|ci-server-jane-2)')
        baseline = baseline.replace('CI_MEMORY=5000000000', 'CI_MEMORY=6442450944')
        baseline = baseline.replace(
            '          CI_MEMORY=$((CI_MEMORY / $(getconf PAGESIZE) * $(getconf PAGESIZE)))\n', '')
        baseline = baseline.replace('labels: ci-server-jane-5gb\n', 'labels: ci-server-jane\n')
        baseline = baseline.replace('"labels":"ci-server-jane-5gb"', '"labels":"ci-server-jane"')
        self.assertEqual(hashlib.sha256(baseline.encode()).hexdigest(),
                         '00fb516f35614a9f66bf34d840b3eb8287bc9ffcbf7b48a1bb7fdac5107a0f03')

    def test_activated_profile_cannot_restore_the_retired_budget(self):
        active = workflow_path('ci')
        if active.name not in ('fleet-ci-v6.yml', 'fleet-ci-v7.yml', 'fleet-ci-v9.yml', 'fleet-ci-v11.yml'):
            return  # Other independently staged workflow versions keep their own contract.
        guard = re.search(r'case "\$RUNNER_NAME" in\n.*?\besac', active.read_text(), re.S).group()
        self.assertIn('ci-server-jane-1|ci-server-jane-2|ci-server-jane-3)', guard)
        self.assertIn('CI_MEMORY=5000000000', guard)
        self.assertNotIn('CI_MEMORY=6442450944', guard)

    def test_guard_uses_the_exact_kernel_page_aligned_limit(self):
        for workflow in SOURCES:
            text = (ROOT / workflow).read_text()
            guard = re.search(r'case "\$RUNNER_NAME" in\n.*?\besac', text, re.S).group()
            align = re.search(r'^\s*CI_MEMORY=\$\(\(.+$', text, re.M).group()
            result = subprocess.check_output(
                ['bash', '-eu', '-c', "getconf() { printf '4096\\n'; }; " + guard
                 + '\n' + align + '; printf "%s" "$CI_MEMORY"'],
                env=dict(os.environ, RUNNER_NAME='ci-server-jane-3'), text=True)
            self.assertEqual(result, '4999999488')


def load_tests(loader, tests, pattern):
    # The immutable Repository Policy job invokes this required entrypoint.
    tests.addTests(loader.loadTestsFromModule(test_jane_helper_workflows, pattern=pattern))
    return tests


if __name__ == '__main__':
    unittest.main()

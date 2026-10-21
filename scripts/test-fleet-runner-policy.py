#!/usr/bin/env python3
"""Exercise the runner identity table in the fleet guard and the trusted pilot."""
import os
from pathlib import Path
import re
import subprocess
import unittest

from fleet_workflows import workflow_path

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ('.github/actions/fleet-guard/action.yml', '.github/workflows/trusted-vps.yml',
           '.github/workflows/fleet-ci-v6.yml')


class FleetRunnerPolicy(unittest.TestCase):
    def test_only_provisioned_runner_names_pass(self):
        for workflow in SOURCES:
            text = (ROOT / workflow).read_text()
            guard = re.search(r'case "\$RUNNER_NAME" in\n.*?\besac', text, re.S).group()
            for name, allowed in [('ci-vm-1441561', True), ('ci-server-jane', True), ('ci-server-john', True),
                                  ('ci-server-jane-1', True), ('ci-server-jane-2', True), ('ci-server-jane-3', True),
                                  ('ci-server-jane-0', False), ('ci-server-jane-4', False),
                                  ('', False), ('ci-server-jane-extra', False), ('unknown', False)]:
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
        previous = subprocess.check_output(
            ['git', 'show', '9e5b65b9e00f004d83eb583ab3934c863ad7956d:.github/workflows/fleet-ci-v4.yml'], cwd=ROOT, text=True)
        expected = previous.replace('ci-server-jane-1|ci-server-jane-2)',
                                    'ci-server-jane-1|ci-server-jane-2|ci-server-jane-3)')
        expected = expected.replace('CI_MEMORY=6442450944', 'CI_MEMORY=5000000000')
        expected = expected.replace('          CI_CGROUP=',
                                    '          CI_MEMORY=$((CI_MEMORY / $(getconf PAGESIZE) * $(getconf PAGESIZE)))\n          CI_CGROUP=')
        self.assertEqual((ROOT / '.github/workflows/fleet-ci-v6.yml').read_text(), expected)

    def test_activated_profile_cannot_restore_the_retired_budget(self):
        active = workflow_path('ci')
        if active.name == 'fleet-ci-v4.yml':
            return  # This staging change deliberately leaves the current pin intact.
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


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Exercise the pre-checkout runner identity guard in both trusted workflows."""
import os
from pathlib import Path
import re
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FleetRunnerPolicy(unittest.TestCase):
    def test_only_provisioned_runner_names_pass(self):
        for workflow in ('owner-pr.yml', 'trusted-vps.yml'):
            text = (ROOT / '.github/workflows' / workflow).read_text()
            guard = re.search(r'case "\$RUNNER_NAME" in\n.*?\besac', text, re.S).group()
            for name, allowed in [('ci-vm-1441561', True), ('ci-server-jane', True), ('ci-server-john', True),
                                  ('ci-server-jane-1', True), ('ci-server-jane-2', True),
                                  ('ci-server-jane-0', False), ('ci-server-jane-3', False),
                                  ('', False), ('ci-server-jane-extra', False), ('unknown', False)]:
                with self.subTest(workflow=workflow, runner=name):
                    result = subprocess.run(['bash', '-c', guard], env=dict(os.environ, RUNNER_NAME=name),
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    self.assertEqual(result.returncode == 0, allowed)

    def test_profiles_match_each_slots_actual_resources(self):
        for workflow in ('owner-pr.yml', 'trusted-vps.yml'):
            text = (ROOT / '.github/workflows' / workflow).read_text()
            guard = re.search(r'case "\$RUNNER_NAME" in\n.*?\besac', text, re.S).group()
            for name, expected in (
                ('ci-vm-1441561', 'ci-runner /srv/ci ci.slice ci-docker 3221225472 1 1 1'),
                ('ci-server-jane-1', 'ci-slot1 /srv/ci-slot1 cislot1.slice ci-slot1-docker 6442450944 6 3 6'),
                ('ci-server-jane-2', 'ci-slot2 /srv/ci-slot2 cislot2.slice ci-slot2-docker 6442450944 6 3 6'),
            ):
                with self.subTest(workflow=workflow, runner=name):
                    result = subprocess.check_output(
                        ['bash', '-eu', '-c', guard + '\necho "$CI_USER $CI_STORAGE $CI_SLICE $CI_DOCKER $CI_MEMORY $CI_CPUS $CARGO_BUILD_JOBS $RUST_TEST_THREADS"'],
                        env=dict(os.environ, RUNNER_NAME=name), text=True)
                    self.assertEqual(result.strip(), expected)

    def test_cpu_validation_rejects_wrong_or_unbounded_quota(self):
        for workflow in ('owner-pr.yml', 'trusted-vps.yml'):
            text = (ROOT / '.github/workflows' / workflow).read_text()
            program = re.search(r"awk -v cpus=\"\$CI_CPUS\" '([^']+)'", text).group(1)
            for cpus, quota, expected in [(1, '100000 100000', 0), (6, '600000 100000', 0),
                                           (6, '100000 100000', 1), (6, 'max 100000', 1)]:
                result = subprocess.run(['awk', '-v', f'cpus={cpus}', program],
                                        input=quota, text=True)
                self.assertEqual(result.returncode, expected)


if __name__ == '__main__':
    unittest.main()

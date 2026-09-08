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
                                  ('', False), ('ci-server-jane-extra', False), ('unknown', False)]:
                with self.subTest(workflow=workflow, runner=name):
                    result = subprocess.run(['bash', '-c', guard], env=dict(os.environ, RUNNER_NAME=name),
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    self.assertEqual(result.returncode == 0, allowed)


if __name__ == '__main__':
    unittest.main()

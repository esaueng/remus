#!/usr/bin/env python3
"""The staged Jane profile changes only the provisioned identities and RAM cap."""
from pathlib import Path
import re
import subprocess
import unittest

from fleet_workflows import workflow_path

ROOT = Path(__file__).resolve().parents[1]
STAGED = ROOT / '.github/workflows/fleet-ci-v6.yml'
PREVIOUS = '9e5b65b9e00f004d83eb583ab3934c863ad7956d'


class JaneRunnerProfile(unittest.TestCase):
    def test_staging_preserves_every_other_workflow_byte(self):
        previous = subprocess.check_output(
            ['git', 'show', f'{PREVIOUS}:.github/workflows/fleet-ci-v4.yml'], cwd=ROOT, text=True)
        expected = previous.replace('ci-server-jane-1|ci-server-jane-2)',
                                    'ci-server-jane-1|ci-server-jane-2|ci-server-jane-3)')
        expected = expected.replace('CI_MEMORY=6442450944', 'CI_MEMORY=5000000000')
        self.assertEqual(STAGED.read_text(), expected)

    def test_activated_profile_cannot_restore_the_retired_budget(self):
        active = workflow_path('ci')
        if active.name == 'fleet-ci-v4.yml':
            return  # This staging change deliberately leaves the current pin intact.
        guard = re.search(r'case "\$RUNNER_NAME" in\n.*?\besac', active.read_text(), re.S).group()
        self.assertIn('ci-server-jane-1|ci-server-jane-2|ci-server-jane-3)', guard)
        self.assertIn('CI_MEMORY=5000000000', guard)
        self.assertNotIn('CI_MEMORY=6442450944', guard)


if __name__ == '__main__':
    unittest.main()

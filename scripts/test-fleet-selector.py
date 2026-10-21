#!/usr/bin/env python3
"""Keep legacy VPS labels out of every workflow and the trusted pilot's choices.

The availability selector (select-runner.yml) was retired with owner-pr.yml;
fleet jobs choose their runner from the CI_FLEET_TARGET variable.
"""
from pathlib import Path
import unittest


SOURCE = Path(__file__).resolve().parents[1] / '.github/workflows/trusted-vps.yml'


class FleetSelectorTests(unittest.TestCase):
    def test_pilot_has_no_legacy_vps_default_or_direct_target(self):
        pilot = (SOURCE.parent / 'trusted-vps.yml').read_text()
        self.assertIn('default: auto', pilot)
        self.assertIn('options: [auto, ci-server-jane, ci-server-jane-1, ci-server-jane-2, ci-server-jane-3, ci-server-john]', pilot)
        self.assertNotIn('ci-small', pilot)
        self.assertNotIn('labels: ci-vm-1441561', pilot)

    def test_default_workflows_do_not_schedule_legacy_vps_labels(self):
        for workflow in SOURCE.parent.glob('*.yml'):
            text = workflow.read_text()
            self.assertNotIn('labels: ci-small', text, workflow.name)
            self.assertNotIn('labels: ci-vm-1441561', text, workflow.name)
            self.assertNotIn('runs-on: [self-hosted, Linux, X64, ci-small', text, workflow.name)

    def test_retired_selector_and_owner_workflows_stay_retired(self):
        for workflow in ('select-runner.yml', 'owner-pr.yml'):
            self.assertFalse((SOURCE.parent / workflow).exists(), workflow)
        for workflow in SOURCE.parent.glob('*.yml'):
            self.assertNotIn('select-runner.yml', workflow.read_text(), workflow.name)


if __name__ == '__main__':
    unittest.main()

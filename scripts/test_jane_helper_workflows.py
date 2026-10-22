"""Keep the three-slot auxiliary stage exact and tied to its landed guard."""

import json
from pathlib import Path
import re
import subprocess
import unittest

from fleet_workflows import FAMILIES, workflow_manifest

ROOT = Path(__file__).resolve().parents[1]
STAGE = json.loads((ROOT / '.github/fleet-profile-stage.json').read_text())
WORKFLOWS = ROOT / '.github/workflows'


def original(filename):
    return subprocess.check_output(
        ['git', 'show', f"{STAGE['source']}:.github/workflows/{filename}"],
        cwd=ROOT, text=True,
    )


def expected_stage(text):
    guard = 'esaueng/remus/.github/actions/fleet-guard@'
    old = guard + STAGE['previous_guard']
    if old not in text or '\"labels\":\"ci-server-jane\"' not in text:
        raise AssertionError('Expected the reviewed legacy guard and Jane selector')
    return (text.replace(old, guard + STAGE['guard'])
            .replace('\"labels\":\"ci-server-jane\"', '\"labels\":\"ci-server-jane-5gb\"'))


class JaneHelperWorkflowTests(unittest.TestCase):
    def test_every_auxiliary_family_has_a_new_version(self):
        self.assertEqual(set(STAGE['workflows']), FAMILIES - {'ci'})
        for family, entry in STAGE['workflows'].items():
            self.assertNotEqual(entry['previous'], entry['staged'])
            self.assertRegex(entry['staged'], rf'^fleet-{re.escape(family)}-v[1-9][0-9]*\.yml$')

    def test_stage_changes_only_the_guard_pin_and_jane_selector(self):
        for family, entry in STAGE['workflows'].items():
            with self.subTest(family=family):
                self.assertEqual((WORKFLOWS / entry['staged']).read_text(),
                                 expected_stage(original(entry['previous'])))

    def test_staging_keeps_active_helpers_or_activates_the_exact_staged_bytes(self):
        manifest = workflow_manifest()
        for family, entry in STAGE['workflows'].items():
            with self.subTest(family=family):
                self.assertIn(manifest[family], entry.values())
                active = (WORKFLOWS / manifest[family]).read_text()
                expected = original(entry['previous'])
                if manifest[family] == entry['staged']:
                    expected = expected_stage(expected)
                    self.assertFalse((WORKFLOWS / entry['previous']).exists(),
                                     'Activation retires the previous helper file')
                self.assertEqual(active, expected)

    def test_guard_pin_is_on_main_and_has_the_verified_three_slot_contract(self):
        for pin in (STAGE['source'], STAGE['guard']):
            self.assertRegex(pin, r'^[0-9a-f]{40}$')
            self.assertEqual(subprocess.run(
                ['git', 'merge-base', '--is-ancestor', pin, 'origin/main'],
                cwd=ROOT, capture_output=True,
            ).returncode, 0, 'Stage only after the guard lands on main')
        guard = subprocess.check_output(
            ['git', 'show', f"{STAGE['guard']}:.github/actions/fleet-guard/action.yml"],
            cwd=ROOT, text=True,
        )
        self.assertEqual(guard, (ROOT / '.github/actions/fleet-guard/action.yml').read_text())
        self.assertIn('ci-server-jane-1|ci-server-jane-2|ci-server-jane-3)', guard)
        self.assertIn('CI_MEMORY=5000000000', guard)
        self.assertIn('CI_MEMORY=$((CI_MEMORY / $(getconf PAGESIZE)', guard)


if __name__ == '__main__':
    unittest.main()

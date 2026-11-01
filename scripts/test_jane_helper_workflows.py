"""Keep the three-slot auxiliary stage exact and tied to its landed guard."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import unittest

from fleet_workflows import (
    FAMILIES, validate_reference, workflow_calls, workflow_manifest, workflow_text,
)
from test_fleet_actions_stage import revert_action_updates, staged_action_update
from test_dependabot_actions_stage import WORKFLOWS as DEPENDABOT_STAGES, previous_definition

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
                self.assertEqual(workflow_text(entry['staged']),
                                 expected_stage(original(entry['previous'])))

    def test_staging_keeps_active_helpers_or_activates_the_exact_staged_bytes(self):
        manifest = workflow_manifest()
        for family, entry in STAGE['workflows'].items():
            with self.subTest(family=family):
                filename = manifest[family]
                active = workflow_text(filename)
                if family in DEPENDABOT_STAGES and filename == DEPENDABOT_STAGES[family][1]:
                    active = previous_definition(family, active)
                    filename = DEPENDABOT_STAGES[family][0]
                if family == 'benchmark' and filename == 'fleet-benchmark-v4.yml':
                    # Preserve the complete original helper-stage proof. V4's
                    # separately reviewed speed changes must match its landed pin.
                    self.assertEqual(workflow_text(entry['staged']),
                                     expected_stage(original(entry['previous'])))
                    self.assertFalse((WORKFLOWS / entry['previous']).exists(),
                                     'Activation retires the previous helper file')
                    path = '.github/workflows/' + manifest[family]
                    matching = [(p, sha) for _caller, p, sha in workflow_calls() if p == path]
                    self.assertEqual(len(matching), 1)
                    validate_reference(*matching[0], (ROOT / path).read_bytes())
                    continue
                if filename in staged_action_update(family):
                    # #1297's dependency-only update replaced the helper; the
                    # helper's own bytes stay proven at its landed commit.
                    helper = workflow_text(entry['staged'])
                    self.assertEqual(helper, expected_stage(original(entry['previous'])))
                    self.assertFalse((WORKFLOWS / entry['staged']).exists(),
                                     'Activation retires the replaced helper file')
                    self.assertEqual(revert_action_updates(
                        family, active), helper)
                    continue
                self.assertIn(filename, entry.values())
                expected = original(entry['previous'])
                if filename == entry['staged']:
                    expected = expected_stage(expected)
                    self.assertFalse((WORKFLOWS / entry['previous']).exists(),
                                     'Activation retires the previous helper file')
                self.assertEqual(active, expected)

    def test_reviewed_guard_digest_and_helper_call_counts(self):
        # Preserve the independent digest/count oracle from duplicate stage #1268.
        reviewed = '05fca2fc1029be782afe41ee6ebbeae812e5e40c'
        self.assertEqual(STAGE['source'], reviewed)
        self.assertEqual(STAGE['guard'], reviewed)
        guard = subprocess.check_output(
            ['git', 'show', reviewed + ':.github/actions/fleet-guard/action.yml'], cwd=ROOT,
        )
        self.assertEqual(hashlib.sha256(guard).hexdigest(),
                         '599087ce5da442d89e0805db24691a07b5badd6c76f52f56d8314a9fb60079b7')
        counts = {'benchmark': 2, 'fuzz': 2, 'gauntlet': 2, 'mutants-sharded': 2,
                  'mutation-scope': 1, 'osv': 1, 'wasm-candidate': 1, 'wasm-size': 1}
        prefix = 'esaueng/remus/.github/actions/fleet-guard@'
        old, new = prefix + STAGE['previous_guard'], prefix + reviewed
        for family, entry in STAGE['workflows'].items():
            with self.subTest(family=family):
                source = original(entry['previous'])
                actual = workflow_text(entry['staged'])
                self.assertEqual(source.count(old), counts[family])
                self.assertEqual(source.count('"labels":"ci-server-jane"'), 1)
                self.assertEqual(actual.count(new), counts[family])
                self.assertNotIn(old, actual)
                self.assertNotIn('"labels":"ci-server-jane"', actual)

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

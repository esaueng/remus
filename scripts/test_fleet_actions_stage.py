"""Stage reviewed Action updates without changing immutable active callees."""

import json
import re
import subprocess
import unittest

from fleet_workflows import (
    ROOT, validate_reference, workflow_text, workflow_calls, workflow_manifest, workflow_path,
)
from test_dependabot_actions_stage import WORKFLOWS as DEPENDABOT_STAGES, previous_definition


SOURCE = "f167bb2bde923b9d8a252d3ef9f6ebb475487332"
WORKFLOWS = {
    "ci": ("fleet-ci-v7.yml", "fleet-ci-v8.yml"),
    "mutants-sharded": ("fleet-mutants-sharded-v4.yml", "fleet-mutants-sharded-v5.yml"),
    "mutation-scope": ("fleet-mutation-scope-v3.yml", "fleet-mutation-scope-v4.yml"),
    "osv": ("fleet-osv-v5.yml", "fleet-osv-v6.yml"),
    "wasm-candidate": ("fleet-wasm-candidate-v4.yml", "fleet-wasm-candidate-v5.yml"),
}
UPDATES = {
    "taiki-e/install-action": (
        "ab6895351a88b908e96a4697860d4ffea5e57dbe # v2.87.27",
        "8cca0918f8471c33515202b73f56ab27f2bed283 # v2.87.29",
    ),
    "github/codeql-action/upload-sarif": (
        "24c54180a607b1449ed407dd24f251e4e9147c8d # v4.38.3",
        "6f2ac81f5cd81fe351a197ba0b00d91d03cd2aee # v4.39.0",
    ),
}


def original(filename):
    return subprocess.check_output(
        ["git", "show", f"{SOURCE}:.github/workflows/{filename}"], cwd=ROOT,
    )


def updated(content):
    for action, (old, new) in UPDATES.items():
        content = content.replace(f"uses: {action}@{old}".encode(),
                                  f"uses: {action}@{new}".encode())
    return content


def staged_action_update(family):
    """The #1297 auxiliary file for a family, once it may be the active callee."""
    return {WORKFLOWS[family][1]} if family in WORKFLOWS and family != "ci" else set()


def revert_action_updates(family, text):
    """Map an activated #1297 auxiliary back to the bytes it was staged from."""
    if not staged_action_update(family):
        return text
    for action, (old, new) in UPDATES.items():
        text = text.replace(f"uses: {action}@{new}", f"uses: {action}@{old}")
    return text


def validate_stage(content, source):
    if content != updated(source):
        raise ValueError("Staged workflow may change only the reviewed Action pins and comments")


class FleetActionsStageTests(unittest.TestCase):
    def test_source_is_an_immutable_main_ancestor(self):
        self.assertEqual(subprocess.run(
            ["git", "merge-base", "--is-ancestor", SOURCE, "origin/main"],
            cwd=ROOT, capture_output=True,
        ).returncode, 0)

    def test_staged_workflows_only_change_reviewed_actions(self):
        for previous, staged in WORKFLOWS.values():
            with self.subTest(workflow=staged):
                source = original(previous)
                content = workflow_text(staged).encode()
                self.assertNotEqual(content, source)
                validate_stage(content, source)

    def test_intervening_commands_permissions_and_unknown_pins_are_rejected(self):
        source = original(WORKFLOWS["ci"][0])
        content = updated(source)
        for before, after in (
            (b"  contents: read", b"  contents: write"),
            (b"cargo nextest run --workspace", b"echo skipped"),
            (UPDATES["taiki-e/install-action"][1].encode(), b"0" * 40),
        ):
            with self.subTest(change=before), self.assertRaisesRegex(ValueError, "only the reviewed"):
                self.assertIn(before, content)
                validate_stage(content.replace(before, after, 1), source)

    def test_active_and_current_main_callees_are_preserved(self):
        main_manifest = subprocess.check_output(
            ["git", "show", "origin/main:.github/fleet-workflows.json"], cwd=ROOT,
        )
        main_manifest = json.loads(main_manifest)
        manifest = workflow_manifest()
        for family, (previous, staged) in WORKFLOWS.items():
            with self.subTest(family=family):
                for filename, content in (
                    (manifest[family], workflow_path(family).read_bytes()),
                    (main_manifest[family], subprocess.check_output(
                        ["git", "show", f"origin/main:.github/workflows/{main_manifest[family]}"],
                        cwd=ROOT,
                    )),
                ):
                    if family == "ci" and filename in ("fleet-ci-v9.yml", DEPENDABOT_STAGES["ci"][1]):
                        self.assertEqual(workflow_text(previous).encode(),
                                         original(previous))
                        if filename == "fleet-ci-v9.yml":
                            validate_reference(".github/workflows/" + filename,
                                               "4e355573ffe9b1bc0466d0a57fcc98677d5fc5eb", content)
                            continue
                        matching = [(path, sha) for _caller, path, sha in workflow_calls()
                                    if path == ".github/workflows/" + filename]
                        self.assertEqual(len(matching), 1)
                        validate_reference(*matching[0], content)
                        continue
                    if family in DEPENDABOT_STAGES and filename == DEPENDABOT_STAGES[family][1]:
                        content = previous_definition(family, content.decode()).encode()
                        filename = DEPENDABOT_STAGES[family][0]
                    self.assertIn(filename, (previous, staged))
                    expected = original(previous)
                    if filename == staged:
                        expected = updated(expected)
                    self.assertEqual(content, expected,
                                     "Preserve intervening workflow edits before activation")

    def test_windows_arm_receipts_match_the_executed_installer(self):
        from test_dependabot_actions_stage import INSTALLER_PIN
        expected = "taiki-e/install-action@" + INSTALLER_PIN
        for workflow in ("windows-arm-tooling.yml", "parity.yml"):
            with self.subTest(workflow=workflow):
                text = (ROOT / ".github/workflows" / workflow).read_text()
                self.assertIn("uses: " + expected, text)
        for filename in ("scripts/platform-probes/windows-arm.mjs", "tools/parity/host-proof.mjs"):
            with self.subTest(proof=filename):
                text = (ROOT / filename).read_text()
                self.assertEqual(re.findall(r"action: '(taiki-e/install-action@[^']+)'", text),
                                 [expected])


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Keep staged dependency updates and active immutable workflow calls exact."""

import json
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

from fleet_workflows import (
    FAMILIES, ROOT, PREFIX, parse_reference, validate_calls, validate_reference,
    workflow_calls, workflow_manifest, workflow_path,
)

# This rollout starts from an existing main commit. The next workflow version
# changes only these reviewed Action dependencies; commands, guards, permissions,
# required check names, and runner selection must remain byte-identical.
SOURCE = "34d7636d258288476755e5ad6a4769e3f96723ea"
UPDATES = {
    "actions/upload-artifact": ("043fb46d1a93c77aae656e7c1c64a875d1fc6a0a", "cf430e030ddbb5b0abf93d22962f4752f3646cd9"),
    "actions/download-artifact": ("3e5f45b2cfb9172054b4087a40e8e0b5a5461e7c", "9000827ccba6bdab643e8b6fd33ac0654aef8333"),
    "taiki-e/install-action": ("861a07ce7084f55488e375df125cdc99bba60eb7", "ab6895351a88b908e96a4697860d4ffea5e57dbe"),
    "actions/setup-node": ("820762786026740c76f36085b0efc47a31fe5020", "949feb2413d6458794dcd2491c4babbbce0c15c1"),
    "github/codeql-action/upload-sarif": ("2892aa5e19bbd11bc0cff5427e3b750a04d9e3c2", "24c54180a607b1449ed407dd24f251e4e9147c8d"),
}


def normalize_reviewed_updates(text, updated):
    def action(match):
        prefix, repository, sha = match.groups()
        if repository not in UPDATES:
            return match[0]
        expected = UPDATES[repository][1 if updated else 0]
        if sha != expected:
            raise AssertionError(f"Unexpected Action pin for {repository}")
        return prefix + repository + "@" + UPDATES[repository][0]
    return re.sub(
        r"(?m)^(\s*(?:- )?uses: )([^@\s]+)@([0-9a-f]{40})(?:[ \t]+#[^\n]*)?$",
        action, text,
    )


def assert_reviewed_workflow_delta(staged, active, active_updated):
    if normalize_reviewed_updates(staged, updated=True) != normalize_reviewed_updates(active, updated=active_updated):
        raise AssertionError("Staged workflow would discard an active command, guard or permission")


def main_workflow_manifest():
    path = ".github/fleet-workflows.json"
    exists = subprocess.run(["git", "cat-file", "-e", f"origin/main:{path}"], cwd=ROOT, capture_output=True)
    if exists.returncode:
        # The pre-staging main revision used the canonical filenames. A
        # missing callee still fails at git show below; never skip its check.
        return {family: f"fleet-{family}.yml" for family in FAMILIES}
    manifest = json.loads(subprocess.check_output(["git", "show", f"origin/main:{path}"], cwd=ROOT))
    if set(manifest) != FAMILIES:
        raise AssertionError("Main must declare every active workflow family")
    return manifest


class FleetWorkflowTests(unittest.TestCase):
    def test_all_active_callers_keep_exact_bytes_and_main_ancestry(self):
        self.assertGreater(validate_calls(), 0)

    def test_mutable_and_short_pins_are_rejected(self):
        manifest = workflow_manifest()
        for ref in ("main", "v2", "a" * 39, "a" * 41):
            with self.subTest(ref=ref), self.assertRaises(ValueError):
                parse_reference(PREFIX + manifest["ci"] + "@" + ref, manifest)

    def test_inactive_and_unknown_workflow_names_are_rejected(self):
        manifest = workflow_manifest()
        for filename in ("fleet-ci-v999999.yml", "fleet-unreviewed.yml"):
            with self.subTest(file=filename), self.assertRaises(ValueError):
                parse_reference(PREFIX + filename + "@" + "a" * 40, manifest)

    def test_different_checked_in_bytes_are_rejected(self):
        caller, path, sha = workflow_calls()[0]
        with self.assertRaisesRegex(ValueError, "differs"):
            validate_reference(path, sha, (ROOT / path).read_bytes() + b"\n# negative control\n")

    def test_exact_bytes_outside_the_main_history_are_rejected(self):
        _caller, path, sha = workflow_calls()[0]
        # Use the real immutable blob and its parent's history. This exercises
        # Git ancestry without manufacturing commits or bypassing local hooks.
        with self.assertRaisesRegex(ValueError, "main ancestor"):
            validate_reference(path, sha, (ROOT / path).read_bytes(), main_ref=sha + "^")

    def test_manifest_rejects_missing_families_and_path_traversal(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory = root / ".github/workflows"
            directory.mkdir(parents=True)
            manifest = {family: f"fleet-{family}.yml" for family in FAMILIES}
            for filename in manifest.values():
                (directory / filename).write_text("on: workflow_call\n")
            config = root / ".github/fleet-workflows.json"
            for invalid in ({}, {**manifest, "ci": "../../outside.yml"}):
                config.write_text(json.dumps(invalid))
                with self.assertRaises(ValueError):
                    workflow_manifest(root)

    def test_active_workflows_exist(self):
        for family in FAMILIES:
            self.assertTrue(workflow_path(family).is_file())

    def test_active_workflow_drift_is_rejected(self):
        active = workflow_path("ci")
        staged = (ROOT / ".github/workflows/fleet-ci-v2.yml").read_text()
        drifted = active.read_text().replace("  contents: read", "  contents: write", 1)
        self.assertNotEqual(drifted, active.read_text())
        with self.assertRaisesRegex(AssertionError, "active command, guard or permission"):
            assert_reviewed_workflow_delta(staged, drifted, active.name.endswith("-v2.yml"))

    def test_staged_v2_only_changes_reviewed_action_dependencies(self):
        self.assertEqual(subprocess.run(
            ["git", "merge-base", "--is-ancestor", SOURCE, "origin/main"],
            cwd=ROOT, capture_output=True,
        ).returncode, 0)
        main_manifest = main_workflow_manifest()
        for family in sorted(FAMILIES):
            with self.subTest(family=family):
                path = f".github/workflows/fleet-{family}.yml"
                source = subprocess.check_output(["git", "show", f"{SOURCE}:{path}"], cwd=ROOT, text=True)
                staged = ROOT / f".github/workflows/fleet-{family}-v2.yml"
                self.assertEqual(
                    normalize_reviewed_updates(staged.read_text(), updated=True),
                    normalize_reviewed_updates(source, updated=False),
                    "Staging must retain every command, guard, permission and runner contract",
                )
                active = workflow_path(family)
                assert_reviewed_workflow_delta(staged.read_text(), active.read_text(), active.name.endswith("-v2.yml"))
                main_file = main_manifest[family]
                self.assertIn(main_file, (f"fleet-{family}.yml", f"fleet-{family}-v2.yml"))
                main_active = subprocess.check_output(
                    ["git", "show", f"origin/main:.github/workflows/{main_file}"], cwd=ROOT, text=True,
                )
                assert_reviewed_workflow_delta(staged.read_text(), main_active, main_file.endswith("-v2.yml"))


if __name__ == "__main__":
    unittest.main()

"""Stage fleet-ci-v10.yml as the active v9 callee plus the reviewed #1297 installer update.

#1297 staged taiki-e/install-action v2.87.29 in fleet-ci-v8.yml, but CI
activated v9 from v7 without it. v10 carries that update onto v9's landed
bytes; every command, permission, guard and runner expression must be
unchanged until a later PR activates it.
"""

import subprocess
import unittest

from fleet_workflows import ROOT, workflow_manifest
from test_fleet_actions_stage import UPDATES, updated

V9_LANDED = "4e355573ffe9b1bc0466d0a57fcc98677d5fc5eb"
V9 = ".github/workflows/fleet-ci-v9.yml"
V10 = ROOT / ".github/workflows/fleet-ci-v10.yml"
INSTALLER = "taiki-e/install-action"
# Install sites in v9; v9 runs no SARIF upload, so the uploader pin is unused.
INSTALL_SITES = 9


def landed_v9():
    return subprocess.check_output(["git", "show", f"{V9_LANDED}:{V9}"], cwd=ROOT)


def validate_stage(content, source):
    if content != updated(source):
        raise ValueError("Staged v10 may change only the reviewed Action pins and comments")


class FleetCiV10StageTests(unittest.TestCase):
    def test_source_is_an_immutable_main_ancestor(self):
        self.assertEqual(subprocess.run(
            ["git", "merge-base", "--is-ancestor", V9_LANDED, "origin/main"],
            cwd=ROOT, capture_output=True,
        ).returncode, 0)

    def test_v10_only_changes_the_reviewed_installer_pin(self):
        source = landed_v9()
        old, new = (f"uses: {INSTALLER}@{pin}".encode() for pin in UPDATES[INSTALLER])
        self.assertEqual(source.count(old), INSTALL_SITES)
        content = V10.read_bytes()
        validate_stage(content, source)
        self.assertEqual(content.count(new), INSTALL_SITES)
        self.assertNotIn(old, content)

    def test_intervening_commands_permissions_and_unknown_pins_are_rejected(self):
        source = landed_v9()
        content = updated(source)
        for before, after in (
            (b"  contents: read", b"  contents: write"),
            (b"cargo clippy --all-targets --all-features -- -D warnings", b"echo skipped"),
            (UPDATES[INSTALLER][1].encode(), b"0" * 40),
        ):
            with self.subTest(change=before), self.assertRaisesRegex(ValueError, "only the reviewed"):
                self.assertIn(before, content)
                validate_stage(content.replace(before, after, 1), source)

    def test_active_and_current_main_ci_callees_are_v9_bytes(self):
        # An edit to active v9 before activation must be carried into v10 first.
        manifest = workflow_manifest()
        if manifest["ci"] == V10.name:
            return  # Activated: the caller pin check binds v10 to its landed bytes.
        self.assertEqual(manifest["ci"], "fleet-ci-v9.yml")
        self.assertEqual((ROOT / V9).read_bytes(), landed_v9())
        self.assertEqual(subprocess.check_output(["git", "show", f"origin/main:{V9}"], cwd=ROOT),
                         landed_v9())


if __name__ == "__main__":
    unittest.main()

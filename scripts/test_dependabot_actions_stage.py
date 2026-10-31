"""Stage #1449's Action pins without changing any active immutable workflow."""

import json
import re
import subprocess
import unittest

from fleet_workflows import ROOT, validate_calls, workflow_manifest, workflow_text


SOURCE = "3862e7f9f162ed5c1fc651f9eba8d7ea7ef90fd3"
WORKFLOWS = {
    "benchmark": ("fleet-benchmark-v4.yml", "fleet-benchmark-v5.yml"),
    "ci": ("fleet-ci-v9.yml", "fleet-ci-v11.yml"),
    "fuzz": ("fleet-fuzz-v4.yml", "fleet-fuzz-v5.yml"),
    "gauntlet": ("fleet-gauntlet-v4.yml", "fleet-gauntlet-v5.yml"),
    "mutants-sharded": ("fleet-mutants-sharded-v5.yml", "fleet-mutants-sharded-v6.yml"),
    "mutation-scope": ("fleet-mutation-scope-v4.yml", "fleet-mutation-scope-v5.yml"),
    "osv": ("fleet-osv-v6.yml", "fleet-osv-v7.yml"),
    "wasm-candidate": ("fleet-wasm-candidate-v5.yml", "fleet-wasm-candidate-v6.yml"),
}
INSTALLER_PIN = "1e29709c0ec63babbe8f266397a465f29a825f91"
UPDATES = {
    "Swatinem/rust-cache": (
        ("f0d9c3887740aee45f6153b24b3a6b815192ec16 # v2",),
        "c8037a86dde8214a8310e9f54a8fc61ab362aa31 # v2",
    ),
    "taiki-e/install-action": (
        ("ab6895351a88b908e96a4697860d4ffea5e57dbe # v2.87.27",
         "8cca0918f8471c33515202b73f56ab27f2bed283 # v2.87.29"),
        INSTALLER_PIN + " # v2.87.31",
    ),
    "github/codeql-action/upload-sarif": (
        ("6f2ac81f5cd81fe351a197ba0b00d91d03cd2aee # v4.39.0",),
        "9b3254085494b55a0f7c4c2b3d2d52cc712ce370 # v4.39.1",
    ),
}


def original(family):
    return subprocess.check_output(
        ["git", "show", f"{SOURCE}:.github/workflows/{WORKFLOWS[family][0]}"],
        cwd=ROOT,
    )


def updated(content):
    for action, (previous, current) in UPDATES.items():
        for pin in previous:
            content = content.replace(f"uses: {action}@{pin}".encode(),
                                      f"uses: {action}@{current}".encode())
    return content


def validate_stage(content, source):
    if content != updated(source):
        raise ValueError("Stage may change only the reviewed Action pins and comments")


class DependabotActionsStageTests(unittest.TestCase):
    def test_source_is_a_reachable_main_commit(self):
        self.assertEqual(subprocess.run(
            ["git", "merge-base", "--is-ancestor", SOURCE, "origin/main"],
            cwd=ROOT, capture_output=True,
        ).returncode, 0)

    def test_every_staged_file_has_only_the_reviewed_dependency_delta(self):
        self.assertEqual(set(WORKFLOWS), {
            "benchmark", "ci", "fuzz", "gauntlet", "mutants-sharded",
            "mutation-scope", "osv", "wasm-candidate",
        })
        for family, (previous, staged) in WORKFLOWS.items():
            with self.subTest(family=family):
                self.assertNotEqual(previous, staged)
                source = original(family)
                content = (ROOT / ".github/workflows" / staged).read_bytes()
                self.assertNotEqual(content, source)
                validate_stage(content, source)

    def test_commands_permissions_guards_budgets_and_unknown_pins_are_rejected(self):
        source = original("ci")
        content = updated(source)
        for before, after in (
            (b"  contents: read", b"  contents: write"),
            (b"cargo clippy --all-targets --all-features -- -D warnings", b"echo skipped"),
            (b"CI_MEMORY=5000000000", b"CI_MEMORY=9000000000"),
            (b"persist-credentials: false", b"persist-credentials: true"),
            (b"--fail-under-lines 60", b"--fail-under-lines 0"),
            (INSTALLER_PIN.encode(), b"0" * 40),
        ):
            with self.subTest(change=before), self.assertRaisesRegex(ValueError, "only the reviewed"):
                self.assertIn(before, content)
                validate_stage(content.replace(before, after, 1), source)

    def test_active_and_current_main_definitions_are_preserved_until_activation(self):
        main_manifest = json.loads(subprocess.check_output(
            ["git", "show", "origin/main:.github/fleet-workflows.json"], cwd=ROOT,
        ))
        for manifest, on_main in ((workflow_manifest(), False), (main_manifest, True)):
            for family, (previous, staged) in WORKFLOWS.items():
                with self.subTest(family=family, on_main=on_main):
                    filename = manifest[family]
                    self.assertIn(filename, (previous, staged))
                    content = (subprocess.check_output(
                        ["git", "show", f"origin/main:.github/workflows/{filename}"], cwd=ROOT,
                    ) if on_main else workflow_text(filename).encode())
                    source = original(family)
                    self.assertEqual(content, updated(source) if filename == staged else source,
                                     "Carry intervening workflow changes into a new stage before activation")
        self.assertGreater(validate_calls(), 0)

    def test_windows_arm_receipts_bind_the_reviewed_installer(self):
        expected = "taiki-e/install-action@" + INSTALLER_PIN
        for filename in ("windows-arm-tooling.yml", "parity.yml"):
            with self.subTest(workflow=filename):
                text = (ROOT / ".github/workflows" / filename).read_text()
                self.assertIn("uses: " + expected, text)
        for filename in ("scripts/platform-probes/windows-arm.mjs", "tools/parity/host-proof.mjs"):
            with self.subTest(proof=filename):
                text = (ROOT / filename).read_text()
                self.assertEqual(re.findall(r"action: '(taiki-e/install-action@[^']+)'", text),
                                 [expected])


if __name__ == "__main__":
    unittest.main()

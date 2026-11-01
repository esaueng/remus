#!/usr/bin/env python3
"""Keep staged dependency updates and active immutable workflow calls exact."""

import hashlib
import json
import re
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from fleet_workflows import (
    FAMILIES, ROOT, PREFIX, parse_reference, validate_calls, validate_reference,
    RETIRED, workflow_calls, workflow_manifest, workflow_path, workflow_text,
)
from test_jane_helper_workflows import STAGE
from test_fleet_actions_stage import FleetActionsStageTests, revert_action_updates, staged_action_update
from test_dependabot_actions_stage import WORKFLOWS as DEPENDABOT_STAGES, previous_definition

CI_V4_LANDED = '9e5b65b9e00f004d83eb583ab3934c863ad7956d'
CI_V9_LANDED = '4e355573ffe9b1bc0466d0a57fcc98677d5fc5eb'


def historical_workflow(family, text):
    """Separate older dependency/tier proofs from the exact profile/queue proofs.

    test_jane_helper_workflows proves the complete auxiliary delta against its
    immutable source. The required three-slot queue suite proves v6 against v4,
    including every application step and guard. Keep the earlier Action-update
    regressions against their original source after those versions are retired.
    """
    if family in DEPENDABOT_STAGES and workflow_manifest()[family] == DEPENDABOT_STAGES[family][1]:
        text = previous_definition(family, text)
    if family == 'ci' and '  queue-trust:\n' in text:
        return subprocess.check_output(
            ['git', 'show', f'{CI_V4_LANDED}:.github/workflows/fleet-ci-v4.yml'],
            cwd=ROOT, text=True,
        )
    text = revert_action_updates(family, text)
    if family == 'benchmark' and 'auto-push: ${{ github.ref' in text:
        text = retained_workflow('fleet-benchmark-v3.yml')
    return (text.replace('esaueng/remus/.github/actions/fleet-guard@' + STAGE['guard'],
                         'esaueng/remus/.github/actions/fleet-guard@' + STAGE['previous_guard'])
            .replace('\"labels\":\"ci-server-jane-5gb\"', '\"labels\":\"ci-server-jane\"'))


def retained_workflow(filename):
    if filename in RETIRED:
        return workflow_text(filename)
    path = ROOT / '.github/workflows' / filename
    if path.exists():
        return path.read_text()
    return subprocess.check_output(
        ['git', 'show', f"{STAGE['source']}:.github/workflows/{filename}"],
        cwd=ROOT, text=True,
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


# The v2 callees landed here (#1179). The families in V3_FAMILIES were retired
# for v3 copies (#1183) that add only job timeouts and the cargo-fuzz pin.
V2_LANDED = "e07ae8e345e10085a717988e10fc08776c85d84a"
V3_FAMILIES = frozenset({"ci", "fuzz", "osv", "wasm-candidate", "wasm-size"})
V3_LANDED = "59d02a10f498a2b3c24512e8b2d9ba8150bbdb1e"
V3_TIMEOUT = re.compile(r"(?m)^    timeout-minutes: [0-9]+\n")


def landed_v2(family):
    return subprocess.check_output(
        ["git", "show", f"{V2_LANDED}:.github/workflows/fleet-{family}-v2.yml"], cwd=ROOT, text=True,
    )


def strip_v3_deltas(text):
    return V3_TIMEOUT.sub("", text).replace(
        "install cargo-fuzz --version 0.13.2 --locked", "install cargo-fuzz --locked")


# These callees were refreshed (#1195) with comment-only edits, plus the
# gauntlet's tier step now that its caller owns the weekly cron. Each maps to
# the file it replaced and the commit that file landed in.
REFRESHED = {
    "wasm-size": ("fleet-wasm-size-v3.yml", V3_LANDED),
    "osv": ("fleet-osv-v3.yml", V3_LANDED),
    "mutants-sharded": ("fleet-mutants-sharded-v2.yml", V2_LANDED),
    "gauntlet": ("fleet-gauntlet-v2.yml", V2_LANDED),
}
CALLER_OWNED_TIER = (
    "          SCHEDULE: ${{ github.event.schedule }}\n"
    "        run: |\n"
    "          if [ \"$SCHEDULE\" = \"47 3 * * 0\" ]; then\n"
    "            tier=abc-1k\n"
    "          elif [ -n \"$REQUESTED_TIER\" ]; then\n"
    "            tier=$REQUESTED_TIER\n"
    "          else\n"
    "            tier=smoke\n"
    "          fi\n",
    "        run: |\n"
    "          tier=${REQUESTED_TIER:-smoke}\n",
)


def replaced_callee(family):
    filename, sha = REFRESHED[family]
    return subprocess.check_output(
        ["git", "show", f"{sha}:.github/workflows/{filename}"], cwd=ROOT, text=True,
    )


def strip_refresh_deltas(text):
    """Drop comments and the replaced gauntlet tier step; nothing else may differ."""
    text = re.sub(r"(?m)^\s*#.*\n", "", text)
    text = re.sub(r"(?m)^(\s*(?:- )?uses: \S+)[ \t]+#[^\n]*$", r"\1", text)
    return text.replace(*CALLER_OWNED_TIER)


def carries_reviewed_updates(filename):
    version = re.search(r"-v([1-9][0-9]*)\.yml$", filename)
    return version is not None and int(version[1]) >= 2

# The ci v4 callee moves Fuzz Targets Compile and
# Software Rendering from the second tier into the first, so a merge group that
# runs the first tier only still compiles the fuzz targets and renders. These
# are the only reviewed edits to the landed v3 callee.
CI_V4 = ROOT / ".github/workflows/fleet-ci-v4.yml"
CI_V4_TIER_MOVE = (
    ("          # The second tier (coverage, MSRV, fuzz compile, render,\n"
     "          # deny, audit, package build) runs for every main push, merge\n"
     "          # group and dispatch, and for PRs only with the `ci:full` label.\n",
     "          # The second tier (coverage, MSRV, deny, audit, package build)\n"
     "          # runs for every main push and dispatch, for merge groups whose\n"
     "          # diff touches its inputs, and for PRs only with the `ci:full`\n"
     "          # label. Fuzz compile and render run with the first tier: they\n"
     "          # build against most kernel crates and take about a minute each.\n"),
    ("    name: Fuzz Targets Compile\n    needs: [changes, repo-policy, secrets-scan]\n"
     "    if: needs.changes.outputs.full == 'true'\n",
     "    name: Fuzz Targets Compile\n    needs: [changes, repo-policy, secrets-scan]\n"
     "    if: needs.changes.outputs.heavy == 'true'\n"),
    ("    name: Software Rendering\n    needs: [changes, repo-policy, secrets-scan]\n"
     "    if: needs.changes.outputs.full == 'true'\n",
     "    name: Software Rendering\n    needs: [changes, repo-policy, secrets-scan]\n"
     "    if: needs.changes.outputs.heavy == 'true'\n"),
    ('                elif (["coverage", "msrv", "fuzz-check", "render", "deny", "audit"] | index($job)) != null then $full == "false"\n',
     '                elif (["coverage", "msrv", "deny", "audit"] | index($job)) != null then $full == "false"\n'),
)


def revert_ci_v4_tier_move(text):
    """The active ci v4 callee with the reviewed tier move undone, i.e. v3."""
    for old, new in CI_V4_TIER_MOVE:
        text = text.replace(new, old)
    return text


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


# Exact main revision containing the reviewed guard and legacy auxiliary bytes.
THREE_SLOT_GUARD_LANDED = "05fca2fc1029be782afe41ee6ebbeae812e5e40c"
THREE_SLOT_AUXILIARIES = {
    "fleet-benchmark-v3.yml": ("fleet-benchmark-v2.yml", 2),
    "fleet-fuzz-v4.yml": ("fleet-fuzz-v3.yml", 2),
    "fleet-gauntlet-v4.yml": ("fleet-gauntlet-v3.yml", 2),
    "fleet-mutants-sharded-v4.yml": ("fleet-mutants-sharded-v3.yml", 2),
    "fleet-mutation-scope-v3.yml": ("fleet-mutation-scope-v2.yml", 1),
    "fleet-osv-v5.yml": ("fleet-osv-v4.yml", 1),
    "fleet-wasm-candidate-v4.yml": ("fleet-wasm-candidate-v3.yml", 1),
    "fleet-wasm-size-v5.yml": ("fleet-wasm-size-v4.yml", 1),
}


def main_workflow_text(filename):
    """Read a callee as origin/main holds it, or a retired one from its landed commit.

    An activation PR retires the replaced file, so after it lands main no
    longer carries it. Until then main must hold exactly the retired bytes.
    """
    path = f".github/workflows/{filename}"
    on_main = subprocess.run(["git", "cat-file", "-e", f"origin/main:{path}"], cwd=ROOT, capture_output=True)
    if filename not in RETIRED:
        return subprocess.check_output(["git", "show", f"origin/main:{path}"], cwd=ROOT, text=True)
    retired = workflow_text(filename)
    if not on_main.returncode and subprocess.check_output(
            ["git", "show", f"origin/main:{path}"], cwd=ROOT, text=True) != retired:
        raise AssertionError(f"Main differs from the retired bytes of {filename}")
    return retired


class FleetWorkflowTests(unittest.TestCase):
    def test_reviewed_update_versions_include_multi_digit_callees(self):
        for version in (2, 9, 10, 11):
            self.assertTrue(carries_reviewed_updates(f"fleet-ci-v{version}.yml"))
        for filename in ("fleet-ci.yml", "fleet-ci-v1.yml"):
            self.assertFalse(carries_reviewed_updates(filename))

    def test_three_slot_auxiliaries_only_change_guard_pin_and_profile_selector(self):
        self.assertEqual(subprocess.run(
            ["git", "merge-base", "--is-ancestor", THREE_SLOT_GUARD_LANDED, "origin/main"],
            cwd=ROOT, capture_output=True,
        ).returncode, 0)
        old_guard = "esaueng/remus/.github/actions/fleet-guard@c1fe861d46b5e932cc69ccbcff6803cd1a38804b"
        new_guard = "esaueng/remus/.github/actions/fleet-guard@" + THREE_SLOT_GUARD_LANDED
        guard = subprocess.check_output(
            ["git", "show", THREE_SLOT_GUARD_LANDED + ":.github/actions/fleet-guard/action.yml"], cwd=ROOT,
        )
        self.assertEqual(hashlib.sha256(guard).hexdigest(),
                         "599087ce5da442d89e0805db24691a07b5badd6c76f52f56d8314a9fb60079b7")
        for new, (old, guard_count) in THREE_SLOT_AUXILIARIES.items():
            with self.subTest(workflow=new):
                source = subprocess.check_output(
                    ["git", "show", THREE_SLOT_GUARD_LANDED + ":.github/workflows/" + old], cwd=ROOT, text=True,
                )
                self.assertEqual(source.count(old_guard), guard_count)
                self.assertEqual(source.count('"labels":"ci-server-jane"'), 1)
                expected = source.replace(old_guard, new_guard).replace(
                    '"labels":"ci-server-jane"', '"labels":"ci-server-jane-5gb"')
                actual = workflow_text(new)
                self.assertEqual(actual, expected,
                                 "Only the reviewed guard pin and Jane selector may change")
                self.assertEqual(actual.count(new_guard), guard_count)
                self.assertNotIn(old_guard, actual)
                self.assertNotIn('"labels":"ci-server-jane"', actual)

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
        staged = landed_v2("ci")
        drifted = active.read_text().replace("  contents: read", "  contents: write", 1)
        self.assertNotEqual(drifted, active.read_text())
        if active.name == DEPENDABOT_STAGES["ci"][1]:
            with self.assertRaisesRegex(ValueError, "complete reviewed"):
                previous_definition("ci", drifted)
            drifted = previous_definition("ci", active.read_text()).replace(
                "  contents: read", "  contents: write", 1)
        with self.assertRaisesRegex(AssertionError, "active command, guard or permission"):
            assert_reviewed_workflow_delta(strip_v3_deltas(staged), strip_v3_deltas(drifted), True)

    def test_v3_only_adds_job_timeouts_and_the_cargo_fuzz_pin(self):
        # V3 bounds every job and pins cargo-fuzz; every other byte must match
        # the landed v2 callee it replaced.
        # Refreshed families no longer carry their v3 file; read it from the
        # commit it landed in.
        for family in sorted(V3_FAMILIES):
            with self.subTest(family=family):
                text = subprocess.check_output(
                    ["git", "show", f"{V3_LANDED}:.github/workflows/fleet-{family}-v3.yml"], cwd=ROOT, text=True,
                )
                path = ROOT / f".github/workflows/fleet-{family}-v3.yml"
                if family not in REFRESHED and family != "ci":
                    self.assertEqual(retained_workflow(path.name), text)
                jobs = re.split(r"(?m)^(?=  [\w-]+:\n)", text.split("\njobs:\n", 1)[1])
                for job in filter(None, jobs):
                    self.assertRegex(job, r"(?m)^    runs-on: .*\n    timeout-minutes: [0-9]+$")
                self.assertEqual(strip_v3_deltas(text), strip_v3_deltas(landed_v2(family)),
                                 "V3 must retain every command, guard, permission and runner contract")

    def test_refreshed_callees_only_change_comments_and_the_gauntlet_tier(self):
        for family, (filename, _sha) in REFRESHED.items():
            with self.subTest(family=family):
                active = workflow_path(family)
                self.assertNotEqual(active.name, filename)
                self.assertFalse((ROOT / ".github/workflows" / filename).exists(), filename)
                text, previous = historical_workflow(family, active.read_text()), replaced_callee(family)
                self.assertNotEqual(text, previous)
                self.assertEqual(strip_refresh_deltas(text), strip_refresh_deltas(previous),
                                 "A refresh may change only comments and the reviewed gauntlet tier step")
        gauntlet = strip_refresh_deltas(replaced_callee("gauntlet"))
        self.assertNotIn("SCHEDULE", gauntlet)
        self.assertIn("tier=${REQUESTED_TIER:-smoke}", gauntlet)

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
                staged = landed_v2(family)
                self.assertEqual(
                    normalize_reviewed_updates(staged, updated=True),
                    normalize_reviewed_updates(source, updated=False),
                    "Staging must retain every command, guard, permission and runner contract",
                )
                retained = ROOT / f".github/workflows/fleet-{family}-v2.yml"
                if family not in V3_FAMILIES | REFRESHED.keys():
                    self.assertEqual(retained_workflow(retained.name), staged)
                active = workflow_path(family)
                assert_reviewed_workflow_delta(strip_refresh_deltas(strip_v3_deltas(staged)),
                                               strip_refresh_deltas(strip_v3_deltas(
                                                   revert_ci_v4_tier_move(historical_workflow(family, active.read_text())))),
                                               carries_reviewed_updates(active.name))
                main_file = main_manifest[family]
                allowed = {f"fleet-{family}{suffix}.yml" for suffix in ("", "-v2", "-v3", "-v4")}
                allowed.update({'fleet-ci-v6.yml', 'fleet-ci-v7.yml', 'fleet-ci-v9.yml'} if family == 'ci'
                               else {STAGE['workflows'][family]['staged']}
                               | staged_action_update(family))
                if family in DEPENDABOT_STAGES:
                    allowed.add(DEPENDABOT_STAGES[family][1])
                self.assertIn(main_file, allowed)
                main_active = main_workflow_text(main_file)
                if family == 'ci' and main_file == 'fleet-ci-v9.yml':
                    # The historical v2 proof still runs below; additionally bind
                    # active v9 to its separately reviewed, landed definition.
                    validate_reference('.github/workflows/' + main_file,
                                       CI_V9_LANDED, main_active.encode())
                assert_reviewed_workflow_delta(strip_refresh_deltas(strip_v3_deltas(staged)),
                                               strip_refresh_deltas(strip_v3_deltas(
                                                   revert_ci_v4_tier_move(historical_workflow(family, main_active)))),
                                               carries_reviewed_updates(main_file))

    def test_historical_stage_proof_accepts_v9_after_main_activation(self):
        # Model the manifest that selected v9. #1457 retired v9 for v11, so its
        # bytes come from the RETIRED commit and must still match CI_V9_LANDED.
        activated = {**main_workflow_manifest(), 'ci': 'fleet-ci-v9.yml'}
        with mock.patch(__name__ + '.main_workflow_manifest', return_value=activated):
            self.test_staged_v2_only_changes_reviewed_action_dependencies()

    def test_main_reads_retired_callees_from_their_landed_bytes(self):
        self.assertEqual(main_workflow_text('fleet-ci-v9.yml').encode(), subprocess.check_output(
            ['git', 'show', f'{CI_V9_LANDED}:.github/workflows/fleet-ci-v9.yml'], cwd=ROOT))
        # A file main still carries while it is being retired must match exactly.
        with mock.patch.dict(RETIRED, {'fleet-ci-v11.yml': RETIRED['fleet-ci-v9.yml']}), \
                mock.patch(__name__ + '.workflow_text', return_value='drifted\n'):
            with self.assertRaisesRegex(AssertionError, 'retired bytes'):
                main_workflow_text('fleet-ci-v11.yml')

    def test_staged_ci_v4_only_moves_fuzz_and_render_into_tier_one(self):
        expected = subprocess.check_output(
            ["git", "show", f"{V3_LANDED}:.github/workflows/fleet-ci-v3.yml"], cwd=ROOT, text=True,
        )
        for old, new in CI_V4_TIER_MOVE:
            self.assertEqual(expected.count(old), 1, old)
            expected = expected.replace(old, new)
        self.assertEqual(retained_workflow(CI_V4.name), expected,
                         "Staged ci v4 may change only the reviewed tier move")


if __name__ == "__main__":
    unittest.main()

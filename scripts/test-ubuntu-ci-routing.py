#!/usr/bin/env python3
"""Exercise Ubuntu scheduling, immutable guards, and migrated job contracts."""

import copy
import importlib.util
import re
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKFLOWS = ROOT / ".github/workflows"
SPEC = importlib.util.spec_from_file_location("direct", ROOT / "scripts/test-direct-fleet.py")
DIRECT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DIRECT)
FILES = sorted(p for p in WORKFLOWS.glob("fleet-*.yml") if p.name != "fleet-ci.yml")
GUARD_PATH = ".github/actions/fleet-guard/action.yml"
GUARD = (ROOT / GUARD_PATH).read_text()


def jobs(text):
    return dict(re.findall(r"^  ([\w-]+):\n(.*?)(?=^  [\w-]+:\n|\Z)",
                           text.split("\njobs:\n", 1)[1], re.M | re.S))


class UbuntuRoutingTests(unittest.TestCase):
    def setUp(self):
        fixture = DIRECT.DirectFleetTests()
        fixture.setUp()
        self.github = fixture.github
        self.variables = fixture.variables

    def expressions(self):
        for path in FILES:
            expression = re.search(r"^    runs-on: (?:&fleet-runner )?(\$\{\{.+\}\})$",
                                   path.read_text(), re.M)[1]
            yield path.name, expression

    def test_owner_pr_and_protected_main_use_the_selected_server(self):
        for filename, expression in self.expressions():
            for event in ("pull_request", "push", "workflow_dispatch", "schedule"):
                github = copy.deepcopy(self.github)
                github["event_name"] = event
                if event != "pull_request":
                    github["ref"] = "refs/heads/main"
                with self.subTest(file=filename, event=event):
                    self.assertEqual(DIRECT.evaluate(expression, github, self.variables),
                                     {"group": "ci-trusted-main", "labels": "ci-server-jane"})

    def test_forks_other_actors_and_unprotected_refs_remain_hosted(self):
        cases = []
        for field, value in (("actor", "outsider"), ("triggering_actor", "outsider"),
                             ("actor_id", "1"), ("repository_id", "1"),
                             ("event_name", "pull_request_target"),
                             ("event_name", "merge_group")):
            github = copy.deepcopy(self.github)
            github[field] = value
            cases.append(github)
        for section in ("head", "base"):
            github = copy.deepcopy(self.github)
            github["event"]["pull_request"][section]["repo"]["fork"] = True
            cases.append(github)
        for event in ("push", "workflow_dispatch", "schedule"):
            github = copy.deepcopy(self.github)
            github.update(event_name=event, ref="refs/heads/main", ref_protected=False)
            cases.append(github)
            github = copy.deepcopy(github)
            github.update(ref="refs/heads/feature", ref_protected=True)
            cases.append(github)
        for filename, expression in self.expressions():
            for github in cases:
                with self.subTest(file=filename, event=github["event_name"]):
                    self.assertEqual(DIRECT.evaluate(expression, github, self.variables), "ubuntu-latest")

    def test_missing_configuration_keeps_hosted_fallback(self):
        for filename, expression in self.expressions():
            for variables in ({}, {"CI_FLEET_ENABLED": "false"},
                              {"CI_FLEET_ENABLED": "true", "CI_FLEET_TARGET": "unknown"}):
                with self.subTest(file=filename, variables=variables):
                    self.assertEqual(DIRECT.evaluate(expression, self.github, variables), "ubuntu-latest")

    def test_validation_ref_authorizes_only_one_trusted_pr(self):
        variables = {"CI_FLEET_ENABLED": "false", "CI_FLEET_TARGET": "ci-server-jane",
                     "CI_FLEET_VALIDATION_REF": self.github["ref"]}
        for filename, expression in self.expressions():
            with self.subTest(file=filename):
                self.assertIsInstance(DIRECT.evaluate(expression, self.github, variables), dict)
                wrong_actor = dict(self.github, actor="outsider")
                self.assertEqual(DIRECT.evaluate(expression, wrong_actor, variables), "ubuntu-latest")
                other_pr = copy.deepcopy(self.github)
                other_pr["ref"] = "refs/pull/8/merge"
                other_pr["event"]["pull_request"]["number"] = 8
                self.assertEqual(DIRECT.evaluate(expression, other_pr, variables), "ubuntu-latest")

    def test_every_job_uses_the_immutable_guard_before_checkout(self):
        for path in FILES:
            for name, block in jobs(path.read_text()).items():
                with self.subTest(file=path.name, job=name):
                    reference = re.search(r"uses: esaueng/remus/\.github/actions/fleet-guard@([a-f0-9]{40})", block)
                    self.assertIsNotNone(reference)
                    self.assertEqual(subprocess.check_output(
                        ["git", "show", f"{reference[1]}:{GUARD_PATH}"], cwd=ROOT, text=True), GUARD)
                    if "actions/checkout@" in block:
                        self.assertLess(block.index("fleet-guard@"), block.index("actions/checkout@"))
                        self.assertIn("persist-credentials: false", block)
                    if re.search(r"run: (?:\|\n.*)?cargo ", block, re.S):
                        self.assertNotIn("contents: write", block)

    def test_callers_pin_the_checked_in_workflow_content(self):
        for path in WORKFLOWS.glob("*.yml"):
            for name, sha in re.findall(r"uses: esaueng/remus/(.github/workflows/fleet-[\w-]+.yml)@([a-f0-9]{40})", path.read_text()):
                with self.subTest(caller=path.name, workflow=name):
                    self.assertEqual(subprocess.check_output(
                        ["git", "show", f"{sha}:{name}"], cwd=ROOT, text=True), (ROOT / name).read_text())

    def test_guard_retains_identity_and_isolation_checks(self):
        original = (WORKFLOWS / "fleet-ci.yml").read_text()
        for anchor in ("fleet-source", "fleet-isolation"):
            block = re.search(r"      - &" + anchor + r"\n(.*?)(?=^      - )", original, re.M | re.S)[1]
            script = block.split("        run: |\n", 1)[1]
            normalized = "\n".join(line[2:] if line.startswith("  ") else line for line in script.splitlines())
            self.assertIn(normalized, GUARD)

    def test_osv_keeps_base_comparison_and_blocking_reports(self):
        text = (WORKFLOWS / "fleet-osv.yml").read_text()
        self.assertIn("github.event.pull_request.base.sha", text)
        self.assertIn("--old=old-results.json", text)
        self.assertEqual(text.count("--fail-on-vuln=true"), 2)
        self.assertIn("github/codeql-action/upload-sarif@", text)
        for name in ("Report new PR vulnerabilities", "Report main vulnerabilities"):
            block = text.split("      - name: " + name, 1)[1].split("      - name:", 1)[0]
            self.assertNotIn("continue-on-error", block)

    def test_scanners_start_only_after_the_empty_docker_guard(self):
        core = (WORKFLOWS / "fleet-ci.yml").read_text()
        self.assertNotIn("EmbarkStudios/cargo-deny-action", core)
        self.assertIn("cargo-deny@0.20.2", core)
        self.assertIn("cargo deny --log-level warn --manifest-path ./Cargo.toml --all-features check", core)
        osv = (WORKFLOWS / "fleet-osv.yml").read_text()
        self.assertNotIn("uses: google/osv-scanner-action", osv)
        self.assertIn("ghcr.io/google/osv-scanner-action@sha256:", osv)
        self.assertLess(osv.index("fleet-guard@"), osv.index("docker run --rm"))

    def test_rust_caches_restore_after_isolation_and_only_main_saves(self):
        for path in [WORKFLOWS / "fleet-ci.yml", *FILES]:
            for name, block in jobs(path.read_text()).items():
                if "Swatinem/rust-cache@" not in block:
                    continue
                cache = block.split("      - uses: Swatinem/rust-cache@", 1)[1].split("      - ", 1)[0]
                with self.subTest(file=path.name, job=name):
                    self.assertNotIn("runner.environment != 'self-hosted'", cache)
                    self.assertIn("save-if: ${{ github.ref == 'refs/heads/main' }}", cache)
                    if name != "platform-test":
                        guard = "*fleet-isolation" if path.name == "fleet-ci.yml" else "fleet-guard@"
                        self.assertLess(block.index(guard), block.index("Swatinem/rust-cache@"))

    def test_macos_stays_required_and_publish_credentials_stay_separate(self):
        fleet = jobs((WORKFLOWS / "fleet-ci.yml").read_text())
        self.assertIn("os: [macos-latest]", fleet["platform-test"])
        self.assertIn("platform-test", fleet["ci-pass"])
        for name, run in (("benchmark", "bench"), ("gauntlet", "run")):
            blocks = jobs((WORKFLOWS / f"fleet-{name}.yml").read_text())
            self.assertIn("contents: read", blocks[run])
            self.assertNotIn("contents: write", blocks[run])
            self.assertIn("contents: write", blocks["publish"])

    def test_main_probe_has_no_hosted_bootstrap(self):
        text = (WORKFLOWS / "trusted-vps.yml").read_text()
        self.assertNotIn("select-runner.yml", text)
        self.assertNotIn("ubuntu-latest", text)
        block = jobs(text)["policy"]
        condition = block.split("    if: >-\n", 1)[1].split("    runs-on:", 1)[0]
        condition = " ".join(condition.split()).replace("&&", " and ").replace("||", " or ")
        github = dict(self.github, ref="refs/heads/main", event_name="workflow_dispatch")
        variables = dict(self.variables, REMUS_TRUSTED_VPS_ENABLED="true")
        for runner, enabled, protected, target, expected in (
            ("auto", "true", True, "ci-server-jane", True),
            ("auto", "false", True, "ci-server-jane", False),
            ("ci-server-jane-1", "false", True, "ci-server-jane", True),
            ("auto", "true", False, "ci-server-jane", False),
            ("unknown", "false", True, "ci-server-jane", False),
            ("auto", "true", True, "unknown", False),
        ):
            github["ref_protected"] = protected
            variables.update(CI_FLEET_ENABLED=enabled, CI_FLEET_TARGET=target)
            result = eval(condition, {"__builtins__": {}}, {
                "github": DIRECT.Context(github), "vars": DIRECT.Context(variables),
                "inputs": DIRECT.Context({"runner": runner}),
                "fromJSON": DIRECT.json.loads, "contains": lambda values, item: item in values,
            })
            self.assertEqual(bool(result), expected)


if __name__ == "__main__":
    unittest.main()

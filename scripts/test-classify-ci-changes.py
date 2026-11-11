#!/usr/bin/env python3
"""Regression tests for the CI change classifier."""

from __future__ import annotations

import importlib.util
import os
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


sys.dont_write_bytecode = True
MODULE_PATH = Path(__file__).with_name("classify-ci-changes.py")
SPEC = importlib.util.spec_from_file_location("classify_ci_changes", MODULE_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot load {MODULE_PATH}")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


ROOT = MODULE_PATH.resolve().parents[1]
CHECKS = ROOT / ".github/workflows/ci-checks.yml"
# The engine crates below the wasm bindings. A PR touching any of them must
# rebuild and smoke-test the packaged kernel (PR #618 broke main's WASM
# Build & Validate from crates/algo, crates/math and crates/operations alone).
KERNEL_CRATES = (
    "math", "geometry", "topology", "algo", "blend", "check", "heal", "offset",
    "operations", "io", "wasm", "wasm-io",
)
# PR #618's complete diff (2a735434..8539b266): no crates/wasm path, yet it
# broke scripts/test-wasm-smoke.mjs on main.
PR_618_PATHS = [
    ".claude/skills/solid-verification/SKILL.md",
    "crates/algo/src/builder/face_splitter/mod.rs",
    "crates/algo/src/pave_filler/phase_ff.rs",
    "crates/math/src/analytic_intersection.rs",
    "crates/operations/src/tessellate/nonplanar.rs",
    "crates/operations/tests/prop_boolean_invariants.rs",
    "crates/operations/tests/regress_b39_toruscone_composite_pierce.rs",
    "docs/kernel-maturity/b39-bisect-2026-09.md",
    "docs/kernel-maturity/roadmap.md",
]


def fixture_git():
    """Keep shared hooks and use the checkout's identity for local fixtures."""
    command = ["git"]
    for key, fallback in (("user.name", "ci"), ("user.email", "ci@example.invalid")):
        value = subprocess.run(["git", "config", "--get", key], cwd=ROOT,
                               text=True, capture_output=True, check=False)
        configured = value.stdout.strip() if value.returncode == 0 else ""
        command += ["-c", key + "=" + (configured or fallback)]
    return command + ["-c", "commit.gpgsign=false", "-c", "init.defaultBranch=main"]


class FixtureGitTests(unittest.TestCase):
    def test_configured_checkout_identity_is_used_without_overriding_hooks(self):
        def configured(command, **kwargs):
            self.assertEqual(kwargs["cwd"], ROOT)
            return subprocess.CompletedProcess(command, 0, stdout={
                "user.name": "public-fixture", "user.email": "fixture@example.invalid",
            }[command[-1]] + "\n")

        with mock.patch("subprocess.run", side_effect=configured):
            command = fixture_git()
        self.assertIn("user.name=public-fixture", command)
        self.assertIn("user.email=fixture@example.invalid", command)
        self.assertNotIn("core.hooksPath", " ".join(command))

    def test_unconfigured_ci_runner_retains_the_existing_synthetic_fixture_identity(self):
        with mock.patch("subprocess.run", return_value=subprocess.CompletedProcess([], 1, stdout="")):
            command = fixture_git()
        self.assertIn("user.name=ci", command)
        self.assertIn("user.email=ci@example.invalid", command)


def queue_reruns_pr_tier(value: bool):
    """Classify with the merge-queue switch set either way, whatever ships."""
    return mock.patch.object(MODULE, "QUEUE_RERUNS_PR_TIER", value)


def fleet_jobs() -> dict[str, str]:
    text = FLEET_CI.read_text()
    return dict(re.findall(r"^  ([\w-]+):\n(.*?)(?=^  [\w-]+:\n|\Z)",
                           text.split("\njobs:\n", 1)[1], re.M | re.S))


class ClassifyCiChangesTests(unittest.TestCase):
    def classify(self, paths, event="pull_request", **kwargs):
        return MODULE.classify(paths, event, **kwargs)

    def assertSelection(self, result, mode, **expected):
        self.assertEqual(result["mode"], mode)
        for flag in MODULE.FLAGS:
            self.assertEqual(result[flag], expected.get(flag, False), flag)

    def test_source_change_on_a_pr_runs_the_pr_tier(self) -> None:
        self.assertSelection(self.classify(["crates/math/src/lib.rs"]), "pr",
                             heavy=True, docs=True, wasm=True)

    def test_every_kernel_crate_selects_the_package_build(self) -> None:
        for crate in KERNEL_CRATES:
            with self.subTest(crate=crate):
                result = self.classify([f"crates/{crate}/src/lib.rs"])
                self.assertTrue(result["heavy"])
                self.assertTrue(result["wasm"])
        self.assertTrue(self.classify(PR_618_PATHS)["wasm"])

    def test_wasm_affecting_paths_select_the_package_build(self) -> None:
        for path in ("xtask/src/wasm.rs", "tools/vs-bench/workflows/w9-preflight.mjs",
                     "scripts/test-wasm-smoke.mjs", "scripts/test-w9-preflight.sh", "Cargo.lock",
                     "rust-toolchain.toml", ".cargo/config.toml", ".cargo/config", "LICENSE-APACHE",
                     "NOTICE", ".github/workflows/ci.yml", ".github/workflows/ci-checks.yml",
                     ".github/workflows/publish.yml", ".github/workflows/wasm-size-report.yml"):
            with self.subTest(path=path):
                for event in ("pull_request", "push"):
                    result = self.classify([path, "README.md"], event)
                    self.assertTrue(result["heavy"])
                    self.assertTrue(result["wasm"])

    def test_dependency_and_toolchain_inputs_select_their_checks(self) -> None:
        for path in ("Cargo.lock", "Cargo.toml", "crates/math/Cargo.toml", "deny.toml", "clippy.toml",
                     "rust-toolchain.toml", ".cargo/config.toml", ".config/nextest.toml",
                     ".github/workflows/ci-checks.yml", ".github/workflows/gauntlet.yml"):
            for event in ("pull_request", "push"):
                with self.subTest(path=path, event=event):
                    result = self.classify([path], event)
                    self.assertTrue(result["heavy"])
                    self.assertTrue(result["full"])
        self.assertFalse(self.classify(["crates/math/src/lib.rs"])["full"])
        self.assertFalse(self.classify(["crates/math/src/lib.rs"], "push")["full"])

    def test_main_push_runs_the_selected_tiers(self) -> None:
        self.assertSelection(self.classify(["crates/algo/src/gfa.rs"], "push"), "main",
                             heavy=True, docs=True, wasm=True)

    def test_label_dispatch_and_unknown_diffs_run_everything(self) -> None:
        everything = dict(heavy=True, docs=True, full=True, wasm=True)
        self.assertSelection(self.classify(["README.md"], force_full=True), "full", **everything)
        self.assertSelection(self.classify(["README.md"], "workflow_dispatch"), "full", **everything)
        for event in ("pull_request", "push", "merge_group", "workflow_dispatch"):
            with self.subTest(event=event):
                self.assertSelection(self.classify([], event), "full", **everything)

    def test_coverage_runs_only_on_request_with_the_heavy_tier(self) -> None:
        self.assertTrue(self.classify(["crates/math/src/lib.rs"], coverage=True)["coverage"])
        self.assertTrue(self.classify(["README.md"], "workflow_dispatch", coverage=True)["coverage"])
        self.assertFalse(self.classify(["crates/math/src/lib.rs"])["coverage"])
        self.assertFalse(self.classify(["README.md"], coverage=True)["coverage"])
        self.assertFalse(self.classify(["crates/math/src/lib.rs"], "merge_group", coverage=True)["coverage"])

    def test_merge_group_runs_the_light_gates_only(self) -> None:
        for paths in (["crates/math/src/lib.rs"], PR_618_PATHS, ["Cargo.lock"],
                      [".github/workflows/ci-checks.yml"], ["scripts/ci_native.py"],
                      ["crates/wasm/pkg/remus_wasm_bg.wasm", "crates/algo/src/gfa.rs"]):
            with self.subTest(paths=paths):
                self.assertSelection(self.classify(paths, "merge_group"), "queue", docs=True)

    def test_package_refresh_leaves_validation_to_the_integrity_job(self) -> None:
        refresh = ["crates/wasm-io/pkg/package.json", "crates/wasm-io/pkg/remus_wasm_io_bg.wasm",
                   "crates/wasm/pkg/package.json", "crates/wasm/pkg/remus_wasm_bg.wasm"]
        for event in ("pull_request", "push", "merge_group"):
            with self.subTest(event=event):
                self.assertSelection(self.classify(refresh, event), "package")

    def test_package_with_any_other_path_keeps_the_suite(self) -> None:
        # The package mode trusts the base commit's checker; a diff that also
        # touches the checker, its workflow or any source is not a refresh.
        for other in ("scripts/check-wasm-package-integrity.py", ".github/workflows/wasm-version.yml",
                      "xtask/src/wasm.rs", "crates/math/src/lib.rs", "Cargo.lock",
                      "docs/kernel-maturity/roadmap.md"):
            with self.subTest(other=other):
                result = self.classify(["crates/wasm/pkg/remus_wasm_bg.wasm", other])
                self.assertTrue(result["heavy"])
                self.assertTrue(result["wasm"])

    def test_documentation_and_metadata_stay_lightweight(self) -> None:
        self.assertSelection(self.classify(["docs/kernel-maturity/roadmap.md", "README.md"]), "docs", docs=True)
        self.assertSelection(self.classify([".github/dependabot.yml"]), "ci-only")
        self.assertSelection(self.classify([".claude/skills/testing/SKILL.md"]), "ci-only")
        self.assertSelection(self.classify([".github/pull_request_template.md", "CHANGELOG.md"]),
                             "docs", docs=True)
        for event in ("push", "merge_group"):
            self.assertSelection(self.classify(["book/src/intro.md"], event), "docs", docs=True)

    def test_unknown_paths_are_not_assumed_to_be_light(self) -> None:
        for path in ("scripts/classify-ci-changes.py", "some/new/file.bin", "package.json",
                     "crates/io/tests/data/readme.md", "crates/math/README.md"):
            with self.subTest(path=path):
                self.assertTrue(self.classify([path])["heavy"])

    def test_checks_workflow_consumes_the_classifier_outputs(self) -> None:
        text = CHECKS.read_text(encoding="utf-8").replace("\r\n", "\n")
        self.assertIn('python3 scripts/classify-ci-changes.py \\\n            --base "$base" \\\n'
                      '            --head "$GITHUB_SHA" $full_flag | tee -a "$GITHUB_OUTPUT"', text)
        for flag in MODULE.FLAGS:
            self.assertIn(f"{flag}: ${{{{ steps.classify.outputs.{flag} }}}}", text)
        self.assertIn("cargo xtask wasm-build", text)

    def run_cli(self, files: dict[str, str], *args: str, event: str = "pull_request",
                removed: tuple[str, ...] = ()) -> dict[str, str]:
        """Classify a real two-commit diff through the CLI the changes job runs."""
        with tempfile.TemporaryDirectory(prefix="remus-classify-") as tmp:
            git = fixture_git()
            def run(*args: str) -> None:
                subprocess.run([*git, *args], cwd=tmp, check=True, capture_output=True)
            run("init", "-q")
            (Path(tmp) / "seed.txt").write_text("seed\n")
            run("add", "-A")
            run("commit", "-q", "-m", "base")
            for relative, content in files.items():
                path = Path(tmp) / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
            for relative in removed:
                (Path(tmp) / relative).unlink()
            run("add", "-A")
            run("commit", "-q", "-m", "head")
            result = subprocess.run(
                [sys.executable, str(MODULE_PATH), "--base", "HEAD~1", "--head", "HEAD", *args],
                cwd=tmp, check=True, capture_output=True, text=True,
                env={**os.environ, "GITHUB_EVENT_NAME": event},
            )
        return dict(line.split("=", 1) for line in result.stdout.splitlines())

    def test_cli_selects_wasm_for_an_algo_only_diff(self) -> None:
        outputs = self.run_cli({"crates/algo/src/pave_filler/phase_ff.rs": "// change\n"})
        self.assertEqual(outputs["heavy"], "true")
        self.assertEqual(outputs["wasm"], "true")
        self.assertEqual(outputs["full"], "false")
        self.assertEqual(outputs["mode"], "pr")
        self.assertEqual(outputs["base"], "HEAD~1")
        self.assertEqual(outputs["changed_count"], "1")

    def test_cli_reads_the_event_and_the_full_flag(self) -> None:
        queued = self.run_cli({"crates/algo/src/gfa.rs": "// change\n"}, event="merge_group")
        self.assertEqual((queued["heavy"], queued["docs"], queued["mode"]), ("false", "true", "queue"))
        labelled = self.run_cli({"docs/kernel-maturity/roadmap.md": "# change\n"}, "--full")
        self.assertEqual((labelled["heavy"], labelled["full"], labelled["mode"]), ("true", "true", "full"))

    def test_cli_classifies_both_sides_of_a_rename(self) -> None:
        # Git reports a rename by its destination alone unless told otherwise.
        outputs = self.run_cli({"docs/seed.txt": "seed\n"}, removed=("seed.txt",))
        self.assertEqual(outputs["changed_count"], "2")
        self.assertEqual(outputs["heavy"], "true")

    def test_cli_runs_no_heavy_tier_for_a_package_refresh(self) -> None:
        outputs = self.run_cli({"crates/wasm/pkg/package.json": "{}\n"}, event="push")
        self.assertEqual((outputs["heavy"], outputs["wasm"], outputs["mode"]), ("false", "false", "package"))

    def test_integrity_job_rebuilds_and_compares_committed_packages(self) -> None:
        # The package mode is safe only while this required job rebuilds both
        # packages from source and compares every committed file, on PRs and
        # merge groups alike.
        text = (ROOT / ".github/workflows/wasm-version.yml").read_text()
        triggers = text.split("\njobs:\n", 1)[0]
        self.assertRegex(triggers, r"(?m)^  pull_request:$")
        self.assertRegex(triggers, r"(?m)^  merge_group:$")
        self.assertNotIn("paths", triggers)
        jobs = text.split("\njobs:\n", 1)[1]
        integrity, version = re.split(r"(?m)^  version:\n", jobs, maxsplit=1)
        self.assertIn("git diff --quiet \"$BASE_SHA\" HEAD -- crates/wasm/pkg crates/wasm-io/pkg",
                      integrity)
        rebuild = integrity.split("name: Rebuild both packages and compare every committed file", 1)[1]
        self.assertIn("if: steps.packages.outputs.changed == 'true'", rebuild)
        self.assertIn("rm -rf -- crates/wasm/pkg crates/wasm-io/pkg", rebuild)
        self.assertIn("cargo xtask wasm-build", rebuild)
        self.assertIn("python3 scripts/check-wasm-package-integrity.py --revision HEAD", rebuild)
        # CI's WASM Size Gate skips a package-only PR, so this job applies it,
        # with the base commit's scripts, to the modules it just verified.
        gate = rebuild.split("name: Enforce the kernel size policy and growth review", 1)[1]
        self.assertIn("if: steps.packages.outputs.changed == 'true' && "
                      "github.event_name == 'pull_request'", gate)
        self.assertIn('git worktree add --detach "$base" "$BASE_SHA"', gate)
        self.assertIn('node "$base/scripts/wasm-size-gate.mjs" --github-pr-body --base-dir "$base"',
                      gate)
        self.assertIn('exit "$status"', gate)
        self.assertIn("name: Changed kernel requires a new version", version)
        self.assertIn("needs: integrity", version)
        self.assertIn('if [[ "$INTEGRITY_RESULT" != "success" ]]; then', version)


if __name__ == "__main__":
    unittest.main()

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
from fleet_workflows import workflow_path


sys.dont_write_bytecode = True
MODULE_PATH = Path(__file__).with_name("classify-ci-changes.py")
SPEC = importlib.util.spec_from_file_location("classify_ci_changes", MODULE_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot load {MODULE_PATH}")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


ROOT = MODULE_PATH.resolve().parents[1]
FLEET_CI = workflow_path("ci")
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


def queue_reruns_pr_tier(value: bool):
    """Classify with the merge-queue switch set either way, whatever ships."""
    return mock.patch.object(MODULE, "QUEUE_RERUNS_PR_TIER", value)


def fleet_jobs() -> dict[str, str]:
    text = FLEET_CI.read_text()
    return dict(re.findall(r"^  ([\w-]+):\n(.*?)(?=^  [\w-]+:\n|\Z)",
                           text.split("\njobs:\n", 1)[1], re.M | re.S))


class ClassifyCiChangesTests(unittest.TestCase):
    def test_source_change_on_a_pr_runs_the_pr_tier(self) -> None:
        result = MODULE.classify_paths(["crates/math/src/lib.rs"])
        self.assertTrue(result.heavy)
        self.assertTrue(result.docs)
        self.assertFalse(result.full)
        self.assertTrue(result.wasm)
        self.assertEqual(result.mode, "pr")

    def test_distribution_licenses_always_select_a_tested_package(self) -> None:
        for path in ("LICENSE-APACHE", "NOTICE"):
            for paths in ([path], [path, "README.md"], [path, "crates/wasm/pkg/package.json"]):
                for force_full in (False, True):
                    with self.subTest(paths=paths, full=force_full):
                        result = MODULE.classify_paths(paths, force_full=force_full)
                        self.assertTrue(result.heavy)
                        self.assertTrue(result.wasm)

    def test_source_change_with_full_runs_everything(self) -> None:
        result = MODULE.classify_paths(["crates/math/src/lib.rs"], force_full=True)
        self.assertTrue(result.heavy)
        self.assertTrue(result.full)
        self.assertTrue(result.wasm)
        self.assertEqual(result.mode, "full")

    def test_wasm_affecting_paths_select_the_package_build(self) -> None:
        for path in (
            "crates/wasm/src/kernel.rs",
            "crates/wasm-io/Cargo.toml",
            "xtask/src/wasm.rs",
            "tools/vs-bench/workflows/w9-preflight.mjs",
            "scripts/test-wasm-smoke.mjs",
            "scripts/test-w9-preflight.sh",
            "Cargo.lock",
            "rust-toolchain.toml",
        ):
            with self.subTest(path=path):
                result = MODULE.classify_paths([path])
                self.assertTrue(result.heavy)
                self.assertTrue(result.wasm)
                self.assertFalse(result.full)

    def test_algo_only_pr_selects_the_package_build(self) -> None:
        result = MODULE.classify_paths(["crates/algo/src/pave_filler/phase_ff.rs"])
        self.assertTrue(result.heavy)
        self.assertTrue(result.wasm)
        self.assertFalse(result.full)
        self.assertEqual(result.mode, "pr")

    def test_every_kernel_crate_selects_the_package_build(self) -> None:
        found = {path.parent.name for path in (ROOT / "crates").glob("*/Cargo.toml")}
        # A renamed or removed engine crate must fail here, not silently
        # shrink the set this test walks.
        self.assertLessEqual(set(KERNEL_CRATES), found)
        for crate in sorted(found):
            for path in (f"crates/{crate}/src/lib.rs", f"crates/{crate}/Cargo.toml",
                         f"crates/{crate}/tests/case.rs"):
                with self.subTest(path=path):
                    result = MODULE.classify_paths([path])
                    self.assertTrue(result.heavy)
                    self.assertTrue(result.wasm)
                    self.assertFalse(result.full)

    def test_pr_618_diff_selects_the_package_build(self) -> None:
        result = MODULE.classify_paths(PR_618_PATHS)
        self.assertTrue(result.heavy)
        self.assertTrue(result.wasm)
        self.assertFalse(result.full)
        self.assertEqual(result.mode, "pr")

    def test_docs_and_ci_only_prs_skip_the_package_build(self) -> None:
        for paths in (
            ["docs/kernel-maturity/roadmap.md"],
            ["book/src/guide.md", "README.md"],
            [".github/dependabot.yml"],
            [".github/CODEOWNERS", "docs/architecture.md"],
            [".claude/skills/roadmap/SKILL.md", "CLAUDE.md"],
        ):
            with self.subTest(paths=paths):
                result = MODULE.classify_paths(paths)
                self.assertFalse(result.heavy)
                self.assertFalse(result.wasm)
                self.assertFalse(result.full)

    def test_cargo_config_selects_the_package_build(self) -> None:
        # It defines the `cargo xtask` alias the package build runs.
        self.assertTrue(MODULE.classify_paths([".cargo/config.toml"]).wasm)
        self.assertFalse(MODULE.classify_paths([".cargo/mutants.toml"]).wasm)

    def test_fleet_ci_runs_the_wasm_job_from_the_classifier_output(self) -> None:
        jobs = fleet_jobs()
        wasm = jobs["wasm"]
        self.assertIn("name: WASM Build & Validate", wasm)
        self.assertIn('needs: [queue-trust, changes, repo-policy, secrets-scan]', wasm)
        self.assertIn("if: ${{ !cancelled() && needs.changes.result == 'success' && "
                      "needs.repo-policy.result == 'success' && needs.secrets-scan.result == 'success' && "
                      "(needs.changes.outputs.wasm == 'true') }}", wasm)
        self.assertIn("cargo xtask wasm-build", wasm)
        changes = jobs["changes"]
        self.assertIn("wasm: ${{ steps.classify.outputs.wasm }}", changes)
        self.assertRegex(changes, r"python3 scripts/classify-ci-changes\.py \\\n\s+--base \"\$base\" \\\n"
                                  r"\s+--head \"\$GITHUB_SHA\" \$full_flag \| tee -a \"\$GITHUB_OUTPUT\"")

    def run_cli(self, files: dict[str, str], *args: str,
                event: str = "pull_request") -> dict[str, str]:
        """Classify a real two-commit diff through the CLI the changes job runs."""
        with tempfile.TemporaryDirectory(prefix="remus-classify-") as tmp:
            git = ["git", "-c", "user.name=ci", "-c", "user.email=ci@example.invalid",
                   "-c", "commit.gpgsign=false", "-c", "init.defaultBranch=main"]
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
        self.assertEqual(outputs["changed_count"], "1")

    def test_cli_skips_wasm_for_a_docs_only_diff(self) -> None:
        outputs = self.run_cli({"docs/kernel-maturity/roadmap.md": "# change\n"})
        self.assertEqual(outputs["heavy"], "false")
        self.assertEqual(outputs["wasm"], "false")
        self.assertEqual(outputs["mode"], "docs")

    def test_package_refresh_leaves_validation_to_the_integrity_job(self) -> None:
        # The refresh PR's own diff (#1248): both packages, nothing else.
        refresh = ["crates/wasm-io/pkg/package.json", "crates/wasm-io/pkg/remus_wasm_io_bg.wasm",
                   "crates/wasm/pkg/package.json", "crates/wasm/pkg/remus_wasm.d.ts",
                   "crates/wasm/pkg/remus_wasm_bg.wasm", "crates/wasm/pkg/remus_wasm_node.cjs"]
        for force_full in (False, True):
            for merge_group in (False, True):
                with self.subTest(force_full=force_full, merge_group=merge_group):
                    result = MODULE.classify_paths(
                        refresh, force_full=force_full, merge_group=merge_group
                    )
                    self.assertFalse(result.heavy)
                    self.assertFalse(result.docs)
                    self.assertFalse(result.full)
                    self.assertFalse(result.wasm)
                    self.assertEqual(result.mode, "package")

    def test_package_with_any_other_path_keeps_the_suite(self) -> None:
        # The light mode trusts the base commit's checker; a diff that also
        # touches the checker, its workflow or any source is not a refresh.
        for other in ("scripts/check-wasm-package-integrity.py",
                      ".github/workflows/wasm-version.yml", "xtask/src/wasm.rs",
                      "crates/math/src/lib.rs", "Cargo.lock"):
            with self.subTest(other=other):
                result = MODULE.classify_paths(["crates/wasm/pkg/remus_wasm_bg.wasm", other])
                self.assertTrue(result.heavy)
                self.assertTrue(result.wasm)
                self.assertNotEqual(result.mode, "package")
        result = MODULE.classify_paths(
            ["crates/wasm/pkg/remus_wasm_bg.wasm", "docs/kernel-maturity/roadmap.md"]
        )
        self.assertTrue(result.heavy)

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

    def test_package_refresh_with_source_change_is_heavy(self) -> None:
        result = MODULE.classify_paths(
            ["crates/wasm/pkg/remus_wasm_bg.wasm", "crates/wasm/src/kernel.rs"]
        )
        self.assertTrue(result.heavy)
        self.assertTrue(result.wasm)
        self.assertEqual(result.mode, "pr")

    def test_agent_instructions_stay_lightweight(self) -> None:
        result = MODULE.classify_paths([".claude/skills/roadmap/SKILL.md", "CLAUDE.md"])
        self.assertFalse(result.heavy)
        self.assertTrue(result.docs)
        self.assertEqual(result.mode, "docs")
        result = MODULE.classify_paths([".claude/settings.json"])
        self.assertFalse(result.heavy)
        self.assertFalse(result.docs)
        self.assertEqual(result.mode, "ci-only")

    def test_unknown_path_fails_closed(self) -> None:
        result = MODULE.classify_paths(["deny.toml"])
        self.assertTrue(result.heavy)
        self.assertEqual(result.mode, "pr")
        self.assertEqual(MODULE.classify_paths(["deny.toml"], force_full=True).mode, "full")

    def test_empty_diff_fails_closed(self) -> None:
        result = MODULE.classify_paths([])
        self.assertTrue(result.heavy)
        self.assertTrue(result.full)
        self.assertTrue(result.wasm)
        self.assertEqual(result.mode, "full")

    def test_docs_only_builds_docs_without_heavy_jobs(self) -> None:
        result = MODULE.classify_paths(["book/src/guide.md", "README.md"])
        self.assertFalse(result.heavy)
        self.assertTrue(result.docs)
        self.assertFalse(result.wasm)
        self.assertEqual(result.mode, "docs")

    def test_workflow_change_runs_everything(self) -> None:
        result = MODULE.classify_paths([".github/workflows/ci.yml"])
        self.assertTrue(result.heavy)
        self.assertTrue(result.docs)
        self.assertEqual(result.mode, "pr")

    def test_github_metadata_only_skips_docs_and_heavy_jobs(self) -> None:
        result = MODULE.classify_paths([".github/dependabot.yml"])
        self.assertFalse(result.heavy)
        self.assertFalse(result.docs)
        self.assertFalse(result.wasm)
        self.assertEqual(result.mode, "ci-only")

    def test_docs_and_github_metadata_changes_stay_lightweight(self) -> None:
        result = MODULE.classify_paths(
            [".github/CODEOWNERS", "docs/architecture.md"]
        )
        self.assertFalse(result.heavy)
        self.assertTrue(result.docs)
        self.assertEqual(result.mode, "docs")

    def test_scripts_are_not_assumed_to_be_ci_only(self) -> None:
        result = MODULE.classify_paths(["scripts/classify-ci-changes.py"])
        self.assertTrue(result.heavy)

    def test_nested_markdown_fixture_is_not_assumed_to_be_docs(self) -> None:
        result = MODULE.classify_paths(["crates/io/tests/fixtures/case.md"])
        self.assertTrue(result.heavy)

    def test_nested_readme_is_not_assumed_to_be_docs(self) -> None:
        result = MODULE.classify_paths(["crates/io/README.md"])
        self.assertTrue(result.heavy)


    def test_merge_group_source_change_leaves_the_pr_tier_to_pr_and_main(self) -> None:
        # The PR ran tier 1 and the package build; the main push of the same
        # tree runs every tier after the merge.
        self.assertFalse(MODULE.QUEUE_RERUNS_PR_TIER)
        for paths in (PR_618_PATHS, ["fuzz/fuzz_targets/nurbs_surface.rs"],
                      ["crates/render/src/lib.rs"], ["scripts/check-boundaries.sh"]):
            with self.subTest(paths=paths):
                result = MODULE.classify_paths(paths, force_full=True, merge_group=True)
                self.assertFalse(result.heavy)
                self.assertTrue(result.docs)
                self.assertFalse(result.full)
                self.assertFalse(result.wasm)
                self.assertEqual(result.mode, "queue")

    def test_merge_group_skip_needs_the_queue_event_and_its_full_flag(self) -> None:
        # A PR or labelled PR never takes the queue shortcut.
        for force_full in (False, True):
            with self.subTest(force_full=force_full):
                result = MODULE.classify_paths(PR_618_PATHS, force_full=force_full)
                self.assertTrue(result.heavy)
                self.assertTrue(result.wasm)
        result = MODULE.classify_paths(PR_618_PATHS, merge_group=True)
        self.assertTrue(result.heavy)
        self.assertEqual(result.mode, "pr")

    def test_merge_group_pairing_a_refresh_with_source_skips_the_pr_tier(self) -> None:
        # wasm-version.yml's integrity job rebuilds and compares the packages.
        result = MODULE.classify_paths(
            ["crates/wasm/pkg/remus_wasm_bg.wasm", "crates/wasm-io/pkg/package.json",
             "crates/math/src/lib.rs", "docs/kernel-maturity/roadmap.md"],
            force_full=True, merge_group=True,
        )
        self.assertFalse(result.heavy)
        self.assertFalse(result.full)
        self.assertEqual(result.mode, "queue")

    def test_restored_merge_group_source_change_runs_the_pr_tier(self) -> None:
        # The main push of the same tree runs the second tier after the merge.
        with queue_reruns_pr_tier(True):
            result = MODULE.classify_paths(
                PR_618_PATHS, force_full=True, merge_group=True
            )
        self.assertTrue(result.heavy)
        self.assertTrue(result.docs)
        self.assertFalse(result.full)
        self.assertTrue(result.wasm)
        self.assertEqual(result.mode, "queue")

    def test_restored_merge_group_leaves_fuzz_and_render_to_the_first_tier(self) -> None:
        # Both jobs run with tier 1 (fleet-ci-v4.yml), so their sources alone
        # do not select tier 2.
        for path in ("fuzz/fuzz_targets/nurbs_surface.rs", "crates/render/src/lib.rs"):
            with self.subTest(path=path), queue_reruns_pr_tier(True):
                result = MODULE.classify_paths([path], force_full=True, merge_group=True)
                self.assertTrue(result.heavy)
                self.assertFalse(result.full)
                self.assertEqual(result.mode, "queue")

    def test_restored_merge_group_without_wasm_paths_skips_the_package_build(self) -> None:
        with queue_reruns_pr_tier(True):
            result = MODULE.classify_paths(
                ["scripts/check-boundaries.sh"], force_full=True, merge_group=True
            )
        self.assertTrue(result.heavy)
        self.assertFalse(result.full)
        self.assertFalse(result.wasm)

    def test_merge_group_keeps_the_second_tier_for_its_inputs(self) -> None:
        for path in (
            "Cargo.lock", "Cargo.toml", "crates/math/Cargo.toml", "deny.toml",
            "rust-toolchain.toml", "clippy.toml", ".cargo/config.toml",
            ".config/nextest.toml", ".github/workflows/ci.yml",
        ):
            for restored in (False, True):
                with self.subTest(path=path, restored=restored), queue_reruns_pr_tier(restored):
                    result = MODULE.classify_paths(
                        ["crates/math/src/lib.rs", path], force_full=True, merge_group=True
                    )
                    self.assertTrue(result.heavy)
                    self.assertTrue(result.full)
                    self.assertTrue(result.wasm)
                    self.assertEqual(result.mode, "full")

    def test_merge_group_never_relaxes_what_fails_closed(self) -> None:
        for restored in (False, True):
            with self.subTest(restored=restored), queue_reruns_pr_tier(restored):
                result = MODULE.classify_paths([], force_full=True, merge_group=True)
                self.assertTrue(result.heavy)
                self.assertTrue(result.full)
                self.assertEqual(result.mode, "full")
        # A queue batch that pairs a package refresh with a source PR.
        with queue_reruns_pr_tier(True):
            result = MODULE.classify_paths(
                ["crates/wasm/pkg/remus_wasm_bg.wasm", "crates/wasm-io/pkg/package.json",
                 "crates/math/src/lib.rs", "docs/kernel-maturity/roadmap.md"],
                force_full=True, merge_group=True,
            )
        self.assertTrue(result.full)
        self.assertEqual(result.mode, "full")
        # A merge group never selects more than the event's own tier.
        result = MODULE.classify_paths(["Cargo.lock"], merge_group=True)
        self.assertFalse(result.full)

    def test_main_push_and_labelled_pr_keep_the_full_suite(self) -> None:
        result = MODULE.classify_paths(PR_618_PATHS, force_full=True)
        self.assertTrue(result.full)
        self.assertEqual(result.mode, "full")

    def test_cli_reads_the_merge_group_event_from_the_runner(self) -> None:
        files = {"crates/algo/src/pave_filler/phase_ff.rs": "// change\n"}
        queued = self.run_cli(files, "--full", event="merge_group")
        self.assertEqual(queued["heavy"], "false")
        self.assertEqual(queued["docs"], "true")
        self.assertEqual(queued["full"], "false")
        self.assertEqual(queued["wasm"], "false")
        self.assertEqual(queued["mode"], "queue")
        pushed = self.run_cli(files, "--full", event="push")
        self.assertEqual(pushed["full"], "true")
        self.assertEqual(pushed["mode"], "full")
        locked = self.run_cli({**files, "Cargo.lock": "# lock\n"}, "--full", event="merge_group")
        self.assertEqual(locked["heavy"], "true")
        self.assertEqual(locked["full"], "true")

    def test_cli_runs_no_heavy_tier_for_a_package_refresh(self) -> None:
        files = {"crates/wasm/pkg/remus_wasm_bg.wasm": "wasm\n",
                 "crates/wasm-io/pkg/package.json": "{}\n"}
        for event, args in (("pull_request", ()), ("merge_group", ("--full",)),
                            ("push", ("--full",))):
            with self.subTest(event=event):
                outputs = self.run_cli(files, *args, event=event)
                self.assertEqual(outputs["heavy"], "false")
                self.assertEqual(outputs["full"], "false")
                self.assertEqual(outputs["wasm"], "false")
                self.assertEqual(outputs["docs"], "false")
                self.assertEqual(outputs["mode"], "package")

if __name__ == "__main__":
    unittest.main()

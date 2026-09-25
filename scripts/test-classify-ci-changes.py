#!/usr/bin/env python3
"""Regression tests for the CI change classifier."""

from __future__ import annotations

import importlib.util
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


sys.dont_write_bytecode = True
MODULE_PATH = Path(__file__).with_name("classify-ci-changes.py")
SPEC = importlib.util.spec_from_file_location("classify_ci_changes", MODULE_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot load {MODULE_PATH}")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


ROOT = MODULE_PATH.resolve().parents[1]
FLEET_CI = ROOT / ".github/workflows/fleet-ci.yml"
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
        self.assertRegex(wasm, r"(?m)^    if: needs\.changes\.outputs\.wasm == 'true'$")
        self.assertIn("cargo xtask wasm-build", wasm)
        changes = jobs["changes"]
        self.assertIn("wasm: ${{ steps.classify.outputs.wasm }}", changes)
        self.assertRegex(changes, r"python3 scripts/classify-ci-changes\.py \\\n\s+--base \"\$base\" \\\n"
                                  r"\s+--head \"\$GITHUB_SHA\" \$full_flag \| tee -a \"\$GITHUB_OUTPUT\"")

    def run_cli(self, files: dict[str, str]) -> dict[str, str]:
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
                [sys.executable, str(MODULE_PATH), "--base", "HEAD~1", "--head", "HEAD"],
                cwd=tmp, check=True, capture_output=True, text=True,
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

    def test_committed_package_paths_cannot_bypass_validation(self) -> None:
        for force_full in (False, True):
            result = MODULE.classify_paths(
                ["crates/wasm/pkg/remus_wasm_bg.wasm", "crates/wasm-io/pkg/package.json"],
                force_full=force_full,
            )
            self.assertTrue(result.heavy)
            self.assertTrue(result.docs)
            self.assertTrue(result.full)
            self.assertTrue(result.wasm)
            self.assertEqual(result.mode, "full")

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


if __name__ == "__main__":
    unittest.main()

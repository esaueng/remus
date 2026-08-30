#!/usr/bin/env python3
"""Regression tests for the CI change classifier."""

from __future__ import annotations

import importlib.util
import sys
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


class ClassifyCiChangesTests(unittest.TestCase):
    def test_source_change_runs_everything(self) -> None:
        result = MODULE.classify_paths(["crates/math/src/lib.rs"])
        self.assertTrue(result.heavy)
        self.assertTrue(result.docs)
        self.assertEqual(result.mode, "full")

    def test_unknown_path_fails_closed(self) -> None:
        result = MODULE.classify_paths(["rust-toolchain.toml"])
        self.assertTrue(result.heavy)
        self.assertEqual(result.mode, "full")

    def test_empty_diff_fails_closed(self) -> None:
        result = MODULE.classify_paths([])
        self.assertTrue(result.heavy)
        self.assertEqual(result.mode, "full")

    def test_docs_only_builds_docs_without_heavy_jobs(self) -> None:
        result = MODULE.classify_paths(["book/src/guide.md", "README.md"])
        self.assertFalse(result.heavy)
        self.assertTrue(result.docs)
        self.assertEqual(result.mode, "docs")

    def test_workflow_change_runs_everything(self) -> None:
        result = MODULE.classify_paths([".github/workflows/ci.yml"])
        self.assertTrue(result.heavy)
        self.assertTrue(result.docs)
        self.assertEqual(result.mode, "full")

    def test_github_metadata_only_skips_docs_and_heavy_jobs(self) -> None:
        result = MODULE.classify_paths([".github/dependabot.yml"])
        self.assertFalse(result.heavy)
        self.assertFalse(result.docs)
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

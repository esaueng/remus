#!/usr/bin/env python3
"""Tests for the package-source boundary (scripts/check_package_source.py)."""

from __future__ import annotations

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import check_package_source as gate  # noqa: E402

REPO = "esaueng/remus"


def pr(login="petertoday", head_repo=REPO, ref="fix/something", changed=1):
    return {
        "user": {"login": login},
        "head": {"repo": {"full_name": head_repo}, "ref": ref},
        "changed_files": changed,
    }


def files(*paths, previous=None):
    entries = [{"filename": path} for path in paths]
    if previous:
        entries[0]["previous_filename"] = previous
    return entries


BOT = dict(login=gate.PACKAGE_BOT, ref="codex/wasm-refresh-" + "a" * 40)


class VerdictTests(unittest.TestCase):
    def test_a_change_outside_the_packages_passes(self):
        ok, _ = gate.verdict(pr(changed=2), files("crates/io/src/lib.rs", "SECURITY.md"), REPO)
        self.assertTrue(ok)

    def test_a_human_pr_changing_a_package_file_fails(self):
        for path in ("crates/wasm/pkg/remus_wasm_bg.wasm", "crates/wasm-io/pkg/package.json"):
            ok, message = gate.verdict(pr(), files(path), REPO)
            self.assertFalse(ok, path)
            self.assertIn(path, message)

    def test_a_rename_out_of_a_package_counts(self):
        ok, _ = gate.verdict(pr(), files("elsewhere.wasm", previous="crates/wasm/pkg/remus_wasm_bg.wasm"), REPO)
        self.assertFalse(ok)

    def test_the_package_bot_refresh_passes(self):
        ok, _ = gate.verdict(pr(**BOT), files("crates/wasm/pkg/remus_wasm_bg.wasm"), REPO)
        self.assertTrue(ok)

    def test_the_bot_needs_its_refresh_branch_and_this_repository(self):
        for override in ({"ref": "codex/other"}, {"head_repo": "someone/remus"}):
            ok, _ = gate.verdict(pr(**{**BOT, **override}), files("crates/wasm/pkg/x.js"), REPO)
            self.assertFalse(ok, override)

    def test_a_refresh_branch_name_alone_is_not_the_bot(self):
        ok, _ = gate.verdict(pr(ref=BOT["ref"]), files("crates/wasm/pkg/x.js"), REPO)
        self.assertFalse(ok)

    def test_lookalike_paths_are_not_packages(self):
        ok, _ = gate.verdict(pr(changed=2), files("crates/wasm/pkgs/x", "crates/wasm/src/pkg/x"), REPO)
        self.assertTrue(ok)

    def test_an_incomplete_file_list_fails_closed(self):
        ok, _ = gate.verdict(pr(changed=2), files("README.md"), REPO)
        self.assertFalse(ok)
        ok, _ = gate.verdict(pr(changed=gate.MAX_LISTED_FILES + 1), files("README.md"), REPO)
        self.assertFalse(ok)
        ok, _ = gate.verdict({**pr(), "changed_files": None}, files("README.md"), REPO)
        self.assertFalse(ok)


class QueueRefTests(unittest.TestCase):
    def test_reads_the_pr_number_from_a_queue_ref(self):
        ref = "refs/heads/gh-readonly-queue/main/pr-1712-" + "0" * 40
        self.assertEqual(gate.queued_pr_number(ref), 1712)

    def test_rejects_other_refs(self):
        for ref in ("refs/heads/main", "gh-readonly-queue/main/pr-0-" + "0" * 40, "", "gh-readonly-queue/main/pr-12-abc"):
            with self.assertRaises(ValueError, msg=ref):
                gate.queued_pr_number(ref)


class LinkTests(unittest.TestCase):
    def test_finds_the_next_page(self):
        header = '<https://api.github.com/x?page=2>; rel="next", <https://api.github.com/x?page=5>; rel="last"'
        self.assertEqual(gate.next_link(header), "https://api.github.com/x?page=2")
        self.assertIsNone(gate.next_link('<https://api.github.com/x?page=1>; rel="prev"'))
        self.assertIsNone(gate.next_link(""))


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Tests for the package-source boundary (scripts/check_package_source.py)."""

from __future__ import annotations

import pathlib
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import check_package_source as gate  # noqa: E402

REPO = "esaueng/remus"
A, B, BASE, BASE2 = "a" * 40, "b" * 40, "c" * 40, "d" * 40
PKG = "crates/wasm/pkg/remus_wasm_bg.wasm"


def pr(login="petertoday", head_repo=REPO, ref="fix/something", changed=1, head=A, base=BASE):
    return {
        "user": {"login": login},
        "head": {"repo": {"full_name": head_repo}, "ref": ref, "sha": head},
        "base": {"sha": base},
        "changed_files": changed,
    }


def files(*paths, previous=None):
    entries = [{"filename": path} for path in paths]
    if previous:
        entries[0]["previous_filename"] = previous
    return entries


BOT = dict(login=gate.PACKAGE_BOT, ref="codex/wasm-refresh-" + "e" * 40)


class VerdictTests(unittest.TestCase):
    def check(self, the_pr, the_files, expected="auto"):
        count = len(the_files) if expected == "auto" else expected
        return gate.verdict(the_pr, the_files, REPO, count)[0]

    def test_a_change_outside_the_packages_passes(self):
        self.assertTrue(self.check(pr(), files("crates/io/src/lib.rs", "SECURITY.md")))

    def test_a_human_change_to_a_package_file_fails(self):
        for path in (PKG, "crates/wasm-io/pkg/package.json"):
            ok, message = gate.verdict(pr(), files(path), REPO, 1)
            self.assertFalse(ok, path)
            self.assertIn(path, message)

    def test_a_rename_out_of_a_package_counts(self):
        self.assertFalse(self.check(pr(), files("elsewhere.wasm", previous=PKG)))

    def test_the_package_bot_refresh_passes(self):
        self.assertTrue(self.check(pr(**BOT), files(PKG)))

    def test_the_bot_needs_its_refresh_branch_and_this_repository(self):
        for override in ({"ref": "codex/other"}, {"head_repo": "someone/remus"}):
            self.assertFalse(self.check(pr(**{**BOT, **override}), files(PKG)), override)

    def test_a_refresh_branch_name_alone_is_not_the_bot(self):
        self.assertFalse(self.check(pr(ref=BOT["ref"]), files(PKG)))

    def test_lookalike_paths_are_not_packages(self):
        self.assertTrue(self.check(pr(), files("crates/wasm/pkgs/x", "crates/wasm/src/pkg/x")))

    def test_an_incomplete_file_list_fails_closed(self):
        self.assertFalse(self.check(pr(), files("README.md"), expected=2))
        self.assertFalse(self.check(pr(), files("README.md"), expected=gate.MAX_LISTED_FILES + 1))
        self.assertFalse(self.check(pr(), [{"filename": "x"}] * gate.MAX_LISTED_FILES, expected=None))


def fake_api(routes):
    """A request() stand-in: routes maps a URL substring to successive replies."""
    calls = []

    def request(method, url, token, body=None):
        calls.append((method, url))
        for needle, replies in routes.items():
            if needle in url:
                reply = replies.pop(0) if len(replies) > 1 else replies[0]
                return reply, ""
        raise AssertionError(f"unexpected request {method} {url}")

    return request, calls


class PullRequestModeTests(unittest.TestCase):
    event = {"pull_request": {"number": 7, "head": {"sha": A}}}

    def run_check(self, routes):
        request, _ = fake_api(routes)
        with mock.patch.object(gate, "request", request):
            return gate.check_pull_request(self.event, REPO, "t")

    def test_checks_the_event_head(self):
        sha, ok, _ = self.run_check({"/pulls/7/files": [files(PKG)], "/pulls/7": [pr()]})
        self.assertEqual((sha, ok), (A, False))

    def test_a_head_that_moved_before_the_listing_publishes_nothing(self):
        with self.assertRaises(gate.Stale):
            self.run_check({"/pulls/7/files": [files("README.md")], "/pulls/7": [pr(head=B)]})

    def test_a_head_that_moved_during_the_listing_publishes_nothing(self):
        # The listing could belong to the clean head B, not the package head A.
        with self.assertRaises(gate.Stale):
            self.run_check({"/pulls/7/files": [files("README.md")], "/pulls/7": [pr(), pr(head=B)]})

    def test_a_retarget_during_the_listing_publishes_nothing(self):
        with self.assertRaises(gate.Stale):
            self.run_check({"/pulls/7/files": [files("README.md")], "/pulls/7": [pr(), pr(base=BASE2)]})


class MergeGroupModeTests(unittest.TestCase):
    def event(self, **run):
        base = {
            "event": "merge_group",
            "head_repository": {"full_name": REPO},
            "head_sha": A,
            "head_branch": f"gh-readonly-queue/main/pr-7-{BASE}",
        }
        return {"workflow_run": {**base, **run}}

    def run_check(self, event, commit, the_pr):
        request, calls = fake_api({f"/commits/{A}": [commit], "/pulls/7": [the_pr]})
        with mock.patch.object(gate, "request", request):
            return gate.check_merge_group(event, REPO, "t"), calls

    def test_checks_the_queue_commit_diff(self):
        (sha, ok, _), calls = self.run_check(self.event(), {"parents": [{}], "files": files(PKG)}, pr())
        self.assertEqual((sha, ok), (A, False))
        self.assertTrue(any(f"/commits/{A}" in url for _, url in calls))

    def test_the_bot_refresh_passes_in_the_queue(self):
        (_, ok, _), _ = self.run_check(self.event(), {"parents": [{}], "files": files(PKG)}, pr(**BOT))
        self.assertTrue(ok)

    def test_a_queue_commit_with_several_parents_fails_closed(self):
        (_, ok, _), _ = self.run_check(self.event(), {"parents": [{}, {}], "files": files("README.md")}, pr())
        self.assertFalse(ok)

    def test_only_merge_group_runs_of_this_repository_count(self):
        for run in ({"event": "push"}, {"head_repository": {"full_name": "someone/remus"}}, {"head_branch": "main"}):
            with self.assertRaises(ValueError, msg=run):
                self.run_check(self.event(**run), {"parents": [{}], "files": []}, pr())


class QueueBranchTests(unittest.TestCase):
    def test_reads_the_pr_number_from_a_queue_branch(self):
        self.assertEqual(gate.queued_pr_number("gh-readonly-queue/main/pr-1712-" + "0" * 40), 1712)

    def test_rejects_other_branches(self):
        for branch in ("main", "gh-readonly-queue/main/pr-0-" + "0" * 40, "", "gh-readonly-queue/dev/pr-12-" + "0" * 40):
            with self.assertRaises(ValueError, msg=branch):
                gate.queued_pr_number(branch)


class LinkTests(unittest.TestCase):
    def test_finds_the_next_page(self):
        header = '<https://api.github.com/x?page=2>; rel="next", <https://api.github.com/x?page=5>; rel="last"'
        self.assertEqual(gate.next_link(header), "https://api.github.com/x?page=2")
        self.assertIsNone(gate.next_link('<https://api.github.com/x?page=1>; rel="prev"'))
        self.assertIsNone(gate.next_link(""))


if __name__ == "__main__":
    unittest.main()

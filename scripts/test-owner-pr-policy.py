#!/usr/bin/env python3
"""Exercise the exact policy embedded in the immutable VPS workflow."""

from __future__ import annotations

import copy
import re
import textwrap
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
WORKFLOW = ROOT / ".github/workflows/owner-pr.yml"
TEXT = WORKFLOW.read_text()
POLICY = {"__name__": "owner_policy"}
SOURCE = TEXT.split("python3 - <<'PY'\n", 1)[1].split("\n          PY", 1)[0]
exec(compile(textwrap.dedent(SOURCE), str(WORKFLOW), "exec"), POLICY)


def fixtures():
    repo = {"id": 1334765869, "full_name": "esaueng/remus", "fork": False}
    owner = {"id": 171875562, "login": "petergstfsn"}
    pr = {
        "number": 300, "state": "open", "user": owner,
        "head": {"repo": copy.deepcopy(repo), "sha": "a" * 40, "ref": "fix/example"},
        "base": {"repo": copy.deepcopy(repo), "sha": "b" * 40, "ref": "main"},
    }
    event = {"pull_request": pr, "sender": copy.deepcopy(owner)}
    context = {
        "ENABLED": "true", "EVENT_NAME": "pull_request", "REPOSITORY": repo["full_name"],
        "REPOSITORY_ID": str(repo["id"]), "ACTOR": owner["login"],
        "ACTOR_ID": str(owner["id"]), "TRIGGERING_ACTOR": owner["login"],
        "REF": "refs/pull/300/merge", "SHA": "c" * 40,
    }
    commit = {"sha": context["SHA"], "parents": [{"sha": "b" * 40}, {"sha": "a" * 40}]}
    return event, context, commit


class OwnerPolicyTests(unittest.TestCase):
    def test_owner_same_repository_pr_is_eligible(self):
        event, context, commit = fixtures()
        self.assertTrue(POLICY["eligible"](event, context))
        self.assertTrue(POLICY["current_merge"](
            event["pull_request"], event["pull_request"], commit, context["SHA"]))

    def test_other_event_actor_repository_or_ref_is_denied(self):
        event, context, _ = fixtures()
        cases = {
            "ENABLED": ["false", "", "True", "1"],
            "EVENT_NAME": ["pull_request_target", "push", "workflow_dispatch",
                           "workflow_run", "workflow_call", ""],
            "REPOSITORY": ["attacker/remus", "esaueng/other", ""],
            "REPOSITORY_ID": ["42", ""],
            "ACTOR": ["contributor", "dependabot[bot]", ""],
            "ACTOR_ID": ["42", ""],
            "TRIGGERING_ACTOR": ["contributor", "dependabot[bot]", ""],
            "REF": ["refs/heads/main", "refs/pull/301/merge", "refs/pull/300/head", ""],
            "SHA": ["main", "a" * 39, "a" * 41, "$(echo unsafe)", ""],
        }
        for key, values in cases.items():
            for value in values:
                with self.subTest(key=key, value=value):
                    modified = dict(context, **{key: value})
                    self.assertFalse(POLICY["eligible"](event, modified))

    def test_other_author_or_sender_cannot_use_an_owner_rerun(self):
        event, context, _ = fixtures()
        for target in ("user", "sender"):
            for field, value in (("id", 42), ("login", "contributor")):
                with self.subTest(target=target, field=field):
                    changed = copy.deepcopy(event)
                    actor = changed["pull_request"]["user"] if target == "user" else changed["sender"]
                    actor[field] = value
                    self.assertFalse(POLICY["eligible"](changed, context))

    def test_owner_forks_and_other_base_branches_are_denied(self):
        event, context, _ = fixtures()
        for side in ("head", "base"):
            for field, value in (("id", 42), ("full_name", "petergstfsn/remus"),
                                 ("fork", True), ("fork", None)):
                with self.subTest(side=side, field=field):
                    changed = copy.deepcopy(event)
                    changed["pull_request"][side]["repo"][field] = value
                    self.assertFalse(POLICY["eligible"](changed, context))
        event["pull_request"]["base"]["ref"] = "unprotected-topic"
        self.assertFalse(POLICY["eligible"](event, context))

    def test_missing_or_closed_pr_fails_closed(self):
        event, context, _ = fixtures()
        for missing in ({}, {"pull_request": None}, {"pull_request": {}},
                        {"pull_request": event["pull_request"]}):
            self.assertFalse(POLICY["eligible"](missing, context))
        for key, value in (("state", "closed"), ("number", True), ("number", 0),
                           ("head", None), ("base", None), ("user", None)):
            with self.subTest(key=key):
                changed = copy.deepcopy(event)
                changed["pull_request"][key] = value
                self.assertFalse(POLICY["eligible"](changed, context))

    def test_stale_head_base_or_closed_pr_is_denied(self):
        event, context, commit = fixtures()
        pr = event["pull_request"]
        for side in ("head", "base"):
            with self.subTest(side=side):
                current = copy.deepcopy(pr)
                current[side]["sha"] = "d" * 40
                self.assertFalse(POLICY["current_merge"](pr, current, commit, context["SHA"]))
        for key, value in (("state", "closed"), ("number", 301)):
            current = dict(pr, **{key: value})
            self.assertFalse(POLICY["current_merge"](pr, current, commit, context["SHA"]))

    def test_merge_commit_must_have_exact_two_parents_in_order(self):
        event, context, commit = fixtures()
        pr = event["pull_request"]
        for parents in ([], commit["parents"][:1], list(reversed(commit["parents"])),
                        commit["parents"] + [{"sha": "d" * 40}],
                        [{"sha": "d" * 40}, {"sha": "a" * 40}]):
            with self.subTest(parents=parents):
                altered = dict(commit, parents=parents)
                self.assertFalse(POLICY["current_merge"](pr, pr, altered, context["SHA"]))
        self.assertFalse(POLICY["current_merge"](pr, pr, commit, "d" * 40))

    def test_titles_and_branch_names_are_data(self):
        event, context, _ = fixtures()
        event["pull_request"]["title"] = "$(touch /tmp/should-not-run)"
        event["pull_request"]["head"]["ref"] = "feature/quote'and$commands"
        self.assertTrue(POLICY["eligible"](event, context))

    def test_authorization_has_no_checkout_or_caller_selected_ref(self):
        gate, rust = TEXT.split("\n  rust:\n", 1)
        self.assertNotIn("actions/checkout@", gate)
        self.assertNotIn("inputs.ref", TEXT)
        self.assertNotIn("inputs.repository", TEXT)
        self.assertIn("needs: select", rust)
        self.assertIn("needs.select.outputs.trusted == 'true'", rust)
        self.assertIn("ref: ${{ github.sha }}", rust)
        self.assertIn("persist-credentials: false", rust)
        self.assertNotIn("secrets.", TEXT)
        self.assertNotRegex(TEXT, r": write\b")
        self.assertNotIn("actions/cache", rust)
        self.assertNotIn("rust-cache", rust)
        self.assertNotIn("pull_request_target:", TEXT)

    def test_workflow_and_actions_have_fixed_access_boundaries(self):
        self.assertIn("  workflow_call:\n", TEXT)
        self.assertIn("group: ci-trusted-main", TEXT)
        self.assertIn("labels: ci-small", TEXT)
        for reference in re.findall(r"uses: (\S+)", TEXT):
            self.assertRegex(reference, r"@[0-9a-f]{40}$")
        self.assertIn('CARGO_BUILD_JOBS: "1"', TEXT)
        self.assertIn('RUST_TEST_THREADS: "1"', TEXT)
        self.assertIn("3221225472", TEXT)
        self.assertIn("memory.swap.max", TEXT)


if __name__ == "__main__":
    unittest.main()


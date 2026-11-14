#!/usr/bin/env python3
"""Let only the package bot change the committed WASM packages.

Consumers install `crates/wasm/pkg` and `crates/wasm-io/pkg` straight from
`main`, and a source rebuild cannot vouch for a PR that also controls the build
code it runs (SECURITY.md, Supply Chain). So only the package bot's refresh PRs,
which `publish.yml` builds from protected `main`, may change those directories.

This script and `.github/workflows/package-source.yml` always run from the
default branch (`pull_request_target` and `workflow_run`), never from PR or
merge-group code, and treat everything they read as data. The verdict is
published as the `package-source` check run by a GitHub App whose key lives in
an environment that PR and merge-group workflows cannot use, so requiring the
check with that App as its source makes it unforgeable by PR code.

- Merge queue (the boundary): `Package source request` runs on `merge_group`
  without credentials; its completion triggers the trusted run, which reads the
  queue commit's own diff against its single parent. That diff is exactly what
  the queued PR adds to `main` and cannot change after the fact.
- Pull request (early feedback): checks the PR's file list for the event's head
  SHA, and publishes nothing when the PR head moved during the check.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import urllib.error
import urllib.request

PACKAGE_ROOTS = ("crates/wasm/pkg/", "crates/wasm-io/pkg/")
PACKAGE_BOT = "remus-package-bot[bot]"
REFRESH_BRANCH_PREFIX = "codex/wasm-refresh-"
CHECK_NAME = "package-source"
# The pull request files and commit APIs list at most this many files.
MAX_LISTED_FILES = 3000
QUEUE_BRANCH = re.compile(r"^(?:refs/heads/)?gh-readonly-queue/main/pr-([1-9][0-9]*)-[0-9a-f]{40}$")
SHA = re.compile(r"^[0-9a-f]{40}$")
API = "https://api.github.com"


class Stale(Exception):
    """The PR moved while it was being checked; a newer run owns the verdict."""


def package_paths(files: list[dict]) -> list[str]:
    """Package paths a change touches, including the old path of a rename."""
    touched = []
    for entry in files:
        for key in ("filename", "previous_filename"):
            path = entry.get(key)
            if path and path.startswith(PACKAGE_ROOTS) and path not in touched:
                touched.append(path)
    return touched


def is_package_bot_pr(pr: dict, repository: str) -> bool:
    head = pr.get("head") or {}
    return (
        (pr.get("user") or {}).get("login") == PACKAGE_BOT
        and ((head.get("repo") or {}).get("full_name")) == repository
        and str(head.get("ref", "")).startswith(REFRESH_BRANCH_PREFIX)
    )


def verdict(pr: dict, files: list[dict], repository: str, expected: int | None) -> tuple[bool, str]:
    """(passes, summary) for one change. `expected` is the file count, if known."""
    if len(files) >= MAX_LISTED_FILES or (expected is not None and (expected > MAX_LISTED_FILES or len(files) != expected)):
        return False, "Could not list every changed file; refusing to vouch for the packages."
    touched = package_paths(files)
    if not touched:
        return True, "No committed package files change."
    if is_package_bot_pr(pr, repository):
        return True, "Package refresh from the package bot."
    shown = ", ".join(touched[:3]) + (" ..." if len(touched) > 3 else "")
    return False, f"Only the package bot may change committed packages: {shown}"


def queued_pr_number(branch: str) -> int:
    match = QUEUE_BRANCH.match(branch or "")
    if not match:
        raise ValueError(f"unexpected merge queue branch: {branch!r}")
    return int(match.group(1))


def request(method: str, url: str, token: str, body: dict | None = None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("Accept", "application/vnd.github+json")
    req.add_header("Authorization", f"Bearer {token}")
    req.add_header("X-GitHub-Api-Version", "2022-11-28")
    with urllib.request.urlopen(req, timeout=30) as response:
        payload = response.read()
        return (json.loads(payload) if payload else None), response.headers.get("Link", "")


def next_link(link_header: str) -> str | None:
    for part in link_header.split(","):
        section = part.split(";")
        if len(section) > 1 and section[1].strip() == 'rel="next"':
            return section[0].strip().strip("<>")
    return None


def paged_files(url: str, token: str, key: str | None = None) -> list[dict]:
    """Every file from a paginated list (`key` selects it from an object page)."""
    files: list[dict] = []
    next_url: str | None = url
    while next_url:
        page, link = request("GET", next_url, token)
        files.extend(page[key] if key else page)
        next_url = next_link(link)
    return files


def check_pull_request(event: dict, repository: str, token: str) -> tuple[str, bool, str]:
    """Verdict for the head SHA named by a pull_request_target event."""
    number = int(event["pull_request"]["number"])
    sha = event["pull_request"]["head"]["sha"]
    if not SHA.match(sha):
        raise ValueError("unexpected head SHA")
    pr_url = f"{API}/repos/{repository}/pulls/{number}"
    before, _ = request("GET", pr_url, token)
    if before["head"]["sha"] != sha:
        raise Stale(sha)
    files = paged_files(f"{pr_url}/files?per_page=100", token)
    after, _ = request("GET", pr_url, token)
    # The file list belongs to this head and base only if neither moved meanwhile.
    if after["head"]["sha"] != sha or after["base"]["sha"] != before["base"]["sha"]:
        raise Stale(sha)
    passed, summary = verdict(after, files, repository, after.get("changed_files"))
    return sha, passed, summary


def check_merge_group(event: dict, repository: str, token: str) -> tuple[str, bool, str]:
    """Verdict for a queue commit, from its own diff against its single parent."""
    run = event["workflow_run"]
    if run.get("event") != "merge_group" or (run.get("head_repository") or {}).get("full_name") != repository:
        raise ValueError("not a merge group run of this repository")
    sha = run["head_sha"]
    if not SHA.match(sha):
        raise ValueError("unexpected head SHA")
    number = queued_pr_number(run.get("head_branch", ""))
    commit_url = f"{API}/repos/{repository}/commits/{sha}?per_page=100"
    commit, _ = request("GET", commit_url, token)
    if len(commit.get("parents") or []) != 1:
        return sha, False, "Queue commit is not a single squash commit; refusing to vouch for the packages."
    files = paged_files(commit_url, token, key="files")
    pr, _ = request("GET", f"{API}/repos/{repository}/pulls/{number}", token)
    passed, summary = verdict(pr, files, repository, None)
    return sha, passed, summary


def publish(repository: str, sha: str, token: str, passed: bool, summary: str) -> None:
    request(
        "POST",
        f"{API}/repos/{repository}/check-runs",
        token,
        {
            "name": CHECK_NAME,
            "head_sha": sha,
            "status": "completed",
            "conclusion": "success" if passed else "failure",
            "output": {"title": summary[:200], "summary": summary},
        },
    )


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--publish", action="store_true", help="publish the check run (CI only)")
    args = parser.parse_args(argv)

    token = os.environ["GH_TOKEN"]
    repository = os.environ["GITHUB_REPOSITORY"]
    event_name = os.environ["GITHUB_EVENT_NAME"]
    with open(os.environ["GITHUB_EVENT_PATH"], encoding="utf-8") as handle:
        event = json.load(handle)

    check = {"pull_request_target": check_pull_request, "workflow_run": check_merge_group}.get(event_name)
    if check is None:
        print(f"unsupported event: {event_name}", file=sys.stderr)
        return 1
    try:
        sha, passed, summary = check(event, repository, token)
    except Stale:
        print("The PR head or base moved during the check; the newer run publishes the verdict.")
        return 0
    except (OSError, KeyError, ValueError, TypeError, json.JSONDecodeError) as error:
        # Without a trustworthy SHA there is nothing to publish on: a missing
        # required check blocks the merge, which is the fail-closed outcome.
        print(f"Could not verify the package source: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(summary)
    if args.publish:
        publish(repository, sha, os.environ["CHECKS_TOKEN"], passed, summary)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

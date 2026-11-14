#!/usr/bin/env python3
"""Let only the package bot change the committed WASM packages.

Consumers install `crates/wasm/pkg` and `crates/wasm-io/pkg` straight from
`main`, and a source rebuild cannot vouch for a PR that also controls the build
code it runs (SECURITY.md, Supply Chain). So only the package bot's refresh PRs,
which `publish.yml` builds from protected `main`, may change those directories.

Under `pull_request_target` this script and its workflow come from the base
branch, never from the PR. It reads the PR's changed files from the API as data
and publishes the `package-source` commit status on the PR head, which a fork
PR's read-only token cannot forge. Under `merge_group` it re-checks the queued
PR the same way; that copy comes from the group head, so the PR run is the
boundary and the queue run only keeps the required status present.
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
STATUS_CONTEXT = "package-source"
# The pull request files API lists at most this many files.
MAX_LISTED_FILES = 3000
QUEUE_REF = re.compile(r"^(?:refs/heads/)?gh-readonly-queue/[^/].*/pr-([1-9][0-9]*)-[0-9a-f]{40}$")
API = "https://api.github.com"


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


def verdict(pr: dict, files: list[dict], repository: str) -> tuple[bool, str]:
    """(passes, description) for one PR and its full changed-file list."""
    expected = pr.get("changed_files")
    if not isinstance(expected, int) or expected > MAX_LISTED_FILES or len(files) < expected:
        return False, "Could not list every changed file; refusing to vouch for the packages."
    touched = package_paths(files)
    if not touched:
        return True, "No committed package files change."
    if is_package_bot_pr(pr, repository):
        return True, "Package refresh from the package bot."
    shown = ", ".join(touched[:3]) + (" ..." if len(touched) > 3 else "")
    return False, f"Only the package bot may change committed packages: {shown}"


def queued_pr_number(head_ref: str) -> int:
    match = QUEUE_REF.match(head_ref or "")
    if not match:
        raise ValueError(f"unexpected merge queue ref: {head_ref!r}")
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


def list_files(repository: str, number: int, token: str) -> list[dict]:
    url: str | None = f"{API}/repos/{repository}/pulls/{number}/files?per_page=100"
    files: list[dict] = []
    while url:
        page, link = request("GET", url, token)
        files.extend(page)
        url = next_link(link)
    return files


def publish(repository: str, sha: str, token: str, state: str, description: str, target: str) -> None:
    request(
        "POST",
        f"{API}/repos/{repository}/statuses/{sha}",
        token,
        {"state": state, "context": STATUS_CONTEXT, "description": description[:140], "target_url": target},
    )


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--publish", action="store_true", help="post the commit status (CI only)")
    args = parser.parse_args(argv)

    token = os.environ["GH_TOKEN"]
    repository = os.environ["GITHUB_REPOSITORY"]
    event_name = os.environ["GITHUB_EVENT_NAME"]
    with open(os.environ["GITHUB_EVENT_PATH"], encoding="utf-8") as handle:
        event = json.load(handle)
    target = f"{os.environ.get('GITHUB_SERVER_URL', 'https://github.com')}/{repository}/actions/runs/{os.environ.get('GITHUB_RUN_ID', '')}"

    if event_name == "pull_request_target":
        number = int(event["pull_request"]["number"])
        sha = event["pull_request"]["head"]["sha"]
    elif event_name == "merge_group":
        sha = event["merge_group"]["head_sha"]
        number = queued_pr_number(event["merge_group"]["head_ref"])
    else:
        print(f"unsupported event: {event_name}", file=sys.stderr)
        return 1

    if args.publish:
        publish(repository, sha, token, "pending", "Checking who changes the committed packages.", target)
    try:
        pr, _ = request("GET", f"{API}/repos/{repository}/pulls/{number}", token)
        passed, description = verdict(pr, list_files(repository, number, token), repository)
    except (urllib.error.URLError, KeyError, ValueError, TypeError) as error:
        passed, description = False, f"Could not verify the package source: {type(error).__name__}"
    print(description)
    if args.publish:
        publish(repository, sha, token, "success" if passed else "failure", description, target)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

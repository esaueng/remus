#!/usr/bin/env python3
"""Check public commit identities without printing them.

Covers the author, the committer and every `Co-authored-by:` and
`Signed-off-by:` trailer, since GitHub publishes all of them.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys

DEFAULT_NAMES = {
    "default",
    "first last",
    "name",
    "root",
    "runner",
    "ubuntu",
    "unknown",
    "user",
    "your name",
    "your name here",
}
ALLOWED_EMAIL_DOMAINS = {"users.noreply.github.com"}
# Exact public automation identities only; personal addresses still fail.
# Claude's attribution trailers may name the model, as in "Claude Opus 5.5";
# Dependabot signs off with GitHub's support address.
PUBLIC_BOT_IDENTITIES = {
    "noreply@anthropic.com": re.compile(r"claude(?: code| (?:opus|sonnet|haiku|fable) [0-9]+(?:\.[0-9]+)*)?"),
    "support@github.com": re.compile(r"dependabot\[bot\]"),
}
EMAIL_PATTERN = re.compile(r"^([^\s@<>]+)@([^\s@<>]+)$")
# Identity trailers anywhere in the message, not only in git's final trailer
# block: a line GitHub might attribute is checked rather than skipped.
TRAILER_PATTERN = re.compile(r"^[ \t]*(co-authored-by|signed-off-by)[ \t]*:(.*)$", re.IGNORECASE)
TRAILER_IDENTITY = re.compile(r"^(.*?)[ \t]*<([^<>]*)>$")


def validate_identity(name: str, email: str) -> list[str]:
    problems: list[str] = []
    if not name.strip() or name != name.strip() or any(ord(char) < 32 or ord(char) == 127 for char in name):
        problems.append("name is blank or contains control characters")
    elif name.strip().lower() in DEFAULT_NAMES:
        problems.append("name is an obvious placeholder")

    match = EMAIL_PATTERN.fullmatch(email.lower())
    bot_name = PUBLIC_BOT_IDENTITIES.get(email.lower())
    if not match:
        problems.append("email is malformed")
    elif (
        match.group(2) not in ALLOWED_EMAIL_DOMAINS
        and email.lower() != "noreply@github.com"
        and not (bot_name and bot_name.fullmatch(name.strip().lower()))
    ):
        problems.append("email must use a GitHub noreply domain or an approved public bot identity")
    return problems


def trailer_identities(message: str) -> list[tuple[str, str | None, str | None]]:
    """(label, name, email) for each identity trailer; name and email are None when unparseable."""
    identities: list[tuple[str, str | None, str | None]] = []
    for line in message.splitlines():
        trailer = TRAILER_PATTERN.match(line)
        if not trailer:
            continue
        label = f"{trailer.group(1).lower()} trailer"
        identity = TRAILER_IDENTITY.match(trailer.group(2).strip())
        if identity:
            identities.append((label, identity.group(1), identity.group(2)))
        else:
            identities.append((label, None, None))
    return identities


def validate_commits(commits: list[dict[str, str]]) -> list[str]:
    findings: list[str] = []
    for commit in commits:
        for label, name_key, email_key in (
            ("author", "author_name", "author_email"),
            ("committer", "committer_name", "committer_email"),
        ):
            for problem in validate_identity(commit[name_key], commit[email_key]):
                findings.append(f"{commit['hash'][:12]}: {label} {problem}")
        for label, name, email in trailer_identities(commit.get("message", "")):
            if name is None or email is None:
                findings.append(f"{commit['hash'][:12]}: {label} is not of the form 'Name <email>'")
                continue
            for problem in validate_identity(name, email):
                findings.append(f"{commit['hash'][:12]}: {label} {problem}")
    return findings


def read_commits(base: str, head: str) -> list[dict[str, str]]:
    fmt = "%H%x00%an%x00%ae%x00%cn%x00%ce%x00%B%x1e"
    output = subprocess.run(
        ["git", "log", f"--format={fmt}", "--no-decorate", f"{base}..{head}"],
        check=True,
        capture_output=True,
        # Messages carry non-ASCII text; never depend on the platform codec.
        encoding="utf-8",
        errors="replace",
    ).stdout
    commits: list[dict[str, str]] = []
    for record in output.split("\x1e"):
        if not record.strip():
            continue
        fields = record.removeprefix("\n").split("\x00")
        if len(fields) != 6:
            raise ValueError("could not parse commit metadata")
        commits.append(
            {
                "hash": fields[0],
                "author_name": fields[1],
                "author_email": fields[2],
                "committer_name": fields[3],
                "committer_email": fields[4],
                "message": fields[5],
            }
        )
    return commits


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, help="Base ref for the checked range")
    parser.add_argument("--head", required=True, help="Head ref for the checked range")
    args = parser.parse_args()
    try:
        findings = validate_commits(read_commits(args.base, args.head))
    except (OSError, subprocess.CalledProcessError, ValueError) as error:
        print(f"Commit metadata check could not inspect the requested range: {type(error).__name__}.", file=sys.stderr)
        return 1
    if findings:
        print(f"Commit metadata check failed ({len(findings)} issue(s)):", file=sys.stderr)
        for finding in findings:
            print(f"- {finding}", file=sys.stderr)
        return 1
    print("Commit metadata check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

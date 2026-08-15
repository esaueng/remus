#!/usr/bin/env python3
"""Validate the checked-in Apache contribution provenance ledger."""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
LEDGER_PATH = ROOT / "docs/production-readiness/apache-replay-provenance.json"
EXPECTED_LEDGER_SHA256 = (
    "5b540c03e5f2222d5dfb896ad51e78909fbd84da06e3fab2c9ba53feb646e199"
)
EXPECTED_ALL_PRS = set(range(127, 231)) | set(range(233, 248))
EXPECTED_PRIOR_PRS = {
    127, 128, 129, 130, 131, 132, 133, 134, 138, 139, 140, 141, 142, 143,
    144, 146, 147, 150, 151, 152, 155, 156, 157, 158, 159, 160, 161, 162,
    163,
}
EXPECTED_PHASE_TWO_PRS = {
    145, 149, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,
    176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189,
    190, 191, 192, 193, 194, 195, 198, 200, 201, 202, 203, 204, 208, 209,
    210, 211, 212, 215, 216, 218, 220, 221, 222, 223, 224, 225, 226, 227,
    228, 229, 230, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243,
    244, 245, 247,
}
EXPECTED_EXCLUDED_PRS = {
    135, 136, 137, 148, 153, 154, 196, 197, 199, 205, 206, 207, 213, 214,
    217, 219, 246,
}
EXPECTED_AUTHOR = "petergstfsn"
EXPECTED_REPLAY = {
    "parent_commit": "e142b5727f56188014ebec723b81e8104063fd1d",
    "parent_tree": "4c4650e0aa43cc3443c8d6eddcf53b5031198d13",
    "first_commit": "a49092d2fdfe9472794cb77f58ffdbff51d38b43",
    "last_commit": "7aeb36a802188de0e158326c15049ebaaa634ddc",
}
EXPECTED_ALLOWED_AUTHORS = {
    ("Peter", "171875562+petergstfsn@users.noreply.github.com"),
    ("Peter", "171875562+petergstfsn@users.noreply.github.com"),
}
SHA_PATTERN = re.compile(r"^[0-9a-f]{40}$")


def fail(message: str) -> None:
    print(f"Apache replay provenance violation: {message}", file=sys.stderr)
    raise SystemExit(1)


def require(condition: bool, message: str) -> None:
    if not condition:
        fail(message)


def git(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *args],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def load_ledger() -> dict[str, Any]:
    try:
        value = json.loads(LEDGER_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        fail(f"cannot read {LEDGER_PATH.relative_to(ROOT)}: {error}")
    require(isinstance(value, dict), "ledger root must be an object")
    canonical = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode()
    require(
        hashlib.sha256(canonical).hexdigest() == EXPECTED_LEDGER_SHA256,
        "pinned provenance evidence changed",
    )
    return value


def validate_pr_record(record: dict[str, Any], *, field: str) -> int:
    number = record.get("number")
    require(isinstance(number, int), f"{field} record has no PR number")
    require(
        record.get("url") == f"https://github.com/esaueng/brepkit/pull/{number}",
        f"PR #{number} URL changed",
    )
    require(record.get("author") == EXPECTED_AUTHOR, f"PR #{number} author changed")
    require(
        SHA_PATTERN.fullmatch(str(record.get("audited_head_sha"))) is not None,
        f"PR #{number} has an invalid audited head",
    )
    require(
        isinstance(record.get("title"), str) and bool(record["title"]),
        f"PR #{number} has no title",
    )
    return number


def validate_partition(ledger: dict[str, Any]) -> None:
    audit = ledger.get("audit_window")
    require(isinstance(audit, dict), "missing audit window")
    require(audit.get("pull_requests_audited") == 119, "audit count must be 119")
    require(
        audit.get("replayed_or_superseding_pull_requests") == 102,
        "replayed PR count must be 102",
    )
    require(
        audit.get("excluded_or_deferred_pull_requests") == 17,
        "excluded PR count must be 17",
    )

    prior = ledger.get("prior_source_pull_requests")
    phase_two = ledger.get("source_pull_requests")
    excluded = ledger.get("deliberately_excluded")
    require(isinstance(prior, list), "missing prior replay records")
    require(isinstance(phase_two, list), "missing phase-two replay records")
    require(isinstance(excluded, list), "missing exclusion records")

    prior_numbers = {
        validate_pr_record(record, field="prior")
        for record in prior
        if isinstance(record, dict)
    }
    phase_two_numbers = {
        validate_pr_record(record, field="phase-two")
        for record in phase_two
        if isinstance(record, dict)
    }
    excluded_numbers = {
        validate_pr_record(record, field="excluded")
        for record in excluded
        if isinstance(record, dict)
    }
    require(prior_numbers == EXPECTED_PRIOR_PRS, "prior replay PR set changed")
    require(
        phase_two_numbers == EXPECTED_PHASE_TWO_PRS,
        "phase-two replay PR set changed",
    )
    require(
        excluded_numbers == EXPECTED_EXCLUDED_PRS,
        "excluded or deferred PR set changed",
    )
    require(
        not (prior_numbers & phase_two_numbers)
        and not (prior_numbers & excluded_numbers)
        and not (phase_two_numbers & excluded_numbers),
        "PR audit partitions overlap",
    )
    require(
        prior_numbers | phase_two_numbers | excluded_numbers == EXPECTED_ALL_PRS,
        "post-cutoff PR audit is incomplete",
    )
    for record in excluded:
        require(
            isinstance(record.get("reason"), str) and bool(record["reason"]),
            f"PR #{record.get('number')} has no exclusion reason",
        )


def validate_staging_lineage(ledger: dict[str, Any]) -> None:
    staging = ledger.get("staging_lineage")
    require(isinstance(staging, dict), "missing staging lineage")
    require(
        staging.get("fork_cutoff_commit")
        == "1886e873fa4c24bf1880f2d3a868905c9d5e407f",
        "fork cutoff changed",
    )
    final_upstream = staging.get("final_permissive_upstream")
    require(
        final_upstream
        == {
            "tag": "v2.129.15",
            "commit": "a878e2b9c42cd36e4f9d2c00504502a6ef2f9687",
        },
        "final permissive upstream changed",
    )
    migration = staging.get("migration_pull_requests")
    require(isinstance(migration, list), "missing staging migration PRs")
    require(
        {record.get("number") for record in migration} == {248, 249, 250, 251, 252},
        "staging migration PR set changed",
    )
    for record in migration:
        validate_pr_record(record, field="migration")
        require(
            SHA_PATTERN.fullmatch(str(record.get("base_sha"))) is not None,
            f"migration PR #{record.get('number')} has an invalid base",
        )


def validate_source_commits(ledger: dict[str, Any]) -> None:
    records = ledger.get("prior_source_commits")
    require(
        isinstance(records, list) and len(records) == 11,
        "expected 11 source-commit mappings for staging PR #250",
    )
    for record in records:
        require(
            SHA_PATTERN.fullmatch(str(record.get("sha"))) is not None,
            "invalid prior source commit",
        )
        require(
            (record.get("author_name"), record.get("author_email"))
            in EXPECTED_ALLOWED_AUTHORS,
            f"unexpected author on source commit {record.get('sha')}",
        )
        require(
            SHA_PATTERN.fullmatch(str(record.get("replay_commit"))) is not None,
            f"invalid replay mapping for source commit {record.get('sha')}",
        )
        require(
            record.get("staging_pull_request") == 250,
            "prior source commit mapped to wrong staging PR",
        )


def validate_phase_two(ledger: dict[str, Any]) -> tuple[set[str], dict[str, str]]:
    replay = ledger.get("replay")
    require(isinstance(replay, dict), "missing replay object")
    require(replay.get("commit_count") == 73, "replay commit count must be 73")
    require(
        replay.get("source_pull_request_count") == 73,
        "source pull-request count must be 73",
    )
    require(replay.get("source_author") == EXPECTED_AUTHOR, "wrong source author")
    for field, expected in EXPECTED_REPLAY.items():
        require(replay.get(field) == expected, f"replay {field} changed")

    allowed = replay.get("allowed_replay_authors")
    require(isinstance(allowed, list), "missing replay author allowlist")
    require(
        {
            (record.get("name"), record.get("email"))
            for record in allowed
            if isinstance(record, dict)
        }
        == EXPECTED_ALLOWED_AUTHORS,
        "replay author allowlist changed",
    )

    mapped_commits: set[str] = set()
    shared_commits: dict[str, set[int]] = {}
    for record in ledger["source_pull_requests"]:
        commits = record.get("replay_commits")
        require(
            isinstance(commits, list) and commits,
            f"PR #{record['number']} is unmapped",
        )
        for commit in commits:
            require(
                isinstance(commit, str) and SHA_PATTERN.fullmatch(commit) is not None,
                f"PR #{record['number']} has an invalid replay commit",
            )
            mapped_commits.add(commit)
            shared_commits.setdefault(commit, set()).add(record["number"])

    expected_shared = {
        "abc839b2fef0ec8cb9b07b75f2ce73207616b375": {
            174, 176, 185, 208, 210,
        },
    }
    actual_shared = {
        commit: prs for commit, prs in shared_commits.items() if len(prs) > 1
    }
    require(actual_shared == expected_shared, "shared replay mapping changed")

    independent = ledger.get("independent_replay_commits")
    require(
        isinstance(independent, list) and len(independent) == 1,
        "expected one independent replay commit",
    )
    independent_record = independent[0]
    independent_sha = independent_record.get("sha")
    require(
        independent_sha == "7aeb36a802188de0e158326c15049ebaaa634ddc",
        "independent replay commit changed",
    )
    mapped_commits.add(independent_sha)
    require(len(mapped_commits) == 73, "mapped replay commit union must contain 73 SHAs")

    adaptations = ledger.get("adaptations")
    require(isinstance(adaptations, list), "missing adaptation records")
    adaptation_sets = {
        tuple(record.get("source_pull_requests", []))
        for record in adaptations
        if isinstance(record, dict)
    }
    require(
        adaptation_sets
        == {
            (243,),
            (229,),
            (218,),
            (174, 176, 185, 208, 210),
            (224,),
        },
        "adaptation set changed",
    )
    return mapped_commits, {independent_sha: independent_record.get("subject", "")}


def validate_available_history(
    ledger: dict[str, Any],
    mapped_commits: set[str],
    recorded_subjects: dict[str, str],
) -> None:
    available = {
        commit
        for commit in mapped_commits
        if git("cat-file", "-e", f"{commit}^{{commit}}").returncode == 0
    }
    if not available:
        print("Replay commits are not present; static provenance ledger verified.")
        return
    require(
        available == mapped_commits,
        "only part of the replay commit set is available in Git",
    )

    for commit in sorted(mapped_commits):
        result = git("show", "-s", "--format=%an%x09%ae%x09%s", commit)
        require(result.returncode == 0, f"cannot inspect replay commit {commit}")
        author_name, author_email, subject = result.stdout.rstrip("\n").split("\t", 2)
        require(
            (author_name, author_email) in EXPECTED_ALLOWED_AUTHORS,
            f"unexpected author on replay commit {commit}",
        )
        if commit in recorded_subjects:
            require(
                subject == recorded_subjects[commit],
                f"independent commit subject changed for {commit}",
            )

    replay = ledger["replay"]
    parent = replay["parent_commit"]
    last = replay["last_commit"]
    count_result = git("rev-list", "--count", f"{parent}..{last}")
    require(count_result.returncode == 0, "cannot count replay history")
    require(
        count_result.stdout.strip() == "73",
        "Git replay range must contain 73 commits",
    )
    tree_result = git("rev-parse", f"{parent}^{{tree}}")
    require(tree_result.returncode == 0, "cannot inspect replay parent tree")
    require(
        tree_result.stdout.strip() == replay["parent_tree"],
        "replay parent tree does not match the recorded PR #252 tree",
    )
    ancestry = git("merge-base", "--is-ancestor", replay["first_commit"], last)
    require(
        ancestry.returncode == 0,
        "first replay commit is not an ancestor of last",
    )
    print("Static ledger and all 73 phase-two replay commits verified.")


def main() -> None:
    ledger = load_ledger()
    require(ledger.get("schema_version") == 1, "unsupported schema version")
    validate_partition(ledger)
    validate_staging_lineage(ledger)
    validate_source_commits(ledger)
    mapped_commits, recorded_subjects = validate_phase_two(ledger)
    validate_available_history(ledger, mapped_commits, recorded_subjects)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Classify a Git diff for CI without changing required check names."""

from __future__ import annotations

import argparse
import subprocess
from dataclasses import dataclass
from pathlib import PurePosixPath


DOC_DIRECTORIES = ("book/", "docs/", "rfcs/")
DOC_FILENAMES = {
    "CHANGELOG",
    "CHANGELOG.md",
    "CONTRIBUTING",
    "CONTRIBUTING.md",
    "LICENSE",
    "LICENSE-APACHE",
    "NOTICE",
    "README",
    "README.md",
}


@dataclass(frozen=True)
class Classification:
    heavy: bool
    docs: bool
    mode: str


def is_documentation(path: str) -> bool:
    normalized = PurePosixPath(path).as_posix()
    parsed = PurePosixPath(normalized)
    return (
        normalized.startswith(DOC_DIRECTORIES)
        or (
            len(parsed.parts) == 1
            and (normalized.endswith(".md") or normalized in DOC_FILENAMES)
        )
    )


def is_lightweight_github_metadata(path: str) -> bool:
    normalized = PurePosixPath(path).as_posix()
    return normalized.startswith(".github/") and not normalized.startswith(
        ".github/workflows/"
    )


def classify_paths(paths: list[str]) -> Classification:
    # An empty or unrecognized diff must receive the complete suite.
    if not paths:
        return Classification(heavy=True, docs=True, mode="full")

    has_docs = any(is_documentation(path) for path in paths)
    lightweight = all(
        is_documentation(path) or is_lightweight_github_metadata(path)
        for path in paths
    )
    if not lightweight:
        return Classification(heavy=True, docs=True, mode="full")

    return Classification(
        heavy=False,
        docs=has_docs,
        mode="docs" if has_docs else "ci-only",
    )


def changed_paths(base: str, head: str) -> list[str]:
    result = subprocess.run(
        ["git", "diff", "--name-only", "-z", base, head],
        check=True,
        capture_output=True,
    )
    return [
        raw.decode("utf-8", errors="surrogateescape")
        for raw in result.stdout.split(b"\0")
        if raw
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", required=True)
    parser.add_argument("--head", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    paths = changed_paths(args.base, args.head)
    result = classify_paths(paths)
    print(f"heavy={str(result.heavy).lower()}")
    print(f"docs={str(result.docs).lower()}")
    print(f"mode={result.mode}")
    print(f"changed_count={len(paths)}")


if __name__ == "__main__":
    main()

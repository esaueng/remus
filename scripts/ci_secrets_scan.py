#!/usr/bin/env python3
"""Scan the commits a CI run adds for secrets; the whole history otherwise.

A pull request, merge group or push names the commit it starts from. Only the
commits after it are new, and every earlier one was scanned by the run that
added it and is scanned again daily (daily-validation.yml). When no usable
base is named (a manual dispatch, a new branch, a rewritten one), every ref is
scanned.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys

ALL_REFS = "--all"
OID = re.compile(r"[0-9a-f]{40}")


def is_commit(oid: str) -> bool:
    return subprocess.run(
        ["git", "cat-file", "-e", f"{oid}^{{commit}}"],
        check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0


def log_opts(bases: list[str], exists=is_commit) -> str:
    """Return gitleaks' `--log-opts`: the first usable base, else every ref."""
    for base in bases:
        if OID.fullmatch(base) and set(base) != {"0"} and exists(base):
            return f"{base}..HEAD"
    return ALL_REFS


def main() -> int:
    opts = log_opts([os.environ.get(name, "") for name in ("MERGE_BASE", "PR_BASE", "PUSH_BASE")])
    print(f"Scanning {'every ref' if opts == ALL_REFS else opts}", flush=True)
    # --redact keeps any match's secret material out of the job log.
    return subprocess.run(["gitleaks", "git", f"--log-opts={opts}", "--redact", "."], check=False).returncode


if __name__ == "__main__":
    sys.exit(main())

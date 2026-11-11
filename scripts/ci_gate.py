#!/usr/bin/env python3
"""Fail closed when any selected CI job is missing, skipped or unsuccessful."""

import json
import os

FLAGS = ("heavy", "full", "wasm", "coverage", "docs")
GROUPS = {
    "always": {"changes", "repo-policy", "secrets-scan"},
    "heavy": {"test", "test-shards", "clippy", "complexity-guards", "approx-census",
              "wasm-no-io", "fuzz-check", "render"},
    "full": {"msrv", "deny", "audit"},
    "wasm": {"wasm", "wasm-runtime", "w9-preflight"},
    "coverage": {"coverage"},
    "docs": {"docs"},
    "size": {"wasm-size-gate"},
}


def validate(results, flags, event):
    if not event or set(flags) != set(FLAGS) or any(v not in ("true", "false") for v in flags.values()):
        raise ValueError("Invalid CI selection flags")
    if any(flags[tier] == "true" and flags["heavy"] != "true" for tier in ("full", "wasm", "coverage")):
        raise ValueError("Inconsistent CI selection flags")
    if set(results) != set.union(*GROUPS.values()):
        raise ValueError("Missing or unknown CI job result")
    for group, jobs in GROUPS.items():
        selected = (group == "always"
                    or (group == "size" and flags["wasm"] == "true" and event == "pull_request")
                    or flags.get(group) == "true")
        for job in jobs:
            result = results[job].get("result")
            if result != "success" and not (result == "skipped" and not selected):
                raise ValueError(f"CI job {job} did not succeed: {result}")


if __name__ == "__main__":
    validate(json.loads(os.environ["NEEDS"]),
             {name: os.environ[name.upper()] for name in FLAGS},
             os.environ["EVENT"])

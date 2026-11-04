#!/usr/bin/env python3
"""Fail closed when any selected daily-CI job is missing, skipped or unsuccessful."""

import json
import os

GROUPS = {
    "always": {"changes", "repo-policy", "secrets-scan"},
    "native": {"test", "test-shards"},
    "heavy": {"clippy", "complexity-guards", "approx-census", "wasm-no-io", "fuzz-check", "render"},
    "full": {"msrv", "deny", "audit"},
    "wasm": {"wasm", "wasm-runtime", "w9-preflight"},
    "coverage": {"coverage"},
    "docs": {"docs"},
    "size": {"wasm-size-gate"},
}


def validate(results, flags, event):
    if not event or set(flags) != {"heavy", "native", "full", "wasm", "coverage", "docs"} or any(v not in ("true", "false") for v in flags.values()):
        raise ValueError("Invalid CI selection flags")
    if ((flags["heavy"] == "true" and flags["native"] != "true")
            or (flags["full"] == "true" and flags["heavy"] != "true")
            or (flags["coverage"] == "true" and flags["native"] != "true")
            or (flags["wasm"] == "true" and flags["heavy"] != "true")):
        raise ValueError("Inconsistent CI selection flags")
    if set(results) != set.union(*GROUPS.values()) | {"queue-trust"}:
        raise ValueError("Missing or unknown CI job result")
    for group, jobs in GROUPS.items():
        selected = (group == "always" or (group == "size" and flags["wasm"] == "true" and event == "pull_request")
                    or (group in flags and flags[group] == "true"))
        for job in jobs:
            result = results[job].get("result")
            if result != "success" and not (result == "skipped" and not selected):
                raise ValueError(f"CI job {job} did not succeed: {result}")


if __name__ == "__main__":
    validate(json.loads(os.environ["NEEDS"]),
             {name: os.environ[name.upper()] for name in ("heavy", "native", "full", "wasm", "coverage", "docs")},
             os.environ["EVENT"])

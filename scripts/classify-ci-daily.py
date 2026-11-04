#!/usr/bin/env python3
"""Select v12 CI tiers independently from optional instrumented coverage."""

import importlib.util
import os
from pathlib import Path
import sys

spec = importlib.util.spec_from_file_location("legacy_classifier", Path(__file__).with_name("classify-ci-changes.py"))
legacy = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = legacy
spec.loader.exec_module(legacy)


def classify(paths, event, force_full=False, coverage=False):
    selected = legacy.classify_paths(paths, force_full=force_full, merge_group=event == "merge_group")
    full = selected.full
    # Main still validates and publishes the exact package artifact. Broad
    # dependency/configuration checks run daily unless their inputs changed.
    if event == "push" and paths:
        full = selected.heavy and any(legacy.needs_second_tier(path) for path in paths)
    native = selected.heavy or selected.mode == "queue"
    return {"heavy": selected.heavy, "docs": selected.docs, "full": full,
            "wasm": selected.wasm, "native": native,
            "coverage": bool(coverage and native), "mode": selected.mode}


def main():
    args = legacy.parse_args()
    paths = legacy.changed_paths(args.base, args.head)
    result = classify(paths, os.environ.get("GITHUB_EVENT_NAME", ""), args.full,
                      os.environ.get("COVERAGE_REQUESTED") == "true")
    for key, value in result.items():
        print(f"{key}={str(value).lower() if isinstance(value, bool) else value}")
    print(f"base={args.base}")
    print(f"changed_count={len(paths)}")


if __name__ == "__main__":
    main()

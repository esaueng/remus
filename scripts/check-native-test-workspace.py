#!/usr/bin/env python3
"""Bind archived tests' compiled fixture paths to their source and checkout."""

import argparse
import json
import os
from pathlib import Path
import re


def validate(receipt, source_sha, workspace):
    expected = {"schema": 1, "source_sha": source_sha,
                "workspace": str(Path(workspace).resolve())}
    if not re.fullmatch(r"[0-9a-f]{40}", source_sha or ""):
        raise ValueError("Native archive requires an exact source commit")
    if receipt != expected or type(receipt.get("schema")) is not int:
        raise ValueError("Native archive source or compiled fixture workspace differs from this checkout")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("receipt", type=Path)
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args()
    try:
        source = os.environ.get("GITHUB_SHA", "")
        workspace = os.environ["GITHUB_WORKSPACE"]
        if Path(workspace).resolve() != Path.cwd().resolve():
            raise ValueError("Native archive command must run at the checkout root")
        if args.record:
            receipt = {"schema": 1, "source_sha": source,
                       "workspace": str(Path.cwd().resolve())}
            validate(receipt, source, workspace)
            args.receipt.write_text(json.dumps(receipt, sort_keys=True) + "\n")
        else:
            if args.receipt.stat().st_size > 4096:
                raise ValueError("Native archive receipt is oversized")
            validate(json.loads(args.receipt.read_text()), source, workspace)
            print("Native archive source and compiled fixture workspace match this checkout")
    except (KeyError, OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Bind coverage profiles to their source/archive and require complete partitions."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def digest(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def identities(inventory):
    return sorted(f"{binary}\0{name}" for binary, suite in inventory["rust-suites"].items()
                  for name, case in suite["testcases"].items()
                  if not case["ignored"] and case["filter-match"]["status"] == "matches")


def record(directory, profiles, inventory, source, archive, shard, total):
    tests = identities(inventory)
    files = sorted(profiles.glob("*.profraw"))
    if not tests or not files or not 1 <= shard <= total:
        raise ValueError("Coverage partition has no tests/profiles or an invalid index")
    directory.mkdir(parents=True, exist_ok=False)
    hashes = {}
    for profile in files:
        shutil.copyfile(profile, directory / profile.name)
        hashes[profile.name] = digest(directory / profile.name)
    receipt = {"schema": 1, "source": source, "archive": digest(archive),
               "shard": shard, "total": total, "tests": tests, "profiles": hashes}
    (directory / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")


def verify(directories, inventory, source, archive, total):
    if len(directories) != total:
        raise ValueError("Missing coverage partition")
    seen_shards, seen_tests = set(), set()
    expected = set(identities(inventory))
    verified = []
    for directory in directories:
        receipt = json.loads((directory / "receipt.json").read_text())
        if set(receipt) != {"schema", "source", "archive", "shard", "total", "tests", "profiles"}:
            raise ValueError("Malformed coverage receipt")
        shard = receipt["shard"]
        if (type(receipt["schema"]) is not int or receipt["schema"] != 1
                or type(shard) is not int or not 1 <= shard <= total or shard in seen_shards
                or type(receipt["total"]) is not int or receipt["total"] != total
                or receipt["source"] != source or receipt["archive"] != digest(archive)):
            raise ValueError("Coverage source, archive or partition does not match")
        tests = receipt["tests"]
        if not isinstance(tests, list) or not tests or len(set(tests)) != len(tests) or seen_tests.intersection(tests):
            raise ValueError("Duplicate or empty coverage test partition")
        seen_tests.update(tests)
        seen_shards.add(shard)
        hashes = receipt["profiles"]
        if not isinstance(hashes, dict) or not hashes or set(hashes) != {p.name for p in directory.glob("*.profraw")}:
            raise ValueError("Missing or extra coverage profile")
        for name, sha in hashes.items():
            if Path(name).name != name or not name.endswith(".profraw") or (directory / name).is_symlink() or digest(directory / name) != sha:
                raise ValueError("Invalid coverage profile content")
            verified.append((shard, directory / name))
    if seen_tests != expected:
        raise ValueError("Coverage partitions do not equal the complete runnable suite")
    return verified


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("record", "merge"))
    parser.add_argument("--inventory", required=True, type=Path)
    parser.add_argument("--archive", required=True, type=Path)
    parser.add_argument("--profiles", required=True, type=Path)
    parser.add_argument("--artifacts", required=True, type=Path)
    parser.add_argument("--shard", type=int)
    parser.add_argument("--total", type=int, default=4)
    args = parser.parse_args()
    source = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    inventory = json.loads(args.inventory.read_text())
    if args.command == "record":
        record(args.artifacts, args.profiles, inventory, source, args.archive, args.shard, args.total)
    else:
        directories = sorted(p for p in args.artifacts.iterdir() if p.is_dir())
        profiles = verify(directories, inventory, source, args.archive, args.total)
        if list(args.profiles.glob("*.profraw")):
            raise ValueError("Refusing to merge into stale coverage profiles")
        args.profiles.mkdir(parents=True, exist_ok=True)
        for shard, path in profiles:
            shutil.copyfile(path, args.profiles / f"shard-{shard}-{path.name}")
        print(f"Verified {args.total} partitions covering {len(identities(inventory))} tests")


if __name__ == "__main__":
    main()

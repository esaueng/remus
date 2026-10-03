#!/usr/bin/env python3
"""Compare clean rebuilt package trees with an immutable committed revision."""

import argparse
import json
from pathlib import Path
import subprocess

ROOTS = ("crates/wasm/pkg", "crates/wasm-io/pkg")
REPOSITORY = "https://github.com/esaueng/remus"


def manifest_contract(data, generated=False):
    package = json.loads(data)
    # The publisher stamps these exact fields after xtask finishes. Validate
    # their committed values rather than ignoring attacker-supplied metadata.
    provenance = {"homepage": REPOSITORY,
                  "repository": {"type": "git", "url": "git+" + REPOSITORY + ".git"}}
    if generated:
        package.update(provenance)
    elif any(package.get(key) != value for key, value in provenance.items()):
        raise ValueError("Committed package provenance does not match the publisher")
    # Release numbering has its own gate; xtask may advance it when it detects
    # rebuilt output. All other fields and nested export order must match.
    package.pop("version", None)
    return json.dumps({key: package[key] for key in sorted(package)})


def check(revision, directory=Path(".")):
    def git(*args):
        return subprocess.check_output(["git", *args], cwd=directory)

    # Resolve first so all reads use the same immutable commit and cannot
    # accidentally interpret a supplied ref as an option.
    commit = git("rev-parse", "--verify", "--end-of-options", revision + "^{commit}").decode().strip()
    mismatches = []
    for root in ROOTS:
        committed = {}
        for record in git("ls-tree", "-r", "-z", commit, "--", root).split(b"\0"):
            if not record:
                continue
            metadata, raw_name = record.split(b"\t", 1)
            mode, kind, _ = metadata.split()
            name = raw_name.decode()
            if mode != b"100644" or kind != b"blob":
                raise ValueError(f"Unsupported committed package entry: {name}")
            committed[name] = git("show", f"{commit}:{name}")
        if not committed:
            raise ValueError(f"Committed package is missing: {root}")
        package = directory / root
        if package.is_symlink() or not package.is_dir():
            raise ValueError(f"Rebuilt package is missing or a symlink: {root}")
        rebuilt = {}
        for path in package.rglob("*"):
            if path.is_symlink():
                raise ValueError(f"Symlink in rebuilt package: {path}")
            if path.is_file():
                rebuilt[path.relative_to(directory).as_posix()] = path.read_bytes()
        for name in sorted(committed.keys() | rebuilt.keys()):
            before, after = committed.get(name), rebuilt.get(name)
            label = name
            if before is not None and after is not None and name.endswith("/package.json"):
                expected = manifest_contract(before)
                actual = manifest_contract(after, generated=True)
                equal = expected == actual
                if not equal:
                    expected_fields, actual_fields = json.loads(expected), json.loads(actual)
                    changed = [key for key in sorted(expected_fields.keys() | actual_fields.keys())
                               if key not in expected_fields or key not in actual_fields or
                               json.dumps(expected_fields[key]) != json.dumps(actual_fields[key])]
                    label += " (manifest fields: " + ", ".join(changed) + ")"
            else:
                equal = before is not None and after is not None and before == after
            if not equal:
                mismatches.append(label)
    if mismatches:
        raise ValueError("Committed packages differ from the source rebuild: " + ", ".join(mismatches))
    print("Committed WASM package integrity passed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--revision", default="HEAD")
    check(parser.parse_args().revision)

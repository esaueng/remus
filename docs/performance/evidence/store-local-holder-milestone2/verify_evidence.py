#!/usr/bin/env python3
"""Verify the shared evidence inventory, without running builds or benchmarks."""

import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parent
    manifest_name = "EVIDENCE-MANIFEST.json"
    manifest = json.loads((root / manifest_name).read_text())
    expected = manifest["files"]
    actual = {
        file.relative_to(root).as_posix()
        for file in root.rglob("*")
        if file.is_file() and file != root / manifest_name
    }
    assert actual == set(expected), "Evidence file inventory changed"
    for name, record in expected.items():
        relative = Path(name)
        assert not relative.is_absolute() and ".." not in relative.parts
        file = root / relative
        assert not file.is_symlink(), name
        payload = file.read_bytes()
        assert len(payload) == record["bytes"], name
        assert hashlib.sha256(payload).hexdigest() == record["sha256"], name
    print(f"Verified {len(expected)} shared evidence files.")


if __name__ == "__main__":
    main()

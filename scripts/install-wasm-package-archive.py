#!/usr/bin/env python3
"""Install generated WASM packages from a narrowly scoped tar archive.

Usage: install-wasm-package-archive.py ARCHIVE ROOT [ROOT ...]

Every ROOT (for example ``crates/wasm/pkg``) is both the archive prefix the
member must sit under and the checkout directory it is installed into.
Members outside every ROOT are refused.
"""

import os
import shutil
import sys
import tarfile
from pathlib import Path, PurePosixPath


def owning_root(path: PurePosixPath, roots: list[PurePosixPath]) -> PurePosixPath | None:
    for root in roots:
        if path == root or root in path.parents:
            return root
    return None


def main() -> None:
    archive = Path(sys.argv[1])
    roots = [PurePosixPath(root) for root in sys.argv[2:]]
    if not roots:
        raise ValueError("at least one package root is required")
    seen: set[PurePosixPath] = set()

    with tarfile.open(archive, "r:gz") as package:
        members = package.getmembers()
        for member in members:
            path = PurePosixPath(member.name)
            if (
                path in seen
                or path.is_absolute()
                or ".." in path.parts
                or owning_root(path, roots) is None
                or not (member.isdir() or member.isreg())
            ):
                raise ValueError(f"unsafe archive member: {member.name!r}")
            seen.add(path)

        for root in roots:
            if not any(path != root and root in path.parents for path in seen):
                raise ValueError(f"archive contains no package files under {root}")

        for root in roots:
            destination = Path(*root.parts)
            shutil.rmtree(destination, ignore_errors=True)
            destination.mkdir(parents=True)
        for member in members:
            path = PurePosixPath(member.name)
            root = owning_root(path, roots)
            assert root is not None
            target = Path(*root.parts).joinpath(*path.relative_to(root).parts)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            source = package.extractfile(member)
            if source is None:
                raise ValueError(f"could not read archive member: {member.name!r}")
            with source, target.open("wb") as output:
                shutil.copyfileobj(source, output)
            os.chmod(target, 0o755 if member.mode & 0o111 else 0o644)


if __name__ == "__main__":
    main()

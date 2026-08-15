#!/usr/bin/env python3
"""Install a generated WASM package from a narrowly scoped tar archive."""

import os
import shutil
import sys
import tarfile
from pathlib import Path, PurePosixPath


ARCHIVE_ROOT = PurePosixPath("crates/wasm/pkg")


def main() -> None:
    archive = Path(sys.argv[1])
    destination = Path(sys.argv[2])
    seen: set[PurePosixPath] = set()

    with tarfile.open(archive, "r:gz") as package:
        members = package.getmembers()
        for member in members:
            path = PurePosixPath(member.name)
            if (
                path in seen
                or path.is_absolute()
                or ".." in path.parts
                or (path != ARCHIVE_ROOT and ARCHIVE_ROOT not in path.parents)
                or not (member.isdir() or member.isreg())
            ):
                raise ValueError(f"unsafe archive member: {member.name!r}")
            seen.add(path)

        if not any(path != ARCHIVE_ROOT for path in seen):
            raise ValueError("archive contains no package files")

        shutil.rmtree(destination, ignore_errors=True)
        destination.mkdir(parents=True)
        for member in members:
            relative = PurePosixPath(member.name).relative_to(ARCHIVE_ROOT)
            target = destination.joinpath(*relative.parts)
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

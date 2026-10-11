#!/usr/bin/env python3
"""Check the scoped Truck reuse ledger and notices without network access."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
LEDGER = ROOT / "docs/production-readiness/truck-reuse-provenance.json"
PIN = "88ed005249e5e3a6b07f62425399435905cd3ab6"
SOURCE_FILES = {
    "LICENSE",
    "truck-geometry/Cargo.toml",
    "truck-geometry/src/nurbs/bspsurface.rs",
    "truck-geometry/src/nurbs/bspcurve.rs",
}
DESTINATION_FILES = {
    "crates/math/src/nurbs/reduction.rs",
    "crates/math/src/nurbs/cubic_fit.rs",
}
FUNCTIONS = {
    "BSplineSurface::try_remove_uknot": ("bspsurface.rs", 851, 897),
    "BSplineSurface::try_remove_vknot": ("bspsurface.rs", 949, 996),
    "BSplineSurface::optimize": ("bspsurface.rs", 1542, 1557),
    "BSplineCurve::optimize": ("bspcurve.rs", 714, 723),
    "BSplineCurve::cubic_bezier_interpolation": ("bspcurve.rs", 1287, 1301),
    "BSplineCurve::sub_cubic_approximation": ("bspcurve.rs", 1303, 1355),
    "BSplineCurve::cubic_approximation": ("bspcurve.rs", 1373, 1391),
}


def require(condition, message):
    """Fail with a scoped diagnostic instead of accepting an incomplete record."""
    if not condition:
        raise ValueError(message)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def validate(truck_source=None):
    ledger = json.loads(LEDGER.read_text())
    require(ledger.get("schema") == 1, "unexpected ledger schema")
    upstream = ledger["upstream"]
    require(upstream["repository"] == "https://github.com/ricosjp/truck", "unexpected upstream")
    require(upstream["commit"] == PIN, "upstream source pin changed")
    require(upstream["license"] == "Apache-2.0", "upstream is not Apache-2.0")
    require(upstream["license_file"] == "LICENSE", "unexpected upstream license file")
    require(upstream["package_manifest"] == "truck-geometry/Cargo.toml", "unexpected manifest")
    require(upstream["notice_files"] == [], "upstream NOTICE audit changed")
    require(set(ledger["file_sha256"]) == SOURCE_FILES, "source extraction boundary changed")
    require(
        all(re.fullmatch(r"[a-f0-9]{64}", digest) for digest in ledger["file_sha256"].values()),
        "invalid upstream SHA-256 digest",
    )
    for key in (
        "copied_upstream_test_files", "copied_fixture_files", "extracted_helper_files", "new_dependencies"
    ):
        require(ledger[key] == [], f"unreviewed extraction expansion: {key}")
    require(set(ledger["independent_files"]) == {
        "crates/math/src/nurbs/reuse.rs", "crates/math/src/nurbs/reuse_bounds.rs"
    }, "independent certificate boundary changed")
    require(ledger["excluded_sources"] == ["OCCT", "Remus historical upstream v3 or later"],
            "excluded source boundary changed")

    adaptations = ledger["adaptations"]
    require(len(adaptations) == len(FUNCTIONS), "adaptation inventory count changed")
    require({item["source_function"] for item in adaptations} == set(FUNCTIONS),
            "adaptation function boundary changed")
    require({item["destination_file"] for item in adaptations} == DESTINATION_FILES,
            "adapted destination boundary changed")
    for item in adaptations:
        basename, start, end = FUNCTIONS[item["source_function"]]
        require(item["source_file"] == f"truck-geometry/src/nurbs/{basename}", "source mapping changed")
        require(item["source_lines"] == [start, end], "source function range changed")
        destination = "cubic_fit.rs" if "cubic" in item["source_function"] else "reduction.rs"
        require(item["destination_file"] == f"crates/math/src/nurbs/{destination}",
                "source-to-destination mapping changed")
        require(item["license"] == "Apache-2.0", "adaptation license changed")
        require(re.fullmatch(r"[a-f0-9]{64}", item["source_range_sha256"]), "invalid range digest")
        require(bool(item["scope"]), "adaptation scope is empty")

    license_text = (ROOT / "LICENSE-APACHE").read_text()
    require("Apache License" in license_text and "Version 2.0, January 2004" in license_text,
            "Remus Apache license copy is missing")
    notice = (ROOT / "NOTICE").read_text()
    require("brepkit contributors" in notice and "Esau Engineering" in notice,
            "existing Remus attribution was removed")
    require("Truck contributors" in notice and PIN in notice and "Apache License, Version 2.0" in notice,
            "Truck attribution or pinned source notice is missing")
    for name in ("LICENSE-APACHE", "NOTICE"):
        require((ROOT / "crates/math" / name).read_bytes() == (ROOT / name).read_bytes(),
                f"remus-math source distribution has stale or missing {name}")
    for relative in DESTINATION_FILES:
        header = "\n".join((ROOT / relative).read_text().splitlines()[:30])
        require("Truck" in header and PIN in header and "Apache-2.0" in header,
                f"adapted-file license/source notice missing: {relative}")
        require("modified" in header.lower() or "adapted" in header.lower(),
                f"adapted-file change notice missing: {relative}")
        require("truck-reuse-provenance.md" in header,
                f"adapted-file provenance pointer missing: {relative}")
    for relative in ledger["independent_files"]:
        require((ROOT / relative).is_file(), f"independent implementation missing: {relative}")

    if truck_source is None:
        print("Truck reuse ledger and Remus notices verified; upstream bytes not re-read.")
        return

    source = truck_source.resolve()
    commit = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    require(commit == PIN, "local Truck checkout is not at the pinned commit")
    source_bytes = {}
    for relative, digest in ledger["file_sha256"].items():
        data = (source / relative).read_bytes()
        require(sha256(data) == digest, f"upstream file bytes changed: {relative}")
        source_bytes[relative] = data
    for item in adaptations:
        start, end = item["source_lines"]
        lines = source_bytes[item["source_file"]].splitlines(keepends=True)
        require(sha256(b"".join(lines[start - 1:end])) == item["source_range_sha256"],
                f"upstream function bytes changed: {item['source_function']}")
    require(b'license = "Apache-2.0"' in source_bytes["truck-geometry/Cargo.toml"],
            "Truck package Apache declaration missing")
    require(b"Apache License" in source_bytes["LICENSE"] and b"Version 2.0" in source_bytes["LICENSE"],
            "Truck Apache license copy missing")
    tracked = subprocess.check_output(["git", "-C", str(source), "ls-files"], text=True).splitlines()
    require(not any(Path(path).name.upper().startswith("NOTICE") for path in tracked),
            "new upstream NOTICE file requires review")
    print("Truck reuse ledger, Remus notices, pinned upstream files and function digests verified.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--truck-source", type=Path, help="existing local Truck checkout at the pinned commit")
    args = parser.parse_args()
    try:
        validate(args.truck_source)
    except (KeyError, ValueError, OSError, subprocess.CalledProcessError) as error:
        print(f"Truck reuse provenance violation: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

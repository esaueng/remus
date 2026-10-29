"""Check remus STEP exports with an independent CAD reader (FreeCAD).

Optional, local-only tooling: CI does not run it. It catches exports that
remus reads back fine but other CAD systems reject, such as edges whose
geometry is a TRIMMED_CURVE (faces import with no edges, unbounded, and the
solid has no volume).

Generate the exports, then run this with FreeCAD's bundled Python:

    REMUS_STEP_CONFORMANCE_DIR=/tmp/remus-step \
        cargo test -p remus-io --test step_edge_geometry_conformance
    "C:/Program Files/FreeCAD 1.1/bin/python.exe" -I \
        scripts/check-step-external.py /tmp/remus-step/*.step

A `<name>.volume` file next to `<name>.step` holds the expected volume in
mm^3. A file passes when its shape is valid, has at least one solid, every
shell is closed, every face has edges, the bounding box is finite and the
total solid volume matches (relative 1e-6). Prints a Markdown table and exits
non-zero when any file fails.
"""

import math
import os
import sys

try:
    import FreeCAD  # noqa: F401  (initialises the Part module)
    import Part
except ImportError:  # pragma: no cover - depends on the local install
    sys.exit("run this with FreeCAD's python (FreeCAD and Part must import)")

VOLUME_TOLERANCE = 1e-6
BOUND_LIMIT = 1e12


def check(path):
    shape = Part.Shape()
    shape.read(path)
    box = shape.BoundBox
    bounds = (box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax)
    volume = sum(solid.Volume for solid in shape.Solids)
    expected = None
    volume_file = os.path.splitext(path)[0] + ".volume"
    if os.path.exists(volume_file):
        with open(volume_file, encoding="utf-8") as handle:
            expected = float(handle.read().strip())
    problems = []
    if not shape.isValid():
        problems.append("invalid")
    if not shape.Solids:
        problems.append("no solid")
    if not all(shell.isClosed() for shell in shape.Shells):
        problems.append("open shell")
    if any(len(face.Edges) == 0 for face in shape.Faces):
        problems.append("face without edges")
    if not all(math.isfinite(v) and abs(v) < BOUND_LIMIT for v in bounds):
        problems.append("unbounded")
    if expected is not None and abs(volume - expected) > VOLUME_TOLERANCE * max(
        abs(expected), 1.0
    ):
        problems.append("volume")
    return {
        "file": os.path.basename(path),
        "valid": shape.isValid(),
        "closed": all(shell.isClosed() for shell in shape.Shells),
        "faces": len(shape.Faces),
        "bounds": bounds,
        "volume": volume,
        "expected": expected,
        "problems": problems,
    }


def main(paths):
    if not paths:
        sys.exit(__doc__)
    rows = [check(path) for path in paths]
    print("| File | Valid | Closed | Faces | Finite bounds | Volume (mm^3) | Expected | Result |")
    print("|---|---|---|---|---|---|---|---|")
    for row in rows:
        finite = "unbounded" not in row["problems"]
        expected = "-" if row["expected"] is None else f"{row['expected']:.6f}"
        result = "pass" if not row["problems"] else "FAIL: " + ", ".join(row["problems"])
        print(
            f"| {row['file']} | {row['valid']} | {row['closed']} | {row['faces']} | "
            f"{finite} | {row['volume']:.6f} | {expected} | {result} |"
        )
    return 1 if any(row["problems"] for row in rows) else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

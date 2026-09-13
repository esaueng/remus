#!/usr/bin/env python3
"""Generate the performance specification exports from the master roadmap."""

import argparse
import csv
import io
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MASTER = ROOT / "docs/kernel-maturity/roadmap.md"
FIELDS = (
    "id", "area", "priority", "phase", "evidence", "work", "acceptance",
    "effort_risk", "dependencies", "source_references",
)


def parse_inventory(markdown):
    """Read package specifications and reject ambiguous or dangling ownership."""
    anchors = re.findall(r'<a id="([^"]+)"></a>', markdown)
    if len(anchors) != len(set(anchors)):
        raise ValueError("duplicate roadmap anchor")
    inventory = markdown.split("## Performance work packages\n", 1)[1]
    inventory = inventory.split("## Decisions and exclusions\n", 1)[0]
    rows = []
    area = None
    references = []
    for line in inventory.splitlines():
        if line.startswith("### "):
            area = line.removeprefix("### ")
            references = []
        if line.startswith("Inspected entry points: "):
            references = re.findall(r"\]\((https://[^)]+)\)", line)
        if not line.startswith('| <a id="perf-'):
            continue
        cells = [c.strip().replace(r"\|", "|") for c in re.split(r"(?<!\\)\|", line)[1:-1]]
        if len(cells) != 8 or not area or not references:
            raise ValueError(f"malformed performance row: {line}")
        ident = re.fullmatch(r'<a id="perf-([a-z]\d{2})"></a>PERF-([A-Z]\d{2})', cells[0])
        if not ident or ident[1] != ident[2].lower():
            raise ValueError(f"invalid package ID/anchor: {cells[0]}")
        priority, phase, evidence = cells[1].split(" / ")
        if priority not in {"P0", "P1", "P2", "P3"} or phase not in set("012345678") or evidence not in {"M", "S", "H"}:
            raise ValueError(f"invalid priority/phase/evidence: {cells[1]}")
        owner = re.fullmatch(r"\[([^]]+)\]\(#([^)]+)\)", cells[6])
        if not owner or owner[2] not in anchors or owner[2].startswith("perf-"):
            raise ValueError(f"missing implementation owner for {ident[2]}")
        if any(not cells[i] for i in (2, 3, 4, 7)):
            raise ValueError(f"missing specification or state for {ident[2]}")
        rows.append(dict(zip(FIELDS, (
            ident[2], area, priority, int(phase), evidence, cells[2], cells[3],
            cells[4], [] if cells[5] == "—" else cells[5].split(", "), references,
        ))))
    by_id = {row["id"]: row for row in rows}
    if not rows or len(by_id) != len(rows):
        raise ValueError("empty inventory or duplicate performance ID")
    visited = set()
    active = set()

    def visit(ident):
        if ident not in by_id:
            raise ValueError(f"unknown dependency: {ident}")
        if ident in active:
            raise ValueError(f"dependency cycle through {ident}")
        if ident in visited:
            return
        active.add(ident)
        for dependency in by_id[ident]["dependencies"]:
            visit(dependency)
        active.remove(ident)
        visited.add(ident)

    for ident in by_id:
        visit(ident)
    return rows


def exports(rows):
    csv_buffer = io.StringIO(newline="")
    writer = csv.DictWriter(csv_buffer, fieldnames=FIELDS, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow({
            **row,
            "dependencies": "; ".join(row["dependencies"]),
            "source_references": "; ".join(row["source_references"]),
        })
    # Keep the established compact dependency lists in the JSON export.
    json_text = json.dumps(rows, ensure_ascii=False, indent=2)
    json_text = re.sub(
        r'"dependencies": \[\s*([^\[\]]*?)\s*\]',
        lambda match: '"dependencies": [' + " ".join(match[1].split()) + "]",
        json_text,
    )
    return {
        "optimization-backlog.json": json_text + "\n",
        "optimization-backlog.csv": csv_buffer.getvalue(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if exports differ")
    args = parser.parse_args()
    try:
        rows = parse_inventory(MASTER.read_text())
    except (ValueError, IndexError) as error:
        parser.exit(1, f"Invalid master roadmap: {error}\n")
    stale = []
    for name, expected in exports(rows).items():
        path = ROOT / "docs/performance" / name
        if args.check:
            if not path.exists() or path.read_text() != expected:
                stale.append(name)
        else:
            path.write_text(expected)
    if stale:
        parser.exit(1, "Stale exports: " + ", ".join(stale) + "\nRun python3 scripts/sync-roadmap-inventory.py\n")
    print(f"Roadmap inventory OK: {len(rows)} packages, valid owners and acyclic dependencies")


if __name__ == "__main__":
    main()

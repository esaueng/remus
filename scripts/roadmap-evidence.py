#!/usr/bin/env python3
"""Check or fold roadmap evidence fragments into the master roadmap.

Each PR records its delivery evidence in its own file under
`docs/kernel-maturity/roadmap-evidence/` instead of editing its roadmap row,
and a later fold PR appends every fragment to its row in one commit.

The roadmap's rows are single-line table cells, so two PRs that append to the
same row -- or to adjacent rows -- always conflict. With seventeen open PRs
touching the roadmap, every merge made the rest un-mergeable. Fragments are
separate files, so they never conflict.

A fragment is Markdown with a small front matter block:

    ---
    row: 8.1
    state: Partial — stage-3 campaign remaining
    ---
    Evidence text, appended to the row's evidence cell.

`row` names the roadmap row by its anchor id (`p-8-1`, `perf-n04`, `b82`) or
its displayed ID (`8.1`, `PERF-N04`, `B82`); it must match exactly one table
row. `state` is optional and replaces the row's state cell outright, so two
fragments that set the state of one row are refused rather than ordered by file
name.

    python3 scripts/roadmap-evidence.py          # check
    python3 scripts/roadmap-evidence.py --fold   # apply and delete fragments
"""

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROADMAP_PATH = "docs/kernel-maturity/roadmap.md"
FRAGMENT_DIR = "docs/kernel-maturity/roadmap-evidence"

# Fragment files: `<row>--<slug>.md`, lowercase, so the row shows in a listing.
FRAGMENT_NAME = re.compile(r"^[a-z0-9][a-z0-9.-]*--[a-z0-9][a-z0-9-]*\.md$")
FRONT_MATTER = re.compile(r"^---\r?\n(.*?)\r?\n---\r?\n(.*)$", re.DOTALL)
FIELD = re.compile(r"^([a-z]+):\s*(.*?)\s*$")
FRONT_MATTER_KEYS = {"row", "state"}
EMPTY_CELLS = {"", "—", "-"}
ANCHOR = re.compile(r'<a id="([^"]+)"></a>')
DELIMITER = re.compile(r"^\|(\s*:?-+:?\s*\|)+\s*$")


class FragmentError(ValueError):
    """A fragment the fold cannot place; nothing is written."""


def parse_fragment(name, source):
    """Parse one fragment, refusing anything the fold could not place."""
    match = FRONT_MATTER.match(source)
    if not match:
        raise FragmentError(f"{name}: missing front matter (--- row: <ID> ---)")
    fields = {}
    for line in match[1].splitlines():
        if not line.strip():
            continue
        field = FIELD.match(line)
        if not field or field[1] not in FRONT_MATTER_KEYS:
            raise FragmentError(f'{name}: unknown front matter line "{line}"')
        if field[1] in fields:
            raise FragmentError(f'{name}: duplicate front matter key "{field[1]}"')
        fields[field[1]] = field[2]
    if not fields.get("row"):
        raise FragmentError(f"{name}: front matter needs a row")
    if "state" in fields and not fields["state"]:
        raise FragmentError(f"{name}: state is empty")
    evidence = " ".join(match[2].split())
    if not evidence:
        raise FragmentError(f"{name}: no evidence text")
    return {"name": name, "row": fields["row"], "state": fields.get("state"), "evidence": evidence}


def split_row(line):
    """Split a table row into cells; a pipe in a code span or escaped as `\\|` is content."""
    cells, cell, in_code, i = [], "", False, 0
    while i < len(line):
        ch = line[i]
        if ch == "\\" and line[i + 1:i + 2] == "|":
            cell += "\\|"
            i += 1
        elif ch == "`":
            in_code = not in_code
            cell += ch
        elif ch == "|" and not in_code:
            cells.append(cell)
            cell = ""
        else:
            cell += ch
        i += 1
    cells.append(cell)
    # Drop the empty strings outside the leading and trailing pipes.
    return [c.strip() for c in cells[1:-1]]


def join_row(cells):
    return "| " + " | ".join(cells) + " |"


def is_table_row(line):
    return line.startswith("|") and line.rstrip().endswith("|")


def header_for(lines, index):
    """Header cells of the table holding `lines[index]`, or None outside a table body."""
    i = index
    while i > 0 and is_table_row(lines[i - 1]) and not DELIMITER.match(lines[i - 1]):
        i -= 1
    if i < 2 or not DELIMITER.match(lines[i - 1]) or not is_table_row(lines[i - 2]):
        return None
    return split_row(lines[i - 2])


def last_index(cells, test):
    for i in range(len(cells) - 1, -1, -1):
        if test(cells[i]):
            return i
    return -1


def state_column(header):
    return next((i for i, h in enumerate(header) if re.match(r"(status|state)\b", h, re.I)), -1)


def evidence_column(header):
    """The cell that receives evidence: the last evidence column, else outcome, else state.

    A slash-separated header such as "Priority / phase / evidence" holds codes
    that `sync-roadmap-inventory.py` parses, not prose, so it never receives text.
    """
    evidence = last_index(header, lambda h: re.search("evidence", h, re.I) and "/" not in h)
    if evidence >= 0:
        return evidence
    outcome = last_index(header, lambda h: re.match(r"outcome\b", h, re.I))
    if outcome >= 0:
        return outcome
    return state_column(header)


def row_index(lines):
    """Map every name a table body row answers to (anchor id, displayed ID, first word) to its indices.

    Displayed IDs of anchored rows are kept apart (`@` prefix) because summary
    tables repeat an owner row's ID in their first column; the owner row wins.
    """
    index = {}
    for number, line in enumerate(lines):
        if not is_table_row(line) or DELIMITER.match(line) or not header_for(lines, number):
            continue
        cells = split_row(line)
        first = cells[0] if cells else ""
        anchor = ANCHOR.search(first)
        shown = ANCHOR.sub("", first).strip()
        prefix = "@" if anchor else ""
        keys = {f"{prefix}{key}" for key in {shown, *shown.split()[:1]} if key}
        if anchor:
            keys.add(f"#{anchor[1]}")
        for key in keys:
            index.setdefault(key, []).append(number)
    return index


def find_rows(index, row):
    """Indices of the table rows a fragment's `row` names, by anchor id or displayed ID."""
    wanted = row.strip()
    hits = set(index.get(f"#{wanted.lower()}", []) + index.get(f"@{wanted}", []))
    return sorted(hits) if hits else sorted(index.get(wanted, []))


def escape_cell(text):
    return re.sub(r"(?<!\\)\|", r"\\|", text)


def fold_evidence(markdown, fragments):
    """Apply fragments in file-name order; raise, changing nothing, if any cannot be placed."""
    lines = markdown.split("\n")
    rows = row_index(lines)
    errors, placed, state_setters = [], [], {}
    for fragment in sorted(fragments, key=lambda f: f["name"]):
        name, row = fragment["name"], fragment["row"]
        hits = find_rows(rows, row)
        if len(hits) != 1:
            errors.append(f'{name}: row "{row}" matches {len(hits)} roadmap rows, expected 1')
            continue
        index = hits[0]
        header = header_for(lines, index)
        width = len(split_row(lines[index]))
        if width != len(header):
            # An unescaped `|` in the row's text (e.g. `|x|`) shifts every
            # later cell, so the fold would write into the wrong column.
            errors.append(
                f'{name}: row "{row}" has {width} cells but its table has {len(header)}; '
                "escape the raw | in that row as \\| first"
            )
            continue
        target, state = evidence_column(header), state_column(header)
        if target < 0:
            errors.append(f'{name}: row "{row}" has no evidence column')
            continue
        if fragment.get("state") is not None:
            if state < 0:
                errors.append(f'{name}: row "{row}" has no state column')
                continue
            if index in state_setters:
                errors.append(f'{name}: {state_setters[index]} already sets the state of row "{row}"')
                continue
            state_setters[index] = name
        placed.append((fragment, index, target, state))
    if errors:
        raise FragmentError("\n".join(errors))
    # States first, then every body: where a table has no Evidence column the
    # state cell also receives evidence, and a state applied after an earlier
    # fragment's body would overwrite that body.
    row_cells = {}
    for fragment, index, _, state in placed:
        cells = row_cells.setdefault(index, split_row(lines[index]))
        if fragment.get("state") is not None:
            cells[state] = escape_cell(fragment["state"])
    for fragment, index, target, _ in placed:
        cells = row_cells[index]
        text = escape_cell(fragment["evidence"])
        cells[target] = text if cells[target] in EMPTY_CELLS else f"{cells[target]} {text}"
    for index, cells in row_cells.items():
        lines[index] = join_row(cells)
    return "\n".join(lines)


def read_fragments(root=ROOT):
    directory = Path(root) / FRAGMENT_DIR
    if not directory.is_dir():
        return []
    errors, fragments = [], []
    # Every entry is checked, so a misnamed `.MD` or extensionless fragment is
    # refused instead of silently skipped.
    for path in sorted(directory.iterdir()):
        if path.name == "README.md":
            continue
        if not path.is_file() or not FRAGMENT_NAME.match(path.name):
            errors.append(f"{path.name}: name must be <row>--<slug>.md, lowercase")
            continue
        try:
            fragments.append(parse_fragment(path.name, path.read_text(encoding="utf-8")))
        except FragmentError as error:
            errors.append(str(error))
    if errors:
        raise FragmentError("\n".join(errors))
    return fragments


def check_repo(root=ROOT):
    """Validate every fragment against the roadmap without writing anything."""
    fragments = read_fragments(root)
    fold_evidence((Path(root) / ROADMAP_PATH).read_text(encoding="utf-8"), fragments)
    return fragments


def fold_repo(root=ROOT):
    """Fold every fragment into the roadmap and delete the folded files."""
    fragments = read_fragments(root)
    roadmap = Path(root) / ROADMAP_PATH
    roadmap.write_text(fold_evidence(roadmap.read_text(encoding="utf-8"), fragments), encoding="utf-8")
    for fragment in fragments:
        (Path(root) / FRAGMENT_DIR / fragment["name"]).unlink()
    return fragments


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fold", action="store_true", help="apply fragments and delete them")
    args = parser.parse_args()
    try:
        fragments = fold_repo() if args.fold else check_repo()
    except FragmentError as error:
        print(error, file=sys.stderr)
        return 1
    print(f"{'Folded' if args.fold else 'Checked'} {len(fragments)} roadmap evidence fragment(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())

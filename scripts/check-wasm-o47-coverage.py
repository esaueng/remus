#!/usr/bin/env python3
"""O4.7 typed-twin coverage gate for mutating WASM methods.

Discovers the actual public JavaScript exports from the WASM bindings
(`crates/wasm/src`, honoring `js_name` attributes and the `io` feature
gate), classifies every mutating export, and checks it against the
committed baseline (`scripts/wasm-o47-coverage-baseline.json`):

* a discovered mutating export with no baseline row fails
  (new unclassified mutation);
* a baseline row with no discovered export fails
  (stale exception / removed export);
* a row marked covered fails unless its declared typed twin exists with
  the solid-envelope return type, every required batch dispatch path
  exists, and every named runtime witness exists (falsely claimed
  coverage);
* source that looks like an export but parses in no known shape fails
  loudly instead of silently disappearing from discovery.

Queries (`&self`) never need twins and carry no rows. Lifecycle/session
mutations and already-typed special cases carry rows with an owner and a
reason but no twin requirement. Non-solid mutations are never forced
into the solid-envelope schema: only rows with `schema ==
"solid_envelope"` must return the solid envelope, and a covered legacy
method must itself return a single solid handle.

Discovery mirrors the existing source-discovery tooling: crate-relative
file walking in the style of `scripts/check-doc-module-map.py` (the whole
working tree, so uncommitted additions are caught too) with the
`check-wildcard-arms.sh` discipline that a scan failure exits 2 (the gate
could not run) rather than vouching for a tree it could not scan.
Growth/shrinkage semantics distinguish VIOLATION
(a live defect) from STALE (the baseline needs updating in the same PR).

This check is cheap and deterministic: working-tree file reads only,
no cargo build, no network, sorted output. Runtime correctness itself is
proven by `cargo test -p remus-wasm` (the named witnesses) and
`node scripts/test-wasm-smoke.mjs` (the installed-package witnesses);
source-name matching here establishes that the witnesses exist, never
that they pass.

Usage: python3 scripts/check-wasm-o47-coverage.py
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
WASM_SRC = REPO / "crates" / "wasm" / "src"
BASELINE = Path(__file__).with_name("wasm-o47-coverage-baseline.json")
# A `#[wasm_bindgen]` export attribute carrying its JS name, followed by
# the Rust function it exports. Any number of `#[...]` attribute lines
# (including multi-line `#[allow(...)]` whose argument lines carry no
# parens), `//` comments, and `///` doc lines may sit between the export
# attribute and the function; anything else is unknown syntax and must
# fail loudly (see `find_unknown_syntax`).
EXPORT_RE = re.compile(
    r'#\[wasm_bindgen\((?P<attr>[^\)]*js_name\s*=\s*"(?P<js>[^"]+)"[^\)]*)\)\]'
    r'(?P<gap>(?:[ \t]*\n|[ \t]*#[^\n]*\n|[ \t]*//[^\n]*\n|[ \t]*///[^\n]*\n|[ \t]*[\)\]][^\n]*\n|[ \t]*[A-Za-z_][A-Za-z0-9_:,\s]*\n)*[ \t]*)'
    r'pub\s+fn\s+(?P<rust>\w+)\s*\((?P<params>[^)]*)\)'
    r'\s*(?:->\s*(?P<ret>[^\{;]+))?',
)
BATCH_OP_RE = re.compile(r'"(?P<op>[A-Za-z0-9_]+)"\s*=>')
WITNESS_RE_TEMPLATE = r"fn\s+{name}\s*\("
SOLID_ENVELOPE = "SolidOperationDetailedResult"


def discover_from_files(root: Path = WASM_SRC) -> list[dict]:
    """Parse exports from the working tree (used by the gate and tests)."""
    exports: list[dict] = []
    for path in sorted(root.rglob("*.rs")):
        text = path.read_text(encoding="utf-8")
        rel = path.relative_to(root).as_posix()
        for match in EXPORT_RE.finditer(text):
            js = match.group("js")
            rust = match.group("rust")
            params = match.group("params")
            ret = (match.group("ret") or "").strip()
            receiver = (
                "mut"
                if "&mut self" in params
                else ("imm" if "&self" in params else "static")
            )
            gate = "io" if rel.startswith("bindings/io") else "shipped"
            exports.append(
                {
                    "js": js,
                    "rust": rust,
                    "file": rel,
                    "recv": receiver,
                    "ret": " ".join(ret.split()),
                    "gate": gate,
                }
            )
    return exports


def own_attr_block(lines: list[str], index: int) -> str:
    """The contiguous attribute/comment block directly above a `pub fn`.

    Walks upward over `#[...]`, `///`, `//`, and blank lines only, so an
    `impl`-level `#[wasm_bindgen]` header further up is never mistaken
    for this function's own attribute.
    """
    block: list[str] = []
    cursor = index - 1
    while cursor >= 0:
        stripped = lines[cursor].strip()
        if stripped.startswith("#[") or stripped.startswith("///") or stripped.startswith("//") or stripped == "":
            block.append(lines[cursor])
            cursor -= 1
        else:
            break
    return "\n".join(reversed(block))


def find_unknown_syntax(root: Path = WASM_SRC) -> list[str]:
    """Find `pub fn` items under `wasm_bindgen` that match no known shape.

    Known shapes: a `js_name` export (captured by discovery), a
    `constructor`, or a bare `getter` without `js_name` (a query accessor
    under its Rust name). Anything else carrying a `wasm_bindgen`
    attribute -- a new macro spelling, a multi-line attribute the parser
    does not cover, an export without `js_name` (including a misspelled
    `js_nam`) -- must fail the gate loudly. Unknown items must never
    silently disappear from discovery.
    """
    problems: list[str] = []
    known_exports = {(e["rust"], e["file"]) for e in discover_from_files(root)}
    for path in sorted(root.rglob("*.rs")):
        # Test companions never ship exports; production `_impl` bodies
        # live in plain `impl` blocks without `wasm_bindgen`.
        if path.name == "tests.rs" or "/tests/" in path.as_posix():
            continue
        lines = path.read_text(encoding="utf-8").splitlines()
        rel = path.relative_to(root).as_posix()
        for index, line in enumerate(lines):
            match = re.match(r"\s*pub\s+(?:unsafe\s+)?fn\s+(\w+)", line)
            if not match:
                continue
            fn_name = match.group(1)
            block = own_attr_block(lines, index)
            if "wasm_bindgen" not in block:
                continue
            if "constructor" in block:
                continue
            if "js_name" in block:
                if (fn_name, rel) not in known_exports:
                    problems.append(f"{rel}:{index + 1}: {fn_name}")
                continue
            # A `wasm_bindgen` attribute with neither `js_name` nor
            # `constructor`: a bare `#[wasm_bindgen]` export (JS name =
            # Rust name) or a misspelled/unknown attribute spelling.
            # Bare getters without `js_name` are query accessors under
            # their Rust name and are out of the mutation-inventory scope.
            if "getter" in block:
                continue
            problems.append(f"{rel}:{index + 1}: {fn_name}")
    return problems


def batch_ops(root: Path = WASM_SRC) -> set[str]:
    """Every dispatch arm name in `bindings/batch.rs`."""
    text = (root / "bindings" / "batch.rs").read_text(encoding="utf-8")
    return set(BATCH_OP_RE.findall(text))


def witness_exists(name: str, root: Path = WASM_SRC) -> bool:
    """Whether a named Rust witness function exists anywhere in wasm src."""
    pattern = WITNESS_RE_TEMPLATE.format(name=re.escape(name))
    found: list[bool] = []
    for path in sorted(root.rglob("*.rs")):
        text = path.read_text(encoding="utf-8")
        if re.search(pattern, text):
            found.append(True)
            break
    return bool(found)


def load_baseline(path: Path = BASELINE) -> dict:
    """Load the committed baseline, rejecting duplicate rows."""
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = data.get("rows", [])
    by_js: dict[str, dict] = {}
    for row in rows:
        js = row["js"]
        if js in by_js:
            raise ValueError(f"duplicate baseline row for {js}")
        by_js[js] = row
    return {"meta": data.get("meta", {}), "rows": rows, "by_js": by_js}


def verify(
    exports: list[dict],
    baseline: dict,
    ops: set[str],
    root: Path = WASM_SRC,
) -> tuple[list[str], list[str], dict]:
    """Check discovery against the baseline.

    Returns (violations, stale, counts). Violations are live defects;
    stale entries name baseline rows the source no longer contains.
    """
    violations: list[str] = []
    stale: list[str] = []
    discovered_mut = [e for e in exports if e["recv"] == "mut"]
    by_js_source = {e["js"]: e for e in discovered_mut}
    if len(by_js_source) != len(discovered_mut):
        dupes = sorted(
            {e["js"] for e in discovered_mut if sum(1 for x in discovered_mut if x["js"] == e["js"]) > 1}
        )
        violations.append(
            f"VIOLATION: duplicate JS export names in source: {', '.join(dupes)}"
        )
    by_js_baseline: dict[str, dict] = baseline["by_js"]

    for js in sorted(by_js_source):
        source = by_js_source[js]
        row = by_js_baseline.get(js)
        if row is None:
            violations.append(
                f"VIOLATION: new unclassified mutation {js} "
                f"({source['rust']} in {source['file']}) has no baseline row; "
                f"classify it in {BASELINE.name} with an owner and a reason"
            )
            continue
        for field in ("rust", "file", "gate"):
            if row.get(field) != source[field]:
                violations.append(
                    f"VIOLATION: baseline row {js} records {field}="
                    f"{row.get(field)!r} but source has {source[field]!r}; "
                    f"a misspelled js_name, moved file, or changed gate must "
                    f"update the baseline in the same PR"
                )

    for js in sorted(by_js_baseline):
        if js not in by_js_source:
            stale.append(
                f"STALE: baseline row {js} names no discovered export; "
                f"a removed export or obsolete exception must leave the "
                f"baseline in the same PR"
            )

    covered = uncovered = special = 0
    for js in sorted(by_js_baseline):
        row = by_js_baseline[js]
        status = row.get("coverage")
        if status == "covered":
            covered += 1
            twin = row.get("twin", "")
            twin_hit = next(
                (e for e in exports if e["js"] == twin), None
            )
            if twin_hit is None:
                violations.append(
                    f"VIOLATION: {js} claims covered by {twin}, but no such "
                    f"export exists in source"
                )
            elif SOLID_ENVELOPE not in twin_hit.get("ret", ""):
                violations.append(
                    f"VIOLATION: {js} twin {twin} returns "
                    f"{twin_hit.get('ret', '')!r}, not the solid envelope; "
                    f"non-solid methods must not be forced into a "
                    f"solid-result schema"
                )
            if row.get("schema") != "solid_envelope":
                violations.append(
                    f"VIOLATION: {js} is covered but schema is "
                    f"{row.get('schema')!r}; covered solid mutations use "
                    f"the solid envelope"
                )
            legacy_ret = by_js_source.get(js, {}).get("ret", "")
            if "Vec<u32>" in legacy_ret or "()" in legacy_ret.split(", JsError")[0][-6:]:
                violations.append(
                    f"VIOLATION: {js} is covered with the solid envelope "
                    f"but returns {legacy_ret!r}; non-solid methods need "
                    f"their own schema, never a forced solid twin"
                )
            for op in row.get("batch_ops", []):
                if op not in ops:
                    violations.append(
                        f"VIOLATION: {js} requires batch dispatch {op!r}, "
                        f"but bindings/batch.rs has no such arm"
                    )
            for witness in row.get("witnesses", []):
                if not witness_exists(witness, root):
                    violations.append(
                        f"VIOLATION: {js} names witness {witness}, but no "
                        f"such function exists in crates/wasm/src"
                    )
            if not row.get("twin") or not row.get("batch_ops") or not row.get("witnesses"):
                violations.append(
                    f"VIOLATION: {js} is covered but its row is missing "
                    f"twin, batch_ops, or witnesses"
                )
        elif status == "uncovered":
            uncovered += 1
            if not row.get("owner") or not row.get("reason"):
                violations.append(
                    f"VIOLATION: uncovered row {js} needs a roadmap owner "
                    f"and a reason"
                )
        elif status == "special":
            special += 1
            if not row.get("owner") or not row.get("reason"):
                violations.append(
                    f"VIOLATION: special-case row {js} needs a roadmap "
                    f"owner and an explanation"
                )
        else:
            violations.append(
                f"VIOLATION: baseline row {js} has unknown coverage "
                f"{status!r}; use covered, uncovered, or special"
            )

    counts = {
        "discovered_mutating": len(discovered_mut),
        "covered": covered,
        "uncovered": uncovered,
        "special": special,
    }
    return violations, stale, counts


def main() -> int:
    try:
        exports = discover_from_files()
    except OSError as error:
        print(f"check-wasm-o47-coverage: cannot read source: {error}")
        return 2
    unknown = find_unknown_syntax()
    try:
        baseline = load_baseline()
    except (OSError, ValueError) as error:
        print(f"check-wasm-o47-coverage: cannot load baseline: {error}")
        return 2
    try:
        ops = batch_ops()
    except OSError as error:
        print(f"check-wasm-o47-coverage: cannot read batch.rs: {error}")
        return 2

    status = 0
    if unknown:
        print("VIOLATION: unknown export syntax would silently leave discovery:")
        for item in sorted(unknown):
            print(f"  {item}")
        print("  Teach the discovery parser the new spelling instead of skipping it.")
        status = 1

    violations, stale, counts = verify(exports, baseline, ops)
    for line in violations:
        print(line)
        status = 1
    for line in stale:
        print(line)
        status = 1

    if status == 0:
        print(
            f"✅ O4.7 twin coverage OK "
            f"({counts['discovered_mutating']} mutating exports: "
            f"{counts['covered']} covered, {counts['uncovered']} uncovered, "
            f"{counts['special']} special)."
        )
    else:
        print(
            f"❌ O4.7 twin coverage failed "
            f"({counts['discovered_mutating']} mutating exports: "
            f"{counts['covered']} covered, {counts['uncovered']} uncovered, "
            f"{counts['special']} special)."
        )
    return status


if __name__ == "__main__":
    sys.exit(main())

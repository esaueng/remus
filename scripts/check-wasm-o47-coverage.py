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
    r'pub\s+(?:(?:async|unsafe)\s+)*fn\s+(?P<rust>\w+)\s*\((?P<params>[^)]*)\)'
    r'\s*(?:->\s*(?P<ret>[^\{;]+))?',
)
BATCH_OP_RE = re.compile(r'^\s*"(?P<op>[A-Za-z0-9_]+)"\s*=>')
CFG_ATTR_RE = re.compile(r'^\s*#\[\s*(cfg|cfg_attr)\s*\(([^]]*)\)\s*\]', re.MULTILINE)
WITNESS_RE_TEMPLATE = r"fn\s+{name}\s*\("
SOLID_ENVELOPE = "Result<tsify::Ts<SolidOperationDetailedResult>, JsError>"
SOLID_HANDLE = "Result<u32, JsError>"


def discover_from_files(root: Path = WASM_SRC) -> list[dict]:
    """Parse exports from the working tree (used by the gate and tests)."""
    exports: list[dict] = []
    for path in sorted(root.rglob("*.rs")):
        text = path.read_text(encoding="utf-8")
        lines = text.splitlines()
        depths, _ = rust_lines_with_depth(lines)
        rel = path.relative_to(root).as_posix()
        parent_gate = file_gate(root, rel)
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
            fn_index = text.count("\n", 0, match.start("rust"))
            gate = export_gate(lines, depths, fn_index, rel, parent_gate)
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


def own_attr_block(lines: list[str], index: int, depths: list[int]) -> str:
    """Attributes and comments directly above an item, including multiline attrs."""
    block: list[str] = []
    cursor = index - 1
    while cursor >= 0 and depths[cursor] == depths[index]:
        stripped = lines[cursor].strip()
        if re.match(r"^(?:(?:pub(?:\([^)]*\))?)\s+)?(?:fn|async|impl|mod|const|type|use|struct|enum)\b", stripped) or stripped.startswith("}") or stripped.endswith(";"):
            break
        block.append(lines[cursor])
        cursor -= 1
    return "\n".join(reversed(block))


def combine_gates(*gates: str) -> str:
    if "conditional" in gates:
        return "conditional"
    return "io" if "io" in gates else "shipped"


def attribute_gate(attributes: str) -> str:
    """Conservatively classify cfgs for the shipped no-feature build."""
    gates: list[str] = []
    for match in CFG_ATTR_RE.finditer(attributes):
        expr = re.sub(r"\s+", "", match.group(2))
        if match.group(1) == "cfg" and expr == 'feature="io"':
            gates.append("io")
        elif match.group(1) == "cfg" and expr == 'not(feature="io")':
            gates.append("shipped")
        else:
            gates.append("conditional")
    return combine_gates(*gates)


def inner_file_attrs(lines: list[str]) -> str:
    """Collect file-level inner attributes, including multiline cfgs."""
    attrs: list[str] = []
    collecting = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("#!["):
            collecting = True
        elif not collecting:
            continue
        attrs.append(line.replace("#![", "#[", 1))
        if "]" in line:
            collecting = False
    return "\n".join(attrs)


def file_gate(root: Path, rel: str) -> str:
    """Follow out-of-line module declarations and crate/file attributes."""
    parts = list(Path(rel).with_suffix("").parts)
    if parts[-1] == "mod":
        parts.pop()
    parent = root / "lib.rs"
    gates: list[str] = []
    for index, name in enumerate(parts):
        if parent.is_file():
            lines = parent.read_text(encoding="utf-8").splitlines()
            depths, _ = rust_lines_with_depth(lines)
            gates.append(attribute_gate(inner_file_attrs(lines)))
            declaration = re.compile(rf"^\s*(?:(?:pub(?:\([^)]*\))?)\s+)?mod\s+{re.escape(name)}\s*;")
            for line_index, line in enumerate(lines):
                if depths[line_index] == 0 and declaration.match(line):
                    gates.append(attribute_gate(own_attr_block(lines, line_index, depths)))
        prefix = root.joinpath(*parts[:index + 1])
        parent = prefix / "mod.rs" if (prefix / "mod.rs").is_file() else prefix.with_suffix(".rs")
    if parent.is_file():
        lines = parent.read_text(encoding="utf-8").splitlines()
        gates.append(attribute_gate(inner_file_attrs(lines)))
    return combine_gates(*gates)


def export_gate(lines: list[str], depths: list[int], index: int, rel: str, parent_gate: str) -> str:
    """Shipped or optional-I/O availability from file, method, and ancestor attrs."""
    attributes = [own_attr_block(lines, index, depths)]
    for ancestor in range(index):
        if not re.match(r"^\s*(?:pub\s+)?(?:impl|mod)\b.*\{", lines[ancestor]):
            continue
        if depths[ancestor] >= depths[index]:
            continue
        if all(depths[child] > depths[ancestor] for child in range(ancestor + 1, index + 1)):
            attributes.append(own_attr_block(lines, ancestor, depths))
    return combine_gates(
        "io" if rel.startswith("bindings/io") else parent_gate,
        *(attribute_gate(block) for block in attributes),
    )


def arm_gate(lines: list[str], index: int) -> str:
    """Availability of one batch match arm from its adjacent cfg attributes."""
    block: list[str] = []
    cursor = index - 1
    while cursor >= 0:
        stripped = lines[cursor].strip()
        if not (stripped.startswith("#[") or stripped.startswith("//") or not stripped):
            break
        block.append(lines[cursor])
        cursor -= 1
    attrs = "\n".join(block)
    return attribute_gate(attrs)


def rust_lines_with_depth(lines: list[str]) -> tuple[list[int], list[str]]:
    """Brace depth and comment-masked source lines, ignoring Rust literals."""
    depths: list[int] = []
    code_lines: list[str] = []
    depth = block_depth = 0
    quote = ""
    raw_closer = ""
    for line in lines:
        depths.append(depth)
        code = list(line)
        cursor = 0
        while cursor < len(line):
            tail = line[cursor:]
            if block_depth:
                code[cursor] = " "
                if tail.startswith("/*"):
                    block_depth += 1
                    cursor += 2
                elif tail.startswith("*/"):
                    block_depth -= 1
                    cursor += 2
                else:
                    cursor += 1
                continue
            if raw_closer:
                if tail.startswith(raw_closer):
                    cursor += len(raw_closer)
                    raw_closer = ""
                else:
                    code[cursor] = " "
                    cursor += 1
                continue
            if quote:
                if line[cursor] == "\\":
                    cursor += 2
                elif line[cursor] == quote:
                    quote = ""
                    cursor += 1
                else:
                    cursor += 1
                continue
            if tail.startswith("//"):
                code[cursor:] = " " * (len(line) - cursor)
                break
            if tail.startswith("/*"):
                block_depth = 1
                cursor += 2
                continue
            raw = re.match(r'r(#+)?"', tail)
            if raw:
                raw_closer = '"' + (raw.group(1) or "")
                cursor += len(raw.group())
                continue
            if line[cursor] == '"':
                quote = '"'
                cursor += 1
                continue
            char = re.match(r"'(?:\\u\{[^}]+\}|\\.|[^'\\])'", tail)
            if char:
                cursor += len(char.group())
                continue
            if line[cursor] == "{":
                depth += 1
            elif line[cursor] == "}":
                depth -= 1
            cursor += 1
        code_lines.append("".join(code))
    return depths, code_lines


def wasm_impl_lines(lines: list[str], depths: list[int]) -> set[int]:
    """Direct method lines inside exported `BrepKernel` impls."""
    result: set[int] = set()
    for index, line in enumerate(lines[:-1]):
        if line.strip() != "#[wasm_bindgen]":
            continue
        if not re.match(r"^\s*impl\s+BrepKernel\s*\{", lines[index + 1]):
            continue
        base_depth = depths[index + 1]
        cursor = index + 2
        while cursor < len(lines) and depths[cursor] > base_depth:
            if depths[cursor] == base_depth + 1:
                result.add(cursor)
            cursor += 1
    return result


def find_unknown_syntax(root: Path = WASM_SRC) -> list[str]:
    """Find `pub fn` items under `wasm_bindgen` that match no known shape.

    Known shapes: a `js_name` export (captured by discovery), a
    `constructor`, or a bare `getter` without `js_name` (a query accessor
    under its Rust name). Anything else carrying a `wasm_bindgen`
    attribute, or living inside an exported impl -- a new macro spelling,
    a multi-line attribute the parser does not cover, an export without
    `js_name` (including a misspelled `js_nam`) -- must fail the gate.
    Unknown items must never silently disappear from discovery.
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
        depths, _ = rust_lines_with_depth(lines)
        exported_impl = wasm_impl_lines(lines, depths)
        for index, line in enumerate(lines):
            match = re.match(r"\s*pub\s+(?:(?:async|unsafe)\s+)*fn\s+(\w+)", line)
            if not match:
                continue
            fn_name = match.group(1)
            if (fn_name, rel) in known_exports:
                continue
            block = own_attr_block(lines, index, depths)
            if "wasm_bindgen" not in block and index not in exported_impl:
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


def batch_ops(root: Path = WASM_SRC) -> dict[str, str]:
    """Top-level `dispatch_op` arms and their shipped/optional-I/O gates."""
    text = (root / "bindings" / "batch.rs").read_text(encoding="utf-8")
    lines = text.splitlines()
    depths, code_lines = rust_lines_with_depth(lines)
    dispatch = next(
        (index for index, line in enumerate(lines) if re.match(r"^\s*fn dispatch_op\s*\(", line)),
        None,
    )
    if dispatch is None:
        raise ValueError("batch dispatch_op method is missing")
    dispatch_gate = export_gate(
        lines, depths, dispatch, "bindings/batch.rs",
        file_gate(root, "bindings/batch.rs"),
    )
    method_depth = depths[dispatch]
    method_body = next(
        (index for index in range(dispatch + 1, len(lines)) if depths[index] > method_depth),
        None,
    )
    if method_body is None:
        raise ValueError("batch dispatch_op body is missing")
    method_end = next(
        (index for index in range(method_body + 1, len(lines)) if depths[index] <= method_depth),
        len(lines),
    )
    match_line = next(
        (
            index
            for index in range(dispatch + 1, method_end)
            if depths[index] == method_depth + 1
            and re.match(r"^\s*match\s+op\s*\{", code_lines[index])
        ),
        None,
    )
    if match_line is None:
        raise ValueError("batch dispatch_op match op is missing")
    arm_depth = depths[match_line] + 1
    match_end = next(
        (index for index in range(match_line + 1, method_end) if depths[index] < arm_depth),
        method_end,
    )
    result: dict[str, str] = {}
    for index in range(match_line + 1, match_end):
        if depths[index] != arm_depth or not (match := BATCH_OP_RE.match(code_lines[index])):
            continue
        op = match.group("op")
        gate = combine_gates(dispatch_gate, arm_gate(lines, index))
        if op not in result or gate == "shipped":
            result[op] = gate
    return result


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
    ops: dict[str, str],
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
            if by_js_source.get(js, {}).get("gate") == "conditional":
                violations.append(
                    f"VIOLATION: {js} claims coverage under an unknown feature gate; "
                    f"qualify its shipped availability before marking it covered"
                )
            twin = row.get("twin", "")
            twin_hit = next(
                (e for e in exports if e["js"] == twin), None
            )
            if twin_hit is None:
                violations.append(
                    f"VIOLATION: {js} claims covered by {twin}, but no such "
                    f"export exists in source"
                )
            elif twin_hit.get("ret") != SOLID_ENVELOPE:
                violations.append(
                    f"VIOLATION: {js} twin {twin} returns "
                    f"{twin_hit.get('ret', '')!r}, not the solid envelope; "
                    f"non-solid methods must not be forced into a "
                    f"solid-result schema"
                )
            if twin_hit is not None and twin_hit["gate"] != by_js_source.get(js, {}).get("gate"):
                violations.append(
                    f"VIOLATION: {js} is available under "
                    f"{by_js_source.get(js, {}).get('gate')!r}, but twin {twin} "
                    f"is only available under {twin_hit['gate']!r}"
                )
            if row.get("schema") != "solid_envelope":
                violations.append(
                    f"VIOLATION: {js} is covered but schema is "
                    f"{row.get('schema')!r}; covered solid mutations use "
                    f"the solid envelope"
                )
            legacy_ret = by_js_source.get(js, {}).get("ret", "")
            if legacy_ret != SOLID_HANDLE:
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
                elif ops[op] != by_js_source.get(js, {}).get("gate"):
                    violations.append(
                        f"VIOLATION: {js} requires batch dispatch {op!r} "
                        f"under {by_js_source.get(js, {}).get('gate')!r}, "
                        f"but its arm is only available under {ops[op]!r}"
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

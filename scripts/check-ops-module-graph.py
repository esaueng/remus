#!/usr/bin/env python3
"""Keep `remus-operations`' production module graph splittable (RFC 0007).

RFC 0007 (`docs/design/rfc-0007-operations-crate-split.md`) assigns every
top-level module of `crates/operations/src` to a future L3 crate. A crate split
only works if no module depends on a module of a crate that is not below its
own, so this check fails on any production reference that points the wrong
way. Test code (`#[cfg(test)]` and `#[cfg(all(test, ..))]` items and files)
is exempt: the RFC moves upward-reaching tests to integration tests at each
stage. `#[cfg(any(test, ..))]` code is checked, since a feature compiles it.

A new top-level module must be added to TIERS. Pick the lowest tier whose
modules it uses; the failure message names what it references.

References counted: `crate::`, `super::` and `self::` paths, both in `use`
statements (with nested `{...}` groups expanded) and as qualified paths in
code, plus bare `module::` paths in `lib.rs`. `lib.rs` items belong to the
core tier, except `pub use` re-exports, which belong to the facade.
"""

from __future__ import annotations

import os
import re
import sys
from dataclasses import dataclass

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "crates", "operations", "src")

# Future crate -> top-level modules. `ROOT` is the crate root (`lib.rs`).
TIERS: dict[str, list[str]] = {
    "remus-ops-core": [
        "ROOT", "performance", "winding", "evolution", "boundary_evolution",
        "polygon", "error_payloads", "test_utils", "cache",
    ],
    "remus-transform": ["transform", "copy", "mirror"],
    "remus-tessellate": ["tessellate"],
    "remus-measure": [
        "measure", "distance", "classify", "validate", "query",
        "feature_recognition", "projection", "triangle_intersection",
    ],
    "remus-construct": [
        "primitives", "extrude", "revolve", "sweep", "loft", "pipe", "helix",
        "cap", "fill_face", "thicken",
    ],
    "remus-boolean": [
        "boolean", "mesh_boolean", "compound_ops", "heal", "section", "split",
        "project_curve",
    ],
    "remus-features": [
        "fillet", "chamfer", "blend_ops", "blend_material", "face_face_blend",
        "resize_blend", "affine_blend_caps", "local_wound", "remove_blends",
        "defeature", "push_pull", "replace_surface", "offset_face",
        "offset_trim", "offset_v2", "shell_op", "draft",
    ],
    "remus-operations": [
        "journal_ops", "prepared_move", "imprint", "pattern", "assembly",
        "sketch", "linked_sketch", "sew", "untrim", "offset_wire", "gear",
    ],
}

# Direct predecessors in the crate DAG; a tier may use everything below it.
# `remus-transform` and `remus-tessellate` are siblings: neither may use the other.
BELOW: dict[str, list[str]] = {
    "remus-ops-core": [],
    "remus-transform": ["remus-ops-core"],
    "remus-tessellate": ["remus-ops-core"],
    "remus-measure": ["remus-tessellate"],
    "remus-construct": ["remus-transform", "remus-measure"],
    "remus-boolean": ["remus-construct"],
    "remus-features": ["remus-boolean"],
    "remus-operations": ["remus-features"],
}

FACADE = "remus-operations"


def allowed_tiers() -> dict[str, set[str]]:
    out: dict[str, set[str]] = {}

    def below(tier: str) -> set[str]:
        if tier not in out:
            acc: set[str] = set()
            for lower in BELOW[tier]:
                acc |= {lower} | below(lower)
            out[tier] = acc
        return out[tier]

    for tier in BELOW:
        below(tier)
    return out


def strip_comments_and_strings(text: str) -> str:
    """Blank out comments, strings and char literals, keeping offsets and newlines."""
    out = list(text)
    i, n = 0, len(text)

    def blank(a: int, b: int) -> None:
        for k in range(a, min(b, n)):
            if out[k] != "\n":
                out[k] = " "

    while i < n:
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            blank(i, j)
            i = j
        elif text.startswith("/*", i):
            depth, j = 1, i + 2
            while j < n and depth:
                if text.startswith("/*", j):
                    depth, j = depth + 1, j + 2
                elif text.startswith("*/", j):
                    depth, j = depth - 1, j + 2
                else:
                    j += 1
            blank(i, j)
            i = j
        elif (m := re.match(r'(?:b?r(#*)")', text[i:i + 12])) and (i == 0 or not (text[i - 1].isalnum() or text[i - 1] == "_")):
            end = '"' + m.group(1)
            j = text.find(end, i + len(m.group(0)))
            j = n if j < 0 else j + len(end)
            blank(i, j)
            i = j
        elif text[i] == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            blank(i, j + 1)
            i = j + 1
        elif text[i] == "'" and (m := re.match(r"'(?:\\u\{[0-9a-fA-F]+\}|\\.|[^\\'])'", text[i:i + 12])):
            blank(i, i + len(m.group(0)))
            i += len(m.group(0))
        else:
            i += 1
    return "".join(out)


def matching_brace(s: str, open_at: int) -> int:
    depth = 0
    for k in range(open_at, len(s)):
        if s[k] == "{":
            depth += 1
        elif s[k] == "}":
            depth -= 1
            if depth == 0:
                return k
    return len(s) - 1


# `cfg(any(test, feature = ..))` is not exempt: a feature compiles it outside tests.
TEST_ATTR = re.compile(r"#\[cfg\((?:test|all\(test\b[^\]]*)\)\]")


def test_ranges(s: str) -> list[tuple[int, int]]:
    """Character ranges of items gated by `#[cfg(test)]` (and `all/any(test, ..)`)."""
    ranges = []
    for m in TEST_ATTR.finditer(s):
        j = m.end()
        while (a := re.match(r"\s*#\[[^\]]*\]", s[j:])):
            j += a.end()
        semi, brace = s.find(";", j), s.find("{", j)
        if brace < 0 or 0 <= semi < brace:
            ranges.append((m.start(), semi + 1))
        else:
            ranges.append((m.start(), matching_brace(s, brace) + 1))
    return ranges


def inline_modules(s: str) -> list[tuple[int, int, str]]:
    return [
        (m.end() - 1, matching_brace(s, m.end() - 1), m.group(1))
        for m in re.finditer(r"\bmod\s+(\w+)\s*\{", s)
    ]


def expand_use(body: str) -> list[list[str]]:
    body = re.sub(r"\s+as\s+\w+", "", " ".join(body.split()))
    i = body.find("{")
    if i < 0:
        return [[p.strip() for p in body.split("::") if p.strip()]]
    prefix = [p.strip() for p in body[:i].split("::") if p.strip()]
    inner = body[i + 1:body.rfind("}")]
    parts, depth, cur = [], 0, ""
    for ch in inner:
        depth += ch == "{"
        depth -= ch == "}"
        if ch == "," and depth == 0:
            parts.append(cur)
            cur = ""
        else:
            cur += ch
    parts.append(cur)
    out = []
    for part in parts:
        if part.strip():
            for tail in expand_use(part):
                out.append(prefix if tail == ["self"] else prefix + tail)
    return out


def read(path: str) -> str:
    with open(path, encoding="utf-8") as fh:
        return fh.read()


@dataclass(frozen=True)
class SourceFile:
    path: str
    modpath: tuple[str, ...]
    is_test: bool


def module_files(src: str) -> list[SourceFile]:
    files: list[SourceFile] = []

    def walk(path: str, modpath: tuple[str, ...], is_test: bool) -> None:
        raw = read(path)
        s = strip_comments_and_strings(raw)
        files.append(SourceFile(path, modpath, is_test))
        tests = test_ranges(s)
        base = os.path.dirname(path)
        stem = os.path.splitext(os.path.basename(path))[0]
        child_dir = base if stem in ("lib", "mod") else os.path.join(base, stem)
        inline = inline_modules(s)
        for m in re.finditer(r"((?:#\[[^\]]*\]\s*)*)(?:pub(?:\([^)]*\))?\s+)?mod\s+(\w+)\s*;", s):
            name = m.group(2)
            nested = [n for a, b, n in inline if a < m.start() < b]
            gated = is_test or any(a <= m.start() < b for a, b in tests) or "cfg(test" in m.group(1)
            path_attr = re.search(r'#\[path\s*=\s*"([^"]+)"\]', raw[m.start():m.end()])
            directory = os.path.join(child_dir, *nested)
            candidates = [os.path.join(directory, name + ".rs"), os.path.join(directory, name, "mod.rs")]
            if path_attr:
                # `#[path]` on a file module is relative to the declaring file's directory.
                candidates.insert(0, os.path.join(base, *nested, path_attr.group(1)))
            found = next((c for c in candidates if os.path.exists(c)), None)
            if found is None:
                raise SystemExit(f"error: cannot resolve `mod {name};` in {os.path.relpath(path, REPO)}")
            walk(found, modpath + tuple(nested) + (name,), gated)

    walk(os.path.join(src, "lib.rs"), (), False)
    return files


@dataclass(frozen=True)
class Reference:
    path: str
    line: int
    source: str
    target: str
    item: str


def references(src: str, top_modules: set[str]) -> list[Reference]:
    """Production references between distinct top-level modules."""
    out: list[Reference] = []
    for f in module_files(src):
        if f.is_test:
            continue
        s = strip_comments_and_strings(read(f.path))
        tests = test_ranges(s)
        inline = inline_modules(s)
        is_root = not f.modpath
        uses = [(m.start(), m.end(), m.group(1)) for m in re.finditer(r"\buse\s+([^;]+);", s)]

        def scope(pos: int) -> tuple[str, ...]:
            return f.modpath + tuple(n for a, b, n in inline if a < pos < b)

        def resolve(pos: int, segs: list[str]) -> list[str] | None:
            if not segs:
                return None
            if segs[0] == "crate":
                return segs[1:]
            if segs[0] in ("super", "self"):
                here = list(scope(pos))
                k = 0
                while k < len(segs) and segs[k] == "super":
                    k += 1
                if k == 0:
                    return here + segs[1:]
                return here[:max(0, len(here) - k)] + segs[k:]
            if is_root and not scope(pos) and segs[0] in top_modules:
                return segs
            return None

        candidates: list[tuple[int, list[str], bool]] = []
        for a, _, body in uses:
            facade = is_root and re.match(r"pub\s+use\b", s[max(0, a - 4):a + 3].strip() + " ") is not None
            facade = facade or (is_root and s[max(0, a - 4):a] == "pub ")
            for segs in expand_use(body):
                candidates.append((a, segs, facade))
        for m in re.finditer(r"(?<![\w:])((?:crate|super|self)(?:\s*::\s*\w+)+)", s):
            if any(a <= m.start() < b for a, b, _ in uses):
                continue
            candidates.append((m.start(), [x.strip() for x in m.group(1).split("::")], False))
        if is_root:
            for m in re.finditer(r"(?<![\w:])(\w+)\s*::\s*(\w+)", s):
                if m.group(1) in top_modules and not any(a <= m.start() < b for a, b, _ in uses):
                    candidates.append((m.start(), [m.group(1), m.group(2)], False))

        for pos, segs, facade in candidates:
            if any(a <= pos < b for a, b in tests):
                continue
            target_path = resolve(pos, segs)
            if not target_path:
                continue
            source = "ROOT" if is_root and not scope(pos) else scope(pos)[0]
            target = target_path[0] if target_path[0] in top_modules else "ROOT"
            if facade:
                source = "FACADE"
            if source == target:
                continue
            out.append(Reference(
                os.path.relpath(f.path, REPO), s.count("\n", 0, pos) + 1,
                source, target, "::".join(target_path),
            ))
    return out


def check(src: str = SRC) -> list[str]:
    tier_of = {m: t for t, mods in TIERS.items() for m in mods}
    tier_of["FACADE"] = FACADE
    top = {
        os.path.splitext(e)[0]
        for e in os.listdir(src)
        if (e.endswith(".rs") and e != "lib.rs") or os.path.isdir(os.path.join(src, e))
    }
    problems = [
        f"{os.path.relpath(os.path.join(src, m), REPO)}: top-level module `{m}` has no tier; "
        "add it to TIERS in scripts/check-ops-module-graph.py"
        for m in sorted(top - set(tier_of))
    ]
    allowed = allowed_tiers()
    for ref in references(src, top):
        src_tier, dst_tier = tier_of.get(ref.source), tier_of.get(ref.target)
        if src_tier is None or dst_tier is None or src_tier == dst_tier:
            continue
        if dst_tier not in allowed[src_tier]:
            problems.append(
                f"{ref.path}:{ref.line}: `{ref.source}` ({src_tier}) uses `{ref.item}` "
                f"from `{ref.target}` ({dst_tier}), which is not below it"
            )
    return problems


def main() -> int:
    problems = check()
    if problems:
        print("❌ remus-operations module graph check failed (RFC 0007):")
        for p in problems:
            print(f"  {p}")
        print("   Move the item to a module of a lower tier (re-export it at its old path),")
        print("   or move the using module up. Test-only references are exempt.")
        return 1
    print("✅ remus-operations module graph matches the RFC 0007 crate order.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Attribute kernel WASM bytes by crate and module from `twiggy top` CSV.

Method follows docs/design/kernel-wasm-size-architecture.md: function bytes
before wasm-opt (names kept), grouped by the first `remus_*` crate in the
symbol, so std generics instantiated for a Remus type count toward that crate.
"""
import csv
import re
import subprocess
import sys
from collections import Counter

CRATE = re.compile(r"remus_([a-z_]+?)(?=::|\[|<|>|\$|\.|,| |$)")
STD = re.compile(r"\b(core|alloc|std|hashbrown)::")


def rows(path):
    out = subprocess.run(
        ["twiggy", "top", "-n", "10000000", "-f", "csv", path],
        check=True, capture_output=True, text=True,
    ).stdout
    for row in csv.DictReader(out.splitlines()):
        yield row["Name"], int(row["ShallowSize"])


def group(name):
    if name.startswith("data[") or name.startswith("data segment"):
        return "data segments"
    if name.startswith('"') or name.startswith("custom section") or name in ("name", "producers", "target_features"):
        return "custom/name sections"
    m = CRATE.search(name)
    if m:
        crate = m.group(1)
        return "remus_" + crate
    if "wasm_bindgen" in name or name.startswith("__wbg") or "__wbindgen" in name:
        return "wasm-bindgen glue"
    if re.search(r"\b(serde|serde_json|tsify|serde_wasm_bindgen)\b", name):
        return "serde/tsify"
    if STD.search(name):
        return "std/core/alloc/hashbrown"
    if re.search(r"\brobust::", name):
        return "robust"
    return "other"


def module(name):
    m = re.search(r"remus_([a-z_]+)::([a-z_0-9]+)", name)
    return f"remus_{m.group(1)}::{m.group(2)}" if m else None


def summarize(path):
    groups, modules, total = Counter(), Counter(), 0
    for name, size in rows(path):
        total += size
        groups[group(name)] += size
        mod = module(name)
        if mod:
            modules[mod] += size
    return total, groups, modules


def main(base, cur):
    bt, bg, bm = summarize(base)
    ct, cg, cm = summarize(cur)
    print(f"total item bytes: base {bt:,}  cur {ct:,}  delta {ct - bt:+,}\n")
    print(f"{'group':34} {'base':>12} {'cur':>12} {'delta':>11} {'share of growth':>16}")
    growth = ct - bt
    for g in sorted(set(bg) | set(cg), key=lambda g: -(cg[g] - bg[g])):
        d = cg[g] - bg[g]
        print(f"{g:34} {bg[g]:>12,} {cg[g]:>12,} {d:>+11,} {100 * d / growth:>15.1f}%")
    print("\nTop modules by growth:")
    for m in sorted(set(bm) | set(cm), key=lambda m: -(cm[m] - bm[m]))[:40]:
        d = cm[m] - bm[m]
        print(f"{m:60} {bm[m]:>11,} {cm[m]:>11,} {d:>+10,}")
    print("\nTop modules by shrink:")
    for m in sorted(set(bm) | set(cm), key=lambda m: (cm[m] - bm[m]))[:10]:
        d = cm[m] - bm[m]
        if d < 0:
            print(f"{m:60} {bm[m]:>11,} {cm[m]:>11,} {d:>+10,}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])

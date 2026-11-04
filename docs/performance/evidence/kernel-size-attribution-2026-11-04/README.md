# Kernel size attribution, 2026-11-04

Evidence for "Status (2026-11-04)" in
[`docs/design/kernel-wasm-size-architecture.md`](../../../design/kernel-wasm-size-architecture.md).
Method, scripts and toolchain are those of the
[2026-10-29 attribution](../kernel-size-attribution-2026-10-29/README.md)
(`attribute.py`, `normdiff.py`, twiggy 0.8.0, wasm-bindgen-cli 0.2.126,
Binaryen `version_123`, no `--remap-path-prefix`), so absolute bytes are
approximate and deltas are internally consistent.

| Build | Source | Raw | gzip | Brotli q11 |
| --- | --- | ---: | ---: | ---: |
| base | `d8195032` (v2026.1.86, 2026-10-29) | 10,974,205 | 4,079,305 | 2,904,227 |
| cur | `84bf696` (`main`, 2026-11-04) | 12,053,275 | 4,526,738 | 3,238,707 |
| new | cur + the sort-lint change in this PR | 12,030,994 | 4,518,329 | 3,232,478 |

Base to cur is +1,079,070 B raw / +447,433 B gzip in six days, about
1.26 MB raw and 0.52 MB gzip per week.

## Where the growth is

From `attribution-post-opt.txt` (function bytes after `wasm-opt`, names kept;
the names section is excluded from the shipped figures):

| Area | Growth | Main contributors |
| --- | ---: | --- |
| Gears and threads | ≈ +330 KB | `operations::gear` +77K, `operations::thread` +38K, and their bindings +166K / +48K (the operation code is inlined into each binding's single call site) |
| Blends, chamfers and blend edits | ≈ +530 KB | `resize_blend` +150K (`move_with_cap_reblend` alone 107 KB), `blend_ops` +114K, `chamfer` +79K, `blend::ridge_chain` +73K, `blend::seam_chain` +72K, `fillet_builder` +27K |
| Everything else | ≈ +220 KB | `extrude` +29K, `check::classify` +24K, `algo::builder` +22K, data segments +40K |
| std generics | −33 KB | |

Almost all of it is new, single-caller feature code. `function-diff-post-opt.txt`
shows no new per-call-site duplication family; `monos-cur.txt` shows the
std sorts are still the largest monomorphization family.

## Recoverable duplication

- **Std sorts outside the lint.** `slice::sort` was not disallowed, and the
  wasm bindings had no sort rule. This PR closes both gaps:
  **−22,281 B raw / −8,409 B gzip** (five stable-sort copies). The remaining
  80 stable-sort copies (`stable-sort-callers.txt`) are `// hot:` sites, the
  sketch solver (no `remus-math` dependency, timed by the sketch suite),
  `remus-io`'s arena serializer (not under the lint), the `CompactSort`
  instantiation itself, and sorts reached through `collect()` into
  `BTreeMap`/`BTreeSet`, which the call graph cannot separate after inlining.
- **Per-carrier integrator copies** were already reduced in #1373.
- Nothing else found is larger than a few kilobytes.

The duplication lever is close to exhausted: everything found here is about
four hours of growth at the current rate.

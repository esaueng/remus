# O4.7 twin-coverage inventory

The executable inventory behind the O4.7 typed-result rollout: every
mutating `BrepKernel` JavaScript export, whether it has a typed-result
twin, and what remains. The gate is
`scripts/check-wasm-o47-coverage.py`; the data is
`scripts/wasm-o47-coverage-baseline.json`; the gate's own proof is
`scripts/test-wasm-o47-coverage.py`. All three run in the `repo-policy`
CI job with no build and no network.

## What the gate checks

Discovery parses `crates/wasm/src` for `#[wasm_bindgen(js_name = "...")]`
exports and records, per export, the Rust name, the file, the receiver
(`&mut self` vs `&self`), the return type, and the feature gate
(`bindings/io.rs` ships only with the `io` feature, which the kernel
package builds without). Queries (`&self`) carry no rows. Every
**mutating** export carries exactly one baseline row:

| `coverage` | Meaning |
| --- | --- |
| `covered` | A legacy method with a typed twin. The gate verifies the twin export exists with the solid-envelope return type, every required batch dispatch path exists, and every named runtime witness exists. |
| `uncovered` | No twin yet. The row carries a roadmap owner and a reason, and stays visible until a twin lands. |
| `special` | Explained exception: the twin itself, an already-typed quality/evolution/journal surface, a batch dispatcher, or a lifecycle/session mutation. Carries an owner and a reason, never a twin requirement. |

For `covered` rows the gate additionally enforces:

* the twin returns `SolidOperationDetailedResult` (the solid envelope);
* the legacy method itself returns a single solid handle -- a `Vec<u32>`,
  `()`, or JSON-string method can never be `covered` with the solid
  envelope. Non-solid and lifecycle methods need their own schema, never
  a forced solid twin;
* every `batch_ops` entry has a dispatch arm in `bindings/batch.rs`.
  Boolean twins (`fuseDetailed`, `cutDetailed`, `intersectDetailed`)
  intentionally have no same-named batch arm: their contract is code
  parity with the legacy `fuse`/`cut`/`intersect` batch ops, and the
  witnesses pin that equality. Construction and modifier twins have
  same-named batch ops returning the identical envelope;
* every `witnesses` entry names a Rust function that exists in
  `crates/wasm/src`. Name existence is all the gate proves -- runtime
  correctness is proven by `cargo test -p remus-wasm` (the named
  contract tests) and `node scripts/test-wasm-smoke.mjs` (the
  installed-package witnesses in the same file).

Failures distinguish `VIOLATION` (a live defect: new unclassified
mutation, missing batch arm, missing witness, falsely claimed coverage,
unknown syntax) from `STALE` (the baseline names an export the source no
longer contains: a removed export or an obsolete exception). Both fail
the gate; both are fixed in the same PR that changes the source.

Unknown source syntax -- a `pub fn` carrying a `wasm_bindgen` attribute
that is neither a `js_name` export, a `constructor`, nor a bare `getter`
-- fails loudly. It must never silently disappear from discovery. Known
non-inventory shapes are `constructor` (session creation) and bare
`getter` without `js_name` (query accessors under their Rust name).

## Adding a twin (closing a row)

1. Implement the `*Detailed` twin and its batch arm(s) following the
   `construction_detailed.rs` / `modifier_detailed.rs` pattern (shared
   `_impl` body, direct and batch identical by construction, refusal as
   data with rollback).
2. Add contract tests proving success, refusal, prior-state preservation,
   and direct/batch code parity. Name them; the gate references them.
3. Flip the legacy row to `coverage: covered` with `twin`, `batch_ops`,
   `witnesses`, and `schema: solid_envelope`. Add a `special` row for
   the twin itself ("the typed surface itself").
4. Extend the installed-package witnesses in
   `scripts/test-wasm-smoke.mjs` for at least the success and refusal
   paths, and run the smoke test against freshly built packages.
5. Run the gate, its fixtures, and the affected contract tests. The
   roadmap O4.7 row's covered list and remaining count move in the same
   PR.

## Adding a method (opening a row)

A new mutating export without a baseline row fails the gate as a new
unclassified mutation. Classify it in the same PR: `geometry_mutation`
with `coverage: uncovered`, a roadmap owner, and a reason -- or `covered`
with the full twin evidence, or `special` with an explanation. Queries
need no row. Removing or renaming an export updates or deletes its row
in the same PR; a stale row fails the gate.

## Source coverage vs package provenance

The baseline inventories **source** (`crates/wasm/src`). What ships in
`crates/wasm/pkg` (the committed, version-locked build) is provenance,
not coverage: the smoke test proves the twins exist and behave in the
built package, and `cargo xtask wasm-build` plus the version guard own
that channel. Never cite package bytes as coverage evidence, and never
let a package-only diff close a baseline row.

## Current counts

Regenerate the headline numbers with the gate itself:

```bash
python3 scripts/check-wasm-o47-coverage.py
python3 scripts/test-wasm-o47-coverage.py
```

At introduction: 218 mutating exports -- 13 covered (the boolean,
construction, and modifier solid-envelope families), 111 uncovered with
an owner and a reason each, 94 explained special cases. The roadmap O4.7
row is the human-readable mirror; this baseline is the executable one.

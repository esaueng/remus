# B26 boolean-correctness campaign — progress log

Worktree: `/home/peter/code/Remus-boolean-correctness-20260912-0115`
Branch: `codex/boolean-correctness-20260912-0115`
Base: `origin/main` @ `4e6499fc` (2026-09-12; #406 bench baseline).
Constraint: the task's `/home/peter/opencode/Remus` checkout is an empty repo
skeleton (no remotes, no commits); all work happens here. `~/code/Remus`
and `~/claude/remus` checkouts are preserved untouched.

Roadmap item: §B row **B26** — boolean and mesh invariant proptests
(`operations/tests/prop_boolean_invariants.rs` (new)).

## Baseline (origin/main @ 66bd1045, rebased to 4e6499fc)

- Open PRs at first fetch: **zero open** (newest: #397 MERGED 2026-09-12).
  At rebase time (2026-09-12 evening): #408, #407, #402, #401, #409 open —
  none boolean-correctness work; no duplication.
- Existing coverage (must not duplicate):
  - `crates/operations/tests/boolean_invariants.rs` (655 lines): volume
    conservation (1-D + 3-D offsets), fuse/intersect commutativity,
    cut-complement, anti-commutativity, self-fuse/self-intersect,
    manifold + Euler genus-0 on box results, cylinder+box conservation,
    edge-on-edge refusal, vertex-on-face fuse, identical-cut contract,
    near-coincident split/join, containment, thin-shell fuse, cyl-from-box.
  - `crates/operations/tests/proptest_operations.rs` (~20 cases):
    box-offset conservation/commutativity, rigid-motion volume.
  - `crates/operations/tests/coincident_proptest.rs` (16 cases):
    face-stack translation/rotation invariance, sub-tolerance perturbation.
  - `fuzz/fuzz_targets/{boolean_tree,shapegen,invariants}.rs`: bounded
    primitive trees with closed-form leaf oracles, per-node
    closed-manifold + volume-bounds + disjoint-exact checks, root
    exact-volume / deflection-stability / scale / determinism /
    self-fuse-idempotence battery.
- Related-but-dirty prior art (NOT cherry-picked):
  - `/tmp/remus-b26-phase1` (branch `codex/b26-fuzz-oracles`) carries
    **uncommitted** oracle helpers (+516 lines). Reused as design input
    only; the committed harness re-derives everything from hand closed
    forms so no kernel-sharing oracle is inherited.

## Design decisions (recorded before implementing)

1. New integration test file `crates/operations/tests/prop_boolean_invariants.rs`
   per the B26 row — 7 generated families (box-pair, box-cylinder plug,
   cavity, rigid, scale, spheres, nudge) + opt-in campaign — rather than
   extending the fuzz target: CI replay must be small/deterministic
   (`cargo test`), with a documented larger opt-in campaign.
2. Independent oracles only: hand closed forms (box, cylinder, sphere,
   equal-sphere lens), set identities, topology (ops-validator as enforced
   by the pipeline + check-crate supplement + position-quantized geometric
   edge recount), mesh (boundary/non-manifold counts). `solid_volume` is
   only the reading under test; operands are pinned first.
3. Outcome taxonomy per case: `exact_ok` / `typed_refusal` / `incorrect`.
   Each family requires ≥1 exact success (non-vacuity gate).
4. Boundary vs instability: tangency/contact refusals pass; a 1e-13 nudge
   flipping refusal↔success fails.
5. Cavity solids via `shell()`; no public-contract changes; transactional
   rollback preserved.
6. Cost control (measured 2026-09-12): sphere∩sphere fuse ~60–80 s/call,
   sphere cut ~36–66 s/call, sphere intersect ~1–2 s/call at 8 segments
   (volumes identical across 8/12/16). CI covers intersect only for the
   sphere family; fuse/cut live in the opt-in campaign.

## Harness findings (all closed as harness bugs, kernel exonerated)

1. Disjoint-box fuse "invalid": `check`-crate `ShellConnected` rejects
   legitimate two-component disjoint unions by construction (no shared
   edge). Fix: authoritative gate is the ops-validator (accepts closed
   Euler-consistent components); check-crate runs with `ShellConnected`
   disabled + summed-Euler warning filtered.
2. Empty intersection "invalid": disjoint intersect returns a faceless
   empty solid (`ShellEmpty`). Fix: accept faceless results, judge by
   volume (~0).
3. `shell()` hollow-box "invalid" under strict ops gate: cavity wall is a
   second edge-connected component of the outer shell. Fix: strict, fall
   back to `validate_solid_relaxed` (the gate `shell()` itself enforces).
4. Campaign false positives on disjoint pairs: same empty-intersection
   cause; fixed with the disjoint-fuse branch + shrink.
5. `family_nudge` dead stores: cleaned; operand-pin failures now print.

## Commands

- `cargo test -p remus-operations --test prop_boolean_invariants`
  (CI replay, ~5 s)
- `PROP_BOOL_CASES=<n> PROP_BOOL_SEED=<u64> cargo test -p remus-operations --test prop_boolean_invariants -- --nocapture`
  (opt-in; n clamped to [1, 4096]; ~2–3 s/case with curved kinds)
- `cargo +nightly fuzz run boolean_tree -- -max_total_time=<s>`
  (structured fuzz; needs nightly for -Zsanitizer)

## Original PR work (historical)

- Harness implemented (1330 lines), clippy-clean, fmt-clean.
- CI replay green: 26 exact_ok / 0 incorrect across 7 families (~5 s).
- Opt-in campaigns: seeds 11/12/13 × 40 cases green; seed 21 × 200 green
  (160 ok / 40 refusal); seed 22 × 64 green; determinism verified
  (seed 42 × 8 twice identical).
- Nightly fuzz `boolean_tree` 120 s: 2860 runs, 0 findings.
- Rebases: 66bd1045 → 4e6499fc (ff-only; no conflicts).

## Minimized reproductions

- None: the campaign found zero kernel defects. Every red during
  development minimized to a harness-oracle bug (see findings above) and
  was fixed in the harness with the kernel exonerated by an independent
  check (volumes matched closed forms to 4+ decimals; validators agreed).

## Original PR verification (historical)

- `cargo test -p remus-operations --test prop_boolean_invariants`: 2/2 pass.
- `cargo clippy -p remus-operations --test prop_boolean_invariants`: clean.
- `cargo fmt --check` on the new file: clean.
- Pending: full workspace checks, boundaries, WASM regressions, roadmap
  update, PR.

## Review corrections (2026-09-13)

The original green campaigns did not establish oracle completeness. Review
injected an unchanged 4x4x2 stock as Fuse success while Intersect/Cut refused;
the valid stock incorrectly escaped the expected union volume of 32 + pi/2.
The permanent `wrong_success_is_not_hidden_by_sibling_refusals` witness now
requires Incorrect. Independent box, cylinder-plug, cavity and scaled-box
volumes are checked before classifying any sibling refusal; generated boxes
also use their analytic overlap volume. Non-finite comparison inputs fail.

The refreshed fixed matrix and injected witnesses pass (4 tests). Historical
campaign counts above belong to their recorded source, not this integration.
The fuzz companion's four injected regressions cover successful NaN/Inf,
refused-root nudge execution, both refusal/success flip directions, and
closed-curve authority/geometric identity. Kernel defects found by future
campaigns must retain minimized witnesses and stay with their owning rows.

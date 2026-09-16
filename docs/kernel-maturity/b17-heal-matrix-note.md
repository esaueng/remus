# B17 heal matrix note — first slice (wire/face/shell/duplicate/sewing)

Date: 2026-09-16. Matrix: `crates/operations/tests/qualify_heal.rs`
(10 tests, defect class x severity x repair policy x scale over a box at
1e-3 / 1 / 1e3). heal has no distilled campaign knowledge; this note and the
test-file header carry every surprise from the slice. Nothing here is a
kernel fix — kernel bugs found would become new §B rows, and none were found.

## What the slice covers

`fix_shape_verified` / `run_heal_pipeline_verified` (commit only a fully
disclosed result accepted by both validators) over: wire order, wire closure,
wire gaps, small edges, face orientation, small faces, shell orientation,
sewing/free bounds, duplicate faces (+ keeps). Every cell asserts dual
`validate_solid` clean + (6, 12, 8) census + all-plane surface census +
closed-form volume (24·scale³, 1e-6 relative; skipped only at 1e-3 with a
5e-8-class injection where the defect itself moves ~8e-6), or a typed refusal
(`HealingRepairRefused` / `ConfiguredHealingValidationFailed`).

## Findings (mirrored in the test header)

1. **The roadmap trap is stale.** `fix_duplicate_faces`
   (`crates/heal/src/fix/solid.rs`) no longer compares centroid/normal/edge
   count: it matches effective plane normals (1 − cos < 1e-6) plus ordered
   same-winding outer boundaries (cyclic shift allowed), planar line-bounded
   faces only. Same-centroid/different-boundary, opposite-winding, and
   reversed-flag pairs are all KEPT for distinct, named reasons. Recommend
   updating the B11 "winding-blind comparison" remainder and the roadmap
   skill trap note to describe the current comparator.
2. **Sewing is off the `fix_shape` path.** No fix_wire/face/shell/solid step
   consults `fix_wireframe`; only the `fix_wireframe` pipeline op reaches it.
   A disjoint shell through `fix_shape` refuses typed; through the pipeline it
   sews 12/12 pairs. Both pinned — a future wiring change flips the matrix
   visibly.
3. **Shell orientation needs flag toggling, not wire-flag reversal.**
   Wire-flag reversal also disconnects the joints (needs wire fixers on top);
   flag toggling is the pure orientation defect. The shell analysis AND
   `fix_orientation` both read the raw wire flag (blind to the face flag), so
   Auto gates off and only On repairs — pinned as On-repair vs
   Auto/Off-refusal. BFS is seed-sensitive: face 0 (all-reversed natively)
   refuses under every policy after 5 flips; face 1 repairs with exactly 1.
4. **Swap is a reorder+gap defect.** Reorder-only Auto commits exactly;
   reorder+gap On refuses typed (`ClosureGapTooLarge` from the widened second
   pass). Pinned as Auto-repair vs On/Off-refusal.
5. **At-tolerance closure splits by policy at 1e3** (stored 1e-7 rounds to
   ~1.0000008e-7): Auto commits, On refuses typed. Below 1e3 both repair.
6. **Mid-wire gaps are On-only** (split pair keeps distinct IDs at coincident
   positions; nominal analysis reports no gap). Closure joints differ
   (`fix_closed` measures its own distance; Auto repairs). Strict
   below-nominal (`<` vs `<=`) keeps the at-tol mid-wire gap unrepaired
   everywhere — pinned as refusal.
7. **Super-tolerance splice is not a small-edge defect** (analysis: 0 gaps,
   0 small edges; still breaks connectivity → refusal everywhere). Sub-tol
   splice is connectivity-clean → small-edge-only Auto repairs.
8. **Off fails closed** (`ConfiguredHealingValidationFailed` + rollback)
   wherever the defect is invalid; the flipped-normal Off cell commits
   (validator-clean at L3) and is asserted as a no-op commit.

## Open B17 remainder (unchanged)

Seams (detection-only by design), continuity splits, representation
conversion, the #244 faceted sew/unify contract, the operand
self-interference report. Future observation for a new row (not filed — no
failing repro on its own): shell analysis orientation-consistency is blind to
the face `reversed` flag, same as `fix_orientation` was before its comment.

## Cell counts

10 tests: order (9 cells), closure (27), gaps (27), small-edge (18),
face-orientation (9), shell-orientation (18), small-face (9),
duplicate-face (9), duplicate-keeps (3), sewing (6). Repair cells assert the
disclosed action kinds (`WireReordered`, `WireGapClosed`, `SmallEdgeRemoved`,
`FaceOrientationFixed`, `SmallFaceRemoved`, `DuplicateFaceRemoved`,
`FreeEdgePairSewn`, `ShellFaceOrientationFixed`); refusal cells assert the
typed error variant.

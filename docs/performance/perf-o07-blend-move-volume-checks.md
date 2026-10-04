# O07 subset: whole-body mesh volumes out of the blend-aware planar move

`push_pull::move_faces` on a planar face that meets a blend goes through
`resize_blend::move_planar_faces_with_blends`. On an imported body with
NURBS and sphere patches, that path spent most of its time not on the move
but on `measure::solid_volume`, which for such a body meshes the whole solid
at `diag × 5e-5`: the remove/rebuild attempt read the source volume twice
before `remove_blend_region` refused, and the translation-invariant fallback
then read source and result once each, only to assert that the volume had
changed in the direction of the distance.

This is a bounded subset of backlog row O07 (plan local direct edits against
the affected neighbourhood). It changes no public contract, no tolerance of
the remove/rebuild volume checks, and no refusal class: a body whose moved
faces decline to integrate keeps the whole-body mesh comparison.

## Change

- Remove/rebuild: one whole-body volume per stage. Every stage allocates its
  result as a new solid and leaves its input in the arena, so one reading is a
  stage's "after" and the next stage's "before" (`stage_volume`,
  `current_volume`). Nothing is meshed before a band has actually come off,
  which is where the path refuses on bodies it cannot serve. For N bands the
  path now reads 2N + 2 volumes instead of 4N + 4.
- Translation-invariant fallback: the two whole-body volumes become
  `translated_faces_volume_change`, which integrates only the translated
  faces before and after about one reference and triples the difference.
  Translating a face set S by δ with every bounding face invariant under δ
  sweeps a prism of volume δ · ∫_S n dA; by the divergence theorem the moved
  faces' own flux (1/3)∮ P·n dA accounts for exactly one third of it and the
  lateral band the invariant carriers grow or lose for the other two thirds.
  Faces that are neither moved nor carriers cancel exactly; closure is
  proven separately by `validate_exact_result`. On this path the change is
  `distance` times the region's shadow along the move, so the gate's slack is
  `1e-9 × region area × |distance| + 1e-7`: only a degenerate region (its
  shadow negligible next to its own area) is refused, independent of body
  size. The integral vouches for itself before it is used: every reading must
  be finite, each face's quadrature area must read the same before and after
  (a rigid translation preserves area, so a different reading means the two
  sides did not sample the same trim), and a face from a family whose UV
  outline the trimmed quadrature can get wrong (`gauss_unqualified_face`:
  scalloped sphere collars, torus trims outside the two-rim band family, walls
  whose rim winds the period) must agree with an independent reading of the
  same quantity that never touches the UV outline: its shadow along the move
  taken from its boundary by Stokes (`boundary_shadow_along`, magnitude only,
  2 % tolerance). Any of these failing keeps the whole-body mesh comparison
  with its original slack (`|V_source| × 1e-9 + 1e-7`), so such bodies behave
  exactly as before. On the fixture the torus corner patch carried by the move
  is such a face; it passes the cross-check.
- Disclosed behaviour changes on the integral path: a move whose volume
  change fell inside the old slack band (sub-micron moves on this body) was
  refused by mesh noise and is now judged on its shadow; a body whose
  `solid_volume` returns `Unsupported` used to fail the move up front and can
  now succeed when its moved faces integrate. An inside-out body used to be
  caught here by the magnitude comparison and is still caught by the strict
  orientation probe that follows every move.
- Closed-form oracles, each built by translating vertices, edges and surfaces
  the way the production path does: a 2 × 3 box lid (6d, both signs,
  cross-checked against the closed-form box volume) and lid plus floor
  together (the shadowless region the gate refuses); an r = 1.5 cylinder cap
  (π r² d, both signs); the drilled top of a 6 × 6 × 3 plate (36 − π)d with
  the hole as an inner wire; the ceiling of a 4-cube cavity inside a 10-cube
  (+16d, a reversed face moved into the void); the support plane plus
  quarter-cylinder band of a filleted 10-cube (100d, region area
  90 + 5π); the same with the band carried as its exact rational NURBS
  (the production hot path, trimmed by projection on both sides); and two
  filleted top edges whose corner patch rides along (100d). The boundary
  shadow has its own closed forms (rectangle, circle, quarter band, and zero
  along the face's own plane). The end-to-end test of the rigid translation
  now also checks the result volume and a negative distance.

## Measurement

Native, `profiling` profile (release + debug info, no LTO), Apple Silicon
laptop, `crates/remus/examples/offset_face_perf.rs` on
`crates/io/tests/data/shapr3d_hammer_holder.step` (160 faces: 52 planes,
42 cylinders, 42 NURBS, 24 sphere/torus/cone patches), moving the 1045.93 mm²
+X planar face by −6 mm. Same binary flags before and after; the result is
identical (160 faces, volume 40 548.365, Δ = −9 693.349 mm³).

| `push_pull::move_faces`                | before   | after    |
|----------------------------------------|----------|----------|
| wall, 5 runs                           | 1 831–1 900 ms | 998–1 010 ms |
| whole-body mesh volumes (`BK_VOL_TRACE`)| 4        | 0        |
| share: whole-body volumes              | 61 %     | 0 %      |
| share: strict `validate_solid` probe   | 38 %     | ~68 %    |
| share: translated-face integrals       | —        | ~31 %    |

Sampled with `samply record --save-only`, symbolicated against the binary.

WASM, through the OpenZCAD kernel adapter in Node 22 (one `syncDocument` of
the same offset-face direct edit, every kernel call timed): the pinned
package (`bf46fdc0`, main at the time of the consumer's pin) against a scratch
`wasm-pack --target nodejs` build of this branch with the shipped flags
(`--no-default-features`, SIMD) but without `wasm-opt`. The pin is 137
commits older than this branch, so this is the consumer-facing before/after,
not a same-source A/B; the native table above is the same-source A/B.

| kernel call per sync        | pinned package | this branch |
|-----------------------------|----------------|-------------|
| `moveFacesJournaled`        | 4 412 ms       | 1 780–1 910 ms |
| whole sync (adapter + app analysis, unchanged here) | 14 503 ms | 11 494–12 046 ms |

Result volume, face count and warnings are identical on both.

What remains is the strict validator's orientation probe (Gauss order 5 over
every face, 42 NURBS patches dominate; 0.7 s native on this body) and the
translated NURBS supports' two integrations in the new check, which share the
same evaluator. Both are the next row, not this one.

## Reproduce

```sh
cargo build --profile profiling -p remus --example offset_face_perf
BK_VOL_TRACE=1 OFFSET_PERF_ONLY_MOVE=5 \
  target/profiling/examples/offset_face_perf \
  crates/io/tests/data/shapr3d_hammer_holder.step -6 1045.93
```

`BK_VOL_TRACE` prints one line per whole-body volume route taken; the move
prints none after this change.

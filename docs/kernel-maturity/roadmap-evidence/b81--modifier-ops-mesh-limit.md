---
row: B81
---
**Fuzz Smoke 2026-11-15 timeout (run 43708174365, `modifier_ops`
timeout-6b0f8bb9) was this row's committed seed, and the harness's fault.**
The artifact is byte-identical to
`fuzz/corpus/modifier_ops/fillet-after-box-fused-with-tilted-torus-meridian-bite`.
The seed was never fast: the earlier weekly runs logged it as the slowest unit
at 74 s (2026-10-25), 92 s (2026-11-01) and 62 s (2026-11-08), each without
reporting the 10 s timeout. Locally the timeout fires on it 3/3. Replayed
outside libFuzzer (release with debug assertions), the case takes 9 to 25 s:
the fuse 0.03 s, each `fillet_v2` 2.1 to 2.7 s (all of it
`blend.volume_guard`, two `solid_volume` calls), and the harness's own
oracles about 6 s. Every hot phase is `mesh.curved-faces`. `solid_volume`
defers a holed torus to tessellation and clamps every request to diag × 5e-5,
so each reading meshes the bitten torus at about 300,000 triangles. The cost
is linear in the mesh (2.4 to 2.8 µs a triangle from d = 8.6e-3 to 5.4e-4,
against 0.4 to 0.7 µs for the same torus unbitten), so the kernel has no
unbounded loop or super-linear term here. The instrumented fuzz build takes
218 s on this seed locally. Fix (harness only): `fuzz/fuzz_targets/modifier_ops.rs`
gains `TRIMMED_CURVED_LIMIT`, which stops a case before the first measurement
when the base body's sphere and torus faces that carry a hole or a NURBS
section edge hold more than 30,000 triangles at the watertightness
deflection. This seed's bitten torus holds 74,482. A first, whole-mesh limit
(60,000) let a local 120 s session find a 5.4 s sibling at 52,338 (a torus
fused with a box through its tube), now committed as
`fillet-after-torus-fused-with-box-through-the-tube`. The frustum seed's
trimmed torus holds 22,154 and still runs; untrimmed spheres and tori and
cylinders and cones measure on exact routes and are not counted. Regressions
in the target's own tests (`test = true`):
`bitten_torus_fillet_seeds_stop_at_the_trimmed_curved_limit` (a work bound,
not a wall-clock one; it fails with the limit disabled) and
`other_committed_seeds_stay_under_the_trimmed_curved_limit`. Not chased here:
a fillet on a large bitten torus still spends seconds in its volume guard,
whose two `solid_volume` calls ask for 0.1 and are clamped to the finest
deflection; the shipped kernel's module caches (volume memo, face-mesh cache)
cut this seed's native case only to 1.9 s. The committed seed
`fillet-small-radius-on-large-disjoint-body` no longer builds its base body
(the fuse refuses), so it never reaches the modifier.

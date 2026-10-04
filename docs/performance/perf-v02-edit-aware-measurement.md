# V02 / Q02 / O07: measurement that survives a direct edit

After the V02 memo subset (#956) and the O07 blend-move subset (#954), a
direct edit on the Shapr3D hammer holder still paid for its measurements
three times, all on geometry the edit had not changed:

- `push_pull::move_faces` ends with a strict validation of its result. The
  orientation probe integrates every face about the shell's vertex-box
  centre; the move shifts that centre, and the face-integral cache keyed
  the reference point, so all 160 faces missed although 128 of them were
  untouched and 18 more were rigid translations.
- The translation-invariant blend move reads its swept prism from the
  moved faces before and after (`translated_faces_volume_change`): two
  fresh integrations of each moved face, the NURBS supports included.
- The application reads the result's volume. The `solid_volume` memo was
  keyed by the topology's cache identity, and between the move and that
  reading OpenZCAD takes a checkpoint (the next mutation clones the
  topology, a new lineage) and runs recognition probes in the same kernel
  (hundreds of generations), so it always missed and meshed the whole
  body again.

This touches backlog rows [V02](optimization-backlog.csv) (cache face
property contributions, recombine only what changed), [Q02](optimization-backlog.csv)
(persistent caching after mutation invalidation is proven) and
[O07](optimization-backlog.csv) (plan direct edits against the affected
neighbourhood). It changes no Gauss order, tolerance, threshold, refusal
class or verdict.

## Change

**Vector area from the same samples** (`remus-check`). The fixed-order
integrator accumulates `N = Σ wᵢ (S_u × S_v)ᵢ` next to the volume term on
every parametric path (trimmed quadrature, torus tube band) and records
`n̂ · area` on both planar paths, whose volume is `n̂ · ∫ (P − R) dA`. The
volume term `(1/3) Σ wᵢ (Pᵢ − R) · nᵢ` is affine in `R`, so it is exactly
re-expressible about another reference. `FaceContribution` is unchanged;
the adaptive integrator does not report `N`.

**Edit-aware face cache.** A new request, `integrate_face_volume_about`
(area and volume term), is answered from any entry of the same content —
about any reference, `I(R') = I(R) − (1/3)(R' − R) · N` — or of a rigid
translation of it, `I(f + δ, R) = I(f, R) + (1/3) δ · N`. The key no
longer holds the reference, and compares positions relative to an anchor
(the face's first boundary vertex), so a translated face finds its
source's entry and `δ` is the anchor difference. A translation is a
translation only to rounding (`p + δ` rounds; `transform_edges` rebuilds a
circle's normal as `u × v` and re-normalises its axes), so positions are
matched within `64 ε` of the larger coordinate magnitude, unit directions
within `64 ε`, lengths within `64 ε` relative, and everything else —
knots, weights, trims, tolerances, counts, flags, incidences — exactly.
The hash quantises the movable reals far more coarsely than that, so a
collision costs a comparison and a quantisation boundary a miss. The full
contribution (moments included) and `face_area` are still served only from
identical content (and, for the contribution, the identical reference),
bit for bit.

`shell_signed_volume` (the strict probe, `solid_is_inverted`,
`offset_v2`) and `translated_faces_volume_change` read through it. In the
latter the "after" side is now the "before" entry shifted by
`(1/3) δ · N` — exactly the prism the function is after, from the same
samples — so its same-area self-check holds by construction; the
boundary-shadow cross-check for unqualified families is unchanged.

**Content-keyed volume memo** (`remus-operations`). A `solid_volume`
reading is found by identity first and then by content: every field the
volume routes read about the solid — shells, faces, wires, edges,
vertices and the pcurve of every oriented edge use the tessellator may
consult — recorded bit for bit together with every handle. The volume
routes read nothing else of the topology (no other solid, registry,
journal, attribute or whole-arena index; the face adjacency they build is
solid-local), so equal content means an equal reading even after the
identity moved on, and any edit of the solid's content still retakes it.
Requests are matched by the deflection the reading is taken at — the
request clamped to `diag × 5e-5` before any mesh, every route before the
clamp being deflection-independent — so a 0.05 reading answers a 0.08
request. Keys are bounded by a 16 MiB budget (64 readings); a key over
budget keeps its reading findable by identity only. In WASM that budget
comes on top of the face cache's 16 MiB, and both stay allocated for the
life of the module.

**Seeded result volume.** The remove/rebuild and prismatic paths measure
their result with `solid_volume`, so the memo already holds it. The
translation-invariant path does not measure its result: when the memo
holds a measured reading of the source, the move hands back
`source + prism` and `push_pull` seeds it once the result has passed
strict validation (the sum assumes an outward-wound body). Seeded
readings are never used to derive another, so edits do not compound them.

## Accuracy

- Re-referenced volume terms agree with fresh integration to 3.7e-16 of
  the face's own scale (`area × reach`) over the exact and sampled planar,
  torus-band, reversed and NURBS paths, at three references up to ~900
  units away; translated ones to 7e-16, including the `transform_edges`
  re-normalisation. Pinned at 1e-13 and 1e-12.
- The hammer holder's probe on the moved result: 40 552.044 764 151 08
  against 40 552.044 764 151 18 uncached (2.5e-15). Verdicts identical.
- Within one body, a face that is a translation of another (the holder has
  such pairs) is read from its sibling, so even a cold probe may differ
  from the uncached one in its last bits. `b28_probe_memo_identity` now
  pins the signed volume at the 1e-12 it already pinned the recorded
  values at; reports, probe shells, orders, face counts and face areas
  stay bit-identical, and warm and deserialized-copy probes replay the
  cold one bit for bit.
- The seeded result volume is the source's closed-mesh reading plus the
  exact prism; a fresh reading meshes the result at its own clamp, whose
  bias differs on the swept band and by the small change of clamp. On the
  hammer holder: 40 548.188 seeded against 40 548.365 fresh (−0.18 mm³,
  −4.4e-6), against a chord bound (clamped deflection × surface area) of
  65.5 mm³. The import volume is unchanged; the result's displayed volume
  therefore moves by that amount when the source was measured first. On an
  exactly integrated body (the filleted box) the two agree to 1e-9.

## Measurement

Native, `profiling` profile, Apple Silicon, `offset_face_perf` in its new
`OFFSET_PERF_MEMOS` mode on `crates/io/tests/data/shapr3d_hammer_holder.step`:
both memos on as the WASM kernel runs them, the source measured and
strictly validated first (the import), the −6 mm move of the 1045.93 mm²
+X face on a clone, then the result validated and measured on another
clone. Baseline `aa5a85b3` (#956 merged with #954), candidate this branch
(`81c81a4f`); three interleaved rounds per row on a machine at load
average 9–40, so the ranges are wide.

| memos on | baseline | this branch |
|---|---:|---:|
| `push_pull::move_faces` | 392–411 ms | 14.7–15.2 ms |
| — its final strict validation | 279–585 ms | 4.1–6.2 ms |
| — `translated_faces_volume_change` | 109–242 ms | 0.2–0.3 ms |
| `solid_volume(result, 0.08)` afterwards | 280–387 ms | 0.1 ms |
| `validate_solid(result)` afterwards | 0.6–0.8 ms | 0.7 ms |

What the move reads, on the candidate: the 15 moved faces (7 cylinders,
2 NURBS supports, 3 tori, 2 spheres, 1 plane) are 30 cache hits and no
integration in the prism check; the final probe's 160 faces are 128
re-referenced, 18 translated and 14 integrated — the carriers whose
boundary the move re-limited. No whole-body volume is meshed; one is
seeded. The consumer's validation and volume afterwards are all hits.
Retained after the edit: 168 face entries (633 KB), 2 volume readings
(842 KB of content keys).

With the memos off (library default) both builds integrate everything;
the extra vector-area accumulation is three multiply-adds per Gauss point.
Interleaved strict validations at load average 30–42 read 340–694 ms
(baseline) and 365–966 ms (candidate), medians 417 and 441 ms: within that
noise, not resolved further.

WASM, Node 22, the OpenZCAD harness `test/perf/offset-face-perf.test.ts`
(one `syncDocument` of the offset-face direct edit, every kernel call
timed) against scratch `wasm-pack --target nodejs --release --no-opt`
builds of both trees with the shipped flags (`--no-default-features`,
SIMD), three interleaved runs each:

| kernel call in the edit sync | baseline | this branch |
|---|---:|---:|
| `moveFacesJournaled` | 1 083–1 091 ms | 44.7–45.0 ms |
| `volume` (the result, 0.08) | 766–788 ms | < 1 ms |
| `validateSolid` × 12 | 189–198 ms | 2.7–2.9 ms |
| whole edit sync | 7 282–7 685 ms | 5 306–5 344 ms |
| cold import sync | 7 341–7 419 ms | 7 172–7 287 ms |

This base does not include #955, so `recognizeFeatures` (3.0–3.4 s) is
unchanged here; display tessellation (0.92 s) is untouched. Warnings and
face counts are identical; the import volume is identical and the edited
body's volume reads 40 548.186 instead of 40 548.363 (see *Accuracy*).

## Reproduce

```sh
cargo build --profile profiling -p remus --example offset_face_perf
F=crates/io/tests/data/shapr3d_hammer_holder.step
OFFSET_PERF_MEMOS=3 target/profiling/examples/offset_face_perf $F -6 1045.93
OFFSET_PERF_ONLY_MOVE=5 target/profiling/examples/offset_face_perf $F -6 1045.93
```

The WASM rows build each tree with
`cd crates/wasm && CARGO_TARGET_DIR=<scratch> CARGO_ENCODED_RUSTFLAGS=$'-C\x1ftarget-feature=+simd128' wasm-pack build --target nodejs --release --no-opt --out-dir <scratch>/pkg-node -- --no-default-features`
(and the same for `crates/wasm-io`), then run, from an OpenZCAD checkout,
`REMUS_WASM_PKG=<pkg-node> REMUS_WASM_IO_PKG=<pkg-io-node> OFFSET_PERF_STEP=$F pnpm vitest run test/perf/offset-face-perf.test.ts --reporter=verbose`.

Tests: `face_cache` unit tests (re-referencing on every path, translated
copies including re-normalised axes, non-translations missing),
`measurement_memos` (content reuse and retake per mutation path, clamped
deflection, pcurve writes, clones, byte budget), the blend-move tests in
`resize_blend` (prism through the cache, seeded volume, remove/rebuild
reading), `edit_aware_measurement` (the hammer move end to end) and the
WASM `a_moved_result_volume_survives_checkpoints_and_unrelated_allocations`.

## Not done

- **A volume seed for the remaining moves.** Only the translation path
  derives a value. Other moves are found in the memo only if they measured
  their result with `solid_volume` themselves; the translation path's
  whole-body fallback measures a scratch copy that carries other handles,
  so its reading is not the result's and nothing is seeded there.
- **Recombining a whole-body mesh volume.** The mesh route is not a sum of
  per-face readings (shared boundary sampling), so the seed derives from
  the source reading instead; exact routes would allow per-face
  recombination, not needed for the measured body.
- **Translated `face_area` and full contributions.** Reported areas and
  moments stay bit-identical; only the probe's volume terms are reused
  across references and translations.

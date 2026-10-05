# V02 subset: strict validation's orientation probe and repeated measurement reads

Strict `validate_solid` proves each shell faces the right way by integrating
its signed volume over every face at the default Gauss order
(`ValidationOptions::default()` → `OrientationCheck::Order(5)`) and reading one
sign. On the Shapr3D hammer holder (160 faces, 42 NURBS patches up to 38 × 58
bicubic, 24 sphere/torus/cone patches) that probe was the whole cost of strict
validation, and applications pay it repeatedly. OpenZCAD's import and
offset-face syncs each make twelve `validateSolid` calls (2.2 s in WASM on the
consumer's pinned package); on import the holder itself is strictly validated
three times under different handles — `validateSolidDetailed` on the imported
body, `validateSolid` on its deserialized copy in a short-lived probe kernel,
and on a sibling with one face rebuilt — and `push_pull::move_faces` validates
every result before the consumer validates its copy again.

This is a bounded subset of backlog row V02 (cache face property contributions,
recombine only what changed), with a slice of N06 (reuse per-axis basis values
across quadrature abscissae). It changes no tolerance, Gauss order, sample
count, threshold or verdict: every reading is bit-identical to `main`.

## Where the time went

`samply` on five strict validations of the holder (baseline `4437751c`,
`profiling` profile, symbolicated against the binary):

| share of `validate_solid` (inclusive) | before |
|---|---:|
| `shell_signed_volume` (the orientation probe) | 99.9 % |
| NURBS first-order solve (`derivatives_into_with_spans`) | 77.6 % |
| of which A2.3 basis (`ders_basis_funs_into`) | 32.2 % |
| trim spans (`UvTrim::v_spans`) | 10.4 % |
| trim containment (`UvTrim::accepts`, partly inside `v_spans`) | 6.4 % |

Per face, the 42 NURBS patches took 638 of 720 ms; the four largest alone
378 ms. The 38 × 58 patch is integrated at 469 425 abscissae (its trim outline
splits `u` into 498 windows), each a full first-order rational solve: both
bases recomputed, and every one of the 16 control-point terms divided by the
weight scale although every weight of the imported B-splines is 1. That came
to ~250 ns an abscissa against ~23 ns on a cylinder.

## Change

**First-order NURBS solve** (`remus-math`, all scratch entry points:
quadrature, the SSI marcher's partials and normals):

- the u-direction basis is memoized in `DerivativeScratch` while the abscissa
  repeats — tensor-product quadrature walks every `v` at one `u` — keyed by
  exactly what A2.3 reads (clamped `u` bits, span, degree, the `2p` local
  knots), so a hit is the value a fresh call returns, also when one scratch
  meets another surface;
- the three needed homogeneous cells are contracted in one pass, each still
  summed `i`-major, `j`-minor from `0.0`;
- when every weight carries the same finite, positive bits (cached on the
  surface, like `max_weight`), the per-term `w / max_weight` is exactly `1.0`
  and `(c · x) · 1.0 = c · x`, so the division and product are skipped;
- `basis::ders_basis_funs_first_into` is the `n_derivs = 1` case of A2.3
  without the 121-entry `ndu` table: same operations, same order.

Bit-identity to `derivatives(u, v, 1)` is pinned over unit, uniform, rational,
one-ulp-off, ruled, bilinear, repeated-knot and degree-9 fixtures, and for the
basis over every degree 0–12; the tests fail when an operation is reordered.
The `simd` feature keeps its 4-lane general solve unchanged.

**Content-keyed face-integral cache** (`remus_check::properties::face_cache`):
`FaceIntegralCache` memoizes `integrate_face_fixed_about` and
`integrate_face_area` by everything those integrators read — request kind,
Gauss order and reference point, the reversal flag, every surface field, and
per wire the closed flag and oriented edges with every curve field, trim, edge
tolerance and both vertices — with handles replaced by first-occurrence
ordinals. A hit is therefore the value a fresh integration computes, and there
is no invalidation to get wrong: an edited face simply stops matching. Whole
keys are compared (a hash collision costs a miss), the cache is bounded at
8 192 entries and 16 MiB of keys with deterministic FIFO eviction, and errors
are never cached. A per-thread instance backs `*_memoized` functions; it is
**off by default**, so library callers, the B28 guard and the
`nurbs_properties` cold bench see the uncached integrators.

The probe's reference point (the shell's vertex-box midpoint, B58) is part of
the key. Re-expressing each face about its own reference plus a vector-area
correction would let a face survive its shell's bounding box moving, but
would change the probe's signed volume in its last bits; it was not done (see
below).

**Operations** (`remus-operations`): `shell_signed_volume` (strict
validation, `solid_is_inverted`, `offset_v2`) and `face_area`'s two quadrature
paths go through the thread cache. `solid_volume` gains an opt-in per-thread
memo keyed by the PERF-Q02 topology cache identity (lineage + generation), the
solid and the deflection bits: every allocation, `*_mut` access, replacement,
retirement, registry write and restore/rollback bumps the generation, so
`transform_solid` (which edits in place), healing in place,
`RollbackSnapshot::restore`, `restore_preserving_handle_slots` and
`delete_solid` all retire a reading. Tests drive each of those paths and fail
when the memo ignores the generation.

**WASM**: `BrepKernel::new` enables both memos for the thread. The face cache
is module-wide on purpose — the probe kernel's validation of a deserialized
copy, and a consumer's re-validation of a result `moveFaces` has just
validated, are hits.

## Evaluated and not adopted

- **NURBS `face_area` by trimmed quadrature instead of tessellation.** On the
  holder's 42 patches the tessellation at 0.08 costs 42 ms; the fixed-order
  quadrature costs 949 ms at order 5 and 2 466 ms at order 8, and differs from
  the tessellated reading by up to 6e-3 relative (it is the more accurate one:
  against a 0.001 tessellation the 0.08 reading is off by up to 6e-3, in either
  direction, and the order-8 quadrature by at most 6e-4). Not adopted; `face_area`'s cost on this body is the
  analytic faces' order-8 quadrature, which the cache now serves on repeats.
- **A degree-aware Gauss order for NURBS faces in the probe.** Order 3 on the
  NURBS faces cuts their probe cost from ~210–275 ms to ~100 ms but moves the
  holder's signed volume from 50 245.5213 to 50 245.4672 (1.1e-6 relative; the
  verdict needs only a sign against a 1.2e-3 floor). The degree argument does
  not make it exact: the probe integrates `P · n`, of degree `3p − 1` per axis
  on a non-rational degree-`p` patch (order ⌈3p/2⌉ = 5 for bicubic), not the
  `2p − 1` vector area, and the fixed rule's cells are not knot-aligned. The
  B28 guard pins the order (`probe.order == PropertiesOptions::default()
  .gauss_order`, "a change of order is a threshold change, not a timing win"),
  so this is left as an option for the guard's owner: record a per-surface
  order in `ShellOrientationProbe` and re-baseline the signed volume.
- **Face-local references in the probe** (above): would let unchanged faces
  hit after a move changes the shell's box, at the cost of last-bit changes to
  the signed volume. On the measured move it would not have paid: the −6 mm
  move of the holder's +X face rebuilds 69 of 160 faces, including all eight
  large NURBS patches (562 of 674 ms of the result's probe), and shifts the
  box centre from (11, 33, 33.5) to (14, 33, 33.5).

## Identity

Native, same platform, `main` (`4437751c`) against this branch:

- every per-face orientation integral of the holder (160 faces) is
  bit-identical;
- for the holder and the three OpenZCAD plate fixtures, the strict report, the
  budget probe (shell, order, face count, signed-volume bits), the hash of all
  face areas and the solid volume are bit-identical with the memos off, on,
  and warm;
- through WASM (the Node harness below), the strict verdicts, the face-area
  sum and the volumes of the body and of the moved result are identical;
- `b28_probe_memo_identity` pins the same in CI (off, cold, warm, deserialized
  copy) and pins the signed volumes to the recorded values (1e-12 relative,
  since platform `libm` may move the last bits of analytic faces).

## Measurement

Native, `profiling` profile (release + debug info, no LTO), Apple Silicon,
`crates/remus/examples/measurement_memo_perf.rs` on
`crates/io/tests/data/shapr3d_hammer_holder.step`. Baseline `main` at
`4437751c`; candidate this branch. Each cell is the median over several fresh
processes with baseline and candidate interleaved (alternating order per
round), on a shared machine at load average 6–11; minima sit within 2 % of
the medians. Memos are off unless the row says otherwise.

| read | `main` | this branch |
|---|---:|---:|
| strict `validate_solid`, n = 20 | 705.8 ms | 271.1 ms |
| — repeated, face cache on | — | 0.5–0.7 ms |
| — deserialized copy in a fresh topology, cache on | — | 0.6 ms |
| `solid_volume(0.08)`, n = 10 | 273.9 ms | 267.2 ms |
| — repeated, volume memo on | — | < 0.05 ms |
| `face_area(0.08)` over all 160 faces, n = 10 | 118.9 ms | 113.1 ms |
| — repeated, face cache on | — | 15.6 ms |
| `push_pull::move_faces`, +X face by −6 mm, n = 6 | 1 808.8 ms | 1 365.2 ms |
| the same on top of #954 (`bc42f525`), n = 9 | 1 023.2 ms | 387.9 ms |

The cold `solid_volume` is unchanged by design: this body is measured on its
closed whole-solid mesh, which the first-order solve does not touch; only the
memo serves a repeat. A repeated `face_area` still tessellates the 42 NURBS
faces (the 15.6 ms); everything else comes from the cache. On top of #954 the
move gains more than the probe alone because #954's translated-face check
integrates the moved NURBS supports with the same first-order solve.

The cached run retains 231 entries (160 probe integrals, 71 quadrature areas)
in 682 KB of keys.

WASM, Node 22: `wasm-pack build crates/wasm --target nodejs --release
--no-opt -- --no-default-features` with `RUSTFLAGS="-C
target-feature=+simd128"` (the shipped flags without `wasm-opt`) for `main`
and this branch, driven by the script under *Reproduce*: one kernel loads the
holder's arena document, a second short-lived kernel validates a deserialized
copy, then face areas, volume, and a move of the +X face. Six fresh processes
per build, interleaved, at load average ~30 (other sessions), so the cold rows
are noisier than the native table; medians:

| kernel call | `main` | this branch |
|---|---:|---:|
| `validateSolid` (first) | 1 509 ms | 887 ms |
| `validateSolid` (repeat) | 1 484 ms | 2.2 ms |
| `validateSolid` (deserialized copy, probe kernel) | 1 479 ms | 1.4 ms |
| `faceArea` × 160 (first / repeat) | 388 / 386 ms | 386 / 44 ms |
| `volume` (first / repeat) | 911 / 854 ms | 892 / 0.0 ms |
| `moveFaces` (+X face, −6 mm) | 4 943 ms | 3 554 ms |
| `validateSolid` of the moved result | 1 466 ms | 1.2 ms |

The moved result's validation is a hit because `moveFaces` validated it
internally a moment before. Verdicts, the face-area sum and every volume are
identical between the two builds.

## Reproduce

```sh
cargo build --profile profiling -p remus --example measurement_memo_perf
B=target/profiling/examples/measurement_memo_perf
F=crates/io/tests/data/shapr3d_hammer_holder.step
$B validate 5 $F   # strict validate_solid, memos off
$B volume 5 $F     # solid_volume(0.08), memos off
$B areas 5 $F      # face_area(0.08) over all faces, memos off
$B move 3 $F       # push_pull::move_faces of the 1045.93 mm² +X face by -6
$B memo 3 $F       # the same reads through the memos: cold, warm, copy
$B arena 1 $F > hammer.arena   # exact arena document for the WASM run
```

The WASM rows build each tree with
`RUSTFLAGS="-C target-feature=+simd128" wasm-pack build crates/wasm --target
nodejs --release --no-opt --out-dir <pkg> -- --no-default-features` (outside
the committed `crates/wasm/pkg`) and run `node harness.cjs <pkg>
hammer.arena`:

```js
const { performance } = require('node:perf_hooks');
const fs = require('node:fs');
const path = require('node:path');
const pkg = require(path.resolve(process.argv[2], 'remus_wasm.js'));
const t = (label, f) => {
  const s = performance.now();
  const r = f();
  console.log(label.padEnd(44), (performance.now() - s).toFixed(1), 'ms');
  return r;
};
const k = new pkg.BrepKernel();
const [solid] = k.deserializeSolids(fs.readFileSync(process.argv[3]));
t('validateSolid (first)', () => k.validateSolid(solid));
t('validateSolid (repeat)', () => k.validateSolid(solid));
const probe = new pkg.BrepKernel();
const [copy] = probe.deserializeSolids(k.serializeSolids(Uint32Array.of(solid)));
t('validateSolid (copy in a probe kernel)', () => probe.validateSolid(copy));
const faces = Array.from(k.getSolidFaces(solid));
const areas = () => faces.reduce((a, f) => a + k.faceArea(f, 0.08), 0);
t('faceArea x160 (first)', areas);
t('faceArea x160 (repeat)', areas);
t('volume (first)', () => k.volume(solid, 0.08));
t('volume (repeat)', () => k.volume(solid, 0.08));
let best, gap = Infinity;
for (const f of faces) {
  if (k.getSurfaceType(f).toLowerCase() !== 'plane' || k.getFaceNormal(f)[0] <= 0.99) continue;
  const g = Math.abs(k.faceArea(f, 0.05) - 1045.93);
  if (g < gap) { gap = g; best = f; }
}
const moved = t('moveFaces (+X face, -6)', () => k.moveFaces(solid, [best], -6));
t('validateSolid (moved result)', () => k.validateSolid(moved));
t('volume (moved result)', () => k.volume(moved, 0.08));
```

## Next

- Trim bookkeeping is the next-largest share of a cold probe (the NURBS solve
  is ~47 %): `UvTrim::v_spans` scans every outline edge for vertical
  crossings at every `u` abscissa and `accepts` takes a winding number over
  every outline edge per span, ~26 % together. The first feeds a sort and the
  second an integer sum of exact `orient2d` decisions, so a per-loop interval
  index that visits only the edges spanning the query can stay
  bit-identical.
- The volume memo misses after any allocation, because the topology identity
  cannot tell an unrelated allocation from an edit. A generation that skips
  pure appends would let a solid's reading survive other operations; it needs
  an audit that no volume path reads whole-arena state or indices an
  allocation extends (the edge-to-coedge index grows when a new face reuses an
  existing edge).

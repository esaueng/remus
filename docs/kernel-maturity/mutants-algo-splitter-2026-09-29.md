# remus-algo face-splitter and arrangement mutation triage — 2026-09-29

One row per survivor of the 2026-09-27 weekly mutation run in
`crates/algo/src/builder/face_splitter/arrangement.rs`, `arrangement/{geometry,periodic,regions}.rs`
(the isolated O2.3b arrangement engine that PR #822/#837 promote to production) and
`crates/algo/src/builder/face_splitter/mod.rs`. Every survivor is either killed by a new test whose
oracle is independent of the code under test, or proven equivalent or unkillable in one line. No
survivor exposed a defect. One open engine limitation was found while writing the oracles (not by a
mutant) and is filed below as a ready-repro. There is no production change.

## Source and scope

- Source of truth: the weekly Mutation Testing run of 2026-09-27 on `main` at `763ad346` (every
  shard red). Its survivor list for these files, one mutant per line, was supplied with the task and
  is reproduced in the table at the end; `/tmp/splitter-mutants.txt` during the session.
- In scope: 171 survivors — 114 in the arrangement engine (`arrangement/geometry.rs` 61,
  `arrangement/periodic.rs` 26, `arrangement.rs` 22, `arrangement/regions.rs` 5) and 57 in
  `mod.rs` (`split_periodic_face_by_rim_chains` 40, `loops_orphan_boundary_edges` 14,
  `split_face_2d_impl` 7).
- The five files are byte-identical between `763ad346` and this branch's base
  (`c8075051`, current `main`); `git diff 763ad346 origin/main -- crates/algo/src/builder/face_splitter/`
  is empty, so the CI line numbers hold on this tree.
- Why they survived: the existing arrangement tests build every use with a pcurve range starting
  at 0 (so `t − range[0]` and `t + range[0]` coincide), never place two seam heights or two rulings
  where the quotient's bookkeeping differs, and never probe a comparison at exactly its bound; the
  `mod.rs` splitter tests use symmetric notches whose chain is flat on the probe meridian.

## Method

- Tool: `cargo-mutants 27.0.0` (the CI pin), Rust 1.96.0, the committed `.cargo/mutants.toml`
  (profile `ci-test`, nextest, `--tests`, first-failure stop, per-package oracle) with only the
  `examine_globs` narrowed to the files in scope (`--config` with a copy of the file; the committed
  config's `crates/algo/src/**` glob is unioned with `-f`, so `-f` alone cannot narrow it). The
  oracle is therefore the whole `remus-algo` suite, as in CI.
- Before: one run of all 688 arrangement-file mutants on the branch base, before any new test
  (`688 tested: 471 caught, 61 unviable, 156 missed, 0 timeout`). After: one run of the same 688 and one of all `3510` `mod.rs` mutants
  on the final test tree (`688 tested: 598 caught, 61 unviable, 29 missed, 0 timeout`; `3510 tested: 605 caught, 504 unviable, 2401 missed, 0 timeout`).
- Kill attribution: the files are unchanged from the CI tree, so a CI survivor caught after can
  only be caught by a new test. The table names the first failing test from each mutant's log.
- Verdicts: (a) killed by a new test; (b) equivalent (no input distinguishes it) or unkillable
  (distinguishable only below a stated resolution, or not at the public boundary); (c) real defect.
  There are no (c) rows.

## New tests and their oracles

| Test module | What it pins | Oracle |
| --- | --- | --- |
| `builder/face_splitter/arrangement/oracle_tests.rs` | `roundoff`, `finite`, `validate_use` | The formula asserted at three magnitudes and its monotonicity; one malformed input per disjunct of every `\|\|` chain (a NaN in exactly one 3D coordinate, a `Circle2D` whose `2π` end overflows while its `π` end is finite, a `Circle3D` centre poisoning one coordinate), with the exact variant and a neighbouring accepted input; a 3D end exactly `tolerance` (0.5) off its stored point |
| same | `intersections`, line–line | Vertical/horizontal integer segments whose four orientation signs are written out (endpoint on the other support, both sides, certified-disjoint pairs that must charge no work step), the 3-4-5 crossing of an 8×6 box's diagonals, collinear touch vs overlap, endpoint-certificate snapping that must not move a hit 2 away from the certified end |
| same | `intersections`, line–circle | The `y = 3` chord of the radius-5 circle (roots `(∓4, 3)`, angles `atan2(3, 4)`); tangency band `64·ε·(64 + 64 + 9)` probed with a line `δ` past the tangent height so the discriminant is `−6δ`: 133 units inside, 300 outside, exact tangent in and out of the line's range; the `y = 2` chord roots `−8 ∓ √5` |
| same | `intersections`, circle–circle | Radius-5 circles 8 apart meet at `(4, ±3)`; radii 5 and 3 at distance 5 (`x = 4.1`, `h² = 8.19`); external tangency at `(5, 0)` refused only when on both arcs; band `64·ε·(25 + x²) ≈ 50` probed at 40 and 300 units with `h² ≈ −3.75δ`; an overflowing radius refused as non-finite; coincident carriers meeting at one native parameter, across the seam, and overlapping by 1e-9 |
| same | `tangent_order` | An eight-point compass fan sorted from a scrambled order; strict comparisons both ways; parallel tangents `Equal`; a reversed use flips the half-plane |
| same | `ray_crossing` | A seven-row probe table on a vertical span traversed both ways (interior, right, on the span, bottom end included, top end excluded, below, above), the `(0,0)→(4,4)` diagonal at height 3, and the radius-5 quadrants at height 4 (`x = ±3`) |
| same | `distance` | Point-to-segment clamps (4, 5, 5, 2, 2 on a length-10 segment) and the unit quarter arc's interior/nearer-end/equidistant cases |
| same | `source_parameter`, `point_3d`, `uv_distance`, `capacity` | Affine maps on the range `[2, 6]` → `[10, 20]` and 3D `(0,0,0)→(4,8,12)`; `hypot(6, 4) = √52` with `u_scale = 2`; `capacity(queue_size)` accepted, one more refused, one step per call |
| same | `build_arrangement` refusals and budgets | Unsupported radii refused before any step (zero-budget context); duplicate use ids; shared-endpoint gaps of exactly `tolerance` (2D and 3D, `1/16`) accepted, `1e-12` more refused, `1.0` refused as `InvalidBoundary`; a full circle needs exactly four event records and a chord-closed quarter arc four; a square needs exactly four segments; a 3D side whose interior point overflows in one axis refused as `NonFiniteInput` |
| same | `quotient` | Plain strip (χ = 0, area `12π`, bottom rim winding +1 / top −1, both seam orientations), one band (three unit-lift identifications at heights 0, 1.5, 3; two `6π` annuli), two rulings (a joined disc and a lone disc, `12` and `6(2π − 2)`); every seam-domain disjunct refused before work; seam spacing exactly at the roundoff bound (`x = 2π + 2⁻⁴³`, `y₀ = 8 − (1 + x)`) passes the spacing check; material outside the seam interval on either side; a 3D seam gap of exactly `tolerance`; rim windings `135·2⁻⁵⁰` too wide refused, `56·2⁻⁵⁰` accepted; a boundary walk detouring through a non-boundary twin refused `OpenRegion`; a duplicated hole entry refused `NonManifoldEmbedding` |
| `builder/face_splitter/rim_chain_oracle_tests.rs` | `loops_orphan_boundary_edges` | Hand-built boundary/loop lists: direction-agnostic cover, zero-extent lines skipped, endpoints `0.24` vs `0.26` cells apart per axis (`tol = 0.5`), arcs on either side of `span·r ≤ tol` with `r = 4`, a full rim keyed, and two co-endpoint rim halves split where `0.5/a = a + 0.5/(2π − a)` |
| same | `split_periodic_face_by_rim_chains`, interior points | A skewed notch `d(1 − x²)(1 + x)`: lens area `(4/3)·α·d·r`, lens interior at `z = d/2` on the centre meridian, band interior at `z = h/2` on `θ = π`; a sector pair whose bowed chain approaches the seam-side probe meridian without crossing it and whose straight chain crosses its antipode, both interiors at `h/2`, clear-sector area `5.8 + 0.3 − 0.4·(2/3)` |
| same | `split_periodic_face_by_rim_chains`, weld gates | Dyadic tolerances (`2⁻¹⁰`, `2⁻²⁰`) so the `100·tol` weld is exact: a vertical section piece exactly one weld long kept (half refused), a junction gap exactly one weld wide not chained in all four attachment orders (half chained), a bottom rim shifted exactly one weld off the seam meridian refused (half accepted), chain ends half a weld before or after the seam refused (two welds accepted) |
| same | `split_face_2d` on a frustum | Two straight rulings at θ = 1 and 3 give sectors of `2h` and `(2π − 2)h` in the developed chart, each ruling shared once by both |

## Before and after, by function

| File | Function | CI survivors | (a) killed | (b) equivalent / unkillable |
| --- | --- | ---: | ---: | ---: |
| `arrangement.rs` | `CurveUse::point_3d` | 2 | 2 | 0 |
| `arrangement.rs` | `Work<'a>::uv_distance` | 3 | 3 | 0 |
| `arrangement.rs` | `Work<'a>::capacity` | 1 | 1 | 0 |
| `arrangement.rs` | `build_arrangement` | 15 | 14 | 1 |
| `arrangement.rs` | `CurveUse::source_parameter` | 1 | 1 | 0 |
| `arrangement/geometry.rs` | `intersections` | 34 | 29 | 5 |
| `arrangement/geometry.rs` | `roundoff` | 2 | 2 | 0 |
| `arrangement/geometry.rs` | `finite` | 2 | 2 | 0 |
| `arrangement/geometry.rs` | `tangent_order` | 6 | 5 | 1 |
| `arrangement/geometry.rs` | `ray_crossing` | 4 | 3 | 1 |
| `arrangement/geometry.rs` | `distance` | 4 | 3 | 1 |
| `arrangement/geometry.rs` | `validate_use` | 9 | 7 | 2 |
| `arrangement/periodic.rs` | `quotient` | 26 | 24 | 2 |
| `arrangement/regions.rs` | `extract` | 4 | 0 | 4 |
| `arrangement/regions.rs` | `left_seed` | 1 | 0 | 1 |
| `mod.rs` | `split_periodic_face_by_rim_chains` | 37 | 17 | 20 |
| `mod.rs` | `loops_orphan_boundary_edges` | 13 | 11 | 2 |
| `mod.rs` | `split_face_2d_impl` | 7 | 3 | 4 |
| **Total** |  | **171** | **127** | **44** |

Survivors after: 44 of 171, all with a one-line proof.

## Equivalent and unkillable survivors

| Mutant | Function | Mutation | Verdict | Proof |
| --- | --- | --- | --- | --- |
| `arrangement.rs:298:29` | `build_arrangement` | replace > with >= | (b) equivalent | `range[0] == range[1]` is refused by `validate_use` (`same(range[0], range[1])` → `InvalidBoundary`) on the line before the swap. |
| `arrangement/geometry.rs:172:29` | `intersections` | replace / with * | (b) unkillable | `aa = |d|²` of a `Line2D::new`-normalised direction lies within 2 ulp of 1, so `q/aa` and `q·aa` differ by at most 2 ulp of the root. |
| `arrangement/geometry.rs:215:35` | `intersections` | replace > with >= | (b) equivalent | `height2 == 0` is caught by the tangency test on the line before (`height2.abs() <= error`, with `error ≥ 0`), so the branch never sees a zero. |
| `arrangement/geometry.rs:230:27` | `intersections` | replace \|\| with && | (b) unkillable | Each hit's two parameters come from one finite 2D point (line–circle: `a.point(t)` and its angle; circle–circle: `p` and its two angles) or from exact orientation-certified endpoints; a lone non-finite coordinate needs a coordinate difference beyond `f64::MAX`, which first breaks the `orient2d` certificates or the `disc`/`height2` finiteness gates guarding the branch. |
| `arrangement/geometry.rs:247:57` | `intersections` | replace > with >= | (b) unkillable | Differs only when a hit's residual `uv_distance` equals `tolerance.linear` to the bit; certified hits have residual exactly 0 and computed roots carry roundoff-scale residual (~1e-16 · scale), neither of which can be made to equal a chosen tolerance. |
| `arrangement/geometry.rs:254:43` | `intersections` | replace && with \|\| | (b) equivalent | Two hits equal in one parameter are the same 2D point (parameters are single-valued in `[0, 2π)` or on a line), hence equal in the other; `dedup_by` removes the same pairs either way. |
| `arrangement/geometry.rs:268:68` | `tangent_order` | replace < with <= | (b) equivalent | Differs only for the zero tangent `(0, 0)`; tangents are a unit line direction or a radius-length circle tangent (radius > 0), and `signum` of a zero-length range is 1. |
| `arrangement/geometry.rs:310:79` | `ray_crossing` | replace * with / | (b) equivalent | `mid.cos().signum()` is ±1 (never 0 for a cardinal-split arc's midpoint), and `x·(±1) == x/(±1)` exactly. |
| `arrangement/geometry.rs:330:56` | `distance` | replace < with <= | (b) equivalent | Differs only when both arc ends are equidistant from `p`; either end then gives the same returned distance. |
| `arrangement/geometry.rs:63:39` | `validate_use` | replace \|\| with && | (b) equivalent | A non-finite line origin or direction makes `u.point(range[0])` non-finite, which line 36 refuses first; neither disjunct is reachable alone. |
| `arrangement/geometry.rs:68:41` | `validate_use` | replace \|\| with && | (b) equivalent | A non-finite circle centre or radius (∞ or NaN; `Circle2D::new` admits NaN) makes `u.point(range[0])` non-finite, refused at line 36; neither disjunct is reachable alone. |
| `arrangement/periodic.rs:163:35` | `quotient` | replace / with * | (b) equivalent | Only non-counterpart edges take the default key; every accepted region holds exactly one half of each such edge (a region holding both halves of an interior edge fails the χ check regardless of the key, see the single-ruling repro), so `h/2` and `2h` count the same edges. |
| `arrangement/periodic.rs:205:44` | `quotient` | replace > with >= | (b) unkillable | Differs only when `|Δu − 2π|` equals `2⁻⁴⁶·(1 + |Δu|)` to the bit; `Δu − 2π` is a multiple of `ulp(2π) = 2⁻⁵⁰` while the bound is `2⁻⁴⁶` times a value near 7.28, never such a multiple. |
| `arrangement/regions.rs:102:84` | `extract` | replace - with + | (b) unkillable | The compensated sum's correction is below one ulp of the running area; adding instead of subtracting it changes the area by at most a few ulps over a cycle's ≤ dozens of edges (~1e-15 relative). |
| `arrangement/regions.rs:129:36` | `extract` | replace > with >= | (b) equivalent | A cycle with `signed_area == 0` is refused as `OpenRegion` (`same(area, 0.0)`) before this classification. |
| `arrangement/regions.rs:151:51` | `extract` | replace < with <= | (b) equivalent | Two outer cycles that both contain a hole's seed are nested (faces of a planar subdivision have disjoint interiors), so the inner one has strictly smaller area; no tie is reachable. |
| `arrangement/regions.rs:175:27` | `extract` | replace += with -= | (b) equivalent | Material is the parity of the crossing count and `rem_euclid(2)` of `−n` equals that of `n`. |
| `arrangement/regions.rs:62:16` | `left_seed` | replace *= with += | (b) equivalent | The seed sits on the sub-curve's own normal at its midpoint, so its distance to that sub-curve is `offset` (≥ `offset/2`) on the first pass for lines and for cardinal-split arcs (`offset ≤ clearance/8 ≤ 0.09·r` never crosses the centre); the shrink statement is unreachable. |
| `mod.rs:1912:40` | `split_periodic_face_by_rim_chains` | replace + with - | (b) equivalent | `(d − π).rem_euclid(2π)` equals `(d + π).rem_euclid(2π)` for every `d` (the arguments differ by one period). |
| `mod.rs:1921:52` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) equivalent | Differs only when `v0 == v1`, which `v_top − v_bot < 100·tol` refuses on the next line whichever rim is called bottom. |
| `mod.rs:1926:22` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) equivalent | Rims exactly `100·tol` apart leave no `v` strictly inside `(v_bot + 100·tol, v_top − 100·tol)`, so every chain's interior samples fail and the split declines later regardless. |
| `mod.rs:1926:22` | `split_periodic_face_by_rim_chains` | replace < with == | (b) equivalent | Rims closer than `100·tol` leave no interior band for the chain samples either, so the split declines at the sample check instead. |
| `mod.rs:1934:37` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) unkillable | A chain end exactly `100·tol` off the top rim in `v` is admitted by `rim_of` under the mutant, but `find_splits_on_circle` then rejects it as off the rim circle (`> tol`), so `split_rim` declines and both trees return `None`. |
| `mod.rs:1993:26` | `split_periodic_face_by_rim_chains` | replace \|\| with && | (b) equivalent | `chains` is never empty when `sections` is not, and three or more chains are neither a notch nor a sector pair, so `!notch && !sectors` declines them one check later. |
| `mod.rs:2006:35` | `split_periodic_face_by_rim_chains` | replace < with == | (b) equivalent | Chain ends closer than `100·tol` (open but near-closed) both resolve to the same rim piece in `split_rim` (`ends_at` matches within `100·tol`), giving `ix == iy` and the same decline. |
| `mod.rs:2031:26` | `split_periodic_face_by_rim_chains` | replace < with == | (b) equivalent | Every chain has 17 samples per piece, so `samples.len()` is never below 17. |
| `mod.rs:2034:34` | `split_periodic_face_by_rim_chains` | replace > with >= | (b) unkillable | Differs only when a NURBS sample height equals `v_bot + 100·tol` to the bit; samples are interpolant evaluations at sixteenths of a marched piece's domain. |
| `mod.rs:2043:26` | `split_periodic_face_by_rim_chains` | replace > with >= | (b) unkillable | Differs only when the chain's unwrapped `u` extent equals π to the bit; `u` values are `atan2` projections of NURBS samples. |
| `mod.rs:2084:30` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) equivalent | Differs only when `ip == iq`, where both branches produce the same pair. |
| `mod.rs:2085:21` | `split_periodic_face_by_rim_chains` | replace \|\| with && | (b) equivalent | `ix == iy` needs both chain ends on one rim split, i.e. ends within `100·tol` (refused as a closed chain at line 2006), and `iy + 1 >= pieces.len()` needs an end within `100·tol` of the seam vertex (refused by the seam clearance at line 2047); neither disjunct is reachable alone. |
| `mod.rs:2094:72` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) unkillable | `from` is one of the chain's own two ends, so the test differs only when the chain's chord is exactly `100·tol`; a chord of a rim circle is `2r·sin(δ/2)`, never a dyadic multiple of `tol`. |
| `mod.rs:2133:57` | `split_periodic_face_by_rim_chains` | replace > with >= | (b) equivalent | Differs only when `vb == va`, i.e. `v_top == v_bot`, refused at line 1926. |
| `mod.rs:2169:23` | `split_periodic_face_by_rim_chains` | replace * with / | (b) unkillable | The sign test differs only when `d1 == 0` with `d0 > 0` (a chain sample's `u` exactly on the probe meridian), a bit-level coincidence between a NURBS sample and a rim-piece midpoint. |
| `mod.rs:2169:53` | `split_periodic_face_by_rim_chains` | replace > with >= | (b) unkillable | Differs only when `d0·d1 == 0`, a chain sample exactly on the probe meridian to the bit. |
| `mod.rs:2169:65` | `split_periodic_face_by_rim_chains` | replace - with / | (b) equivalent | This is the `-` of the zero-step guard `(d1 − d0).abs() < ε`; a zero `u` step has `d0 == d1`, skipped by the sign test unless both are exactly 0, where `d1/d0` is NaN, the comparison is false, the interpolation factor is NaN and the update comparison fails — no change either way. |
| `mod.rs:2169:77` | `split_periodic_face_by_rim_chains` | replace < with == | (b) equivalent | A zero `u` step has `d0 == d1`, skipped by the sign test unless both are exactly 0, where the division yields NaN and the update comparison is false either way. |
| `mod.rs:2264:53` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) unkillable | Differs only when the other chain's bottom end is exactly `100·tol` from `p`; two rim points are a chord `2r·sin(δ/2)` apart, never a dyadic multiple of `tol`. |
| `mod.rs:2285:62` | `split_periodic_face_by_rim_chains` | replace + with * | (b) unkillable | `bx·1 == bx` widens the clear span's piece list by the seam-side piece; with the middle piece still selected for one- or two-piece spans, and any selected piece inside the clear span, the probe meridian and the emitted regions are unchanged (the interior stays interior). |
| `mod.rs:2768:52` | `loops_orphan_boundary_edges` | replace * with + | (b) unkillable | The mid key is a relabelling applied identically to boundary and loop edges; it can only matter through a collision of two distinct arcs with equal quantized endpoints, which under `start + 0.5 + span` needs equal end angles, i.e. the same arc. |
| `mod.rs:2779:19` | `loops_orphan_boundary_edges` | replace <= with > | (b) equivalent | Reversing the inequality picks the other canonical order of the unordered endpoint pair for every edge alike, so boundary and loop keys still agree. |
| `mod.rs:6440:68` | `split_face_2d_impl` | replace && with \|\| | (b) equivalent | The widened snapshot is consumed only at line 7764 behind `u_periodic && Cylinder && !sections.is_empty()`, the same three conditions, so it is never read when it would not have existed. |
| `mod.rs:7947:13` | `split_face_2d_impl` | replace && with \|\| | (b) unkillable at the public boundary | Needs the cone orphan rescue with a DCEL trace that has a degenerate-area loop yet no orphan and no duplicate cover; no cone section set reaching the greedy produces one. |
| `mod.rs:7953:13` | `split_face_2d_impl` | replace && with \|\| | (b) unkillable at the public boundary | Needs the cone orphan rescue with a DCEL trace that self-crosses while the greedy result does not; no cone section set reaching the greedy produces one. |
| `mod.rs:7953:17` | `split_face_2d_impl` | delete ! | (b) unkillable at the public boundary | As 7953:13: the self-cross clause only decides when the DCEL and greedy self-cross flags differ, unreachable from native cone fixtures. |

## Finding filed as a ready-repro (bridge row candidate)

- `arrangement/oracle_tests.rs::one_ruling_joins_two_sectors_into_one_annulus` (`#[ignore]`): a
  cylinder strip cut by ONE ruling is two cells joined across the seam into one annulus (χ = 0, two
  rim boundaries), but `periodic::quotient` treats every non-seam half-edge as boundary, so the
  ruling's two halves — both inside the joined region — are walked as a slit, the rim cycles merge
  into one zero-winding cycle and the χ check refuses with `NonManifoldEmbedding`. Two rulings pass
  only because each ruling then has one half per periodic region. Impact in O2.3c: a lateral whose
  sections chain into a single full-height ruling is refused by the arrangement and falls back;
  the sector rescue at `mod.rs` line 7757 owns that configuration today. Not fixed here (no
  production change in this tranche); the repro's assertions are the acceptance target.

## Run provenance (disclosed)

- Before (arrangement files, branch base, no new tests): 688 tested: 471 caught, 61 unviable, 156 missed, 0 timeout; all 114 named arrangement survivors reproduce as MISSED ({'MISSED': 114}). The `mod.rs` file was not re-run before: its 3,510 mutants cost more than the tranche, and every named survivor there is either caught after or carries a proof.
- After, full files, on the test tree before the rebase onto `main` (`c8075051` plus the three test commits, now `cd315391..e0677b52`): arrangement files 688 tested: 598 caught, 61 unviable, 29 missed, 0 timeout; `mod.rs` 3510 tested: 605 caught, 504 unviable, 2401 missed, 0 timeout. Both are the second attempt: the first after-runs filled the root disk with `cargo-mutants` scratch copies and died (`No space left on device`) at 687/688 and 1,517/3,510; their partial results agreed with these.
- `main` moved while the full runs were in flight: `a2495828` (#902) integrated the O2.3c promotion and touched all five files (the `intersections` endpoint adoption, the cut-join at `arrangement.rs` line 363, the quotient's `y_tol` and `shares_non_seam_edge`, and the `arrangement_prod`/`arrangement_cyl` dispatch in `mod.rs`). The branch was rebased onto it; every oracle test still passes (532 algo tests) and the single-ruling repro still fails. Confirmation on the final rebased tree, restricted by `--re` to the 18 functions that own the 171 survivors: `1825 mutants tested in 2h: 730 missed, 817 caught, 278 unviable`. The regex keys on the `in <fn>` suffix, so it selected 166 of the 171 named mutants (the five whole-function replacements `uv_distance → -1.0/0.0`, `roundoff → 0.0`, `finite → true`, `distance → 1.0` carry no suffix; all five were caught in the full run); of those 166, 122 are caught and 44 missed — the same 44 (b) rows, with their CI lines mapped onto the rebased files by a longest-common-subsequence line map. CI line numbers are the 2026-09-27 run's; on the final tree the arrangement-engine lines sit two to four lines lower and the `mod.rs` lines up to 90 lower.
- Named survivors still MISSED after: 44 of 171 — exactly the (b) rows below; none without a proof.
- Outside the 2026-09-27 list, not pinned: 2386 other mutants in these files are MISSED in the after runs (2375 in `mod.rs`, 11 in the arrangement files). They are the files' pre-existing survivor population (the weekly job mutates only each week's diff, so the whole-file count was never reported), not regressions of this change. Per function, the densest: `split_face_2d_impl` 631, `arrangement_regions_from_inputs` 386, `split_cylinder_band_by_arrangement` 161, `clip_sections_to_outer_region` 132, `integrate_holes_plane` 116, `split_periodic_face_by_winding_chain` 94, `split_sections_at_t_junctions` 89, `split_sections_at_seam_meridian` 86, `plane_internal_line_loops` 85, `interior_point_3d` 67, `plane_closed_loops_separate` 64, `fn-replacement` 54. The raw list is the `missed.txt` of the two after runs; the next session should start from the arrangement-file entries, which are the code O2.3c promotes.

## Every survivor

| Mutant | Function | Mutation | Verdict | Evidence |
| --- | --- | --- | --- | --- |
| `arrangement.rs:108:16` | `CurveUse::point_3d` | replace - with + | (a) killed | `source_parameter_and_point_3d_are_affine_on_a_range_not_starting_at_zero` |
| `arrangement.rs:108:50` | `CurveUse::point_3d` | replace - with + | (a) killed | `source_parameter_and_point_3d_are_affine_on_a_range_not_starting_at_zero` |
| `arrangement.rs:220:26` | `Work<'a>::uv_distance` | replace * with / | (a) killed | `uv_distance_scales_u_by_the_domain_radius` |
| `arrangement.rs:220:9` | `Work<'a>::uv_distance` | replace Work<'a>::uv_distance -> f64 with -1.0 | (a) killed | `uv_distance_scales_u_by_the_domain_radius` |
| `arrangement.rs:220:9` | `Work<'a>::uv_distance` | replace Work<'a>::uv_distance -> f64 with 0.0 | (a) killed | `shared_endpoint_gaps_at_exactly_the_tolerance_are_accepted` |
| `arrangement.rs:234:14` | `Work<'a>::capacity` | replace > with >= | (a) killed | `capacity_accepts_the_queue_size_itself_and_charges_one_step` |
| `arrangement.rs:277:32` | `build_arrangement` | replace \|\| with && | (a) killed | `unsupported_strip_radii_are_refused_before_any_work_is_charged` |
| `arrangement.rs:294:14` | `build_arrangement` | replace > with < | (a) killed | `duplicate_use_ids_are_an_invalid_boundary` |
| `arrangement.rs:298:29` | `build_arrangement` | replace > with >= | (b) equivalent | `range[0] == range[1]` is refused by `validate_use` (`same(range[0], range[1])` → `InvalidBoundary`) on the line before the swap. |
| `arrangement.rs:314:44` | `build_arrangement` | replace > with == | (a) killed | `shared_endpoint_gaps_at_exactly_the_tolerance_are_accepted` |
| `arrangement.rs:314:44` | `build_arrangement` | replace > with >= | (a) killed | `shared_endpoint_gaps_at_exactly_the_tolerance_are_accepted` |
| `arrangement.rs:314:90` | `build_arrangement` | replace > with == | (a) killed | `shared_endpoint_gaps_at_exactly_the_tolerance_are_accepted` |
| `arrangement.rs:314:90` | `build_arrangement` | replace > with >= | (a) killed | `shared_endpoint_gaps_at_exactly_the_tolerance_are_accepted` |
| `arrangement.rs:333:38` | `build_arrangement` | replace * with + | (a) killed | `cardinal_cuts_are_the_interior_multiples_of_a_quarter_turn` |
| `arrangement.rs:334:22` | `build_arrangement` | replace > with >= | (a) killed | `cardinal_cuts_are_the_interior_multiples_of_a_quarter_turn` |
| `arrangement.rs:334:40` | `build_arrangement` | replace < with <= | (a) killed | `cardinal_cuts_are_the_interior_multiples_of_a_quarter_turn` |
| `arrangement.rs:389:37` | `build_arrangement` | replace \|\| with && | (a) killed | `a_single_overflowing_3d_coordinate_is_refused_as_non_finite` |
| `arrangement.rs:389:61` | `build_arrangement` | replace \|\| with && | (a) killed | `a_single_overflowing_3d_coordinate_is_refused_as_non_finite` |
| `arrangement.rs:392:80` | `build_arrangement` | replace > with == | (a) killed | `shared_endpoint_gaps_at_exactly_the_tolerance_are_accepted` |
| `arrangement.rs:463:29` | `build_arrangement` | replace / with % | (a) killed | `segment_budget_is_the_number_of_undirected_edges` |
| `arrangement.rs:463:29` | `build_arrangement` | replace / with * | (a) killed | `segment_budget_is_the_number_of_undirected_edges` |
| `arrangement.rs:97:18` | `CurveUse::source_parameter` | replace - with + | (a) killed | `source_parameter_and_point_3d_are_affine_on_a_range_not_starting_at_zero` |
| `arrangement/geometry.rs:121:17` | `intersections` | replace \|\| with && | (a) killed | `line_pairs_certified_disjoint_by_orientation_charge_no_work` |
| `arrangement/geometry.rs:121:37` | `intersections` | replace < with <= | (a) killed | `line_endpoint_on_the_other_support_is_reported_at_the_exact_parameters` |
| `arrangement/geometry.rs:122:17` | `intersections` | replace \|\| with && | (a) killed | `line_pairs_certified_disjoint_by_orientation_charge_no_work` |
| `arrangement/geometry.rs:122:24` | `intersections` | replace > with >= | (a) killed | `line_endpoint_on_the_other_support_is_reported_at_the_exact_parameters` |
| `arrangement/geometry.rs:122:36` | `intersections` | replace > with == | (a) killed | `endpoint_certificates_snap_only_when_both_ends_agree` |
| `arrangement/geometry.rs:123:24` | `intersections` | replace < with <= | (a) killed | `endpoint_certificates_snap_only_when_both_ends_agree` |
| `arrangement/geometry.rs:123:24` | `intersections` | replace < with == | (a) killed | `endpoint_certificates_snap_only_when_both_ends_agree` |
| `arrangement/geometry.rs:123:36` | `intersections` | replace < with <= | (a) killed | `line_endpoint_on_the_other_support_is_reported_at_the_exact_parameters` |
| `arrangement/geometry.rs:123:36` | `intersections` | replace < with == | (a) killed | `line_endpoint_on_the_other_support_is_reported_at_the_exact_parameters` |
| `arrangement/geometry.rs:162:17` | `intersections` | replace * with / | (a) killed | `line_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:162:31` | `intersections` | replace * with / | (a) killed | `line_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:162:62` | `intersections` | replace + with * | (a) killed | `line_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:162:80` | `intersections` | replace * with / | (a) killed | `line_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:172:29` | `intersections` | replace / with * | (b) unkillable | `aa = |d|²` of a `Line2D::new`-normalised direction lies within 2 ulp of 1, so `q/aa` and `q·aa` differ by at most 2 ulp of the root. |
| `arrangement/geometry.rs:18:46` | `roundoff` | replace + with - | (a) killed | `rim_winding_must_be_a_whole_turn_within_its_own_roundoff_bound` |
| `arrangement/geometry.rs:18:5` | `roundoff` | replace roundoff -> f64 with 0.0 | (a) killed | `seam_spacing_exactly_at_the_roundoff_bound_passes_the_spacing_check` |
| `arrangement/geometry.rs:190:51` | `intersections` | replace < with <= | (a) killed | `coincident_carrier_arcs_meet_only_at_a_shared_native_parameter` |
| `arrangement/geometry.rs:197:40` | `intersections` | replace - with / | (a) killed | `coincident_carrier_arcs_meet_only_at_a_shared_native_parameter` |
| `arrangement/geometry.rs:204:38` | `intersections` | replace * with / | (a) killed | `circle_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:204:66` | `intersections` | replace * with / | (a) killed | `circle_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:205:43` | `intersections` | replace * with / | (a) killed | `circle_pair_crossings_are_the_3_4_5_points` |
| `arrangement/geometry.rs:206:41` | `intersections` | replace \|\| with && | (a) killed | `circle_pair_with_an_overflowing_radius_is_refused_as_non_finite` |
| `arrangement/geometry.rs:209:49` | `intersections` | replace * with / | (a) killed | `circle_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:209:64` | `intersections` | replace * with + | (a) killed | `circle_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:209:78` | `intersections` | replace + with * | (a) killed | `circle_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:209:82` | `intersections` | replace * with + | (a) killed | `circle_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:209:82` | `intersections` | replace * with / | (a) killed | `circle_circle_tangency_band_is_the_stated_multiple_of_epsilon` |
| `arrangement/geometry.rs:211:54` | `intersections` | replace / with % | (a) killed | `external_tangency_refuses_only_when_the_contact_is_on_both_arcs` |
| `arrangement/geometry.rs:212:49` | `intersections` | replace && with \|\| | (a) killed | `external_tangency_refuses_only_when_the_contact_is_on_both_arcs` |
| `arrangement/geometry.rs:215:35` | `intersections` | replace > with >= | (b) equivalent | `height2 == 0` is caught by the tangency test on the line before (`height2.abs() <= error`, with `error ≥ 0`), so the branch never sees a zero. |
| `arrangement/geometry.rs:21:23` | `finite` | replace && with \|\| | (a) killed | `every_non_finite_disjunct_refuses_on_its_own` |
| `arrangement/geometry.rs:21:5` | `finite` | replace finite -> bool with true | (a) killed | `every_non_finite_disjunct_refuses_on_its_own` |
| `arrangement/geometry.rs:230:27` | `intersections` | replace \|\| with && | (b) unkillable | Each hit's two parameters come from one finite 2D point (line–circle: `a.point(t)` and its angle; circle–circle: `p` and its two angles) or from exact orientation-certified endpoints; a lone non-finite coordinate needs a coordinate difference beyond `f64::MAX`, which first breaks the `orient2d` certificates or the `disc`/`height2` finiteness gates guarding the branch. |
| `arrangement/geometry.rs:238:21` | `intersections` | replace && with \|\| | (a) killed | `cardinal_cuts_are_the_interior_multiples_of_a_quarter_turn` |
| `arrangement/geometry.rs:238:68` | `intersections` | replace <= with > | (a) killed | `endpoint_certificates_snap_only_when_both_ends_agree` |
| `arrangement/geometry.rs:239:68` | `intersections` | replace <= with > | (a) killed | `endpoint_certificates_snap_only_when_both_ends_agree` |
| `arrangement/geometry.rs:247:57` | `intersections` | replace > with >= | (b) unkillable | Differs only when a hit's residual `uv_distance` equals `tolerance.linear` to the bit; certified hits have residual exactly 0 and computed roots carry roundoff-scale residual (~1e-16 · scale), neither of which can be made to equal a chosen tolerance. |
| `arrangement/geometry.rs:254:43` | `intersections` | replace && with \|\| | (b) equivalent | Two hits equal in one parameter are the same 2D point (parameters are single-valued in `[0, 2π)` or on a line), hence equal in the other; `dedup_by` removes the same pairs either way. |
| `arrangement/geometry.rs:268:32` | `tangent_order` | replace < with > | (a) killed | `parallel_tangents_compare_equal_and_reversal_flips_the_half_plane` |
| `arrangement/geometry.rs:268:68` | `tangent_order` | replace < with <= | (b) equivalent | Differs only for the zero tangent `(0, 0)`; tangents are a unit line direction or a radius-length circle tangent (radius > 0), and `signum` of a zero-length range is 1. |
| `arrangement/geometry.rs:271:17` | `tangent_order` | replace > with >= | (a) killed | `parallel_tangents_compare_equal_and_reversal_flips_the_half_plane` |
| `arrangement/geometry.rs:273:24` | `tangent_order` | replace < with <= | (a) killed | `parallel_tangents_compare_equal_and_reversal_flips_the_half_plane` |
| `arrangement/geometry.rs:273:24` | `tangent_order` | replace < with == | (a) killed | `parallel_tangents_compare_equal_and_reversal_flips_the_half_plane` |
| `arrangement/geometry.rs:273:24` | `tangent_order` | replace < with > | (a) killed | `tangent_order_is_upper_half_plane_first_then_counterclockwise` |
| `arrangement/geometry.rs:299:42` | `ray_crossing` | replace > with >= | (a) killed | `ray_crossing_counts_half_open_monotone_spans_strictly_right_of_the_probe` |
| `arrangement/geometry.rs:300:44` | `ray_crossing` | replace > with >= | (a) killed | `ray_crossing_counts_half_open_monotone_spans_strictly_right_of_the_probe` |
| `arrangement/geometry.rs:310:79` | `ray_crossing` | replace * with / | (b) equivalent | `mid.cos().signum()` is ±1 (never 0 for a cardinal-split arc's midpoint), and `x·(±1) == x/(±1)` exactly. |
| `arrangement/geometry.rs:314:10` | `ray_crossing` | replace > with >= | (a) killed | `ray_crossing_on_cardinal_arcs_uses_the_3_4_5_height` |
| `arrangement/geometry.rs:322:5` | `distance` | replace distance -> f64 with 1.0 | (a) killed | `distance_clamps_to_the_span_and_picks_the_nearer_arc_end` |
| `arrangement/geometry.rs:330:56` | `distance` | replace < with <= | (b) equivalent | Differs only when both arc ends are equidistant from `p`; either end then gives the same returned distance. |
| `arrangement/geometry.rs:330:56` | `distance` | replace < with == | (a) killed | `distance_clamps_to_the_span_and_picks_the_nearer_arc_end` |
| `arrangement/geometry.rs:330:56` | `distance` | replace < with > | (a) killed | `distance_clamps_to_the_span_and_picks_the_nearer_arc_end` |
| `arrangement/geometry.rs:33:9` | `validate_use` | replace \|\| with && | (a) killed | `every_non_finite_disjunct_refuses_on_its_own` |
| `arrangement/geometry.rs:35:63` | `validate_use` | replace \|\| with && | (a) killed | `every_non_finite_disjunct_refuses_on_its_own` |
| `arrangement/geometry.rs:37:9` | `validate_use` | replace \|\| with && | (a) killed | `every_non_finite_disjunct_refuses_on_its_own` |
| `arrangement/geometry.rs:47:37` | `validate_use` | replace \|\| with && | (a) killed | `degenerate_ranges_refuse_separately_and_unsupported_curves_are_named` |
| `arrangement/geometry.rs:53:35` | `validate_use` | replace \|\| with && | (a) killed | `circle_3d_end_checks_probe_each_coordinate_and_the_exact_tolerance` |
| `arrangement/geometry.rs:53:57` | `validate_use` | replace \|\| with && | (a) killed | `circle_3d_end_checks_probe_each_coordinate_and_the_exact_tolerance` |
| `arrangement/geometry.rs:56:51` | `validate_use` | replace > with >= | (a) killed | `circle_3d_end_checks_probe_each_coordinate_and_the_exact_tolerance` |
| `arrangement/geometry.rs:63:39` | `validate_use` | replace \|\| with && | (b) equivalent | A non-finite line origin or direction makes `u.point(range[0])` non-finite, which line 36 refuses first; neither disjunct is reachable alone. |
| `arrangement/geometry.rs:68:41` | `validate_use` | replace \|\| with && | (b) equivalent | A non-finite circle centre or radius (∞ or NaN; `Circle2D::new` admits NaN) makes `u.point(range[0])` non-finite, refused at line 36; neither disjunct is reachable alone. |
| `arrangement/periodic.rs:100:17` | `quotient` | replace \|\| with && | (a) killed | `seam_vertex_3d_gap_is_accepted_up_to_exactly_the_tolerance` |
| `arrangement/periodic.rs:101:21` | `quotient` | replace > with >= | (a) killed | `seam_vertex_3d_gap_is_accepted_up_to_exactly_the_tolerance` |
| `arrangement/periodic.rs:106:61` | `quotient` | replace == with != | (a) killed | `plain_strip_is_one_annulus_with_signed_rim_windings` |
| `arrangement/periodic.rs:113:57` | `quotient` | replace + with * | (a) killed | `plain_strip_is_one_annulus_with_signed_rim_windings` |
| `arrangement/periodic.rs:113:57` | `quotient` | replace + with - | (a) killed | `plain_strip_is_one_annulus_with_signed_rim_windings` |
| `arrangement/periodic.rs:114:57` | `quotient` | replace + with * | (a) killed | `plain_strip_is_one_annulus_with_signed_rim_windings` |
| `arrangement/periodic.rs:163:35` | `quotient` | replace / with * | (b) equivalent | Only non-counterpart edges take the default key; every accepted region holds exactly one half of each such edge (a region holding both halves of an interior edge fails the χ check regardless of the key, see the single-ruling repro), so `h/2` and `2h` count the same edges. |
| `arrangement/periodic.rs:191:23` | `quotient` | replace += with -= | (a) killed | `plain_strip_is_one_annulus_with_signed_rim_windings` |
| `arrangement/periodic.rs:198:21` | `quotient` | replace \|\| with && | (a) killed | `boundary_walk_through_a_non_boundary_half_edge_is_open` |
| `arrangement/periodic.rs:205:44` | `quotient` | replace > with == | (a) killed | `seam_spacing_exactly_at_the_roundoff_bound_passes_the_spacing_check` |
| `arrangement/periodic.rs:205:44` | `quotient` | replace > with >= | (b) unkillable | Differs only when `|Δu − 2π|` equals `2⁻⁴⁶·(1 + |Δu|)` to the bit; `Δu − 2π` is a multiple of `ulp(2π) = 2⁻⁵⁰` while the bound is `2⁻⁴⁶` times a value near 7.28, never such a multiple. |
| `arrangement/periodic.rs:205:51` | `quotient` | replace * with + | (a) killed | `seam_spacing_exactly_at_the_roundoff_bound_passes_the_spacing_check` |
| `arrangement/periodic.rs:205:51` | `quotient` | replace * with / | (a) killed | `seam_spacing_exactly_at_the_roundoff_bound_passes_the_spacing_check` |
| `arrangement/periodic.rs:205:66` | `quotient` | replace * with + | (a) killed | `rim_winding_must_be_a_whole_turn_within_its_own_roundoff_bound` |
| `arrangement/periodic.rs:205:73` | `quotient` | replace + with * | (a) killed | `rim_winding_must_be_a_whole_turn_within_its_own_roundoff_bound` |
| `arrangement/periodic.rs:206:17` | `quotient` | replace \|\| with && | (a) killed | `rim_winding_must_be_a_whole_turn_within_its_own_roundoff_bound` |
| `arrangement/periodic.rs:20:28` | `quotient` | replace \|\| with && | (a) killed | `quotient_domain_refusals_happen_before_any_work_is_charged` |
| `arrangement/periodic.rs:226:28` | `quotient` | replace \|\| with && | (a) killed | `euler_characteristic_mismatch_is_refused_even_with_balanced_windings` |
| `arrangement/periodic.rs:31:9` | `quotient` | replace \|\| with && | (a) killed | `quotient_domain_refusals_happen_before_any_work_is_charged` |
| `arrangement/periodic.rs:32:9` | `quotient` | replace \|\| with && | (a) killed | `quotient_domain_refusals_happen_before_any_work_is_charged` |
| `arrangement/periodic.rs:33:9` | `quotient` | replace \|\| with && | (a) killed | `quotient_domain_refusals_happen_before_any_work_is_charged` |
| `arrangement/periodic.rs:34:29` | `quotient` | replace - with + | (a) killed | `strip_with_a_shifted_left_seam_is_an_annulus_too` |
| `arrangement/periodic.rs:34:60` | `quotient` | replace > with == | (a) killed | `quotient_domain_refusals_happen_before_any_work_is_charged` |
| `arrangement/periodic.rs:34:60` | `quotient` | replace > with >= | (a) killed | `seam_spacing_exactly_at_the_roundoff_bound_passes_the_spacing_check` |
| `arrangement/periodic.rs:34:9` | `quotient` | replace \|\| with && | (a) killed | `quotient_domain_refusals_happen_before_any_work_is_charged` |
| `arrangement/periodic.rs:51:39` | `quotient` | replace \|\| with && | (a) killed | `material_outside_the_seam_interval_is_refused_on_either_side` |
| `arrangement/regions.rs:102:84` | `extract` | replace - with + | (b) unkillable | The compensated sum's correction is below one ulp of the running area; adding instead of subtracting it changes the area by at most a few ulps over a cycle's ≤ dozens of edges (~1e-15 relative). |
| `arrangement/regions.rs:129:36` | `extract` | replace > with >= | (b) equivalent | A cycle with `signed_area == 0` is refused as `OpenRegion` (`same(area, 0.0)`) before this classification. |
| `arrangement/regions.rs:151:51` | `extract` | replace < with <= | (b) equivalent | Two outer cycles that both contain a hole's seed are nested (faces of a planar subdivision have disjoint interiors), so the inner one has strictly smaller area; no tie is reachable. |
| `arrangement/regions.rs:175:27` | `extract` | replace += with -= | (b) equivalent | Material is the parity of the crossing count and `rem_euclid(2)` of `−n` equals that of `n`. |
| `arrangement/regions.rs:62:16` | `left_seed` | replace *= with += | (b) equivalent | The seed sits on the sub-curve's own normal at its midpoint, so its distance to that sub-curve is `offset` (≥ `offset/2`) on the first pass for lines and for cardinal-split arcs (`offset ≤ clearance/8 ≤ 0.09·r` never crosses the centre); the shrink statement is unreachable. |
| `mod.rs:1880:87` | `split_periodic_face_by_rim_chains` | replace < with <= | (a) killed | `section_piece_exactly_one_weld_long_is_kept` |
| `mod.rs:1880:87` | `split_periodic_face_by_rim_chains` | replace < with == | (a) killed | `section_piece_exactly_one_weld_long_is_kept` |
| `mod.rs:1912:40` | `split_periodic_face_by_rim_chains` | replace + with - | (b) equivalent | `(d − π).rem_euclid(2π)` equals `(d + π).rem_euclid(2π)` for every `d` (the arguments differ by one period). |
| `mod.rs:1916:42` | `split_periodic_face_by_rim_chains` | replace < with <= | (a) killed | `rim_start_exactly_one_weld_off_the_seam_meridian_is_refused` |
| `mod.rs:1921:52` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) equivalent | Differs only when `v0 == v1`, which `v_top − v_bot < 100·tol` refuses on the next line whichever rim is called bottom. |
| `mod.rs:1926:22` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) equivalent | Rims exactly `100·tol` apart leave no `v` strictly inside `(v_bot + 100·tol, v_top − 100·tol)`, so every chain's interior samples fail and the split declines later regardless. |
| `mod.rs:1926:22` | `split_periodic_face_by_rim_chains` | replace < with == | (b) equivalent | Rims closer than `100·tol` leave no interior band for the chain samples either, so the split declines at the sample check instead. |
| `mod.rs:1934:37` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) unkillable | A chain end exactly `100·tol` off the top rim in `v` is admitted by `rim_of` under the mutant, but `find_splits_on_circle` then rejects it as off the rim circle (`> tol`), so `split_rim` declines and both trees return `None`. |
| `mod.rs:1971:51` | `split_periodic_face_by_rim_chains` | replace < with <= | (a) killed | `junction_gap_exactly_one_weld_is_not_chained` |
| `mod.rs:1973:51` | `split_periodic_face_by_rim_chains` | replace < with <= | (a) killed | `junction_gap_exactly_one_weld_is_not_chained` |
| `mod.rs:1993:26` | `split_periodic_face_by_rim_chains` | replace \|\| with && | (b) equivalent | `chains` is never empty when `sections` is not, and three or more chains are neither a notch nor a sector pair, so `!notch && !sectors` declines them one check later. |
| `mod.rs:2006:35` | `split_periodic_face_by_rim_chains` | replace < with == | (b) equivalent | Chain ends closer than `100·tol` (open but near-closed) both resolve to the same rim piece in `split_rim` (`ends_at` matches within `100·tol`), giving `ix == iy` and the same decline. |
| `mod.rs:2031:26` | `split_periodic_face_by_rim_chains` | replace < with == | (b) equivalent | Every chain has 17 samples per piece, so `samples.len()` is never below 17. |
| `mod.rs:2034:34` | `split_periodic_face_by_rim_chains` | replace > with >= | (b) unkillable | Differs only when a NURBS sample height equals `v_bot + 100·tol` to the bit; samples are interpolant evaluations at sixteenths of a marched piece's domain. |
| `mod.rs:2043:26` | `split_periodic_face_by_rim_chains` | replace > with >= | (b) unkillable | Differs only when the chain's unwrapped `u` extent equals π to the bit; `u` values are `atan2` projections of NURBS samples. |
| `mod.rs:2047:38` | `split_periodic_face_by_rim_chains` | replace + with - | (a) killed | `chain_ends_half_a_weld_from_the_seam_are_refused` |
| `mod.rs:2047:50` | `split_periodic_face_by_rim_chains` | replace \|\| with && | (a) killed | `chain_ends_half_a_weld_from_the_seam_are_refused` |
| `mod.rs:2047:69` | `split_periodic_face_by_rim_chains` | replace - with + | (a) killed | `chain_ends_half_a_weld_from_the_seam_are_refused` |
| `mod.rs:2047:69` | `split_periodic_face_by_rim_chains` | replace - with / | (a) killed | `chain_ends_half_a_weld_from_the_seam_are_refused` |
| `mod.rs:2084:30` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) equivalent | Differs only when `ip == iq`, where both branches produce the same pair. |
| `mod.rs:2085:21` | `split_periodic_face_by_rim_chains` | replace \|\| with && | (b) equivalent | `ix == iy` needs both chain ends on one rim split, i.e. ends within `100·tol` (refused as a closed chain at line 2006), and `iy + 1 >= pieces.len()` needs an end within `100·tol` of the seam vertex (refused by the seam clearance at line 2047); neither disjunct is reachable alone. |
| `mod.rs:2094:72` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) unkillable | `from` is one of the chain's own two ends, so the test differs only when the chain's chord is exactly `100·tol`; a chord of a rim circle is `2r·sin(δ/2)`, never a dyadic multiple of `tol`. |
| `mod.rs:2133:57` | `split_periodic_face_by_rim_chains` | replace > with >= | (b) equivalent | Differs only when `vb == va`, i.e. `v_top == v_bot`, refused at line 1926. |
| `mod.rs:2169:23` | `split_periodic_face_by_rim_chains` | replace * with / | (b) unkillable | The sign test differs only when `d1 == 0` with `d0 > 0` (a chain sample's `u` exactly on the probe meridian), a bit-level coincidence between a NURBS sample and a rim-piece midpoint. |
| `mod.rs:2169:41` | `split_periodic_face_by_rim_chains` | replace - with + | (a) killed | `sector_interiors_ignore_chains_beside_and_opposite_the_probe_meridian` |
| `mod.rs:2169:53` | `split_periodic_face_by_rim_chains` | replace > with == | (a) killed | `sector_interiors_ignore_chains_beside_and_opposite_the_probe_meridian` |
| `mod.rs:2169:53` | `split_periodic_face_by_rim_chains` | replace > with >= | (b) unkillable | Differs only when `d0·d1 == 0`, a chain sample exactly on the probe meridian to the bit. |
| `mod.rs:2169:58` | `split_periodic_face_by_rim_chains` | replace \|\| with && | (a) killed | `sector_interiors_ignore_chains_beside_and_opposite_the_probe_meridian` |
| `mod.rs:2169:65` | `split_periodic_face_by_rim_chains` | replace - with / | (b) equivalent | This is the `-` of the zero-step guard `(d1 − d0).abs() < ε`; a zero `u` step has `d0 == d1`, skipped by the sign test unless both are exactly 0, where `d1/d0` is NaN, the comparison is false, the interpolation factor is NaN and the update comparison fails — no change either way. |
| `mod.rs:2169:77` | `split_periodic_face_by_rim_chains` | replace < with == | (b) equivalent | A zero `u` step has `d0 == d1`, skipped by the sign test unless both are exactly 0, where the division yields NaN and the update comparison is false either way. |
| `mod.rs:2172:32` | `split_periodic_face_by_rim_chains` | replace + with - | (a) killed | `skewed_notch_interiors_sit_halfway_to_the_chain_on_the_probe_meridians` |
| `mod.rs:2172:58` | `split_periodic_face_by_rim_chains` | replace / with % | (a) killed | `skewed_notch_interiors_sit_halfway_to_the_chain_on_the_probe_meridians` |
| `mod.rs:2172:58` | `split_periodic_face_by_rim_chains` | replace / with * | (a) killed | `skewed_notch_interiors_sit_halfway_to_the_chain_on_the_probe_meridians` |
| `mod.rs:2172:64` | `split_periodic_face_by_rim_chains` | replace - with + | (a) killed | `skewed_notch_interiors_sit_halfway_to_the_chain_on_the_probe_meridians` |
| `mod.rs:2172:64` | `split_periodic_face_by_rim_chains` | replace - with / | (a) killed | `skewed_notch_interiors_sit_halfway_to_the_chain_on_the_probe_meridians` |
| `mod.rs:2264:53` | `split_periodic_face_by_rim_chains` | replace < with <= | (b) unkillable | Differs only when the other chain's bottom end is exactly `100·tol` from `p`; two rim points are a chord `2r·sin(δ/2)` apart, never a dyadic multiple of `tol`. |
| `mod.rs:2285:62` | `split_periodic_face_by_rim_chains` | replace + with * | (b) unkillable | `bx·1 == bx` widens the clear span's piece list by the seam-side piece; with the middle piece still selected for one- or two-piece spans, and any selected piece inside the clear span, the probe meridian and the emitted regions are unchanged (the interior stays interior). |
| `mod.rs:2749:20` | `loops_orphan_boundary_edges` | replace * with + | (a) killed | `endpoint_quantization_rounds_each_axis_to_the_nearest_cell` |
| `mod.rs:2749:20` | `loops_orphan_boundary_edges` | replace * with / | (a) killed | `endpoint_quantization_rounds_each_axis_to_the_nearest_cell` |
| `mod.rs:2750:20` | `loops_orphan_boundary_edges` | replace * with / | (a) killed | `endpoint_quantization_rounds_each_axis_to_the_nearest_cell` |
| `mod.rs:2751:20` | `loops_orphan_boundary_edges` | replace * with + | (a) killed | `endpoint_quantization_rounds_each_axis_to_the_nearest_cell` |
| `mod.rs:2751:20` | `loops_orphan_boundary_edges` | replace * with / | (a) killed | `endpoint_quantization_rounds_each_axis_to_the_nearest_cell` |
| `mod.rs:2764:43` | `loops_orphan_boundary_edges` | replace - with + | (a) killed | `tiny_arcs_are_skipped_only_below_the_span_times_radius_floor` |
| `mod.rs:2764:43` | `loops_orphan_boundary_edges` | replace - with / | (a) killed | `tiny_arcs_are_skipped_only_below_the_span_times_radius_floor` |
| `mod.rs:2765:25` | `loops_orphan_boundary_edges` | replace * with + | (a) killed | `tiny_arcs_are_skipped_only_below_the_span_times_radius_floor` |
| `mod.rs:2765:25` | `loops_orphan_boundary_edges` | replace * with / | (a) killed | `co_endpoint_rim_halves_never_alias` |
| `mod.rs:2768:52` | `loops_orphan_boundary_edges` | replace * with + | (b) unkillable | The mid key is a relabelling applied identically to boundary and loop edges; it can only matter through a collision of two distinct arcs with equal quantized endpoints, which under `start + 0.5 + span` needs equal end angles, i.e. the same arc. |
| `mod.rs:2768:52` | `loops_orphan_boundary_edges` | replace * with / | (a) killed | `co_endpoint_rim_halves_never_alias` |
| `mod.rs:2775:26` | `loops_orphan_boundary_edges` | replace && with \|\| | (a) killed | `orphan_detection_is_direction_agnostic_and_skips_zero_extent_lines` |
| `mod.rs:2779:19` | `loops_orphan_boundary_edges` | replace <= with > | (b) equivalent | Reversing the inequality picks the other canonical order of the unordered endpoint pair for every edge alike, so boundary and loop keys still agree. |
| `mod.rs:6440:68` | `split_face_2d_impl` | replace && with \|\| | (b) equivalent | The widened snapshot is consumed only at line 7764 behind `u_periodic && Cylinder && !sections.is_empty()`, the same three conditions, so it is never read when it would not have existed. |
| `mod.rs:7920:9` | `split_face_2d_impl` | replace && with \|\| | (a) killed | `frustum_cut_by_two_rulings_is_two_sectors_through_the_dispatcher` |
| `mod.rs:7921:9` | `split_face_2d_impl` | replace && with \|\| | (a) killed | `frustum_cut_by_two_rulings_is_two_sectors_through_the_dispatcher` |
| `mod.rs:7922:9` | `split_face_2d_impl` | replace && with \|\| | (a) killed | `frustum_cut_by_two_rulings_is_two_sectors_through_the_dispatcher` |
| `mod.rs:7947:13` | `split_face_2d_impl` | replace && with \|\| | (b) unkillable at the public boundary | Needs the cone orphan rescue with a DCEL trace that has a degenerate-area loop yet no orphan and no duplicate cover; no cone section set reaching the greedy produces one. |
| `mod.rs:7953:13` | `split_face_2d_impl` | replace && with \|\| | (b) unkillable at the public boundary | Needs the cone orphan rescue with a DCEL trace that self-crosses while the greedy result does not; no cone section set reaching the greedy produces one. |
| `mod.rs:7953:17` | `split_face_2d_impl` | delete ! | (b) unkillable at the public boundary | As 7953:13: the self-cross clause only decides when the DCEL and greedy self-cross flags differ, unreachable from native cone fixtures. |

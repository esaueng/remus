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
  (`BEFORE_SUMMARY`). After: one run of the same 688 and one of all `MOD_TOTAL` `mod.rs` mutants
  on the final test tree (`AFTER_ARR_SUMMARY`; `AFTER_MOD_SUMMARY`).
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

BY_FUNCTION_TABLE

## Equivalent and unkillable survivors

EQUIVALENT_TABLE

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

PROVENANCE

## Every survivor

EVERY_TABLE

# Face-splitter mutation triage — 2026-09-25 (B19)

The face-splitter share of the 2026-09-25 weekly mutation run: every
survivor in `crates/algo/src/builder/face_splitter/mod.rs` and
`special_cases.rs`, triaged into killed, equivalent/unkillable, or real
defect.

## Source and method

- Source of truth: GitHub Actions run 36075171651 (Mutation Testing,
  sharded, head `1fa7d150`; the week ending 2026-09-24). The shard
  artifacts could not be downloaded from this session (artifact storage is
  outside its network policy), so the missed list was rebuilt from the 19
  shard job logs: 829 unique `MISSED` lines against the run summary's 825,
  of which 201 are in this family (192 in `mod.rs`, 9 in
  `special_cases.rs`). Shard 9 was interrupted by a runner shutdown; its
  four logged survivors are included.
- Line numbers: the table uses the current tree. `mod.rs` lines are the
  run's plus 2 (this change adds a two-line `#[cfg(test)] mod`
  declaration above the function); the one `split_face_2d_impl` survivor
  (run line 6150) is now line 6222 after later merges.
- Local tool: `cargo-mutants 27.1.0` (CI pins 27.0.0), the repository's
  `.cargo/mutants.toml` (nextest, `--tests`, per-package oracle,
  first-failure stop) with `--profile dev` instead of `ci-test` for build
  speed; the tests run are identical. `--in-diff` against the week's diff
  selects nothing on the current tree, so before/after were measured over
  whole functions (a superset of the week's survivors).
- Per-mutant oracle is the `remus-algo` package only (the config's
  `test_workspace = false`). Assertions added to operations or io fixtures
  cannot kill an algo mutant, so every new test lives in `remus-algo`
  (`crates/algo/src/builder/face_splitter/closed_form_split_tests.rs`).

## New tests and their oracles

All drive `split_face_2d` (and, for the rim-chain cell, the splitter
directly) on a primitive-style cylinder lateral or a faceted hemisphere
(`make_sphere`'s inscribed-equator construction). No oracle re-reads the
splitter's arithmetic: each region is re-sampled from its 3D carriers and
its area compared with a closed form; interiors are checked by closed-form
membership and must sit in the middle half of their region's extent along
their meridian; section edges' stored `(u, v)` must map back to their 3D
ends under the cylinder map.

| Test | Geometry | Closed form |
| --- | --- | --- |
| `rim_notch_splits_lateral_into_band_and_lens_of_closed_form_area` | planar-ellipse notch on either rim; 4 positions (symmetric, far half, both sides of the seam); rims at z = 0, below, straddling and above zero; 3 scales; scrambled and reversed pieces; reversed faces | lens `r·d·2(sin α − α cos α)/(1 − cos α)`, band `2πrh − lens` |
| `rim_to_rim_chains_split_lateral_into_two_sectors_of_closed_form_area` | two helix chains of unequal lean | clear sector `r·h·((θ_b − θ_a) + (λ_b − λ_a)/2)` |
| `rim_chains_weld_fit_error_gaps_at_the_weld_scale` | 3e-6 joint gap | lens as above |
| `rim_chains_decline_outside_their_cell` | lines, seam-crossing, > π, far-rim touch, closed loop, dangling, mixed, three chains, crossing, lone rim-to-rim, seam-crossing sector, one-rim boundary, W chain touching the rim | declines |
| `closed_rings_split_lateral_into_bands_of_closed_form_area` | 1–2 closed rings | `2πr·Δz` per band |
| `one_ruling_splits_lateral_into_two_sectors_of_closed_form_area` | one full-height ruling | `r·θ·h`, `r·(2π − θ)·h` |
| `box_walls_split_faceted_hemisphere_into_collar_and_four_caps` | four box walls, a > r/√2 | cap = circular segment `r² acos(a/r) − a√(r² − a²)` from +z |
| `one_wall_and_lidded_box_split_faceted_hemisphere_exactly` | one wall (two-patch route); four walls + lid below the pole (collar with a hole) | segment; lid disc `π ρ_c²` as the collar's hole and its own cap |

## Before / after (measured, whole functions)

| Function(s) | Mutants | Before caught / missed / unviable | After caught / missed / unviable |
| --- | ---: | ---: | ---: |
| `split_periodic_face_by_rim_chains` (body) + the `split_face_2d_impl` dispatch gate | 232 | 2 / 200 / 30 | 149 / 53 / 30 |
| whole-function stubs of the four splitters | 11 | 2 / 5 / 4 | 7 / 0 / 4 |
| `split_noseam_by_arrangement` (body) | 83 | 0 / 73 / 10 | 36 / 37 / 10 |
| `split_periodic_face_into_bands` (body) | 76 | 0 / 68 / 8 | 26 / 42 / 8 |
| `split_periodic_face_into_sectors` (body) | 118 | 0 / 112 / 6 | 51 / 61 / 6 |

## The week's 201 survivors

| Class | Count | Where |
| --- | ---: | --- |
| (a) Killed by a new test | 145 | 140 `split_periodic_face_by_rim_chains` (138 body + both stubs), 1 `split_face_2d_impl` gate, 4 `split_noseam_by_arrangement` + bands (the `Ok(vec![])` / `None` stubs, `408:45 * → +`, `408:72 * → /`) |
| (b) Equivalent | 39 | rows below |
| (b) Unkillable | 15 | rows below |
| (c) Real defect | 2 | `split_periodic_face_into_sectors -> None` and `-> Some(vec![])`: the one-ruling sector rescue was dead code (fixed, below) |

### (c) Defects

- **Fixed — one-ruling sector rescue never fired.** `split_periodic_face_into_sectors`
  needs the lateral's two CLOSED rim circles; its call site passed the
  boundary after the rims were halved at the seam antipode and split at
  every section endpoint, so it declined on every ruling off the seam since
  it landed. A single full-height ruling came back as one region that
  dropped the bottom rim and the ruling. The call site now receives the
  boundary as the sibling periodic shortcuts read it. Regression:
  `one_ruling_splits_lateral_into_two_sectors_of_closed_form_area` (one
  region before, two sectors of the closed-form areas after).
- **Filed — [B64](roadmap.md#b64).** Found while pinning the collar's lid
  route: sphere ∖ lidded box silently drops the lid-cap lump (raw GFA
  already lacks it). Ready-repro in
  `crates/operations/tests/regress_sphere_lidded_box_collar.rs`.
- **Filed — [B65](roadmap.md#b65).** The collar's classification sample
  overshoots a wall close under the lid and the exact boolean is refused.
  A bounded nudge fixes the intersect but would turn the cut from refused
  into B64's wrong result, so it waits on B64. Ready-repros in both test
  files named above.

### (b) Survivor rows

| Mutant (current line) | Function | Mutation | Class | Proof |
| --- | --- | --- | --- | --- |
| `mod.rs:1872:76` | `split_periodic_face_by_rim_chains` | replace \|\| with && | EQUIVALENT | Redundant guard: a non-cylinder/cone periodic face bounds itself with circular seams, declined by the boundary loop's open-circle arm; planes never reach here (caller gate); empty sections leave no chains (declined at 1990). |
| `mod.rs:1877:87` | `split_periodic_face_by_rim_chains` | replace < with == | EQUIVALENT | Redundant guard: a closed section either forms a closed chain (declined at 2003) or is spliced at a rim point whose interior sample then fails the strictly-between check at 2031. |
| `mod.rs:1886:58` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:1909:40` | `split_periodic_face_by_rim_chains` | replace + with - | EQUIVALENT | `(d − π).rem_euclid(2π) − π` equals `(d + π).rem_euclid(2π) − π`: ±π are congruent mod 2π. |
| `mod.rs:1913:42` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:1918:52` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Differs only at `v0 == v1`, where both branches give `v_top == v_bot` and line 1923 declines. |
| `mod.rs:1923:22` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:1923:22` | `split_periodic_face_by_rim_chains` | replace < with == | EQUIVALENT | Redundant guard: coincident rims make the strictly-between sample test at 2031 unsatisfiable, so the input declines there. |
| `mod.rs:1929:30` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:1931:37` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:1966:53` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:1968:51` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:1970:51` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:1972:53` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:1990:26` | `split_periodic_face_by_rim_chains` | replace \|\| with && | EQUIVALENT | Redundant: non-empty sections always yield ≥ 1 chain, and 3+ chains are neither a notch nor a sector pair (declined at 2058). |
| `mod.rs:2003:35` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:2003:35` | `split_periodic_face_by_rim_chains` | replace < with == | EQUIVALENT | Redundant: a closed chain has an end off both rims (`rim_of` → None) or both ends at one rim point, which `split_rim` declines (`ix == iy`). |
| `mod.rs:2028:26` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | `samples.len()` is 17 × pieces ≥ 17, so `< 3`, `<= 3` and `== 3` are all false. |
| `mod.rs:2028:26` | `split_periodic_face_by_rim_chains` | replace < with == | EQUIVALENT | `samples.len()` is 17 × pieces ≥ 17, so `< 3`, `<= 3` and `== 3` are all false. |
| `mod.rs:2031:34` | `split_periodic_face_by_rim_chains` | replace > with >= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:2031:59` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:2040:26` | `split_periodic_face_by_rim_chains` | replace > with >= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:2044:38` | `split_periodic_face_by_rim_chains` | replace + with - | UNKILLABLE | Moves the seam-clearance margin by 2·close_tol (2e-5): only a chain ending within the weld scale of the seam meridian, where chain end and seam vertex are one point, is reclassified. |
| `mod.rs:2044:50` | `split_periodic_face_by_rim_chains` | replace \|\| with && | UNKILLABLE | Disables the `seam_off ≥ 2π − close_tol` disjunct, which only catches a chain starting within close_tol (1e-5) past the seam meridian: below the weld scale, where its end is the seam vertex. |
| `mod.rs:2044:69` | `split_periodic_face_by_rim_chains` | replace - with + | UNKILLABLE | Disables the `seam_off ≥ 2π − close_tol` disjunct, which only catches a chain starting within close_tol (1e-5) past the seam meridian: below the weld scale, where its end is the seam vertex. |
| `mod.rs:2044:69` | `split_periodic_face_by_rim_chains` | replace - with / | UNKILLABLE | Disables the `seam_off ≥ 2π − close_tol` disjunct, which only catches a chain starting within close_tol (1e-5) past the seam meridian: below the weld scale, where its end is the seam vertex. |
| `mod.rs:2076:55` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:2081:30` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Differs only at `ip == iq`, where both orders give `ix == iy` and line 2082 declines. |
| `mod.rs:2082:21` | `split_periodic_face_by_rim_chains` | replace \|\| with && | EQUIVALENT | Unreachable disjuncts: `ix == iy` needs p == q (chains are open); `iy + 1 >= len` needs a chain end at the seam vertex (excluded by the seam clearance at 2044). |
| `mod.rs:2082:27` | `split_periodic_face_by_rim_chains` | replace + with * | EQUIVALENT | Unreachable disjuncts: `ix == iy` needs p == q (chains are open); `iy + 1 >= len` needs a chain end at the seam vertex (excluded by the seam clearance at 2044). |
| `mod.rs:2082:27` | `split_periodic_face_by_rim_chains` | replace + with - | EQUIVALENT | Unreachable disjuncts: `ix == iy` needs p == q (chains are open); `iy + 1 >= len` needs a chain end at the seam vertex (excluded by the seam clearance at 2044). |
| `mod.rs:2091:72` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:2130:57` | `split_periodic_face_by_rim_chains` | replace > with >= | EQUIVALENT | The two seam ends lie on the two rims, more than close_tol apart, so `vb == va` never occurs. |
| `mod.rs:2166:23` | `split_periodic_face_by_rim_chains` | replace * with / | EQUIVALENT | `sign(d0/d1) = sign(d0·d1)` for `d1 ≠ 0`; differs only for a sample exactly on the meridian, which the next pair re-finds. |
| `mod.rs:2166:41` | `split_periodic_face_by_rim_chains` | replace - with + | UNKILLABLE | Weakens the ±π wrap filter, which only fires for a sample pair straddling the meridian antipodal to the sample point. Notch chains span < π around the floor meridian, and in the sector cell the antipode of one rim span's middle is the other span's middle, which the chains do not cross. |
| `mod.rs:2166:53` | `split_periodic_face_by_rim_chains` | replace > with == | UNKILLABLE | Weakens the ±π wrap filter, which only fires for a sample pair straddling the meridian antipodal to the sample point. Notch chains span < π around the floor meridian, and in the sector cell the antipode of one rim span's middle is the other span's middle, which the chains do not cross. |
| `mod.rs:2166:53` | `split_periodic_face_by_rim_chains` | replace > with >= | UNKILLABLE | Weakens the ±π wrap filter, which only fires for a sample pair straddling the meridian antipodal to the sample point. Notch chains span < π around the floor meridian, and in the sector cell the antipode of one rim span's middle is the other span's middle, which the chains do not cross. |
| `mod.rs:2166:58` | `split_periodic_face_by_rim_chains` | replace \|\| with && | UNKILLABLE | Weakens the ±π wrap filter, which only fires for a sample pair straddling the meridian antipodal to the sample point. Notch chains span < π around the floor meridian, and in the sector cell the antipode of one rim span's middle is the other span's middle, which the chains do not cross. |
| `mod.rs:2166:65` | `split_periodic_face_by_rim_chains` | replace - with + | EQUIVALENT | Zero-length pair filter: a pair with `d0 = d1 = 0` gives a NaN crossing that fails the reach bracket anyway; the mutated test only skips a pair whose crossing the neighbouring pair re-finds, or one exactly centred between samples (measure zero). |
| `mod.rs:2166:65` | `split_periodic_face_by_rim_chains` | replace - with / | EQUIVALENT | Zero-length pair filter: a pair with `d0 = d1 = 0` gives a NaN crossing that fails the reach bracket anyway; the mutated test only skips a pair whose crossing the neighbouring pair re-finds, or one exactly centred between samples (measure zero). |
| `mod.rs:2166:77` | `split_periodic_face_by_rim_chains` | replace < with == | EQUIVALENT | Zero-length pair filter: a pair with `d0 = d1 = 0` gives a NaN crossing that fails the reach bracket anyway; the mutated test only skips a pair whose crossing the neighbouring pair re-finds, or one exactly centred between samples (measure zero). |
| `mod.rs:2169:32` | `split_periodic_face_by_rim_chains` | replace + with - | UNKILLABLE | Perturbs the interpolated crossing within its bracketing sample pair (17 samples per piece); results outside `(v_from, reach)` are rejected by the bracket, and the interior at the midpoint stays in the middle half of the region (asserted). |
| `mod.rs:2169:58` | `split_periodic_face_by_rim_chains` | replace / with % | UNKILLABLE | Perturbs the interpolated crossing within its bracketing sample pair (17 samples per piece); results outside `(v_from, reach)` are rejected by the bracket, and the interior at the midpoint stays in the middle half of the region (asserted). |
| `mod.rs:2169:58` | `split_periodic_face_by_rim_chains` | replace / with * | UNKILLABLE | Perturbs the interpolated crossing within its bracketing sample pair (17 samples per piece); results outside `(v_from, reach)` are rejected by the bracket, and the interior at the midpoint stays in the middle half of the region (asserted). |
| `mod.rs:2169:64` | `split_periodic_face_by_rim_chains` | replace - with + | UNKILLABLE | Perturbs the interpolated crossing within its bracketing sample pair (17 samples per piece); results outside `(v_from, reach)` are rejected by the bracket, and the interior at the midpoint stays in the middle half of the region (asserted). |
| `mod.rs:2169:64` | `split_periodic_face_by_rim_chains` | replace - with / | UNKILLABLE | Perturbs the interpolated crossing within its bracketing sample pair (17 samples per piece); results outside `(v_from, reach)` are rejected by the bracket, and the interior at the midpoint stays in the middle half of the region (asserted). |
| `mod.rs:2170:33` | `split_periodic_face_by_rim_chains` | replace * with / | EQUIVALENT | `sign(a/b) = sign(a·b)`; differs only at `reach == v`. |
| `mod.rs:2170:47` | `split_periodic_face_by_rim_chains` | replace > with >= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:2261:53` | `split_periodic_face_by_rim_chains` | replace < with <= | EQUIVALENT | Strict vs non-strict on a continuous float compared with a tolerance or zero: differs only on exact equality, which no constructible input pins (measure zero). |
| `mod.rs:2270:9` | `split_periodic_face_by_rim_chains` | replace \|\| with && | EQUIVALENT | With exactly two chains whose top ends are the two top split points, `top_end(c_yb) ≠ xt` holds iff `top_end(c_xb) ≠ yt`: the disjuncts are always equal. |
| `mod.rs:2282:62` | `split_periodic_face_by_rim_chains` | replace + with * | EQUIVALENT | Prepends the seam-side piece to the slice; `span_mid` indexes `len/2 ≥ 1`, which still selects a piece of the clear span, so the interior meridian stays in the clear sector (asserted with margin). |
| `special_cases.rs:408:45` | `split_noseam_by_arrangement` | replace * with / | UNKILLABLE | `max(tol/r, tol²)` vs `max(tol·r, tol²)` only reclassifies bounded cells of projected area between `tol/r` and `tol·r` (≤ 5e-6 up to r = 50); every non-sliver cell is orders larger and slivers are removed separately. |
| `special_cases.rs:408:72` | `split_noseam_by_arrangement` | replace * with + | UNKILLABLE | `max(tol·r, 2·tol)` vs `max(tol·r, tol²)` differs only for r < 2, in a band below 2e-7 of projected area that no non-sliver cell reaches. |
| `special_cases.rs:413:40` | `split_noseam_by_arrangement` | replace * with + | EQUIVALENT | Unreachable arm: `build_seam_arcs` returns `None` for every non-sphere surface, so the function returns unsplit before `area_tol` is computed. |

## Remainder outside this tranche

The whole-function runs also examined mutants the week's diff did not
select. Still missed on the final tree and not triaged here:

| Function | Missed |
| --- | ---: |
| `split_periodic_face_by_rim_chains` | 2 (`1877:87 < → <=`, `2166:77 < → <=`: equality boundaries, same class as the rows above) |
| `split_periodic_face_into_sectors` | 61 |
| `split_periodic_face_into_bands` | 42 |
| `split_noseam_by_arrangement` | 34 |

Also observed, not changed: the rim-chain and band shortcuts read the
boundary before `split_face_2d_impl` aligns closed-rim UVs to the seam, so
their emitted rim pieces carry UVs in the rim circle's own parameter (a
quarter turn off the cylinder chart for the primitive fixtures) while the
section edges carry chart UVs. The new chart oracle checks section edges
and synthesized seams only.

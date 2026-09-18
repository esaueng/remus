# Perf baseline at 3785eebc vs O3.1 baseline at 86c57df4

SHA-pinned Criterion baseline for current main, with a before/after table
against the SHA recorded in o31-inner-loop-baseline.md. No kernel changes were
made; all measurements were taken in detached worktrees. The O3.1 evidence
stays fixed to its SHA; this file is the new evidence directory entry for the
current SHA (roadmap rule).

## Provenance

- Prior SHA: 86c57df4137af83a38a75721776d3fcca0d34bcf (recorded in
  docs/kernel-maturity/o31-inner-loop-baseline.md)
- Current SHA: 3785eebcede74113f4b5b04fa6d884ae4a580127 (origin/main at fetch)
- 485 commits between the SHAs; the benchmarked paths gained the inner-loop
  suites (math/algo/blend/io benches did not exist at the prior SHA).
- Host: AMD Ryzen 9 5900XT, 16 cores / 32 threads, 31 GiB RAM, Linux x86-64.
- Toolchain: Rust 1.96.0 (ac68faa20 2026-05-25); Criterion 0.8.2 (plotters
  backend, Gnuplot absent).
- Measured 2026-09-18 (UTC) in /tmp/remus-perf-current (detached at current
  SHA) and /tmp/remus-perf-prior (detached at prior SHA). Criterion baselines:
  main-3785eebc / main-3785eebc-run2 (/run3) and prior / prior-run2 (/run3).

## Method

- Bench inventory: cargo bench --workspace -- --list gives 104 benchmarks at
  current (11 bench targets: 5 operations, 3 math, 1 algo, 1 blend with the
  bench-internals feature, 1 io) and 73 at prior (5 operations targets only).
- Each bench target ran twice per SHA with --save-baseline; any benchmark
  whose two medians differed by more than 5% got a third run and a noisy flag.
  Table values are the median of the available runs; spread is
  (max-min)/min over those runs. Medians come from each baseline's
  estimates.json median point estimate.
- Delta = (current_median - prior_median) / prior_median. Positive delta means
  current is slower.
- The box+fillet (bevel) bench aborts on both SHAs (see Blocked section), so
  cad_operations was measured with substring/exact filters that cover the
  other 28 benches; filters accumulate into the same named baselines.
- 32 benchmarks have no prior counterpart (4 added to boolean_tracking, 28 in
  the new math/algo/blend/io suites); they are reported current-only.

## Measurement conditions (read before citing a delta)

- The host was shared: other sessions ran cargo test/clippy/mutants jobs
  during parts of the current-SHA runs (load 5-90, spiking to 38 during
  compound_cut run 2). Prior-SHA runs mostly landed in a quieter window
  (load ~2-5). Run order was current-first, prior-second, so environment
  drift is confounded with SHA; uniform small deltas in one direction must be
  re-measured on an idle host before being treated as regressions.
- Two current-SHA single-run outliers (boolean/fuse_box_box run 3 at 1.79 ms,
  boolean/intersect_box_box run 3 at 0.039 ms vs ~1.1 ms / ~0.021 ms siblings)
  look like host interference; median-of-3 is robust to them and both rows are
  flagged noisy.
- A transient disk-full episode (root at 100%, caused by other sessions'
  /tmp worktrees, since recovered to 67%) corrupted 6 in-progress criterion
  JSON files. All named-baseline estimates.json files were re-validated
  afterwards (150 parse clean); the affected run-3 benches were re-run clean.
- Saved baselines live only in the two detached worktrees' target/criterion
  directories, which are scratch space. Re-running from this file's method
  section reproduces the table; the raw JSON is not committed.

## Comparable benchmarks (72)

| Benchmark | Prior (86c57df4) | Current (3785eebc) | Delta | Noisy |
|---|---|---|---:|:---:|
| boolean 64 cuts (8x8 grid) | 607.612 ms (n=2, spread 1.2%) | 735.342 ms (n=2, spread 1.1%) | +21.0% |  |
| boolean/cut_box_box | 947.03 µs (n=2, spread 0.2%) | 986.59 µs (n=3, spread 7.0%) | +4.2% | noisy |
| boolean/cut_cylinder_through_box | 720.73 µs (n=2, spread 3.7%) | 821.37 µs (n=3, spread 6.4%) | +14.0% | noisy |
| boolean/fuse_box_box | 1.024 ms (n=2, spread 2.9%) | 1.124 ms (n=3, spread 69.2%) | +9.8% | noisy |
| boolean/intersect_box_box | 18.83 µs (n=2, spread 2.2%) | 21.95 µs (n=3, spread 85.7%) | +16.6% | noisy |
| boolean/perforated_cut_36 | 28.277 ms (n=2, spread 0.5%) | 24.976 ms (n=3, spread 14.4%) | -11.7% | noisy |
| boundingBox x100 | 156.05 µs (n=2, spread 2.9%) | 175.31 µs (n=2, spread 2.2%) | +12.3% |  |
| box+chamfer | 527.82 µs (n=3, spread 65.2%) | 856.64 µs (n=3, spread 155.8%) | +62.3% | noisy |
| box+fillet | 12.975 ms (n=2, spread 2.8%) | 54.481 ms (n=2, spread 1.6%) | +319.9% |  |
| compound_cut_cylinders/compound_N=16 | 11.406 ms (n=2, spread 0.2%) | 12.848 ms (n=3, spread 11.0%) | +12.6% | noisy |
| compound_cut_cylinders/compound_N=36 | 25.750 ms (n=2, spread 0.8%) | 29.103 ms (n=3, spread 12.7%) | +13.0% | noisy |
| compound_cut_cylinders/compound_N=4 | 3.083 ms (n=2, spread 0.7%) | 3.516 ms (n=3, spread 13.3%) | +14.0% | noisy |
| compound_cut_cylinders/compound_N=64 | 47.090 ms (n=2, spread 0.2%) | 53.180 ms (n=3, spread 9.8%) | +12.9% | noisy |
| compound_cut_cylinders/sequential_N=16 | 44.111 ms (n=2, spread 0.4%) | 51.438 ms (n=3, spread 12.0%) | +16.6% | noisy |
| compound_cut_cylinders/sequential_N=36 | 198.694 ms (n=2, spread 0.4%) | 235.912 ms (n=3, spread 11.6%) | +18.7% | noisy |
| compound_cut_cylinders/sequential_N=4 | 4.557 ms (n=2, spread 0.7%) | 5.313 ms (n=3, spread 13.1%) | +16.6% | noisy |
| compound_cut_cylinders/sequential_N=64 | 608.814 ms (n=2, spread 0.8%) | 737.979 ms (n=3, spread 13.0%) | +21.2% | noisy |
| compound_cut_honeycomb/compound_rings=1_N=7 | 6.294 ms (n=2, spread 0.6%) | 5.765 ms (n=3, spread 10.9%) | -8.4% | noisy |
| compound_cut_honeycomb/compound_rings=2_N=19 | 16.378 ms (n=2, spread 0.0%) | 15.088 ms (n=3, spread 9.4%) | -7.9% | noisy |
| compound_cut_honeycomb/compound_rings=3_N=37 | 32.377 ms (n=2, spread 0.1%) | 30.106 ms (n=3, spread 11.2%) | -7.0% | noisy |
| compound_cut_honeycomb/compound_rings=5_N=91 | 84.030 ms (n=2, spread 0.5%) | 78.838 ms (n=3, spread 14.6%) | -6.2% | noisy |
| compound_cut_honeycomb/sequential_rings=1_N=7 | 15.199 ms (n=2, spread 0.6%) | 13.462 ms (n=3, spread 11.0%) | -11.4% | noisy |
| compound_cut_honeycomb/sequential_rings=2_N=19 | 85.257 ms (n=2, spread 0.3%) | 74.207 ms (n=3, spread 13.5%) | -13.0% | noisy |
| compound_cut_honeycomb/sequential_rings=3_N=37 | 297.516 ms (n=2, spread 0.3%) | 260.142 ms (n=3, spread 8.7%) | -12.6% | noisy |
| compound_cut_honeycomb/sequential_rings=5_N=91 | 1.932 s (n=2, spread 0.7%) | 1.803 s (n=3, spread 34.3%) | -6.7% | noisy |
| compound_cut_struts/nway_N=10 | 120.800 ms (n=2, spread 0.0%) | 119.760 ms (n=3, spread 20.9%) | -0.9% | noisy |
| compound_cut_struts/nway_N=14 | 306.134 ms (n=2, spread 0.4%) | 307.954 ms (n=3, spread 17.5%) | +0.6% | noisy |
| compound_cut_struts/nway_N=6 | 36.455 ms (n=2, spread 0.4%) | 35.428 ms (n=3, spread 25.5%) | -2.8% | noisy |
| compound_cut_struts/sequential_N=10 | 154.975 ms (n=2, spread 1.0%) | 162.455 ms (n=3, spread 25.8%) | +4.8% | noisy |
| compound_cut_struts/sequential_N=14 | 392.898 ms (n=2, spread 0.5%) | 417.892 ms (n=3, spread 19.0%) | +6.4% | noisy |
| compound_cut_struts/sequential_N=6 | 44.573 ms (n=2, spread 0.4%) | 45.027 ms (n=3, spread 21.6%) | +1.0% | noisy |
| cut(box,cyl) x10 | 11.517 ms (n=2, spread 0.8%) | 14.720 ms (n=2, spread 2.1%) | +27.8% |  |
| fuse(box,box) x10 | 644.91 µs (n=3, spread 9.5%) | 721.37 µs (n=2, spread 3.0%) | +11.9% | noisy |
| fuse_balanced/balanced_N=16 | 77.508 ms (n=2, spread 0.5%) | 65.639 ms (n=2, spread 0.0%) | -15.3% |  |
| fuse_balanced/balanced_N=25 | 235.087 ms (n=2, spread 0.9%) | 223.031 ms (n=2, spread 2.4%) | -5.1% |  |
| fuse_balanced/balanced_N=4 | 5.593 ms (n=2, spread 0.5%) | 3.639 ms (n=2, spread 2.5%) | -34.9% |  |
| fuse_balanced/balanced_N=9 | 23.028 ms (n=2, spread 0.1%) | 16.955 ms (n=2, spread 1.0%) | -26.4% |  |
| fuse_balanced/sequential_N=16 | 57.159 ms (n=2, spread 0.1%) | 54.035 ms (n=2, spread 0.4%) | -5.5% |  |
| fuse_balanced/sequential_N=25 | 136.572 ms (n=2, spread 0.4%) | 128.814 ms (n=2, spread 1.0%) | -5.7% |  |
| fuse_balanced/sequential_N=4 | 4.161 ms (n=2, spread 1.0%) | 3.365 ms (n=2, spread 0.3%) | -19.1% |  |
| fuse_balanced/sequential_N=9 | 19.674 ms (n=2, spread 1.0%) | 16.850 ms (n=2, spread 2.1%) | -14.4% |  |
| fuse_touching/balanced_N=16 | 29.713 ms (n=2, spread 0.7%) | 24.988 ms (n=2, spread 1.6%) | -15.9% |  |
| fuse_touching/balanced_N=4 | 2.720 ms (n=2, spread 1.3%) | 1.902 ms (n=2, spread 1.3%) | -30.1% |  |
| fuse_touching/balanced_N=9 | 10.114 ms (n=2, spread 0.5%) | 7.584 ms (n=2, spread 1.1%) | -25.0% |  |
| gridPattern(box, 3x3) | 121.02 µs (n=2, spread 1.3%) | 135.35 µs (n=2, spread 1.4%) | +11.8% |  |
| gridfinity 1x1 bin (box+shell+chamfer) | 450.29 µs (n=2, spread 0.1%) | 520.56 µs (n=2, spread 3.8%) | +15.6% |  |
| gridfinity 3x3 baseplate (grid+holes) | 4.592 ms (n=2, spread 0.1%) | 5.331 ms (n=2, spread 2.0%) | +16.1% |  |
| intersect(box,sphere) single | 42.22 µs (n=2, spread 0.9%) | 48.23 µs (n=2, spread 3.6%) | +14.2% |  |
| intersect(box,sphere) x10 | 439.27 µs (n=2, spread 4.4%) | 475.53 µs (n=2, spread 4.1%) | +8.3% |  |
| linearPattern(box, 10) | 137.13 µs (n=2, spread 1.5%) | 151.73 µs (n=3, spread 5.6%) | +10.7% | noisy |
| makeBox(10,20,30) x100 | 246.06 µs (n=2, spread 0.8%) | 274.45 µs (n=2, spread 0.6%) | +11.5% |  |
| makeCylinder(5,20) single | 912.5 ns (n=2, spread 4.2%) | 1.04 µs (n=2, spread 2.9%) | +13.7% |  |
| makeCylinder(5,20) x100 | 89.79 µs (n=2, spread 1.4%) | 104.49 µs (n=2, spread 1.5%) | +16.4% |  |
| makeSphere(10) single | 3.06 µs (n=2, spread 1.4%) | 3.35 µs (n=2, spread 2.2%) | +9.6% |  |
| makeSphere(10) x100 | 301.32 µs (n=2, spread 1.9%) | 344.49 µs (n=2, spread 0.8%) | +14.3% |  |
| mesh box (tol=0.1) | 17.11 µs (n=2, spread 0.5%) | 19.36 µs (n=3, spread 7.2%) | +13.1% | noisy |
| mesh sphere (tol=0.01) | 136.43 µs (n=2, spread 2.2%) | 149.89 µs (n=3, spread 8.6%) | +9.9% | noisy |
| multi-boolean model | 5.147 ms (n=3, spread 7.1%) | 6.026 ms (n=2, spread 1.5%) | +17.1% | noisy |
| rotate x100 | 1.155 ms (n=2, spread 2.9%) | 1.302 ms (n=2, spread 3.9%) | +12.7% |  |
| sequential_cylinder_cuts/N=16 | 43.924 ms (n=2, spread 0.5%) | 51.773 ms (n=3, spread 7.0%) | +17.9% | noisy |
| sequential_cylinder_cuts/N=4 | 4.547 ms (n=3, spread 5.8%) | 5.303 ms (n=3, spread 2.6%) | +16.6% | noisy |
| sequential_cylinder_cuts/N=64 | 619.118 ms (n=2, spread 4.6%) | 737.741 ms (n=3, spread 3.0%) | +19.2% |  |
| shell(box, t=1, open_top) | 97.35 µs (n=2, spread 0.5%) | 118.90 µs (n=3, spread 5.7%) | +22.1% | noisy |
| single_boolean_at_face_count/F=6 (bare box) | 726.24 µs (n=2, spread 1.9%) | 855.19 µs (n=3, spread 0.9%) | +17.8% |  |
| single_boolean_at_face_count/F~18 (4 cuts) | 1.843 ms (n=2, spread 0.3%) | 2.211 ms (n=3, spread 2.1%) | +20.0% |  |
| single_boolean_at_face_count/F~54 (16 cuts) | 5.119 ms (n=2, spread 1.4%) | 6.031 ms (n=3, spread 3.6%) | +17.8% |  |
| tessellate 64-hole plate | 28.484 ms (n=2, spread 1.7%) | 31.095 ms (n=3, spread 12.7%) | +9.2% | noisy |
| tessellate box∩sphere (tol=0.01) | 1.028 ms (n=2, spread 0.0%) | 1.164 ms (n=2, spread 4.5%) | +13.2% |  |
| translate x1000 | 14.745 ms (n=2, spread 1.1%) | 12.897 ms (n=3, spread 44.7%) | -12.5% | noisy |
| translate_x1000_ab/copy+transform (old) | 14.535 ms (n=2, spread 1.9%) | 12.601 ms (n=2, spread 3.4%) | -13.3% |  |
| translate_x1000_ab/copy_and_transform (new) | 12.519 ms (n=2, spread 1.5%) | 13.147 ms (n=2, spread 3.7%) | +5.0% |  |
| volume x100 | 2.775 ms (n=2, spread 2.0%) | 3.107 ms (n=3, spread 7.9%) | +11.9% | noisy |

## Current-only benchmarks (32, no prior counterpart)

| Benchmark | Current (3785eebc) | Runs | Spread |
|---|---|---:|:---:|
| bezier_clip_cubic_pair | 89.56 µs | 2 | 0.8% |
| blend_walker/plane_pair_steps | 63.05 µs | 2 | 2.3% |
| boolean/cross_drilled_cylinder | 13.849 ms | 3 | 7.0% |
| boolean/torus_notch_cut | 9.636 ms | 3 | 4.7% |
| boolean/torus_notch_fuse | 9.545 ms | 3 | 3.6% |
| boolean/torus_notch_intersect | 8.889 ms | 3 | 5.5% |
| cdt_insertion/1000 | 643.75 µs | 2 | 0.1% |
| cdt_insertion/10000 | 7.486 ms | 2 | 2.3% |
| flamegraph_hot/analytic_cylinder_evaluate | 11.9 ns | 2 | 3.7% |
| flamegraph_hot/analytic_cylinder_project_point | 25.4 ns | 2 | 1.7% |
| flamegraph_hot/point_in_polygon_64 | 43.6 ns | 2 | 0.0% |
| flamegraph_hot/winding_number_64 | 43.8 ns | 2 | 1.6% |
| gfa_phases/box_cylinder_cut | 531.74 µs | 2 | 0.5% |
| gfa_phases/overlapping_boxes_fuse | 792.28 µs | 2 | 0.3% |
| nurbs/basis/degree3 | 27.2 ns | 2 | 0.4% |
| nurbs/basis/degree9 | 116.4 ns | 2 | 0.3% |
| nurbs/basis_derivatives/degree3 | 74.6 ns | 2 | 1.9% |
| nurbs/basis_derivatives/degree9 | 247.6 ns | 2 | 0.6% |
| nurbs/curve_derivatives/degree3 | 150.6 ns | 2 | 0.6% |
| nurbs/curve_derivatives/degree9 | 361.8 ns | 2 | 1.0% |
| nurbs/curve_evaluate/degree3 | 45.1 ns | 2 | 0.5% |
| nurbs/curve_evaluate/degree9 | 157.4 ns | 2 | 0.2% |
| nurbs/surface_derivatives/degree3 | 548.7 ns | 2 | 0.5% |
| nurbs/surface_derivatives/degree9 | 2.23 µs | 2 | 0.5% |
| nurbs/surface_evaluate/degree3 | 111.2 ns | 2 | 0.2% |
| nurbs/surface_evaluate/degree9 | 523.5 ns | 2 | 0.1% |
| nurbs_properties/mass_properties (hammer holder) | 2.339 s | 2 | 0.9% |
| nurbs_properties/validate_solid strict (hammer holder) | 987.310 ms | 2 | 0.4% |
| ssi/nurbs_march | 463.87 µs | 2 | 0.2% |
| ssi/nurbs_seed | 122.07 µs | 2 | 0.3% |
| ssi/quadric_march | 7.970 ms | 2 | 0.2% |
| ssi/quadric_seed | 405.27 µs | 2 | 0.6% |

## Regressions over 10% (current slower)

Only the box+fillet row is tight on both sides; everything else is noisy,
within the shared-host confound band, or both. Commits listed are the ones
between the SHAs that touched the benchmarked path (git log prior..current
on the path); no causal claim is made.

- box+fillet +319.9% (tight both sides; rolling-ball engine on a 20-unit box,
  all edges, r=1). Path commits: dd70d36c (fillet band continuation), ca91b956
  (B4 planar trimmer), e0cab854 (fillet cascade), 316cc2ad (face-face sheets),
  08fe6fab (typed cliff handling), 216db1a3 and c292b876 (blend queue
  reconciliation and oracle flooring).
- box+chamfer +62.3% (noisy both sides, 65%/156% spreads; do not cite without
  an idle re-run). Same blend-area commits as above.
- cut(box,cyl) x10 +27.8% (tight). Boolean/algo path commits: 281afaa5
  (torus band loops), 97bc7afd (phase_ff exhaustiveness), 9307e73d (converted
  cylinder decapitation, cut-only), 95de1604 and 6195c1af (distance/NURBS
  pruning and edge-face box gating), 1ec84bfa (converted B-spline planes),
  plus the torus-notch/seam batch a764ad34, 39710ddb, b7bda976, 6516a4bf,
  78ffe41c and the scale-band series (fd81f257 through ad0c21eb).
- boolean 64 cuts +21.0%, sequential_cylinder_cuts N=64 +19.2%,
  single_boolean F~18 +20.0%, F=6 +17.8%, F~54 +17.8% (all tight). Same
  boolean/algo commits as above.
- gridfinity 3x3 +16.1%, gridfinity 1x1 +15.6% (tight; shell+chamfer chain).
  Blend/shell-area commits: ca91b956 plus the compound/chamfer-area 85a310df
  (trimmed distances, fuse-all inputs).
- tessellate box∩sphere +13.2% (tight). Tessellation commits: 5558143e (seam
  vertices on stepped walls), bf592b72 (interior supports seeding), 7a5b6b5d
  (sphere-cylinder seam preservation), ac9da9ef (stepped-rim double seeding),
  338a2481 (nonplanar exhaustiveness).
- makeBox/makeCylinder/makeSphere/boundingBox/gridPattern/rotate/intersect
  single +11% to +16% (tight, micro-benchmarks under 350 µs). Transform-area
  commits: 49567b02 and ff4cb012 (endpoint certificates); sphere-frame
  f3e846ec; measure-area 0d87c1cc, 60372661, cfd7586a. These sit inside the
  shared-host confound band and ran while competing jobs were active; treat
  as unconfirmed until an idle re-run.
- Remaining rows over 10% are all flagged noisy (compound cylinders,
  cut_cylinder_through_box, intersect_box_box, fuse(box,box), linearPattern,
  mesh box/sphere, multi-boolean, sequential N=4/N=16, shell, volume,
  tessellate 64-hole).

## Improvements over 10% (current faster)

- fuse_balanced N=4 -34.9%, N=9 -26.4%, sequential N=4 -19.1%,
  balanced N=16 -15.3%, sequential N=9 -14.4% (all tight); fuse_touching N=4
  -30.1%, N=9 -25.0%, N=16 -15.9% (all tight). Compound/fuse-all commits:
  85bb7bd3 (exact-only boolean default) and 85a310df (trimmed distances,
  fuse-all inputs).
- compound_cut_honeycomb sequential rings 1-3 -11% to -13% (noisy).
- boolean/perforated_cut_36 -11.7% (noisy). Same boolean/algo commits as the
  regression list above.
- translate x1000 -12.5% (noisy), translate_x1000_ab old -13.3% (tight).
  Transform commits 49567b02, ff4cb012.

## Blocked: box+fillet (bevel) fails on both SHAs

The deprecated flat-bevel v1 fillet on box(20,20,20), all edges, r=1 fails
postcondition validation identically at both SHAs, aborting a full
cad_operations run at that bench:

fillet postcondition validation failed with 1 error(s): shell has 24 free
(boundary) edges (cad_operations.rs line 306 unwrap)

Reproduced deterministically via the bevel filter on each SHA; the bench file
itself is unchanged between the SHAs. It is excluded from both baselines, so
the comparison is unaffected. Follow-up belongs to the fillet/trimmer owner,
not to this baseline.

## Reproducing

From a clean checkout at either SHA, with an otherwise idle host:

cargo bench -p remus-operations --bench boolean_tracking -- --save-baseline NAME
cargo bench -p remus-operations --bench boolean_perf -- --save-baseline NAME
cargo bench -p remus-operations --bench compound_cut_perf -- --save-baseline NAME
cargo bench -p remus-operations --bench fuse_perf -- --save-baseline NAME
cargo bench -p remus-math --bench nurbs_inner_loops -- --save-baseline NAME
cargo bench -p remus-math --bench cdt_insertion -- --save-baseline NAME
cargo bench -p remus-math --bench intersection_inner_loops -- --save-baseline NAME
cargo bench -p remus-algo --bench gfa_inner_loops -- --save-baseline NAME
cargo bench -p remus-blend --features bench-internals --bench walker_steps -- --save-baseline NAME
cargo bench -p remus-io --bench nurbs_properties -- --save-baseline NAME

For cad_operations, run all benches except the blocked bevel bench via
substring/exact filters (see Method), accumulating into the same baseline
name. Before quoting any delta from this file, re-run both sides on an idle
host; the noisy flags and the conditions section say which rows need it most.

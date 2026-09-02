# O3.1 inner-loop benchmark baseline

O3.1 declares an inclusive CPU-sample threshold of **3%**. Every Remus
function at or above that share in either reference workload is covered by a
Criterion benchmark below. Inclusive shares intentionally keep orchestration
frames: a regression in a child remains visible in the parent operation.

## Profile provenance

- Source: `86c57df4137af83a38a75721776d3fcca0d34bcf`
- Host: AMD Ryzen 9 5900XT, 16 cores / 32 threads, Linux x86-64
- Toolchain: Rust 1.96.0; `cargo-flamegraph` 0.6.14; Linux `perf` 6.8.12
- Sampling: user-process CPU cycles, DWARF stacks capped at 8 KiB, 1,024 mmap
  pages. Both cited captures completed without lost samples.

The 64-cut benchmark constructs its 64-hole fixture before Criterion applies
the name filter, so this command profiles the same sequential 64-cut path even
though the filtered timing loop itself does not run:

```bash
cargo flamegraph --profile profiling --bench cad_operations \
  -p remus-operations \
  --cmd 'record -F 997 --call-graph dwarf,8192 -g -m 1024' \
  --no-inline --deterministic -o /tmp/remus-64cut.svg \
  -- --bench 'boolean 64 cuts (8x8 grid)'

cargo flamegraph --profile profiling --bench cad_operations \
  -p remus-operations \
  --cmd 'record -F 199 --call-graph dwarf,8192 -g -m 1024' \
  --no-inline --deterministic -o /tmp/remus-gridfinity.svg \
  -- --bench gridfinity
```

## Threshold census

Nested frames with the same samples are grouped. The share shown is the
largest observed inclusive share in that stack family.

| Workload | At-or-above-3% Remus stack family | Max share | Criterion coverage |
|---|---|---:|---|
| 64-cut | `operations::boolean`, validation, `check::face_integrator` volume stack | 32.28% | Existing `cad_operations` 64-cut/volume cases and `boolean_tracking` |
| 64-cut | `algo::gfa`, `PaveFiller`, VE, builder perform/build-result | 40.41% | `gfa_inner_loops`: box-cylinder cut and overlapping-box fuse |
| 64-cut | `topology::FaceSurface::{evaluate,project_point}` → analytic cylinder methods | 5.91% | `nurbs_inner_loops`: direct cylinder evaluate/project plus the GFA fixture |
| 64-cut | `math::predicates::{point_in_polygon,winding_number}` | 5.48% | `nurbs_inner_loops`: direct 64-vertex predicate cases |
| Gridfinity | `operations::boolean`, validation, `check::face_integrator` volume stack | 5.25% | Existing Gridfinity, volume, and `boolean_tracking` cases |
| Gridfinity | `algo::gfa`, `run_pave_filler_with_context`, EF | 9.81% | `gfa_inner_loops`: box-cylinder cut and overlapping-box fuse |
| Gridfinity | `operations::chamfer` transaction/core | 3.06% | Existing `gridfinity 1x1 bin` and chamfer cases in `cad_operations` |

## Roadmap-required baselines

The threshold census is supplemented by the complete O3.1 prerequisite set:

- NURBS basis, curve/surface evaluation, and derivatives at degrees 3 and 9
- SSI seeding-only and bounded marching for a rational quadric pair and a
  non-quadric NURBS pair
- cubic Bézier clipping
- Hilbert-ordered CDT insertion at 1,000 and 10,000 points
- full GFA phase stacks on analytic and planar fixtures
- blend-walker section throughput (reported as steps per second)

`scripts/bench-compare.sh` runs every maintained math, algo, and blend suite
plus the boolean tracking, boolean scaling, and Gridfinity reference suites;
`.github/workflows/benchmark.yml` records the focused O3.1 and
boolean-tracking cases on the hosted trend baseline.

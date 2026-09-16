# O3.3 SIMD in NURBS evaluation — evidence note

Row: [O3.3 SIMD in NURBS evaluation](roadmap.md#o-o3-3) (evidence-gated).
Prerequisite: [O3.1 inner-loop baseline](o31-inner-loop-baseline.md).
Related perf item: [PERF-N10](roadmap.md#perf-n10).

## Verdict

**Ship.** The hottest NURBS loop clears the 3% inclusive threshold on a
NURBS-heavy workload on both targets with bit-identical results, so the
manual 4-lane version behind the `simd` cargo feature is merged:

- `remus-math` gains a `simd` feature (`crates/math/Cargo.toml`).
- `NurbsSurface::derivatives_into_with_spans`
  (`crates/math/src/nurbs/surface.rs`, `contract_aw_cells_4lane`) strip-mines
  the `j` (v-direction) leg of the homogeneous contraction in groups of four
  lanes with a scalar tail. Each lane issues the same operations in the same
  order as the scalar nest and lanes never exchange partial sums, so output
  is bit-identical (not 1-ulp: f64-exact), pinned by
  `contract_4lane_bit_identical` and the hammer-holder exact-output record
  below.

This is explicit unroll-and-jam instruction-level parallelism for the
superscalar pipeline — portable to wasm32, where autovectorization of the
scalar nest is target-dependent — not true SIMD intrinsics: no packed
registers, no alignment requirements, no lane-tail masking beyond the scalar
tail loop. The O3.3 exit gate in `open-kernel-implementation.md` names
`wide`/portable SIMD with a ≥1.5×-or-documented-negative-result gate; this
note records the measured outcome against the task's 3% inclusive threshold
on both targets instead: the loop qualifies on NURBS-heavy work, the gain is
a few percent (not ×1.5), and it ships because it clears 3% with zero
accuracy cost, not because it approaches 1.5×.

## Where the loop sits (samply, `--unstable-presymbolicate`, 5 kHz)

Branch `docs/o33-simd-evidence`, host AMD Ryzen 9 5900XT, Linux x86-64,
Rust 1.96.0, `[profile.profiling]` builds. Inclusive shares (a sample counts
for every frame on its stack):

| Workload | NURBS basis/eval share | Detail |
|---|---|---|
| Sequential 64-cut (box minus 8×8 r=2 cylinders, pairwise `boolean`) | `nurbs::basis` 0.49% (22/4521 samples); `NurbsCurve2D` 0.27% | Faces stay analytic (`CylindricalSurface` 18.69%, `Circle3D` 10.24%); `ders_basis_funs_into` never appears. The 64-cut/Gridfinity flamegraphs do **not** name basis/evaluation hot. |
| Gridfinity 1×1 bin (`box+shell+chamfer`) ×400 | 0 NURBS frames in 1441 samples | Boolean 14.43%, chamfer 66.41%, tessellation/CDT the rest. |
| Tessellate 64-hole plate (`tess_profile`, bool + 3× tess) | `nurbs::basis` 0.47% of the 5124-sample boolean thread; 0 in the tess threads | Tess threads are ~90% CDT. |
| Hammer-holder STEP, `mass_properties` (42 NURBS faces) | `nurbs::surface` 78.12%; `ders_basis_funs_into` **29.80%** (8802/29533) | `span_hinted_point_and_partials_from` 77.12% → `derivatives_into_with_spans` 75.63% → `ders_basis_funs_into` 29.80%. |
| Hammer-holder STEP, strict `validate_solid` | `nurbs::surface` 73.38%; `ders_basis_funs_into` **28.44%** (3501/12311) | Same stack shape as mass properties. |

So the O3.1 workloads (box/cylinder analytic booleans, planar chamfer,
CDT tessellation) never touch the NURBS basis path — expected: nothing there
is NURBS. The loop that *is* hot on real NURBS work is the homogeneous
contraction inside `derivatives_into_with_spans`, called per Gauss abscissa
by the face integrator (`integrate_parametric` 96.49% inclusive of the mass
run), with `ders_basis_funs_into` at ~30% inclusive inside it.

## What was prototyped and measured

`contract_aw_cells_4lane`: four independent `[f64; 4]` accumulators over the
`j` leg, folded left-to-right in visit order, scalar tail for `pv+1 mod 4`.
Changed files: `crates/math/Cargo.toml` (`simd` feature),
`crates/math/src/nurbs/surface.rs` (scalar nest factored to
`contract_aw_cells_scalar` + shared `accumulate_aw_cell`, 4-lane variant,
`contract_4lane_bit_identical` test). No new workspace dependency; layer
boundaries unchanged (math-internal only).

Bit-identity:

- `cargo nextest run -p remus-math`: 620 passed (scalar), 621 passed
  (`--features simd`, incl. `contract_4lane_bit_identical`).
- Hammer-holder exact-output record (temporary `o33_hash` example, since
  removed; scalar vs `simd` binaries, full precision, `diff`-clean):
  `valid=true faces=160`, `mass=5.02453881741498190e4`,
  `com=1.09830195196429159e1,2.61402139590076601e1,2.72194240890826897e1`,
  `inertia=2.64260958513019234e7,5.73724290002014264e7,5.03917290279597417e7,`
  `6.66200789452344179e3,-2.66219415201246738e3,-5.65893560358167440e6`.
- WASM `massProperties` probe on the extruded bicubic sheet solid is
  character-identical between base and `simd` packages (volume
  `4083.5754969909453`, full center/inertia JSON equal).

Native Criterion deltas (`[profile.profiling]`, same machine):

| Bench | Baseline | `simd` | Delta |
|---|---|---:|---:|
| `nurbs_properties/validate_solid strict (hammer holder)` | 1.1091 s (median of 10) | 1.0544 s | **−4.9%** |
| `nurbs_properties/mass_properties (hammer holder)` | 2.6471 s | 2.5075 s | **−5.3%** |
| `nurbs/surface_derivatives/degree3` | ~607–621 ns | ~545 ns | −10…−12% (different-session baselines; treat as directional) |
| `nurbs/surface_derivatives/degree9` | ~2.50–2.52 µs | ~1.80 µs | −28% (same caveat; micro-bench, not the gate) |

WASM (Node 24, `wasm-pack --target nodejs --release --no-opt`, shipped
flags `RUSTFLAGS="-Dwarnings -C target-feature=+simd128"` in both builds —
i.e. the shipped package already enables simd128 codegen; the `simd`
feature only switches the contraction source shape):

| Harness | Baseline | `simd` | Delta |
|---|---|---:|---:|
| `evaluateSurfaceNormal` 20k loop (bicubic face, d=1 path), median of 5 | 936.9 ns/iter | 827.2 ns/iter (best run; reruns 852–914 vs 911–931) | −5…−12% run-dependent |
| `massProperties` on extruded 8×8 bicubic sheet solid, median of 5 | 18.6 / 17.7 ms | 15.5 / 15.7 ms | **−11…−12%**, bit-identical JSON |

WASM binary size (unoptimized `--no-opt` node builds, for attribution
only): base 9581008 B vs simd 9582618 B (+1610 B, +0.017%). The shipped
`-Oz --enable-simd` artifact delta is recorded by the WASM Size Report on
the PR; no `MAX_WASM_SIZE` risk from a +1.6 KiB unoptimized delta.

## Threshold accounting

The task's ship rule: clear the 3% inclusive threshold on both targets with
bit-identical results. The ~30% `ders_basis_funs_into` share is *inside* the
contraction call tree being optimized, and the end-to-end NURBS-workload
deltas clear 3% on both targets (native −4.9%/−5.3% on the maintained
`nurbs_properties` benches; WASM −11…−12% on the `massProperties` harness),
with f64-exact output on both. The 64-cut/Gridfinity workloads sit at
~0.5%/0% NURBS — the prototype neither helps nor hurts them (analytic and
CDT paths untouched). Amdahl check on the mass run: ~30% of the workload in
the optimized tree × ~15% loop speedup ≈ 4–5% end-to-end — consistent with
the measured −5.3%.

## Follow-ups (not this row)

- The remaining ~70% of the mass run is `ders_basis_funs_into` internals
  (the triangular `ndu` solve) plus integrator/trim overhead — a harder
  target (division-heavy, carried dependencies). PERF-N10 stays open for it.
- `PowerBasis1D`/Horner cached evaluation (PERF-N05) is a separate,
  workload-dependent question; this row does not adopt it.
- Enabling `simd` for the shipped WASM package (xtask flag / default
  features) is a distribution decision for the PR discussion, not taken
  here: the feature is opt-in, default-off, so shipped bytes are unchanged.

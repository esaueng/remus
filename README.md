<div align="center">

# Remus

Exact B-Rep solid modeling kernel for Rust and WebAssembly.

[![CI](https://github.com/esaueng/remus/actions/workflows/ci.yml/badge.svg)](https://github.com/esaueng/remus/actions/workflows/ci.yml)
[![Commit activity](https://img.shields.io/github/commit-activity/m/esaueng/remus?label=commits%2Fmonth)](https://github.com/esaueng/remus/commits/main)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](#license)
[![Rust 1.88+](https://img.shields.io/badge/rust-1.88%2B-orange.svg)](https://www.rust-lang.org/)
[![unsafe denied](https://img.shields.io/badge/unsafe-denied-success.svg)](#why-a-cad-kernel)

**[Kernel contract](#kernel-contract)** · **[Architecture](#architecture)** · **[Performance](#performance)** · **[Getting started](#getting-started)** · **[Known limitations](#known-limitations)** · **[Contributing](./CONTRIBUTING.md)**

</div>

One exact-geometry engine, from Rust and from JavaScript. Cut a solid, measure it, export it.

```rust
use remus::prelude::*;

# fn main() -> Result<(), Box<dyn std::error::Error>> {
let mut model = Model::new();

// Primitives are anchored at the origin, so this cylinder rounds off the
// block's corner. Transform it first to place the cut elsewhere.
let block = model.make_box(30.0, 20.0, 10.0)?;
let cutter = model.make_cylinder(5.0, 15.0)?;
let notched = model.cut(block, cutter)?;

// Every policy-aware boolean discloses whether its result stayed exact.
assert_eq!(notched.quality, BooleanQuality::Exact);
let volume = model.volume(notched.solid, 0.1)?;
let step = model.write_step(&[notched.solid])?;

assert!(volume > 0.0);
assert!(step.starts_with("ISO-10303-21;"));
# Ok(())
# }
```

```js
import { BrepKernel } from 'remus-wasm';

const kernel = new BrepKernel();

// Primitives are anchored at the origin, so this cylinder rounds off the
// block's corner. Use `transformSolid` to place it somewhere else.
const block = kernel.makeBox(30, 20, 10);
const cutter = kernel.makeCylinder(5, 15);
const notched = kernel.cut(block, cutter);

// Measure and export
const vol = kernel.volume(notched, 0.1);
const step = kernel.exportStep(notched); // Uint8Array
```

## Why a CAD kernel?

Remus is a B-Rep solid modeling kernel written from scratch in Rust. It targets
WebAssembly, so the same kernel runs in the browser and on the desktop.
`unsafe` is denied by lint, as are `unwrap` and `panic`. Every public operation
returns a `Result`.

Parametric CAD in the browser has long meant choosing between proprietary
kernels and large C++ codebases compiled to WASM. Remus exists to be the third
option: a from-scratch Rust kernel with exact geometry and a permanent
Apache-2.0 license. It is maintained by Esau Engineering as the Apache-2.0
continuation of an upstream kernel that relicensed at v3 — see
[Provenance](#provenance) for how that boundary is enforced.

The geometry is exact. Booleans run on analytic and NURBS surfaces and keep
those surfaces through the operation, so a cylinder stays a cylinder instead of
becoming a bag of triangles. That keeps face counts low and round-trips
lossless.

Remus's canonical modeling convention is **millimetres for length** and
**radians for angle**. The kernel does not attach units to scalar values or
silently convert them; applications using another length unit must scale all
coordinates, dimensions, deflections, and linear tolerances consistently at
their boundary. See the [tolerance and robustness guide](book/src/tolerances.md).

## Kernel contract

Remus is being driven from a broad-but-maturing kernel toward a
professional-grade one, and the rules for that are written down rather than
implied. This is the part of the repository worth reading before you trust a
feature label.

- **[Kernel maturity target](docs/kernel-maturity/target.md)** — what
  "professional-grade" means here, the program invariants, and the
  program-wide definition of done.
- **[Capability matrix](docs/kernel-maturity/capability-matrix.md)** — the
  qualification structure. Every cell of every operation family is Qualified,
  Partial, Unqualified, Unsupported-typed, or Unsupported-untyped. It is the
  promotion authority for the feature labels in [Status](#status): no feature
  is promoted on a single successful fixture.
- **[Operation contract](docs/kernel-maturity/operation-contract.md)** — the
  result, quality, fallback, and postcondition contract every operation
  converges on.
- **[Failure taxonomy](docs/kernel-maturity/failure-taxonomy.md)** — stable
  failure categories, and how they map onto the error-code registry.
- **[Testing strategy](docs/kernel-maturity/testing-strategy.md)** — what kind
  of evidence qualifies a capability cell, and what CI gates.
- **[Stability matrix](docs/production-readiness/stability-matrix.md)** — the
  audited disposition of each label shipping *today*, including the rows whose
  advertised domain is not yet fully evidenced.
- **[Stabilization plan](docs/kernel-maturity/stabilization-plan.md)** — the
  working plan for promoting every Beta/Experimental row below to Stable,
  sequenced under the capability-matrix promotion rules.

Four mechanisms carry that contract in code:

| Mechanism | Where | What it gives you |
| --- | --- | --- |
| **Operation context** | `remus_math::context::OperationContext` ([RFC 0001](docs/design/rfc-0001-operation-context.md)) | Tolerances, hard work budgets, fallback policy, and cooperative cancellation as explicit caller-visible policy. Defaults reproduce prior behavior exactly; cancellation is typed and transactional. |
| **Structured diagnostics** | `remus_math::diagnostic` | Every failure carries a stable category plus a stable code, independent of the Rust error type. Codes are explicit literals, never derived from type or variant names, and the registry is additive only. |
| **Coedges and per-use p-curves** | `remus_topology` ([RFC 0002](docs/design/rfc-0002-coedge-architecture.md)) | First-class edge *uses*, so seams, poles, and periodic surfaces are represented correctly. Seam p-curve access is fail-closed rather than silently picking one side. |
| **Reproduction bundles** | `remus_wasm::repro` | Versioned JSON that replays an operation sequence and its expected results through the batch dispatch path — identically on native and WASM. Bundles are the canonical carrier for new regressions; expected *failures* are first-class. |

Structural work lands through versioned RFCs in [`docs/design`](docs/design)
and incremental vertical slices, not repository-wide rewrites. Tests are not
weakened, tolerances are not widened, and a mesh fallback is not introduced to
make a failing case pass.

## Status

Remus is in active development. Core modeling is solid. Each feature below is
marked stable, beta, planned, or experimental;
[Known Limitations](#known-limitations) covers the gaps, and the
[stability matrix](docs/production-readiness/stability-matrix.md) records what
evidence each label currently rests on.

| Category                | Feature                                                                      | Status       |
| ----------------------- | ---------------------------------------------------------------------------- | ------------ |
| **Primitives**          | Box, cylinder, cone, sphere, torus, ellipsoid                                | Stable       |
| **Primitives**          | Convex hull, Minkowski sum (convex inputs)                                   | Stable       |
| **Booleans**            | Union, cut, intersect on plane, cylinder, cone, sphere, NURBS                | Stable       |
| **Booleans**            | Batch fuse-all (disjoint-aware union)                                        | Stable       |
| **Booleans**            | Torus booleans (box ± torus, coaxial torus)                                  | Beta         |
| **Modifiers**           | Validated planar fillet/chamfer and axisymmetric closed-rim fillet; other curved blend geometry (experimental assembly) | Stable / Experimental |
| **Modifiers**           | Resize or remove an analytic blend band (`resize_blend`)                     | Experimental |
| **Modifiers**           | Shell (hollow solid)                                                         | Stable       |
| **Modifiers**           | Offset face, offset solid, thicken, mirror, pattern                          | Stable       |
| **Modifiers**           | Draft (planar faces)                                                         | Stable       |
| **Sweeps**              | Extrude (planar + NURBS profiles)                                            | Stable       |
| **Sweeps**              | Revolve, sweep, loft, pipe (planar profiles)                                 | Stable       |
| **Sweeps**              | Helical sweep                                                                | Stable       |
| **Sweeps**              | Non-planar profiles for loft, sweep, pipe, revolve                           | Stable       |
| **Construction**        | Coons-patch face fill, sew, untrim                                           | Stable       |
| **Sectioning**          | Cross-section faces, split by plane                                          | Stable       |
| **Measurement**         | Bounding box, area, volume, center of mass, inertia tensor + principal axes  | Stable       |
| **Measurement**         | Point-to-solid, solid-to-solid distance, point classification                | Stable       |
| **Drawing**             | Hidden-line edge projection                                                  | Stable       |
| **Geometry**            | NURBS evaluation, derivatives, knot ops, fitting, projection                 | Stable       |
| **Geometry**            | Analytic intersections (plane × cylinder, cone, sphere exact; torus sampled) | Stable       |
| **Geometry**            | Surface-surface intersection (analytic + marching)                           | Stable       |
| **Geometry**            | Curve-curve intersection (Bezier clipping)                                   | Stable       |
| **Tessellation**        | Adaptive deflection, CDT, analytic-surface optimization                      | Stable       |
| **Repair**              | Shape healing (wire, face, shell fixes), sewing, validation                  | Stable       |
| **I/O**                 | STEP import/export (analytic-preserving round-trip)                          | Stable       |
| **I/O**                 | STL, 3MF, OBJ, PLY, glTF (`.glb`) import/export                              | Stable       |
| **I/O**                 | IGES import/export                                                           | Experimental |
| **Sketching**           | 2D constraint solver (DogLeg)                                                | Stable       |
| **Feature Recognition** | Holes, pockets, chamfers, fillets                                            | Stable       |
| **Assemblies**          | Hierarchy, transforms, bill of materials                                     | Stable       |
| **Evolution**           | Face provenance (booleans, blends, patterns, draft, defeature, split, shell) | Stable       |
| **Defeaturing**         | Remove planar faces                                                          | Stable       |
| **Rendering**           | Offscreen wgpu render to image plus face-id buffer (`remus-render`)        | Experimental |

## Known Limitations

A few areas are still maturing. Worth knowing before you build on them:

- **Boolean fallback.** Most booleans run on an exact path that preserves analytic and NURBS surfaces. Hard configurations may use a bounded mesh-based fallback, which tessellates curved faces. If its input/work budgets are exceeded or the welded result is open, non-manifold, or invalid, the operation returns an error instead of a partial solid. Exact tangency and sliver crossings are the two contact configurations that still fall over to that path rather than being answered analytically.
- **Walking fillet/chamfer and offset.** The v2 modifier APIs validate completed topology and reject partial results. Unsupported/no-op trimming and offsetting a solid that already contains cavity shells return explicit errors; they do not silently drop faces or cavities. Radii the rolling ball cannot fit are refused as typed errors naming the edge and the limit, not delivered as a partial result.
- **Torus booleans.** Box-with-torus, coaxial-torus, plane-through-centre, and coaxial-cylinder cases give correct volumes, and coaxial torus×cylinder / axis-centred torus×sphere sections are exact circles. Carving a closed torus face into tube bands is not implemented yet, so those configurations resolve through the bounded mesh fallback (torus×sphere fuse currently refuses on its work budget); general torus-to-torus intersections have known gaps.
- **Non-planar profiles.** Loft, sweep, and pipe close non-planar section boundaries with bilinear (4-sided) or Coons (5-or-more-sided) caps whose boundary iso-curves are exactly the ring chords; holes on a non-planar section remain a typed refusal. Revolve accepts non-planar profile surfaces; a full revolution takes any boundary, and a partial revolution closes non-planar polygonal boundaries with the same caps (curved-edge non-planar boundaries and holes stay typed refusals). Only the miter-corner sweep variant still requires planar profiles (its bisector-plane joint faces would otherwise be non-planar).
- **Evolution coverage.** Face provenance is exact and construction-derived for booleans, the walking and planar blend builders, patterns, draft, defeature, plane split, and shell. Offset and direct edits still journal as explicit barriers, and edge/vertex provenance beyond the boolean path is roadmap work.
- **IGES is experimental.** Export writes planar and NURBS surfaces but skips analytic surfaces and approximates circular and elliptical edges as polylines. Import reconstructs planar placeholder faces only. Use STEP for B-Rep exchange.
- **Declared domains.** Feature recognition claims only its declared feature set (holes, rectangular pockets, chamfers, curved fillet bands) — outside it, absence of a claim is the contract. Defeaturing removes features whose wound lies on planar kept faces (the removed feature itself may be curved); draft targets planar faces. Each refuses outside its domain by name.

The versioned WASM fillet/chamfer provenance payload and its strict decoder are
documented in [WASM face evolution](docs/wasm-face-evolution.md).

## Scope

Remus deliberately does not:

- **Bundle a viewport into the kernel.** The core emits exact geometry and tessellated meshes; camera, lighting, and shading belong to the caller (Three.js and the like). The optional `remus-render` crate provides offscreen wgpu rendering with a face-id buffer, for tests and headless verification, and is not required by any core operation.
- **Plan toolpaths or slice.** Export STEP, STL, or 3MF and pass the output to a CAM tool or slicer.
- **Model with meshes.** The kernel operates on exact B-Rep geometry. Subdivision surfaces, polygon meshes, and voxels are out of scope.
- **Provide a GUI.** Remus is a library. Building a UI around it is the application's job.
- **Simulate physics.** Measurement (volume, area, center of mass) is included. Stress analysis, collision detection, and dynamics are not.

## Architecture

Layered Cargo workspace. Each crate depends only on the same or lower layers,
and CI enforces the boundaries with `scripts/check-boundaries.sh`.

| Layer | Crate                | What it does                                                                                        |
| ----- | -------------------- | --------------------------------------------------------------------------------------------------- |
| L0    | `remus-math`       | Points, vectors, matrices, NURBS curves and surfaces, geometric predicates, CDT, convex hull, operation context, diagnostics |
| L1    | `remus-geometry`   | Curve sampling (uniform, deflection, arc-length, curvature), extrema, analytic-to-NURBS conversion  |
| L1    | `remus-topology`   | Arena-allocated B-Rep: vertex, edge, coedge, loop, wire, face, shell, solid, with an edge-to-face adjacency index |
| L2    | `remus-algo`       | General Fuse boolean engine: pave filler, face classification, solid assembly                       |
| L2    | `remus-blend`      | Walking-based fillet and chamfer with constant, variable, and custom radius laws                    |
| L2    | `remus-heal`       | Shape healing: analysis, fixing, upgrading, sewing, tolerance management, configurable pipeline     |
| L2    | `remus-check`      | Point classification, validation, properties (volume, area, center of mass), distance               |
| L2    | `remus-offset`     | Solid offset and thickening via global face-face intersection                                       |
| L2    | `remus-sketch`     | 2D parametric constraint solver (GCS) using a DogLeg trust-region method                            |
| L3    | `remus-operations` | Booleans, fillet, chamfer, extrude, revolve, sweep, loft, shell, offset, measure, tessellation      |
| L3    | `remus-io`         | Import and export: STEP, IGES, STL, 3MF, OBJ, PLY, glTF                                             |
| L4    | `remus-wasm`       | JavaScript API via wasm-bindgen, with batch execution, checkpoint/restore, and reproduction bundles |
| L4    | `remus-render`     | Offscreen wgpu rendering to a color image plus a face-id buffer. Optional, nothing depends on it    |
| L5    | `remus`            | Native Rust facade: owned model session, explicit operation policy, curated modeling and I/O API  |

The layer DAG is a program invariant: preserving it is a constraint on every
change, and a violation fails both the pre-push hook and CI.

## Performance

Median times from the [brepjs benchmark suite](https://github.com/andymai/brepjs/tree/main/benchmarks)
(5 iterations, Node.js, Linux x86_64). WASM is single-threaded. Native
benchmarks use criterion.

| Operation                | Remus (WASM)   | OCCT (WASM) | Speedup | Remus (native)   |
| ------------------------ | -------------- | ----------- | ------- | ---------------- |
| fuse(box, box) (×10)     | 0.5 ms         | 43.7 ms     | 87x     | 122 µs           |
| cut(box, cylinder) (×10) | 28.3 ms        | 64.3 ms     | 2.3x    | 9.3 ms           |
| box + chamfer            | 0.2 ms         | 5.4 ms      | 27x     | 46 µs            |
| box + fillet             | 0.3 ms         | 6.2 ms      | 21x     | 127 µs           |
| multi-boolean (16 holes) | 4.7 ms         | 30.1 ms     | 6.4x    | 2.8 ms           |
| mesh sphere (tol=0.01)   | 7.1 ms         | 51.9 ms     | 7.3x    | 6.0 ms           |
| exportSTEP (×10)         | 0.9 ms         | 14.3 ms     | 16x     | n/a              |

Every quoted row is output-verified across both kernels before timing is
compared: fuse, chamfer, and sphere volumes match exactly; cut, fillet, and
multi-boolean volumes agree within 0.004%. The sphere mesh densities are
comparable at equal tolerance (9,800 triangles vs 10,176). The
`intersect(box, sphere)` row is excluded: the kernel currently keeps the wrong
sphere region for that configuration (an open, pinned defect), so its ~200x
timing would not be a like-for-like comparison.

Booleans preserve analytic surfaces, so face counts stay low across chained
operations. A nine-step compound boolean settles at 72 faces while a mesh-based
approach would reach roughly 7,000. The same holds for blends: a straight edge
filleted between two planar faces keeps an exact cylindrical wall rather than a
NURBS approximation of one.

> The OCCT comparison uses [occt-wasm](https://www.npmjs.com/package/occt-wasm), an OpenCASCADE build compiled to WebAssembly. Both kernels ran single-threaded in Node.js. Boolean and `exportSTEP` rows were timed as batches of ten operations. WASM figures are medians of `kernel-comparison.bench.test.ts` (5 iterations) against a local `cargo xtask wasm-build` package, hash-verified at the require path. Native figures came from `cargo bench -p remus-operations --bench cad_operations`, except the mesh-sphere row, which used `crates/operations/examples/perf_probe.rs` at matching parameters. Measured 2026-08-06, before the Apache-only line was established; the upstream head-to-head harness is retired, so treat these figures as historical and do not quote them as current. Run `scripts/bench-compare.sh` for the maintained native Criterion baselines.

## Data Exchange

| Format        | Type  | Import  | Export |
| ------------- | ----- | ------- | ------ |
| STEP          | B-Rep | ✓       | ✓      |
| STL           | Mesh  | ✓       | ✓      |
| 3MF           | Mesh  | ✓       | ✓      |
| OBJ           | Mesh  | ✓       | ✓      |
| PLY           | Mesh  | ✓       | ✓      |
| glTF (`.glb`) | Mesh  | ✓       | ✓      |
| IGES          | B-Rep | preview | lossy  |

STEP preserves exact geometry on round-trip. Analytic surfaces (plane,
cylinder, cone, sphere, torus) are written as native STEP surface entities
rather than tessellated, and they read back to the same surface types. NURBS
surfaces are preserved too, as are line, circle, ellipse, and NURBS edges.

Mesh formats export tessellated triangles. glTF is binary `.glb`, with no
materials or scene graph. IGES is experimental, as described in
[Known Limitations](#known-limitations).

All Rust importer entry points apply production defaults through
`ImportLimits`: 256 MiB encoded input, 256 MiB for the uncompressed 3MF model
XML entry, and 3,000,000 format-specific model entities. Use each format's
`*_with_limits` reader to choose stricter or application-specific budgets; the
WASM importers accept optional `maxInputBytes` / `maxEntities` arguments for
the same purpose. Limit violations return `IoError::LimitExceeded` before
avoidable large allocations. The WASM batch API separately limits JSON to
16 MiB and 10,000 operations.

## Getting Started

### Packages

**Remus publishes nothing yet.** No crates.io releases, no npm packages, no
GitHub releases. Release ownership — named maintainers, package identity,
vulnerability intake, signing and provenance, rollback and yank authority —
has to be established first; the gate is documented in
[fork maintenance and release policy](docs/production-readiness/fork-maintenance.md).

Two consequences worth stating plainly:

- A `remus-wasm` package on npm does **not** come from this repository. It
  belongs to the historical upstream line, which is no longer permissively
  licensed. Installing it does not get you this kernel.
- The checked-in `crates/wasm/pkg` (kernel) and `crates/wasm-io/pkg`
  (file-format translators) directories are frozen compatibility snapshots
  for an existing consumer that installs them by git path, pinned to one
  commit. They are not a release channel and not the way to adopt Remus.

Until packages exist, build from source.

### As a Rust dependency

```toml
[dependencies]
remus = { git = "https://github.com/esaueng/remus" }
```

Pin a revision (`rev = "..."`) for anything you intend to reproduce: nothing
is versioned or published yet, so `main` moves.

### Building from source

MSRV is Rust 1.88, and CI holds that floor. Day-to-day development uses the
toolchain pinned in `rust-toolchain.toml`, which rustup picks up automatically
along with the `wasm32-unknown-unknown` target.

```bash
cargo build --workspace
cargo test --workspace
cargo clippy --all-targets -- -D warnings
cargo fmt --all

# WASM packages (kernel + file-format translators): dual-target build,
# merge, and validation
cargo xtask wasm-build

# Plain WASM builds: the kernel as shipped (no translators), the single-module
# kernel with translators bundled, and the translator module
cargo build -p remus-wasm --target wasm32-unknown-unknown --release --no-default-features
cargo build -p remus-wasm --target wasm32-unknown-unknown --release
cargo build -p remus-wasm-io --target wasm32-unknown-unknown --release

# API docs
cargo doc --workspace --no-deps --open
```

Repository invariants have their own checks, all of which CI runs:

```bash
./scripts/check-boundaries.sh              # layer dependency DAG
./scripts/check-doc-paths.sh               # documented file paths still resolve
./scripts/check-apache-lineage.sh          # no prohibited upstream lineage
python3 scripts/check-apache-replay-provenance.py   # provenance ledger integrity
```

### Documentation

| Where | What |
| --- | --- |
| [`book/`](book/src) | Task-oriented guide: getting started, concepts, tolerances, data exchange, WASM, rendering, troubleshooting |
| [`docs/kernel-maturity/`](docs/kernel-maturity) | The maturity contract: target, capability matrix, operation contract, failure taxonomy, testing strategy |
| [`docs/design/`](docs/design) | RFCs and design research, including operation context (0001) and coedge architecture (0002) |
| [`docs/production-readiness/`](docs/production-readiness) | Audit, stability matrix, coverage, release checklist, fork maintenance, Apache replay provenance |
| [`AGENTS.md`](AGENTS.md) | Working guide: module map, ripple-effect checklists, common pitfalls |
| [`CHANGELOG.md`](CHANGELOG.md) | Full history, including the pre-fork series |

Maintainers should use the
[production-readiness audit](docs/production-readiness/audit.md),
[stability matrix](docs/production-readiness/stability-matrix.md), and
[release checklist](docs/production-readiness/release-checklist.md) before
cutting an artifact. The checklist is validation guidance and does not grant
authority to publish.

## Roadmap

Priorities, not dates. Planning is by dependency and acceptance gate; see the
[kernel maturity target](docs/kernel-maturity/target.md) for the full program.

**P0 — foundations and correctness.** Capability and failure contracts across
every operation family; the reproduction and regression corpus; first-class
coedges and explicit curve/p-curve trimming; unified tolerance and operation
context; intersection robustness; General Fuse and boolean robustness
(shrinking the set of inputs that fall back to meshing, starting with torus and
mixed-surface cases); transactional topology mutation; kernel-wide diagnostics.

**P1 — professional modeling behavior.** Complete vertex, edge, and face
evolution with persistent topological naming; general blends, offsets,
shelling, sweeps, and lofts — including the miter-corner sweep, boundaries with
more than four edges, and partial revolutions with non-planar boundaries;
direct face editing; attribute propagation; broad STEP round-trip behavior with
topology attributes; memory compaction and session lifecycle.

**P2 — extended scope.** General and non-manifold bodies; mixed B-rep and facet
modeling; cellular topology; lattice representation; concurrent operations and
large-model scaling — including parallel tessellation on the WASM target, which
native builds already do per face.

## Contributing

See [CONTRIBUTING.md](./CONTRIBUTING.md). Contributions are inbound under
Apache-2.0 and require a Developer Certificate of Origin sign-off. Commits are
conventional commits, enforced by commitlint; the pre-commit hook runs
`cargo fmt` and clippy, and CI gates the full test suite, the layer-boundary
check, and the license-lineage check on every push.

New regressions should land as [reproduction bundles](crates/wasm/src/repro.rs)
where the failure is expressible through the batch API — every discovered
defect is meant to become a permanent, replayable regression.

Security reports: see [SECURITY.md](./SECURITY.md).

## Provenance

Remus continues a codebase whose upstream relicensed to AGPL at v3. This
repository is the permanent Apache-2.0 line of that work, maintained by
Esau Engineering. The last permissive upstream release is `v2.129.15`;
nothing from v3 or later is merged, and behavior from those releases enters
only under an explicit Apache-2.0 grant or as an independent implementation
proven by a regression test.

That boundary is enforced in CI and every replayed contribution is recorded
in an auditable ledger — see
[Apache contribution provenance](docs/production-readiness/apache-replay-provenance.md).

The project's use of AI tooling is disclosed in
[AI-DISCLOSURE.md](./AI-DISCLOSURE.md).

## License

Remus is licensed under the [Apache License, Version 2.0](./LICENSE-APACHE),
permanently — see [Provenance](#provenance) for how the AGPL boundary with
the historical upstream is enforced. Attribution is in [NOTICE](./NOTICE),
and contributions come in under the same license
(see [CONTRIBUTING.md](./CONTRIBUTING.md)).
</content>

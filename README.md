<div align="center">

# Remus

Solid modeling kernel for Rust and WebAssembly.

[![CI](https://github.com/esaueng/remus/actions/workflows/ci.yml/badge.svg)](https://github.com/esaueng/remus/actions/workflows/ci.yml)
[![Commit activity](https://img.shields.io/github/commit-activity/m/esaueng/remus?label=commits%2Fmonth)](https://github.com/esaueng/remus/commits/main)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](#license)
[![Rust 1.88+](https://img.shields.io/badge/rust-1.88%2B-orange.svg)](https://www.rust-lang.org/) [![unsafe denied](https://img.shields.io/badge/unsafe-denied-success.svg)](#why-a-cad-kernel)

**[Architecture](#architecture)** · **[Performance](#performance)** · **[Getting Started](#getting-started)** · **[Known Limitations](#known-limitations)** · **[Contributing](./CONTRIBUTING.md)**

</div>

One exact-geometry engine, from Rust and from JavaScript. Cut a solid, measure it, export it.

```rust
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::boolean::{boolean, BooleanOp};
use remus_operations::measure::solid_volume;
use remus_io::step::write_step;
use remus_topology::Topology;

let mut topo = Topology::new();

// Primitives are anchored at the origin, so this cylinder rounds off the
// block's corner. Use `transform_solid` to place it somewhere else.
let block = make_box(&mut topo, 30.0, 20.0, 10.0)?;
let cutter = make_cylinder(&mut topo, 5.0, 15.0)?;
let notched = boolean(&mut topo, BooleanOp::Cut, block, cutter)?;

// Measure and export
let vol = solid_volume(&topo, notched, 0.1)?;
let step = write_step(&topo, &[notched])?;
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

Remus is a B-Rep solid modeling kernel written in Rust. It targets WebAssembly, so the same kernel runs in the browser and on the desktop. `unsafe` is denied by lint, as are `unwrap` and `panic`. Every public operation returns a `Result`. Its audited Apache-only source history is documented in [PROVENANCE.md](docs/PROVENANCE.md).

It grew out of building [gridfinitylayouttool.com](https://gridfinitylayouttool.com), where the options for parametric CAD in the browser were proprietary or compiled from large C++ codebases.

The geometry is exact. Booleans run on analytic and NURBS surfaces and keep those surfaces through the operation, so a cylinder stays a cylinder instead of becoming a bag of triangles. That keeps face counts low and round-trips lossless.

Remus's canonical modeling convention is **millimetres for length** and
**radians for angle**. The kernel does not attach units to scalar values or
silently convert them; applications using another length unit must scale all
coordinates, dimensions, deflections, and linear tolerances consistently at
their boundary. See the [tolerance and robustness guide](book/src/tolerances.md).

## Status

Remus is in active development. Core modeling is solid. Each feature below is marked stable, beta, planned, or experimental, and [Known Limitations](#known-limitations) covers the gaps.

| Category                | Feature                                                                      | Status       |
| ----------------------- | ---------------------------------------------------------------------------- | ------------ |
| **Primitives**          | Box, cylinder, cone, sphere, torus, ellipsoid                                | Stable       |
| **Primitives**          | Convex hull, Minkowski sum (convex inputs)                                   | Stable       |
| **Booleans**            | Union, cut, intersect on plane, cylinder, cone, sphere, NURBS                | Stable       |
| **Booleans**            | Batch fuse-all (disjoint-aware union)                                        | Stable       |
| **Booleans**            | Torus booleans (box ± torus, coaxial torus)                                  | Beta         |
| **Modifiers**           | Validated planar fillet/chamfer and axisymmetric closed-rim fillet; other curved blend geometry (experimental assembly) | Stable / Experimental |
| **Modifiers**           | Shell (hollow solid)                                                         | Stable       |
| **Modifiers**           | Offset face, offset solid, thicken, mirror, pattern                          | Stable       |
| **Modifiers**           | Draft (planar faces)                                                         | Beta         |
| **Sweeps**              | Extrude (planar + NURBS profiles)                                            | Stable       |
| **Sweeps**              | Revolve, sweep, loft, pipe (planar profiles)                                 | Stable       |
| **Sweeps**              | Helical sweep                                                                | Stable       |
| **Sweeps**              | Non-planar profiles for loft, sweep, pipe, revolve                           | Beta         |
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
| **Rendering**           | wgpu offscreen rendering and face-ID picking                                 | Beta         |
| **Repair**              | Shape healing (wire, face, shell fixes), sewing, validation                  | Stable       |
| **I/O**                 | STEP import/export (analytic-preserving round-trip)                          | Stable       |
| **I/O**                 | STL, 3MF, OBJ, PLY, glTF (`.glb`) import/export                              | Stable       |
| **I/O**                 | IGES import/export                                                           | Experimental |
| **Sketching**           | 2D constraint solver (DogLeg)                                                | Stable       |
| **Feature Recognition** | Holes, pockets, chamfers, fillets                                            | Beta         |
| **Assemblies**          | Hierarchy, transforms, bill of materials                                     | Beta         |
| **Evolution**           | Face provenance through booleans                                             | Beta         |
| **Defeaturing**         | Remove planar faces                                                          | Beta         |
| **Rendering**           | Offscreen wgpu render to image plus face-id buffer (`remus-render`)        | Experimental |

## Known Limitations

A few areas are still maturing. Worth knowing before you build on them:

- **Boolean fallback.** Most booleans run on an exact path that preserves analytic and NURBS surfaces. Hard configurations may use a bounded mesh-based fallback, which tessellates curved faces. If its input/work budgets are exceeded or the welded result is open, non-manifold, or invalid, the operation returns an error instead of a partial solid.
- **Walking fillet/chamfer and offset.** The v2 modifier APIs validate completed topology and reject partial results. Unsupported/no-op trimming and offsetting a solid that already contains cavity shells return explicit errors; they do not silently drop faces or cavities.
- **Torus booleans.** Box-with-torus and coaxial-torus cases work and give correct volumes. General torus-to-torus and torus-with-other-surface intersections have known gaps and may fall back to meshing.
- **Non-planar profiles.** Loft, sweep, and pipe accept profiles with non-planar surfaces, and close non-planar section boundaries with bilinear caps for four-sided rings (boundaries with more than four edges, or holes on a non-planar section, are not yet supported). Revolve accepts non-planar profile surfaces; a full revolution takes any boundary, but a partial revolution still requires a planar boundary for its caps. The smooth, scaled/guided, and multi-section sweep variants accept non-planar profiles too; only the miter-corner variant still requires planar profiles (its bisector-plane joint faces would otherwise be non-planar).
- **IGES is experimental.** Export writes planar and NURBS surfaces but skips analytic surfaces and approximates circular and elliptical edges as polylines. Import reconstructs planar placeholder faces only. Use STEP for B-Rep exchange.
- **Beta subsystems.** Feature recognition, assemblies, evolution tracking, and defeaturing work but are still maturing. Defeaturing handles planar faces only.

The versioned WASM fillet/chamfer provenance payload and its strict decoder are
documented in [WASM face evolution](docs/wasm-face-evolution.md).

## Scope

Remus deliberately does not:

- **Bundle a viewport into the kernel.** The core emits exact geometry and tessellated meshes; camera, lighting, and shading belong to the caller (Three.js and the like). The optional `remus-render` crate provides offscreen wgpu rendering with a face-id buffer, for tests and headless verification, and is not required by any core operation.
- **Plan toolpaths or slice.** Export STEP, STL, or 3MF and pass the output to a CAM tool or slicer.
- **Model with meshes.** The kernel operates on exact B-Rep geometry. Subdivision surfaces, polygon meshes, and voxels are out of scope.
- **Provide a GUI.** Remus is a library. Building a UI around it, like [gridfinitylayouttool.com](https://gridfinitylayouttool.com), is the application's job.
- **Simulate physics.** Measurement (volume, area, center of mass) is included. Stress analysis, collision detection, and dynamics are not.

## Architecture

Layered Cargo workspace. Each crate depends only on the same or lower layers, and CI enforces the boundaries.

| Layer | Crate                | What it does                                                                                        |
| ----- | -------------------- | --------------------------------------------------------------------------------------------------- |
| L0    | `remus-math`       | Points, vectors, matrices, NURBS curves and surfaces, geometric predicates, CDT, convex hull        |
| L1    | `remus-geometry`   | Curve sampling (uniform, deflection, arc-length, curvature), extrema, analytic-to-NURBS conversion  |
| L1    | `remus-topology`   | Arena-allocated B-Rep: vertex, edge, wire, face, shell, solid, with an edge-to-face adjacency index |
| L2    | `remus-algo`       | General Fuse boolean engine: pave filler, face classification, solid assembly                       |
| L2    | `remus-blend`      | Walking-based fillet and chamfer with constant, variable, and custom radius laws                    |
| L2    | `remus-heal`       | Shape healing: analysis, fixing, upgrading, sewing, tolerance management, configurable pipeline     |
| L2    | `remus-check`      | Point classification, validation, properties (volume, area, center of mass), distance               |
| L2    | `remus-offset`     | Solid offset and thickening via global face-face intersection                                       |
| L2    | `remus-sketch`     | 2D parametric constraint solver (GCS) using a DogLeg trust-region method                            |
| L3    | `remus-operations` | Booleans, fillet, chamfer, extrude, revolve, sweep, loft, shell, offset, measure, tessellation      |
| L3    | `remus-io`         | Import and export: STEP, IGES, STL, 3MF, OBJ, PLY, glTF                                             |
| L4    | `remus-wasm`       | JavaScript API via wasm-bindgen, with batch execution and checkpoint/restore                        |
| L4    | `remus-render`     | Offscreen wgpu rendering to a color image plus a face-id buffer. Optional, nothing depends on it    |

## Performance

Median times from the [brepjs benchmark suite](https://github.com/andymai/brepjs/tree/main/benchmarks) (5 iterations, Node.js, Linux x86_64). WASM is single-threaded. Native benchmarks use criterion.

| Operation                | remus (WASM) | OCCT (WASM) | Speedup | remus (native) |
| ------------------------ | -------------- | ----------- | ------- | ---------------- |
| fuse(box, box) (×10)     | 0.5 ms         | 43.7 ms     | 87x     | 122 µs           |
| cut(box, cylinder) (×10) | 28.3 ms        | 64.3 ms     | 2.3x    | 9.3 ms           |
| box + chamfer            | 0.2 ms         | 5.4 ms      | 27x     | 46 µs            |
| box + fillet             | 0.3 ms         | 6.2 ms      | 21x     | 127 µs           |
| multi-boolean (16 holes) | 4.7 ms         | 30.1 ms     | 6.4x    | 2.8 ms           |
| mesh sphere (tol=0.01)   | 7.1 ms         | 51.9 ms     | 7.3x    | 6.0 ms           |
| exportSTEP (×10)         | 0.9 ms         | 14.3 ms     | 16x     | n/a              |

Every quoted row is output-verified across both kernels before timing is compared: fuse, chamfer, and sphere volumes match exactly; cut, fillet, and multi-boolean volumes agree within 0.004%. The sphere mesh densities are comparable at equal tolerance (9,800 triangles vs 10,176). The `intersect(box, sphere)` row is excluded: remus currently keeps the wrong sphere region for that configuration (an open, pinned defect), so its ~200x timing would not be a like-for-like comparison.

Booleans preserve analytic surfaces, so face counts stay low across chained operations. A nine-step compound boolean settles at 72 faces while a mesh-based approach would reach roughly 7,000. The same holds for blends: a straight edge filleted between two planar faces keeps an exact cylindrical wall rather than a NURBS approximation of one.

> The OCCT comparison uses [occt-wasm](https://www.npmjs.com/package/occt-wasm), an OpenCASCADE build compiled to WebAssembly. Both kernels run single-threaded in Node.js. Boolean and `exportSTEP` rows are timed as batches of ten operations. WASM figures are medians of `kernel-comparison.bench.test.ts` (5 iterations) against a local `cargo xtask wasm-build` package, hash-verified at the require path. Native figures: `cargo bench -p remus-operations --bench cad_operations`, except the mesh-sphere row, which is measured at the same parameters as the WASM row (`tessellate_solid_with_tolerance`, deflection 0.01, angular 0.1 rad) via `crates/operations/examples/perf_probe.rs` — the criterion suite's sphere case meshes per-face and is not comparable. Full benchmark source: [brepjs/benchmarks](https://github.com/andymai/brepjs/tree/main/benchmarks). Measured 2026-08-06 on the permissive source lineage now carried by Remus, with the display-sphere tessellation fix.

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

STEP preserves exact geometry on round-trip. Analytic surfaces (plane, cylinder, cone, sphere, torus) are written as native STEP surface entities rather than tessellated, and they read back to the same surface types. NURBS surfaces are preserved too, as are line, circle, ellipse, and NURBS edges.

Mesh formats export tessellated triangles. glTF is binary `.glb`, with no materials or scene graph. IGES is experimental, as described in [Known Limitations](#known-limitations).

All Rust importer entry points apply production defaults through
`ImportLimits`: 128 MiB encoded input, 256 MiB for the uncompressed 3MF model
XML entry, and 2,000,000 format-specific model entities. Use each format's
`*_with_limits` reader to choose stricter or application-specific budgets; the
WASM importers accept optional `maxInputBytes` / `maxEntities` arguments for
the same purpose. Limit violations return `IoError::LimitExceeded` before
avoidable large allocations. The WASM batch API separately limits JSON to
16 MiB and 10,000 operations.

## Getting Started

### As a WASM package

`remus-wasm` is not yet published to npm. Pin a reviewed Remus commit and use
the committed package directory:

```bash
pnpm add 'remus-wasm@github:esaueng/remus#<commit>&path:/crates/wasm/pkg'
```

```js
import { BrepKernel } from 'remus-wasm';

const kernel = new BrepKernel();
const solid = kernel.makeBox(10, 20, 30);
```

For a higher-level TypeScript API, see [brepjs](https://github.com/andymai/brepjs).

### As a Rust dependency

Not yet published to crates.io. Use git dependencies for now:

```toml
[dependencies]
remus-math = { git = "https://github.com/esaueng/remus" }
remus-topology = { git = "https://github.com/esaueng/remus" }
remus-operations = { git = "https://github.com/esaueng/remus" }
remus-io = { git = "https://github.com/esaueng/remus" }        # optional
```

### Building from source

Requires Rust 1.88 or newer.

```bash
cargo build --workspace
cargo test --workspace
cargo clippy --all-targets -- -D warnings
cargo fmt --all

# WASM package, including the Node and bundler entry points
cargo xtask wasm-build --skip-opt

# Validate the package exactly as an installed consumer sees it
node scripts/test-wasm-tarball-consumer.mjs

# API docs
cargo doc --workspace --no-deps --open
```

### Distribution and self-hosting

The generated package is committed under `crates/wasm/pkg`. Applications can
pin that directory through Git, build it locally with the xtask above, or pack
it into an installable tarball:

```bash
cd crates/wasm/pkg
npm pack
```

The resulting tarball can be stored in your own artifact registry or installed
directly. Pin a commit or content digest in production; do not depend on a
moving branch. No public npm or crates.io release should be inferred from the
committed package.

Maintainers should use the
[production-readiness audit](docs/production-readiness/audit.md),
[stability matrix](docs/production-readiness/stability-matrix.md), and
[release checklist](docs/production-readiness/release-checklist.md) before
cutting an artifact. The checklist is validation guidance and does not grant
authority to publish.

## Roadmap

Broad directions, no dates.

- **Boolean robustness.** Harden torus and mixed-surface booleans, and shrink the set of inputs that fall back to meshing.
- **Boundary-aware cylindrical resizing.** Resize partial cylindrical walls while rebuilding adjoining face boundaries, preserving the axis, axial extent, and exact analytic geometry. See [scope and acceptance criteria](docs/roadmap/partial-cylinder-resize.md).
- **Sweep generalization.** Extend non-planar profile support to the miter-corner sweep, to section boundaries with more than four edges, and to partial revolutions with non-planar boundaries.
- **Parallel tessellation in WASM.** Native builds already parallelize per-face meshing. Bring it to the WASM target via threads.
- **Assembly metadata.** Colors, layers, materials, and PMI for richer data exchange.
- **Lossless IGES.** Real B-Rep import and analytic-surface export.
- **Documentation.** Expand task-oriented tutorials and advanced algorithm guides.

## Projects Using Remus

- [brepjs](https://github.com/andymai/brepjs), CAD modeling for JavaScript.
- [Gridfinity Layout Tool](https://github.com/andymai/gridfinity-layout-tool), a web-based Gridfinity storage layout generator.

[Open a PR](https://github.com/esaueng/remus/pulls) to add your project.

## License

Licensed under the [Apache License, Version 2.0](./LICENSE-APACHE).

This Apache line is permanently based on the last permissive upstream series.
It does not merge code from upstream releases published under the AGPL.

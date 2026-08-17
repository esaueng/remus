# Remus

An exact B-Rep solid modeling kernel, written from scratch in Rust and
compiled to WebAssembly. It handles NURBS and analytic geometry, boolean
operations, blends, tessellation, and data exchange.

Remus emits exact geometry and tessellated meshes. It is a library, not an
application: there is no viewport, no GUI, and no CAM. Building those around
it is the caller's job.

## Why Remus?

- **Exact geometry** — booleans run on analytic and NURBS surfaces and keep
  them, so a cylinder stays a cylinder instead of becoming a bag of triangles
- **Pure Rust** — no C/C++ dependencies, no complex build systems
- **WASM-first** — the same kernel runs in the browser and on the desktop
- **Memory-safe** — `unsafe`, `unwrap`, and `panic!` are denied by lint;
  every public operation returns a `Result`
- **Layered architecture** — enforced separation of math, topology,
  operations, and I/O
- **Contract-driven** — feature labels rest on a capability matrix and an
  operation contract, not on individual demonstrations

## Naming

The project and repository are **Remus**. The crates still carry the
`brepkit-` prefix (`brepkit-math`, `brepkit-operations`, …) and the generated
WASM package is still named `brepkit-wasm`; a rename is in flight. Code and
identifiers throughout this book show today's names.

## Where to start

- [Getting Started](./getting-started.md) — build the workspace and the WASM
  package from a checkout
- [Concepts](./concepts.md) — B-Rep, the topology/geometry split, tolerances,
  NURBS
- [Architecture](./architecture.md) — the crate layers and why analytic
  surfaces are special-cased
- [Operation Reference](./operation-reference.md) — what each module provides

Beyond this book, the repository carries the kernel maturity contract in
`docs/kernel-maturity/` (capability matrix, operation contract, failure
taxonomy, testing strategy), design RFCs in `docs/design/`, and the audited
label dispositions in `docs/production-readiness/stability-matrix.md`. Read
the stability matrix before relying on a feature label.

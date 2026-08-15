# Remus — Project Guidelines

Remus is a standalone Apache-2.0 B-Rep modeling kernel for Rust and WebAssembly.
It descends from the final permissively licensed predecessor line plus audited
Apache-compatible contributions; `docs/PROVENANCE.md` is the lineage policy.
The kernel provides geometry, topology, modeling, file I/O, rendering, and
JavaScript bindings. brepjs is one higher-level TypeScript consumer.

## Architecture

Strict layered Cargo workspace. Each layer depends only on layers below it.

```
L4: remus-wasm        → JS bindings (wasm-bindgen)
L4: remus-render      → Offscreen GPU rendering (wgpu) to image + face-id buffer
L3: remus-io          → STEP, 3MF, STL, IGES, OBJ, PLY, glTF import/export
L3: remus-operations  → Booleans, fillets, extrusions, tessellation
L2: remus-algo        → GFA boolean engine, classification, intersection
L2: remus-blend       → Walking-based fillet and chamfer engine
L2: remus-check       → Classification, validation, properties, distance
L2: remus-heal        → Shape healing (analysis, fixing, upgrading)
L2: remus-offset      → Solid offset engine (global face-face intersection)
L2: remus-sketch      → 2D parametric constraint solver (GCS)
L1: remus-topology    → B-Rep data structures (arena-based)
L1: remus-geometry    → Curve sampling, extrema, geometry conversion
L0: remus-math        → Vectors, matrices, NURBS, predicates
```

### Layer dependency rules

Enforced by `scripts/check-boundaries.sh` — run before pushing:

| Crate | Allowed deps |
|-------|-------------|
| `math` | *(none — no workspace deps)* |
| `geometry` | `math` |
| `topology` | `math` |
| `algo` | `math`, `topology` |
| `blend` | `math`, `topology` |
| `heal` | `math`, `topology`, `geometry` |
| `check` | `math`, `topology`, `geometry` |
| `offset` | `math`, `topology`, `geometry` |
| `sketch` | *(none — no workspace deps)* |
| `operations` | `math`, `topology`, `algo`, `blend`, `heal`, `check`, `geometry`, `offset`, `sketch` |
| `io` | `math`, `topology`, `operations` |
| `render` | `math`, `topology`, `operations` (L4 leaf — the script also rejects any crate that depends on `render`) |
| `wasm` | all crates (`blend` only transitively, via `operations`) |

The script checks `[dependencies]` in each `Cargo.toml`. A violation fails the pre-push hook.

**Allowed `use` paths per crate:**
- `math/src/**` → only `std`, external crates
- `geometry/src/**` → `remus_math::*`
- `topology/src/**` → `remus_math::*`
- `algo/src/**` → `remus_math::*`, `remus_topology::*`
- `blend/src/**` → `remus_math::*`, `remus_topology::*`
- `heal/src/**` → `remus_math::*`, `remus_topology::*`, `remus_geometry::*`
- `check/src/**` → `remus_math::*`, `remus_topology::*`, `remus_geometry::*`
- `offset/src/**` → `remus_math::*`, `remus_topology::*`, `remus_geometry::*`
- `sketch/src/**` → only `std`, external crates
- `operations/src/**` → `remus_math::*`, `remus_topology::*`, `remus_geometry::*`, `remus_algo::*`, `remus_blend::*`, `remus_heal::*`, `remus_check::*`, `remus_offset::*`, `remus_sketch::*`
- `io/src/**` → `remus_math::*`, `remus_topology::*`, `remus_operations::*`
- `render/src/**` → `remus_math::*`, `remus_topology::*`, `remus_operations::*`
- `wasm/src/**` → all `remus_*`

## Build and verify

Run from the repository root. `rust-toolchain.toml` selects the normal
toolchain; CI separately verifies Rust 1.88.

```bash
cargo build --workspace --all-features
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo nextest run --workspace --no-fail-fast
cargo test --workspace --doc
cargo nextest run -p remus-operations --features perf-counters -E 'test(scaling_)'
./scripts/check-boundaries.sh
./scripts/check-apache-lineage.sh
```

Change-specific gates:

```bash
# Documentation
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps --all-features
mdbook build book
./scripts/check-doc-paths.sh

# Manifests, dependencies, or release metadata
rustup run 1.88.0 cargo check --workspace --all-features
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo package --workspace --allow-dirty --no-verify

# WASM API, bindings, tooling, or package contents
cargo test --manifest-path xtask/Cargo.toml
cargo clippy -p remus-wasm --target wasm32-unknown-unknown --no-default-features -- -D warnings
cargo test -p remus-wasm --no-default-features
cargo xtask wasm-build --skip-opt
cd crates/wasm/pkg && npm pack --dry-run

# Rename and provenance-sensitive changes
./scripts/check-remus-rename.sh
python3 scripts/check-apache-replay-provenance.py
```

CI additionally gates coverage, software Vulkan rendering, cargo-deny,
RustSec, cargo-machete, Taplo, secret scanning, fuzz-target compilation, and
Linux, macOS, and Windows tests. The SemVer check and WASM size report are
advisory.

## Invariants

- Lengths are millimetres and angles are radians. Scale coordinates,
  dimensions, deflections, and linear tolerances together at application
  boundaries.
- `Tolerance::new()` is scale-aware: linear `1e-7`, angular `1e-12`, relative
  `1e-10`. Use tolerance helpers; never raw float equality.
- Tessellation success does not prove geometry correctness. Check closed and
  valid topology, volume, analytic surface kinds, and STEP round trips when
  the affected path crosses STEP I/O.
- Solid-wide traversal must include cavity shells. Prefer
  `remus_topology::explorer::solid_faces`; iterate per shell only when the
  operation is explicitly shell-scoped.
- Production code denies unsafe, unwrap, expect, and panic. Keep test-only
  lint allowances narrow.
- `crates/wasm/pkg` is generated and committed. Never hand-edit it; rebuild
  with `cargo xtask wasm-build --skip-opt` and validate the tarball consumer.
- Fuzz checks can refresh `fuzz/Cargo.lock`. Exclude that churn unless the task
  changes dependencies.

## Adding a user-visible operation

1. Implement and export it from `remus-operations`, with native geometry and
   topology regression coverage.
2. Add the matching WASM binding under `crates/wasm/src/bindings/`, using the
   existing validators and typed handle converters.
3. Add batch dispatch and batch contract coverage when the operation belongs
   in `executeBatch`.
4. Run the WASM gates and regenerate the committed package from source.

## Apache lineage

- `main` is permanently Apache-2.0-only. Keep `LICENSE-APACHE`, `NOTICE`, and
  the machine-checked provenance ledger intact.
- Never merge or copy the post-license predecessor source line. Equivalent behavior
  must be independently implemented or arrive with an explicit Apache-2.0
  grant from the relevant copyright holder.
- Run the lineage, provenance, and rename checks for any sync, history,
  licensing, package-identity, or repository-metadata change.

## Git Conventions

- Conventional commits enforced by commitlint
- Pre-commit runs fmt and clippy plus optional Taplo and cargo-machete.
- Pre-push delegates the full suite to CI; run relevant local gates yourself.
- Branch: `main` is the primary Apache-2.0 branch
- Use a topic branch and ready-for-review PR. Do not commit directly to `main`
  or merge your own PR, even while GitHub permits it.

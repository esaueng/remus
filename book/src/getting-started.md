# Getting Started

## Prerequisites

- [Rust](https://rustup.rs/) (stable, edition 2024)
- [wasm-bindgen CLI](https://rustwasm.github.io/wasm-bindgen/) for WASM builds
- [Node.js](https://nodejs.org/) 20+ for packaging the WASM module

## Building

```bash
# Clone the repository
git clone https://github.com/esaueng/remus.git
cd remus

# Build all Rust crates
cargo build --workspace

# Run tests
cargo test --workspace

# Build WASM target
cargo build -p remus-wasm --target wasm32-unknown-unknown
```

The MSRV is Rust 1.88. Day-to-day development uses the toolchain pinned in
`rust-toolchain.toml`, which rustup picks up automatically along with the
`wasm32-unknown-unknown` target.

## Using from JavaScript and TypeScript

The JS surface is the WASM package built from `crates/wasm`. It ships its own
TypeScript declarations.

**This repository publishes no packages.** There is no npm release and no
crates.io release; a `remus-wasm` package on npm belongs to the historical
upstream line, not to this repository. See
`docs/production-readiness/fork-maintenance.md` for the release-ownership gate.
Build the package from a checkout instead:

```bash
cargo xtask wasm-build          # dual-target build, merge, and validation
node scripts/test-wasm-smoke.mjs
```

That writes `crates/wasm/pkg`, which you can consume with `npm install
./crates/wasm/pkg` or a workspace link.

```typescript
import init, { BrepKernel } from 'remus-wasm';

await init();
const kernel = new BrepKernel();
const solid = kernel.makeBox(10, 20, 30);
```

## Development

```bash
# Install development tooling
npm install          # Husky hooks, commitlint
cargo install cargo-deny cargo-llvm-cov  # CI tools

# Format and lint
cargo fmt --all
cargo clippy --all-targets

# Check crate boundaries
./scripts/check-boundaries.sh
```

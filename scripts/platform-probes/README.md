# Windows ARM tooling readiness

This isolated probe tests `windows-11-arm` tooling before adding another Remus
geometry parity host. It does not build Remus, change its packages, or qualify
any of the 439 required geometry/translator gates. A workflow label or passing
local control tests is not a hosted ARM result.

The pull-request workflow runs only when its own workflow or this directory
changes. It has read-only repository permissions, no secrets or deployment,
a 25-minute job limit, a 20-minute execution limit, and failure artifact uploads.
Actions are pinned to the same immutable revisions as the parity workflow.

The actual host must establish all of these facts:

- Node 24 runs as `win32,arm64`; its actual `process.execPath` has an ARM64 PE
  machine field. Raw `node:os.machine()` must report `arm64` and the runner must
  identify itself as Windows/ARM64.
- Rust 1.96.0 uses `aarch64-pc-windows-msvc`. `rustup which` resolves the actual
  compiler and Cargo executables; both must have ARM64 PE headers. The compiler
  builds a tiny native program, whose actual PE, runtime architecture, pointer
  width and result 42 are checked before qualification.
- The rustup shim is recorded separately. The pinned wasm-pack 0.15.0 helper
  has an x64 Windows asset and runs under emulation. Its PE/hash/version and
  emulation status are recorded without claiming a wholly native helper stack.
  Available wasm-bindgen helpers are version-probed and exactly one 0.2.126
  candidate must match. That helper's PE/hash/version are an explicit post-build
  observation, not a child-process execution trace.
- A standalone, locked fixture with wasm-bindgen 0.2.126 builds a tiny `nodejs`
  WASM package. The probe records exact argv and its isolated build environment:
  verified `RUSTC`, empty compiler wrappers, a private Cargo home/cache/target
  directory, one Cargo job and a 32 MiB Rust thread stack. It does not set
  `CARGO_BUILD_TARGET`. This is a probe environment, not the canonical Remus
  package build configuration.
- Real npm JavaScript runs through the selected native Node using an argv array.
  The actual archive's SHA256 and npm SHA512 integrity are checked; every packed
  file must equal its actual installed counterpart, including the complete file
  census and the single expected WASM module. The installed package runs its
  exported sum function with the native Node and returns 42. Scripts are disabled
  for packing and installation, and the fixture has no npm dependencies.

The Rust fixture has its own workspace and lockfile. Its wasm-bindgen version
matches the kernel workspace, but it has no dependency on Remus crates and does
not alter the root dependency graph.

Run the portable negative controls without a build:

```sh
node --test scripts/platform-probes/windows-arm.test.mjs
```

The hosted workflow runs:

```sh
node scripts/platform-probes/windows-arm.mjs target/windows-arm-tooling
```

It retains `proof.json`, per-command stdout/stderr, the native program, actual
npm archive, and copies of all packed/installed package members. Failed commands
leave `passed:false`; missing evidence, wrong machine types, mismatched installed
bytes or relaxed build arguments cannot become a successful receipt. Temporary
build directories stay within the disposable hosted runner until it is discarded.

For read-only replay of an extracted success artifact, import `verifyArtifact`
from the exact checked-out revision of `windows-arm.mjs` and call it with the
artifact directory. It checks the receipt and retained PE/archive/package bytes, including the actual
regular USTAR member census and contents. The tiny fixed paths do not need TAR
extensions; links, PAX/GNU extensions, malformed checksums, unsafe/duplicate
names, more than 64 members or more than 16 MiB of inflated data refuse.
Separately bind the server artifact digest, actual workflow/job success, exact
checkout SHA and source-input hashes against immutable Git files. The receipt
identifies the hosted checkout; it does not promote a PR test merge to final main.

The current probe is tooling preparation. Adding the Windows ARM geometry tuple
requires a later change and actual complete reports with the existing assertions
and advisory gaps preserved. Cross-host bitwise identity remains unqualified.

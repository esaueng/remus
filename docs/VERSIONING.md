# Remus versioning

Remus uses `YYYY.RELEASE.PATCH`, beginning at `2026.1.0`. All three fields are
decimal integers without leading zeros. This is a valid Cargo and npm version;
the year is the Remus release line, not an assertion that the API is stable.

- `YYYY` is the year in which a release line is opened. A new year starts at
  `YYYY.1.0` only when maintainers deliberately open that line; builds do not
  change the year automatically.
- `RELEASE` starts at 1 each year and increments for a planned release that
  adds features or changes a public contract. Breaking changes require an
  explicit migration note in the changelog and release notes.
- `PATCH` starts at 0 for each release and increments for compatible fixes,
  performance changes, or a changed distributable WASM payload within that
  release line. A rebuilt package with identical published contents retains
  its version.

The publishable Rust crates start at `2026.1.0` together. Exact sibling
requirements in `Cargo.toml` must match their manifests. The two WASM crate
manifests and their generated npm packages use the same Remus version at a
release cut. Between cuts, the package refresh may advance the two committed
WASM packages together by `PATCH` as their distributable bytes change; this
does not publish Rust crates or declare a new release. Pair both npm packages
from the same commit when consuming them.

The `2.130.x` series belongs to the predecessor lineage. `2026.1.0` is the
first Remus numbering baseline and intentionally sorts after it. Historical
changelog entries and provenance records retain their original numbers.
Package version changes do not alter model, STEP, or serialized-data schema
versions. Existing consumers pinned to a commit keep that exact artifact;
moving them to a `2026.x.y` commit requires normal compatibility review.

For a release candidate, update the first-party crate versions, exact sibling
requirements, lockfiles, and both WASM manifests; rebuild both npm packages
from the same source. Check that the package pair, release notes, and proposed
`vYYYY.RELEASE.PATCH` tag agree. Do not create a tag or publish from a version
bump alone: the [release checklist](production-readiness/release-checklist.md)
and [release ownership policy](production-readiness/fork-maintenance.md) still
apply.

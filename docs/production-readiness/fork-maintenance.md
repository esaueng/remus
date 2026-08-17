# Fork maintenance and release policy

## Upstream relationship

- Historical upstream: `https://github.com/andymai/remus`.
- Production fork remote: `https://github.com/esaueng/remus`
  (renamed from `esaueng/remus`; GitHub redirects the old path).
- Permanent Apache branch: `main` (the former `apache-main`, since deleted;
  workflows and the consumer pin follow `main`).
- Final permissive upstream release: `v2.129.15` (`a878e2b9`).
- Last fork commit before the AGPL upstream merge: `1886e873`.
- Fork-only changes must be conventional commits with an audit or issue
  reference. Do not rewrite upstream history or remove attribution.

## Upstream intake policy

Do not merge upstream v3 or later into `main`. Code from those releases
can enter this project only when its copyright holder provides an explicit
Apache-2.0 grant. Otherwise specify and implement the behavior independently,
with a regression that proves the fork contract. Run
`scripts/check-apache-lineage.sh` before every push and release.

Security fixes are prioritized over feature work. If a vulnerability is
discovered in fork-only code, create a private maintainer record first; do not
promise an upstream disclosure SLA that this fork has not formally adopted.

## Release ownership

This fork must not publish Rust crates or npm packages or create GitHub releases
until named maintainers, package identity, vulnerability intake,
signing/provenance, rollback, and yanking authority are established. The
project and its first-party packages are Apache-2.0-only; contributions use the
same inbound license.

The manual `Build OpenZCAD WASM Candidate` workflow is validation-only: it
builds and uploads a short-lived workflow artifact, but cannot push commits or
create releases. The checked-in `crates/wasm/pkg` directory remains a frozen
compatibility snapshot while OpenZCAD consumes
`github:esaueng/remus#main&path:/crates/wasm/pkg`. Remove that snapshot only
after the consumer has migrated to an independently versioned artifact.

Before any independent release:

1. Confirm the branch contains a recorded upstream base and fork-only diff.
2. Pass the full native, MSRV, WASM, package smoke, npm dry-run, and dependency
   scanning matrix with checked-in lockfiles.
3. Review the production-readiness audit for unresolved P0/P1 findings.
4. Verify artifact contents, checksums, provenance/attestation, and release
   notes against the tag.
5. Document rollback and package-yank decisions with the release record.

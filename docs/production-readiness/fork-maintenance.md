# Origin, license, and release policy

## Project origin

The source boundary and historical repositories are recorded in
[`docs/PROVENANCE.md`](../PROVENANCE.md). Preserve copyright, license, NOTICE,
and Git attribution from those source projects.

## Upstream intake policy

Do not merge the post-license source line into `main`. Code from that line can
enter this project only when its copyright holder provides an explicit
Apache-2.0 grant. Otherwise specify and implement the behavior independently,
with a regression that proves the Remus contract. Run
`scripts/check-apache-lineage.sh` before every push and release.

Security fixes are prioritized over feature work. If a vulnerability is
discovered, create a private maintainer record first; do not promise a
disclosure SLA that the project has not formally adopted.

## Release ownership

This project must not publish Rust crates or npm packages or create GitHub releases
until named maintainers, package identity, vulnerability intake,
signing/provenance, rollback, and yanking authority are established. The
project and its first-party packages are Apache-2.0-only; contributions use the
same inbound license.

The manual `Build OpenZCAD WASM Candidate` workflow is validation-only: it
builds and uploads a short-lived workflow artifact, but cannot push commits or
create releases. The checked-in `crates/wasm/pkg` directory is the current Git
distribution channel and is refreshed from the exact `main` source by the
committed-package workflow. Consumers should pin a reviewed commit with
`github:esaueng/remus#<commit>&path:/crates/wasm/pkg`. Remove that snapshot only
after consumers have migrated to an independently versioned artifact.

Before any independent release:

1. Confirm the branch contains the recorded Apache source base and a reviewed
   Remus-only diff.
2. Pass the full native, MSRV, WASM, package smoke, npm dry-run, and dependency
   scanning matrix with checked-in lockfiles.
3. Review the production-readiness audit for unresolved P0/P1 findings.
4. Verify artifact contents, checksums, provenance/attestation, and release
   notes against the tag.
5. Document rollback and package-yank decisions with the release record.

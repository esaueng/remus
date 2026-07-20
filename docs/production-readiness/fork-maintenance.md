# Fork maintenance and release policy

## Upstream relationship

- Authoritative upstream: `https://github.com/andymai/brepkit`.
- Production fork remote: `https://github.com/esaueng/brepkit`.
- Audit base: `d2893af8807b2e7c6c52e90cba2a3ad9cce3bfa7` (origin/main and
  upstream/main matched at audit start).
- Fork-only changes must be conventional commits with an audit or issue
  reference. Do not rewrite upstream history or remove attribution.

## Sync procedure

1. Fetch both remotes and record the merge base and ahead/behind counts.
2. Review upstream release notes and bug-fix commits before selecting changes.
3. Merge or cherry-pick upstream fixes without rewriting the fork branch.
4. Resolve conflicts by preserving upstream attribution and adding a regression
   that captures the fork-specific behavior.
5. Run the release validation matrix before pushing the synced branch.

Security fixes are prioritized over feature work. If a vulnerability is
discovered in fork-only code, create a private maintainer record first; do not
promise an upstream disclosure SLA that this fork has not formally adopted.

## Release ownership

This fork must not publish Rust crates or npm packages, create GitHub releases,
or alter remote GitHub settings until named maintainers, package identity,
vulnerability intake, signing/provenance, rollback, and yanking authority are
established. The audit did not change licenses, copyright, CLA text, upstream
links, or package names.

The manual `Build OpenZCAD WASM Candidate` workflow is validation-only: it
builds and uploads a short-lived workflow artifact, but cannot push commits or
create releases. The checked-in `crates/wasm/pkg` directory remains a frozen
compatibility snapshot while OpenZCAD consumes
`github:esaueng/brepkit#main&path:/crates/wasm/pkg`. Remove that snapshot only
after the consumer has migrated to an independently versioned artifact.

Before any independent release:

1. Confirm the branch contains a recorded upstream base and fork-only diff.
2. Pass the full native, MSRV, WASM, package smoke, npm dry-run, and dependency
   scanning matrix with checked-in lockfiles.
3. Review the production-readiness audit for unresolved P0/P1 findings.
4. Verify artifact contents, checksums, provenance/attestation, and release
   notes against the tag.
5. Document rollback and package-yank decisions with the release record.

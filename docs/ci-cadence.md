# Development CI cadence

The fast/daily implementation is active in `fleet-ci-v12.yml`. The `ci.yml`
caller and `.github/fleet-workflows.json` select its immutable landed commit
`6701fc2dc528514e4e841270255eacfa5ff1dcd6`. Required check names,
App bindings, review rules and merge-queue settings are unchanged.

## Everyday feedback

The native path always executes the integration binaries listed in
`.github/ci-test-suites.json`: boolean and planar parity, evolution, exact blend
refusal, analytic chamfer, extrusion lineage, malformed mesh input, arena
history, atomic import and hostile input. These are a bounded contract suite,
not complete kernel qualification.

For supported leaf crates, source edits add the affected top-level Rust module's
unit tests. New or changed integration test files add their complete binary;
nested support files select their containing integration binary. The fast archive builds the selected packages' libraries and integration
targets; full archives build the workspace. The
actual archive inventory must contain every selected binary. If an affected
module has no runnable unit tests, the producer rebuilds the full native suite
instead of silently dropping that component.

Foundational crates, crate roots, dependency/toolchain inputs, unknown paths,
deleted files, fixtures and unsupported test layouts select the full suite.
No test's assertions, tolerances or ignored status change. New crates must be
explicitly reviewed before joining `focused_crates`; otherwise they remain full.

Fast selections execute on the archive producer without allocating download
workers. Full native execution uses four deterministic hash partitions of one
archive. Clippy runs beside archive preparation. Workers verify the source commit and
compiled fixture workspace before executing it. The final CI gate requires all
selected jobs and rejects failures, cancellations, missing results and unexpected
skips.

The full-suite producer builds and uploads its archive without extracting a
second complete target directory beside the compiler outputs. The four full
workers remain mandatory and perform archive setup and test execution. Fast
producers still inspect the archive inventory before running their selection,
and any captured setup diagnostic is printed before the job fails.

Ordinary merge candidates receive the critical suite plus changed test binaries.
Foundational changes still select complete native tests. The existing selection
of other merge-candidate checks remains intact. Main pushes retain exact-source
WASM build, runtime and package-artifact qualification; ordinary source pushes
defer broad dependency checks unless their inputs changed.

`ci:full` selects complete native tests and the broader validation tier.
Instrumented coverage has its own `ci:coverage` request or reusable-workflow
`coverage` input. Dependency and CI changes do not implicitly request coverage.
Fork code never receives the coverage job's OIDC permission.

## Daily and weekly qualification

`daily-validation.yml` runs complete native tests, doctests, Clippy, MSRV,
dependency policy and the advisory audit on protected main every day at 06:43
UTC. Its manual dispatch also requires protected main. Its jobs execute the
event's immutable checkout, so later main changes cannot relabel their result.

`coverage.yml` calls `coverage-sharded.yml` on the existing daily coverage
schedule. It builds instrumented binaries once, runs the complete
selected suite across four workers and collects raw LLVM profiles. Each worker
records source/archive identity, its selected test identities and profile
hashes. The merge refuses missing, overlapping, corrupted or mixed-source
partitions, and requires their union to equal the entire runnable suite.
The reporter restores the instrumented binaries and exports one LCOV report
while enforcing the existing **60% line minimum**. Only the separate Codecov
upload job receives OIDC permission. Codecov availability is separate from the
local coverage floor and the downloadable report.

The coverage-only Nextest profile reserves the complete test pool for the
sphere-minus-box oracle replay. Instrumentation plus CPU contention can exhaust
its existing two-minute budget before the identity checks. The reservation
changes scheduling only; its budget, mesh resolution and full oracle battery
remain unchanged. A process-level concurrency fixture and an instrumented
replay verify this policy in the hosted proof workflow.

Weekly fuzzing, mutation testing and the larger corpus campaign retain their
existing schedules. The smaller corpus campaign remains daily. Native platform
workflows retain their existing triggers.

This cadence moves detection of some native integration regressions until the
daily run. Scheduled failures need prompt investigation before further related
changes or release qualification. A passing fast PR result must not be reported
as a full-suite or coverage result. Full coverage is a suite-wide measurement;
this design does not invent a smaller PR coverage percentage.

## Rollout and verification

1. Land the staged workflow, selection manifest, scripts and regressions without
   changing the active workflow bytes or their immutable pin.
2. Read `Fast native suite proof` on that exact PR checkout. It builds and runs
   the real critical kernel suite and separately compares a real single-run
   coverage fixture with four isolated archive workers. Missing-profile and
   gate-failure cases are covered by the required Repository Policy tests.
3. In a separate PR, select v12 in `.github/fleet-workflows.json` and pin the
   `ci.yml` caller to the landed main commit containing byte-identical v12.
   Add the manual `coverage` input and forward it to that caller. Update the
   active-workflow policy tests to enforce the new native/coverage selectors,
   retaining the historical v11 contracts. Replace `coverage.yml`'s inline job
   with a call to `coverage-sharded.yml`, preserving its schedule, protected-main
   guard, concurrency and minimum.
4. Compare fast/full inventories and wall time, queue delay, archive transfer,
   runner minutes and the slowest partition on matched source commits. Verify
   the new gate on the separate merge-group checkout before reporting it active.
5. If the result regresses, restore the prior v11 manifest/caller pin in a PR.
   Scheduled full tests remain available independently of activation.

No fourfold speedup is assumed. Hash partitions do not balance by measured
duration, and compilation, package validation or runner availability may remain
the limiting stage. Add duration-based partition assignment only if recorded
imbalance justifies that extra maintenance.

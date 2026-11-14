# Reusing exact package qualification

Package refresh PRs keep the version/integrity gate and optimized size policy.
The integrity job first looks for the existing generated package artifact from
a completed successful first-attempt `ci.yml` push run on protected main.
It requires the WASM build, both emitted-runtime suites and CI Pass to have
succeeded, and the artifact must predate both runtime jobs.

The run's source commit must be reachable from current main. Every tracked
input outside the two generated package directories must match the candidate,
including the toolchain, lockfile, build flags, generators and test scripts.
Tracked source edits in the working tree also prevent reuse. The download's
SHA-256 digest must match GitHub's artifact digest. Only the expected archive
entry is accepted; the existing package installer validates every tar path
and entry type into a separate temporary directory.

Both rebuilt package trees are compared against the exact candidate's
committed files and manifest contracts using the existing integrity checker.
Only release numbering and the publisher's already-defined provenance
normalization retain their existing treatment. Verified bytes are then
installed for the size gate. A source, binary, glue, export, metadata,
toolchain or test change cannot borrow an unrelated qualification result.

CI runs the integration base commit's copies of the reuse script, the archive
installer and the integrity checker against the pull request checkout
(`--checkout`), so a diff cannot change a committed package and weaken its
checks together. A base whose reuse script predates `--checkout` reuses
nothing and falls back to the clean rebuild.

Expired/missing artifacts, reruns, incomplete jobs, failed comparisons and
retrieval errors select the unchanged clean rebuild and full package suites.
No arbitrary PR artifact or cached green check is used as a proof. Existing
main-push builds and daily cross-platform parity retain independent fresh
builds. Superseded PR version-guard runs are cancelled; every merge candidate
keeps its own group and verdict.

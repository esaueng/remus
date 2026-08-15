# Apache contribution provenance

This is the engineering provenance record for the Apache-only continuation. It
is not legal advice. The machine-readable source of truth is
[`apache-replay-provenance.json`](apache-replay-provenance.json), and
`scripts/check-apache-replay-provenance.py` validates it offline.

## Complete post-cutoff audit

The audit covers every pull request in `esaueng/brepkit` from #127 through
#247. Numbers #231 and #232 are not pull requests in that repository.

| Disposition | Count |
| --- | ---: |
| Replayed in Apache staging before phase two | 29 |
| Credited to the 73-commit phase-two replay | 73 |
| Explicitly excluded, superseded, or deferred | 17 |
| Total audited | 119 |

All 119 source pull requests were authored by `petergstfsn`. Each record pins
the exact head SHA, title, state, and merge date inspected on 2026-08-14. This
matters for PRs that were still open when audited.

## Lineage and staging sequence

- Fork cutoff: `1886e873fa4c24bf1880f2d3a868905c9d5e407f`
- Final permissive upstream: `v2.129.15` at
  `a878e2b9c42cd36e4f9d2c00504502a6ef2f9687`
- Forbidden upstream relicensing commit: `bd7d1ba7`
- Forbidden first fork merge of that line: `8fbaea57`

The checked record pins staging PRs #248 through #252. PR #248 established the
Apache-only line, #249 disabled BrepKit publication from staging, #250 replayed
the initial fork source-commit set, #251 replayed the STEP periodic work, and
#252 is the first security/correctness wave. The eleven fork source commits
used by #250 are recorded separately with their authors and exact replay
mappings. PRs #129 and #133 are credited even though their reconciled successor
PRs #132 and #134 supplied the replayed trees.

## Phase-two replay

- Replay parent: `e142b5727f56188014ebec723b81e8104063fd1d`
- Parent tree matching PR #252:
  `4c4650e0aa43cc3443c8d6eddcf53b5031198d13`
- Replay range: `a49092d2fdfe9472794cb77f58ffdbff51d38b43`
  through `7aeb36a802188de0e158326c15049ebaaa634ddc`
- Local replay commits: 73
- Credited source pull requests: 73
- Replay commit author: Peter, using the two recorded GitHub noreply addresses

The replay preserves contribution deltas rather than importing post-license
branch ancestry. Where the Apache staging architecture differed, the delta was
ported manually and verified against the staging tree.

- PR #243 required a follow-up tessellation adaptation after the ordered edge
  map port.
- PR #229 was split into a clean source subset and the remaining compatible
  fixes.
- PR #218 has a separate lockfile refresh.
- CI PRs #174, #176, #185, #208, and #210 were consolidated into one
  Apache-safe package-refresh workflow.
- PR #224 contributes its regressions; its implementation was superseded.
- `7aeb36a8` is independent fork-authored CI hardening that binds generated
  WASM to the triggering source commit.

## Exclusions and deferred repository prose

Every omitted PR has an exact reason in the JSON ledger. Closed alternatives
#148, #153, #196, #197, #199, #205, #213, and #214 are superseded by the
credited implementations. PR #154's delta was already present in the final
permissive upstream. PRs #206 and #207 target a post-license blend architecture
that does not exist on the Apache line. PR #219 is release-only metadata.

Repository-specific documentation from PRs #135, #137, #217, and #246 must be
regenerated after the standalone successor exists; transplanting their
BrepKit-specific settings would make the new repository inaccurate. PR #136 is
the closed review branch superseded by #135 and #137. The installed-package
test from #217 is superseded by the stronger tarball consumer replayed from
#221.

## Verification behavior

The checker always validates the canonical ledger digest, the exact 119-PR
partition, pinned source heads and authors, staging lineage, source-commit
mappings, phase-two counts, adaptations, and exclusions. When all 73 individual
phase-two replay commits are present, it additionally verifies their Git
authors, subjects, parent tree, and range count. That history check is expected
on the replay PR; after a squash merge or a fresh clone, the structured ledger
remains the durable evidence and is regenerated with the final replay hashes.

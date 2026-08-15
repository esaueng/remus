# Remus provenance

Remus is an independent Apache-2.0 continuation of the earlier BrepKit
codebase. The source lineage was assembled on
[`esaueng/brepkit`](https://github.com/esaueng/brepkit), branch `apache-main`,
from the final permissively licensed upstream line and separately audited
Apache-compatible contributions.

Historical upstream: [`andymai/brepkit`](https://github.com/andymai/brepkit).
The last permissive upstream release used by this lineage is `v2.129.15`
(`a878e2b9`). The fork commit immediately before the post-license upstream
merge is `1886e873`.

The standalone repository was seeded from the `esaueng/brepkit` Apache replay
squash merge `d89b189650fd535529814ea216405e97966854df`. Its tree is
`27a5d4edcd8a767e578184eeb20229474dead507`, exactly matching replay PR #253
head `aeaccea247ef619842a2e111a87c939b544969ff`. GitHub's squash merge means
the 73 individual replay commits are not ancestors of the standalone `main`;
PR #253 retains the review record, while the checked static ledger records and
validates each source-to-replay mapping after the squash.

The post-license BrepKit line is not part of Remus. Code from that line may
enter Remus only under an explicit Apache-2.0 grant from its copyright holder,
or through an independently specified and implemented replacement.

Historical product names and repository links remain in `NOTICE`, the two
changelog files, and the Apache replay ledger and checker because they record
attribution and release provenance. They are not current Remus package names
or endpoints.

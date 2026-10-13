## Change

Describe the problem and the resulting behavior. Link the roadmap row or issue.

## Verification

List the checks run and their results, including checks not run and why. For
bug fixes, describe the regression coverage; for performance or size work,
include before/after numbers.

## Compatibility and risks

Note changes to public APIs, file formats, units, tolerances, dependencies or
the WASM packages. Write "None" if there are none.

## WASM module tier

<!-- A new capability declares its tier: core, lazy:<module> or native
(AGENTS.md "WASM module tiers"). Core needs a reason: the session state it
edits. If the CI "WASM Binary Size" comment shows kernel growth above 64 KiB
raw, add a line that starts with "WASM-Size:" and names the capability and
its tier, for example "WASM-Size: +120 KiB raw, curve projection, Tier: core
(writes the journal)". Write "None" when there is no new capability and no
kernel growth. -->

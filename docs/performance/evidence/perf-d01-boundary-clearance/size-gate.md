## WASM Binary Size

| Kernel `remus_wasm_bg.wasm` | Base | PR | Delta | Policy |
|---|---:|---:|---:|---|
| raw | 9,560,400 B | 9,563,178 B | +2.7 KiB (+0.03%) | review 11.00 MiB, hard 12.00 MiB |
| gzip | 3,529,968 B | 3,531,558 B | +1.6 KiB (+0.05%) | hard 4.50 MiB |
| Brotli q11 | 2,521,461 B | 2,521,794 B | +0.3 KiB (+0.01%) | review 3.00 MiB |

Translator `remus_wasm_io_bg.wasm` (loaded lazily, outside the kernel policy): raw 2,705,941 B → 2,708,719 B, +2.7 KiB (+0.10%).

- ✅ Within the hard limits. Kernel grew by 2.7 KiB raw, within the 64.0 KiB review threshold.
- Declared: ` WASM-Size: +2,778 B raw kernel against current main, nonplanar boundary-distance pruning, Tier: core `

<sub>Measured by the base branch's `scripts/wasm-size-policy.mjs` (Node 24.14.0, zlib 1.3.1-e00f703, Brotli 1.2.0) against the package committed on the base branch. A kernel growth above 64.0 KiB raw needs a `WASM-Size:` line in the PR description.</sub>

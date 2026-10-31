# Kernel size attribution, 2026-10-29

Evidence for "Growth attribution (2026-10-29)" in
[`docs/design/kernel-wasm-size-architecture.md`](../../../design/kernel-wasm-size-architecture.md).

| Build | Source | Shipped package | Rebuilt here (raw / gzip / Brotli q11) |
| --- | --- | ---: | ---: |
| base | `89a3eac5` (#1031, after the Oct 13 size work) | 9,412,128 B (v2026.1.57) | 9,386,948 / 3,481,345 / 2,484,589 B |
| cur | `d8195032` (v2026.1.86 package refresh) | 11,005,504 B | 10,974,155 / 4,079,297 / 2,904,757 B |

The rebuilds land within 0.3% of the shipped packages. The rebuild here used
Binaryen `version_123` from its GitHub release instead of wasm-pack's cached
copy, and no `--remap-path-prefix`. Treat absolute bytes as approximate; the
base-to-cur deltas are internally consistent.

## Reproduce

For each source, in a separate worktree, with a shared target directory:

```
CARGO_ENCODED_RUSTFLAGS=$'-C\x1ftarget-feature=+simd128' \
  cargo build --release --target wasm32-unknown-unknown -p remus-wasm --no-default-features --lib
wasm-bindgen --target bundler --out-dir bg-<v> <v>.wasm          # wasm-bindgen-cli 0.2.126
wasm-opt -Oz --enable-simd bg-<v>/<v>_bg.wasm -o <v>.opt.wasm     # shipped-equivalent bytes
wasm-opt -Oz -g --enable-simd bg-<v>/<v>_bg.wasm -o <v>.optg.wasm # same, names kept for attribution
node scripts/wasm-size-policy.mjs --json <v>.opt.wasm             # size-<v>.json
```

Then, with twiggy 0.8.0:

```
python3 attribute.py base.optg.wasm cur.optg.wasm > attribution-post-opt.txt
python3 attribute.py base.wasm cur.wasm           > attribution-pre-opt.txt
python3 normdiff.py  base.optg.wasm cur.optg.wasm > function-diff-post-opt.txt
twiggy monos -m 15 -n 0 <v>.optg.wasm             > monos-<v>.txt
```

`attribute.py` groups function bytes by the first `remus_*` crate in the
symbol, as the 2026-10-13 attribution did. `normdiff.py` strips the
`::h<hash>` suffix, which changes every build, before comparing functions. The
`-g` builds carry a names section, reported as "custom/name sections"; subtract
it for shipped bytes. The post-`wasm-opt` totals differ from base to cur by
+1,658,310 B. Minus the names section's +71,103 B, that is +1,587,207 B, the
shipped-equivalent raw growth.

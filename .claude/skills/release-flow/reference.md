# release-flow reference

Deep detail for the hops in SKILL.md.

**Repo and paths.** The kernel side is this repo, `esaueng/remus`; paths below
are relative to its root, because it is checked out in several places
(`~/claude/remus`, `~/codex/remus`, plus `.worktrees/*`) and no single absolute
path is correct. Do not confuse it with `esaueng/brepkit`, a separate fork of
`andymai/brepkit`.

## What "publishes nothing" rests on

`docs/production-readiness/fork-maintenance.md`, "Release ownership": this fork
must not publish Rust crates or npm packages, or create GitHub releases, until
named maintainers, package identity, vulnerability intake, signing/provenance,
rollback, and yanking authority are established. That is a governance gate, not
a technical one — a green build does not satisfy it.

Consequences worth stating plainly, because they contradict what a reader may
assume from the repository's shape:

- `release-please-config.json` sets `"component": "remus-wasm"` with
  `"include-component-in-tag": false`, and `.release-please-manifest.json`
  tracks a version. **No workflow consumes either.** Verify with
  `rg -l 'release-please' .github/workflows/`.
- `crates/wasm/Cargo.toml` carries a version (2.x) inherited from the
  pre-fork line. It is not a published version.
- Anything on npm under a Remus-like name came from somewhere else.

## The committed package

`crates/wasm/pkg` is a real distribution channel, not a build leftover: a
consumer installs it by git path, so whatever is committed there is what they
get. Contents:

```
package.json          # name, exports, files list, provenance stamp
<crate>_wasm.js       # bundler entry (ESM)
<crate>_wasm_bg.js
<crate>_wasm_bg.wasm  # the binary
<crate>_wasm.d.ts     # TypeScript declarations
<crate>_wasm_node.cjs # node entry; xtask renames the nodejs build to .cjs
LICENSE-APACHE
```

The `.cjs` rename matters: the node entry is CommonJS, and `package.json` sets
`"type": "module"`, so without the extension change Node would misparse it.

### The snapshot is frozen, and why that is not laziness

The committed binary embeds its own glue module path in the wasm import
section. Renaming the files without regenerating breaks the package outright;
regenerating changes the package name, which breaks a consumer importing the
old one. So the snapshot stays as-is until the consumer migrates, and
`scripts/check-remus-rename.sh` allowlists `crates/wasm/pkg` for exactly that
reason. Check the embedded paths before assuming a rename is safe:

```bash
rg -a --count-matches 'wasm_bg\.js' crates/wasm/pkg/*.wasm   # currently 15
```

## Build anatomy

`cargo xtask wasm-build` (see `xtask/src/wasm.rs`):

1. `wasm-pack build crates/wasm --target bundler --out-dir pkg`
2. `wasm-pack build crates/wasm --target nodejs --out-dir pkg-node`
3. Merges: copies the node entry in as `*_node.cjs`, rewrites `package.json`
   (`main`, `module`, `exports`, `files`).
4. Copies `LICENSE-APACHE` in, removes a stale `LICENSE-MIT` if present.
5. Runs `wasm-opt` unless `--skip-opt`.

Flags: `--no-simd` disables the SIMD build; `--skip-opt` skips `wasm-opt`
(what CI uses, for speed). `cargo xtask wasm-publish` exists and takes
`--dry-run`; **do not run it without the dry-run flag** — see the publishing
gate above.

Note that wasm-pack wipes its `--out-dir`. A local `cargo xtask wasm-build`
therefore destroys the committed snapshot in your working tree. That is
recoverable (`git checkout -- crates/wasm/pkg`) but easy to commit by accident;
check `git status` before staging.

## Provenance stamp

The manual candidate workflow stamps the generated `package.json` before
packing:

```bash
npm pkg set \
  repository.type=git \
  repository.url=git+https://github.com/esaueng/remus.git \
  homepage=https://github.com/esaueng/remus \
  remusSourceCommit="$GITHUB_SHA"
```

`remusSourceCommit` is the audit trail: given a tarball, it identifies the
source commit that produced it. Preserve the field if you touch the stamping
step.

## The consumer regressions

`scripts/openzcad-wasm-consumer-regressions.mjs` is shared by both harnesses
(`test-wasm-smoke.mjs` in-tree, `test-wasm-tarball-consumer.mjs` through a
packed install). Two classes of assertion:

- **Exact-geometry**: e.g. coaxial revolved annuli sharing an exact
  cylindrical wall must fuse analytically. The assertions check face counts and
  surface kinds because a mesh fallback can remain watertight, valid, and
  close on volume — volume alone would not catch the regression.
- **Evolution payload completeness**: every source face must be accounted for
  as modified, deleted, or explicitly unresolved, and likewise for results.
  This mirrors what a consumer needs to keep selections alive across an edit.

If you add a consumer-visible behavior, add its regression here rather than
only in Rust: this is the layer that proves the *packaged* artifact behaves,
not just the workspace.

## Snapshot refresh mechanics

Workflow: `.github/workflows/publish.yml`, "Refresh Apache Staging Package",
`workflow_dispatch` only. The job additionally requires `sync_package` and
`github.ref == 'refs/heads/main'`, so a dispatch from another ref no-ops.

- `install-wasm-package-archive.py` is a deliberate airlock: it accepts only
  regular files and directories beneath `crates/wasm/pkg`, so a build-produced
  archive can never write elsewhere in the checkout.
- The commit job disables git hooks (`core.hooksPath=/dev/null`) while a write
  token is present, and the build job never sees that token.
- `HAS_APP_CREDENTIALS` is computed from `REMUS_BOT_APP_ID` and
  `REMUS_BOT_PRIVATE_KEY`. If either is unset the app-token step is skipped and
  the job falls back to `github.token` — **without failing**. A missing secret
  is therefore silent; check the step's status, not just the job's.
- The push is `HEAD:main` with no rebase, by design. Losing a race is the
  correct outcome: the newer source commit refreshes from the right tree.

## Handing over a candidate without touching main

`.github/workflows/openzcad-wasm-release.yml` ("Build OpenZCAD WASM
Candidate"), `workflow_dispatch`. Builds, stamps provenance, `npm pack`s, and
uploads the tarball as a workflow artifact. It has `contents: read` only, so it
cannot push or release. This is the sanctioned way to give someone a build for
evaluation.

The receiving side can validate it the same way CI does:

```bash
node scripts/test-wasm-tarball-consumer.mjs   # packs and installs into a temp consumer
```

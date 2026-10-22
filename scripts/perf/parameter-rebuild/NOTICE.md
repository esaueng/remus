# Recipe attribution and fixture identity

The numeric plan in `holder_recipe.cjs` adapts the opening-width portion of
OpenZCAD's `packages/command-system/src/growing-holder.ts`, pinned to
[`913bb9ddd015bf2e6284c60a231eaa5afe7f2106`](https://github.com/esaueng/OpenZCAD/tree/913bb9ddd015bf2e6284c60a231eaa5afe7f2106).
The adaptations replace document expressions and application commands with a
fixed numeric kernel replay. Height remains 58 mm; lettering remains enabled.
This is separate benchmark code and does not modify or bundle the consumer.

The pinned source is licensed under Apache 2.0. Its copyright notice is:

> OpenZCAD
> Copyright 2026 Esau Engineering LLC
>
> This product includes third-party software and fonts. See
> THIRD-PARTY-NOTICES.md for attribution and license details.

No fonts or other third-party consumer assets are copied here. The Apache
license is available at the repository root as `LICENSE-APACHE`; original
license/notice/compiler SHA-256 values are recorded in `fixtures/recipe.json`.

`fixtures/base.arena.json` and `fixtures/text.arena.json` are exact geometric
derivatives of the existing repository STEP fixture
`crates/io/tests/data/shapr3d_hammer_holder.step`. They add no external model.
The STEP header is not copied into these derivatives. The pinned consumer's
unchanged `recognizeOpening` and `separatePlanarEmboss` helpers, executed against
the committed v2026.1.69 package pair, measured the recipe and separated the
plain base from seven constant-depth lettering solids. Carrier coefficients,
curves, trims, and boundary authority use arena v3 interchange. The base's
public part-name attribute is retained. No further ownership or licensing
claim about the source model is made here.

`fixtures/recipe.json` records the source hash, measured profiles, cut planes,
recognition evidence, exact derivative hashes and byte lengths, generation
package hashes, and the 31-feature layout. `pinned-source-manifest.json`
records every source Git blob and SHA-256 read for the one-time extraction
(55 files, 988,659 bytes). Runtime-only package loading was bridged to the
chosen Node wrappers in temporary files; all recognition/separation source
bytes were verified against their immutable Git blobs. No consumer checkout
was changed, and no consumer dependency install was required.

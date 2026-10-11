# Truck NURBS reuse provenance

This change preserves Remus's Apache-2.0 license. Its only external source is
selected Apache-2.0 code from
[Truck](https://github.com/ricosjp/truck/tree/88ed005249e5e3a6b07f62425399435905cd3ab6),
pinned to commit `88ed005249e5e3a6b07f62425399435905cd3ab6`. This record
describes source provenance and redistribution notices; it does not establish
numerical qualification or make a broader license claim about other projects.

The machine-readable inventory is
[`truck-reuse-provenance.json`](truck-reuse-provenance.json).
The [design and usage contract](../design/truck-nurbs-reuse.md) describes the
supported domain and quality reporting. The owning roadmap item remains
[P-Class 7.6](../kernel-maturity/roadmap.md#p-7-6).

## License boundary

At the pinned commit, Truck's root `LICENSE` contains the Apache License,
Version 2.0, and `truck-geometry/Cargo.toml` declares `license = "Apache-2.0"`.
The selected source files contain no separate copyright, patent, trademark,
attribution or license headers. The pinned repository contains no `NOTICE`
file. No separately licensed vendored helper is extracted by this change.
Truck's package author field identifies Yoshinori Tanimura; attribution is to
the Truck contributors, without inventing a copyright year or holder notice
that is absent from the selected sources.

Apache-2.0 section 4 permits this adaptation under the same license. Remus
retains [`LICENSE-APACHE`](../../LICENSE-APACHE), preserves its existing
[`NOTICE`](../../NOTICE), adds Truck attribution there, and prominently marks
the adapted Rust files as modified for Remus with the source pin. The standard
Apache license copy covers both Remus and these portions. Future changes must
preserve those notices and re-audit any expansion of the extraction boundary.
The WASM distribution builder copies both `LICENSE-APACHE` and the current
root `NOTICE` into newly built npm packages, includes them in the package
file list, and rejects missing or stale metadata. Frozen generated packages
are not changed by this source implementation.
`remus-math` also carries byte-identical copies in `crates/math/LICENSE-APACHE`
and `crates/math/NOTICE`, so its Cargo source archive includes the license
and attribution for the adapted code. The offline checker requires both
copies to match the current root metadata.

This change imports no OCCT code, translated implementation, documentation,
test script or fixture. It also imports no Truck dependency, external dataset,
STEP fixture, or upstream test implementation. Its new tests construct their
own analytic and rational geometry. Existing Remus code remains subject to
the repository's [Apache contribution policy](../../CONTRIBUTING.md#license-and-sign-off).

## Source-to-destination mapping

All upstream line numbers below refer to the pinned commit. Full source-file
and function-range SHA-256 digests are recorded in the JSON inventory.

| Truck source | Upstream lines | Remus destination | Adaptation boundary |
| --- | --- | --- | --- |
| `truck-geometry/src/nurbs/bspsurface.rs`, `BSplineSurface::try_remove_uknot` | 851–897 | `crates/math/src/nurbs/reduction.rs` | Homogeneous inverse-insertion recurrence for a U control strip; Remus validation, immutable results and whole-domain acceptance replace Truck's local `near` decision. |
| Same file, `BSplineSurface::try_remove_vknot` | 949–996 | Same destination | The corresponding V control strip recurrence, adapted to Remus's storage convention. |
| Same file, `BSplineSurface::optimize` | 1542–1557 | Same destination | Descending repeated-removal strategy, with deterministic U/V ordering, explicit limits and a bound against the original surface. |
| `truck-geometry/src/nurbs/bspcurve.rs`, `BSplineCurve::optimize` | 714–723 | Same destination | Descending repeated-removal strategy. Curve candidates use Remus's existing `curve_knot_remove`, rather than extracting Truck's curve-removal implementation. |
| Same file, `BSplineCurve::cubic_bezier_interpolation` | 1287–1301 | `crates/math/src/nurbs/cubic_fit.rs` | Endpoint/tangent cubic Hermite control construction in Remus types. |
| Same file, `BSplineCurve::sub_cubic_approximation` | 1303–1355 | Same destination | Adaptive bisection orchestration; Truck's hash-selected sample acceptance and concatenation are replaced by Remus whole-interval certificates and bounded segment assembly. |
| Same file, `BSplineCurve::cubic_approximation` | 1373–1391 | Same destination | Endpoint/derivative initialization and orchestration. Truck's trailing `optimize()` is not copied into the fitting path. |

`crates/math/src/nurbs/reuse.rs` and `reuse_bounds.rs` are independently
authored Remus validation, resource-policy and interval-certificate code.
No Truck geometry types, `Tolerance`, `HashGen`, `KnotVec`, `inv_or_zero`,
iterator implementations, trait crates or serialization helpers are extracted.
Call-site glue in the native facade and WASM bindings is independently authored.
No Cargo manifest or dependency license changes are needed for this extraction.

## Reproducible checks

Run the static inventory and adapted-file notice check without network access:

```sh
python3 scripts/check-truck-reuse-provenance.py
```

To additionally verify upstream file bytes and function-range digests, supply
an existing local Truck checkout at the pinned commit:

```sh
python3 scripts/check-truck-reuse-provenance.py --truck-source /path/to/truck
```

The checker does not clone, fetch or modify either repository. Without the
local checkout it validates the durable ledger and Remus notices, but does
not claim to have re-read upstream source bytes. With the checkout it checks
the commit, file hashes, selected line hashes, Apache manifest declaration,
and absence of an upstream `NOTICE` file. This is a scoped provenance check,
not a license audit of Remus's existing dependency graph.

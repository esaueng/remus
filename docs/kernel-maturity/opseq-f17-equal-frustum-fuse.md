# F17: bounded equal-frustum fuse

The original F17 witness fuses native frustums with bottom radius 4.5, top
radius 2.5 and height 5.5, with the second translated 7 along x. Its marched
cone–cone section stopped inside the walls near z 0.35 rather than reaching
the bottom rim. The returned exact solid had four same-sense shared edges.
The witness is now active in `op_seq_81.rs`.

The repair carries the analytic cone–cone section as two exact rational
quadratic spans. The adapter requires identical stored axes and half-angle,
complete native circular rims with the same stored radii, one repeated
straight generator and no holes. Apex axial displacement must be within the
existing linear tolerance; genuine axial shifts and unequal finite bands
remain outside this adapter. The smooth conic turn must lie inside both
finite bands. Other inputs retain their existing path.

One cone's artificial seam crosses the new section. Its seam moves to the
antipodal generator only when outward-rounded bounds on every positive
rational control hull prove the whole new seam clear. Full native circle
trim and winding are retained; moved straight seams drop their old edge
identity. Exact quarter-arc cuts on an untouched relocated rim let the
existing refinement synchronize its cap mate. The shared edge merge policy,
operation tolerances and work budgets are unchanged.

## Independent oracles

For the literal fixture, subtracting the cone equations gives x=7/2. The
bottom crossings have y²=8, z=0; the conic turn is (7/2,0,11/4). The math
helper also has independent positive-nappe implicit-equation tests with
nonzero axial apex shifts, rotated axes, scale and operand order. That
broader math identity does not qualify corresponding downstream booleans.

Quantities use horizontal disc overlap rather than the conic construction.
Let r0=4.5, r1=2.5, h=5.5, q=7/2 and k=(r0−r1)/h. With
L(r)=2[r²acos(q/r)−q√(r²−q²)], the overlap volume is

```text
2/(3k) [r0³ acos(q/r0) − 2q r0 √(r0²−q²) + q³ acosh(r0/q)].
```

Subtracting this from two primitive volumes gives 426.55412467227507.
Subtracting L(r0)[1+√(1+k²)/k] from two primitive exterior areas gives
393.56351122611704. An independent Simpson sum of the disc lens checks the
closed form. Native and packaged acceptance retains 1e−4 relative quantity
accuracy at volume requests 1e−4 and 5e−5 times scale. Quadrature still has
finite trim-outline error; it is not an algebraically exact measurement.

Successful cells require analytic cone/plane faces, strict B-Rep validation,
exactly two opposing uses per edge, Cartesian source/result material grids,
unchanged serialized operands, and closed opposing raw triangle edges at
preview deflections 0.1 and 0.01 times scale. Signed mesh volume must be
positive and agree within the preview's 1% approximation allowance.

## Accepted and refused cells

Both operand orders are checked in every row. The rigid placement is
translation(13s,−7s,9s) times Rx(0.37) times Rz(0.23).

| Construction | Scale | Placement | Expected result |
| --- | --- | --- | --- |
| Translated twin | 1,1000 | Unrotated | Exact; 4 cells |
| Mirror across x=3.5s | 0.001,1,1000 | Unrotated | Exact; 6 cells |
| Translated twin | 0.001 | Unrotated | ExactOnlyUnattainable; 2 cells |
| Either construction | 0.001,1,1000 | Rigid placement above | ExactOnlyUnattainable; 12 cells |

Every refused cell pins that exact error and preserves arena counts and
serialized operands. The packaged direct and both batch surfaces also check
live-slot/journal state, the direct detailed refusal code, and the batch
v2 category/code. Allocated slot high-water marks remain monotonic: each new
slot from a refused call must be a reserved tombstone, preserving the existing
stale-handle contract. They do not accept arbitrary errors as successful tests.

A clean rebuild of an immutable `ed34a9abe1a1cc9de62613d23350f91df44d74c5`
archive executed this complete 24-cell matrix: eight incorrect exact
successes and sixteen typed refusals, with no correct exact cell. The repair
turns four wrong successes into correct exact results, four into atomic
refusals, and six prior refusals into correct exact results. Ten prior
refusals remain, with fourteen total refusals after the repair. A preliminary
run that reused candidate Cargo fingerprints was identified and excluded
from this baseline evidence.

The unit translated and mirrored pairs also have native controls for all three
operators in both orders, with independent inclusion–exclusion volume and
exterior-area targets, Cartesian material grids and opposing raw meshes.
Packaged controls exercise translated cut/intersect and mirrored intersect
through direct and both batch APIs. The 1% preview quantity allowance above
belongs to the fuse matrix; complementary operators retain the strict fine
quantity gates without a new coarse-preview accuracy promise.

The tiny translated refusal still reaches an incomplete cone wire. Rotated
cells have correct exact conic sections but unresolved coplanar bottom-cap
partitioning. These limits, general cone–cone booleans, partial/holed bands,
and the broad X10 campaign exclusion remain open. This closes the literal
F17 orientation witness, not the general X10 family or the 8.1 parent exit.

Regression owners:

- `crates/math/tests/regress_equal_cone_section.rs`
- `crates/algo/src/pave_filler/phase_ff/helper_oracle_tests.rs`
- `crates/algo/src/builder/face_splitter/special_cases.rs`
- `crates/operations/tests/regress_opseq_equal_frustum_fuse.rs`
- `crates/operations/tests/op_seq_81.rs`
- `crates/wasm/tests/opseq81_crosscheck.rs`
- `scripts/opseq-f17-packaged.mjs`, called by both canonical package gates

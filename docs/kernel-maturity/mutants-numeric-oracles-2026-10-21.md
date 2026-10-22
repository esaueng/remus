# Numeric mutation oracles (2026-10-21)

B19 follow-up to scheduled [Mutation Testing run 39105361942](https://github.com/esaueng/remus/actions/runs/39105361942), at source `60adde7788be3d6ccddad1fb26201b56801e3543`.
This bounded slice owns three observed survivors in shard 0. The corresponding
production functions are unchanged at the refreshed base
`ed34a9abe1a1cc9de62613d23350f91df44d74c5`.

## Independent witnesses

| Survivor, original location | New owning-package oracle |
| --- | --- |
| `projection.rs:195:5`, replace `curve_coarse_seeds_public` with `vec![-1.0]` | Piecewise affine and rational curves with independently derived Cartesian distance ordering, nonunit domains and complete span coverage, including candidates beyond the private best-five truncation. |
| `line/subdivision.rs:463:37`, replace `<` with `==` in `split_net` | Degree-elevated coefficients of `(u²-u/2)(1+v)+v²`, evaluated with an explicit Bernstein sum on both halves and axes at degrees 31, 32 and 33. Degree 32 takes an out-of-bounds stack slice under the mutation. |
| `phase_ef/line_sphere.rs:172:13`, replace the final `&&` with `||` in `I::sqrt_nonnegative` | Exact integer comparison proves the binary64 value `0x3ff6a09e667f3bcd` lies strictly between the square roots of `2` and its next binary64 neighbor. The interval must extend to both sides. Exact dyadic roots, zero crossings and unknown/invalid intervals are separate controls. |

These tests exercise the production helper in its owning package. They add no
runtime code, dependency, tolerance change, mutation exclusion or reporting-policy
change. The `SpanHull::for_surface` capacity-expression survivor is outside this
slice: replacing `2*n-1` with `2+n-1` only changes a `Vec` reservation hint;
pushes still construct the same hull. It is not credited as caught.

## Measurement and limits

The scheduled shard listed 429 mutants and examined 171: 108 caught, 43 missed,
20 unviable, no timeouts and 258 unexamined. Its baseline passed; the 300-minute
budget expired. These are historical counts, not a current whole-campaign score.

Local replay with `cargo-mutants 27.0.0` caught all three exact historical
mutants: 3 examined, 3 caught, no misses, unviable mutations or timeouts. The
fresh configured baselines passed 1,090 math tests and 744 algo tests, each with
one existing skip. The split replay reused the same frozen math baseline with
explicit `--baseline skip`; the other two ran their own package baseline.
The committed mutation config retains `ci-test`, nextest, `--tests`, per-package
oracles, first-failure stop and the existing exact expensive-test exclusions.

The remaining survivors and unexamined tail remain open. A bounded local replay
does not establish a green weekly campaign, platform parity or consumer delivery.


| Exact replay | Historical verdict | Current verdict | First failing new oracle |
| --- | --- | --- | --- |
| Coarse-seed replacement | Missed | Caught | `coarse_seeds_rank_a_rational_arc_by_independent_circle_geometry` |
| Degree-32 comparison | Missed | Caught | `split_net_degree_boundary_matches_closed_polynomial_on_both_axes` |
| Final square-root conjunction | Missed | Caught | `square_root_interval_encloses_both_sides_of_rounded_root` |

The selected files are byte-identical between the historical source and the
refreshed base. This branch changes only test modules in those files. The
historical missed evidence is not presented as a new pre-change replay.

Full workspace all-target/all-feature Clippy, formatting, TOML formatting,
dependency checks, architecture boundaries, mutation-scope negative controls,
planner/verdict regressions, roadmap inventory, documentation paths and module
map checks passed locally. Hosted CI remains separate.

### Raw replay identity

Each replay retains its `mutants.out` listing, outcomes, diff and build/test logs.
The source and outcome hashes below identify that local evidence without
requiring a particular checkout path.

| Source file | SHA-256 of tested file |
| --- | --- |
| `math/src/nurbs/projection.rs` | `6cc5fb9fbba5bd017b8046b67f3b16c7089b4d907046669d2a61ec0f224cd03c` |
| `math/src/nurbs/intersection/line/subdivision.rs` | `08276e2b62c4e91b824c3e04cba9d0601504f8c21792bf3171812d784368fd26` |
| `algo/src/pave_filler/phase_ef/line_sphere.rs` | `dca7da53736e3b4d7228c927612515d72f3bd0864218298322ec63b231d24761` |

| Replay | `outcomes.json` SHA-256 |
| --- | --- |
| sqrt | `2b0d3f88cc06060945e1ec841dda5f1f74736c3138d14d61e61e5f6428969fb1` |
| seeds | `44c9fd1a4d151678a450ebde7311fbe4b58ef55a299b1d80d1e8c8fe7d075e62` |
| split | `5d029df5b7663c78a055c24029ff8174bcc1ef09239934c0524a9e9c53ed8063` |


### Reproduce the bounded selection

Version 27.0.0 also listed structural field-deletion mutations outside these
file/regex filters. The fixed 21-entry listing was verified before selecting
round-robin shards 1, 4 and 13. On another revision or tool version, verify the
actual single-mutant identities before running; these indices are not general
function selectors. The combined JSON listing's SHA-256 was
`e7f56e1d461fc272c6aa4305a8d0d286dccab7d7906cb7c18bb36667cc682e3a`.

```bash
export CARGO_BUILD_JOBS=2 RUST_MIN_STACK=33554432
selection=(
  --package remus-math --package remus-algo
  --file crates/math/src/nurbs/projection.rs
  --file crates/math/src/nurbs/intersection/line/subdivision.rs
  --file crates/algo/src/pave_filler/phase_ef/line_sphere.rs
  --re 'replace curve_coarse_seeds_public -> Vec<f64> with vec!\[-1\.0\]|replace < with == in split_net|replace && with \|\| in I::sqrt_nonnegative'
  --exclude-re 'delete field'
)
cargo mutants "${selection[@]}" --list --json > selection.json
# Verify the listing and each selected identity before continuing.
cargo mutants "${selection[@]}" --sharding round-robin --shard 1/21 \
  --jobs 1 --caught --colors never --output /tmp/oracle-sqrt
cargo mutants "${selection[@]}" --sharding round-robin --shard 4/21 \
  --jobs 1 --caught --colors never --output /tmp/oracle-seeds
# Only reuse the successful math baseline while the tested source stays frozen.
cargo mutants "${selection[@]}" --sharding round-robin --shard 13/21 \
  --baseline skip --jobs 1 --caught --colors never --output /tmp/oracle-split
```

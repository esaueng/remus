# Fuzzing

`fuzz/` has bounded libFuzzer targets for public readers and kernel operations.
Reader inputs use strict `ImportLimits`; engine targets use small structured
generators and independent geometry or topology oracles, so a plausible but
wrong result is a finding as well as a panic.

`nurbs_surface` constructs 2–4 control-point rational patches, tests typed
rejection of corrupted construction data, evaluates points and derivatives,
and intersects every valid patch with a horizontal NURBS plane. Returned SSI
points, parameters, and fitted curves must satisfy the plane equation and
re-evaluate on both input surfaces. Its corpus includes a clustered marching
section that previously made a cubic refit leave the known plane.

`topology_mutation` builds a bounded box (census and `dx * dy * dz` known by
construction) and drives byte-selected topology mutations over it:
authoritative face-loop identity (`build_face_loops` is read-only on a
derived face) and retirement through the sanctioned wire replacement,
validated rollback of a deliberately broken wire, rollback of staged
allocations and of in-transaction wire replacements/deletions, checkpoint
restore, unreferenced-solid deletion, and referenced-deletion refusal. The
oracles are the mutation contracts: comprehensive validation and the
closed-manifold census hold after every step, rollback reproduces the exact
live state while checkpoint restore keeps window retirements tombstoned and
promotes the affected face onto fresh handles rather than dangling, retired
handles fail typed lookups forever and are never reissued, a refused
deletion leaves no partial mutation, and an accepted one retires exactly the
unshared tree — a guard box and an unrelated compound must survive. Its
corpus includes a checkpoint re-derivation seed that previously left the
loop-derivation map referencing retired loops, and the two seeds from the
coedge-authority flip (PR #179): a re-derivation whose handles must now be
preserved, and a guard-box sweep whose derivation census must count the
loops `add_face` installs.

`modifier_ops` builds a bored or bossed primitive and applies one fillet,
chamfer, shell or draft, checking hole preservation, closed-manifold census,
watertight tessellation, scale invariance and integrator agreement. Its corpus
includes a draft seed whose 1° outward taper of a narrow facet slid the facet's
corners past each other; the folded face passed validation and the volume sign
check and only showed as four wrongly-wound half-edges in the fine
tessellation, and is now refused by name. It also carries a fillet seed — a
unit box fused with a large disjoint torus, one box edge filleted at r and r/2
— that once tripped the option-honoured invariant: the two fillets differed by
4e-4 as they should, but the check scaled that against the torus's ~800 of
unrelated volume. The invariant now judges the volume each setting changed.

`arena_roundtrip` builds bounded boxes and cylinders (census and closed-form
volumes known by construction) with duplicate roots, shared-shell aliases,
repeated/aliased compound members, precision-hostile tolerances, and public
attributes, then round-trips the document through the native arena format.
Restored solids must validate, stay closed-manifold, and measure their
closed-form volumes per root/member position; tolerances, trims, and
attributes survive bit-exactly; and serialize → deserialize → serialize is
byte-identical. Deliberately corrupted root/member/wire/version references
must be refused with a typed error, leaving a pre-populated destination
topology untouched and leaking no staged allocations. Its corpus includes a
seed covering attributes on a deliberately uncaptured member (correctly
absent from the document). The byte-identity oracle caught serde_json
losing the last bit of arbitrary f64 tolerances without the
`float_roundtrip` feature, which is now enabled workspace-wide.

Run one target locally with nightly Rust and `cargo-fuzz`:

```bash
cargo +nightly fuzz run nurbs_surface -- -max_total_time=60 -rss_limit_mb=2048
```

PR CI compiles every target. The scheduled `Fuzz Smoke` workflow runs the
public model-reader, boolean-tree, modifier, NURBS-surface,
topology-mutation, and arena-roundtrip campaigns for two minutes each and
retains crash artifacts. `arena_reader` and `wasm_batch` currently compile
in PR CI but are not scheduled; curve-intersection and offset-specific
campaigns remain separate S4 follow-ups.

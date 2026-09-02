# RFC 0006: Native swept analytic surfaces

Status: accepted design; implementation is staged as Open Kernel issues
O2.1b–e. This RFC is O2.1a. Its measured wildcard audit is
[rfc-0006-face-surface-wildcard-audit.md](rfc-0006-face-surface-wildcard-audit.md).

## Problem

STEP `SURFACE_OF_REVOLUTION` and `SURFACE_OF_LINEAR_EXTRUSION` already import,
but only elementary cases remain typed. A revolved line can collapse to a
plane, cylinder, or cone; a revolved circle to a sphere or torus; and a normal
circle extrusion to a cylinder. Every other bounded profile is lowered to an
exact rational NURBS surface in `step/reader.rs`. Geometry is retained, but the
construction, periodicity, profile, axis, and ability to re-export the same
STEP entity are lost.

Adding two `FaceSurface` variants is a large semantic ripple. An unhandled
variant can disappear into a wildcard and be skipped, treated as non-periodic,
projected as the input point, or sampled over an unrelated domain. The current
baseline has 92 production `match` expressions containing both a
`FaceSurface` pattern and a wildcard arm. The companion audit dispositions
every one before the variants exist.

## Goals

- Preserve general surfaces of revolution and linear extrusion as exact,
  first-class carriers from import through topology, queries, tessellation,
  measurement, transform, boolean disclosure, serialization, and STEP export.
- Preserve the parameterization currently used by the exact NURBS lowering so
  existing p-curves remain authoritative.
- Put common behavior behind delegates; no caller should infer periodicity or
  project a point by enumerating variants.
- Make unsupported construction, projection, intersection, and transform
  cases typed refusals. A new variant must never enter an old wildcard and
  silently produce a plausible result.
- Recognize only structurally proven NURBS twins. A failed recognition leaves
  the source NURBS untouched.

## Non-goals

- This RFC does not add ruled, tabulated-cylinder, offset, or general sweep
  surface variants.
- It does not make every swept surface pair a closed-form boolean pair.
- It does not reinterpret a swept profile's trim as a face boundary. Faces
  remain bounded by their authoritative loops and p-curves.
- It does not replace `EdgeCurve`, move topology into `remus-math`, or change
  existing elementary-surface parameterizations.

## Data model

### A self-contained profile, not a literal `EdgeCurve`

The implementation-plan shorthand was `profile: EdgeCurve`. Taken literally,
that violates two invariants:

1. `remus-math` cannot depend on `remus-topology` without reversing the layer
   graph needed by O2.1b.
2. `EdgeCurve::Line` stores no line at all; its endpoints in an `Edge` supply
   the geometry. A surface profile has no bounding edge from which to recover
   that placement.

The math layer therefore gains a self-contained `SweptCurve`:

```rust,ignore
pub enum SweptCurve {
    Line(Line3D),
    Circle(Circle3D),
    Ellipse(Ellipse3D),
    Hyperbola(Hyperbola3D),
    Parabola(Parabola3D),
    Nurbs(NurbsCurve),
}
```

This promotes the STEP reader's existing private `SweptProfile` rather than
inventing a second conversion. It represents the untrimmed carrier curve.
`TRIMMED_CURVE` wrappers are unwrapped to their basis as they are today: the
face loops, not the surface value, bound the patch.

`SweptCurve` supplies checked evaluation, first and second derivatives,
projection, natural domain, optional period, exact finite-span NURBS lowering,
finite-data validation, and a stable type tag. A topology `EdgeCurve` can be
converted only with the geometry it lacks supplied explicitly; there is no
blanket `From<EdgeCurve>` that could manufacture a line placement or trim.

### Surface structs and topology variants

`remus-math::surfaces` gains:

```rust,ignore
pub struct SurfaceOfRevolution {
    profile: SweptCurve,
    axis: Line3D,
}

pub struct SurfaceOfLinearExtrusion {
    profile: SweptCurve,
    direction: Vec3,
}
```

The extrusion `direction` is the complete STEP `VECTOR` (unit orientation
times magnitude), not a unit vector. Its magnitude is part of the v-parameter
scale and must round-trip. Constructors reject non-finite or zero directions,
non-finite profiles, and profiles that sweep no regular surface. Local poles
such as a sphere profile meeting its revolution axis are permitted; an entire
profile on the axis, or a line parallel to its extrusion direction, is not.

`FaceSurface` gains concrete variants:

```rust,ignore
Revolution(SurfaceOfRevolution),
LinearExtrusion(SurfaceOfLinearExtrusion),
```

These variants are exact and `is_analytic()` returns true. They are not members
of `AnalyticSurface`, whose closed-form intersection enum remains the quadric
set; `as_analytic()` therefore returns `None` for both.

## Parameterization, domains, and seams

The ordering deliberately matches the current STEP-to-NURBS lowering.

For an axis with origin `A`, unit direction `Z`, rotation `R_Z(u)`, and
profile `C(v)`:

```text
revolution:       S(u, v) = A + R_Z(u) (C(v) - A)
linear extrusion: S(u, v) = C(u) + v D
```

- Revolution `u` is the sweep angle in radians and has period `2π`. Its
  `u = 0` seam is the source profile itself. `v` is the profile parameter and
  also carries the profile's period when the profile is closed.
- Linear-extrusion `u` is the profile parameter. It carries the profile's
  optional period. `v` is dimensionless, non-periodic, and one unit advances
  by the stored full vector `D`.
- Surface domains remain unbounded carrier domains. A face's authoritative
  loops and p-curves select the finite patch. Code needing finite integration
  or tessellation bounds derives them from those loops and refuses if it
  cannot.
- Periodic coordinates are unwrapped to the branch selected by the boundary,
  exactly as the torus-band machinery does. Equality modulo a period is never
  used to collapse a non-empty full-period span to zero.

`FaceSurface` gains `u_period()` and `v_period()` delegates. Direct variant
lists such as "cylinder, cone, sphere, torus means periodic u" migrate to
those delegates. STEP generic `FACE_BOUND` reconstruction adds exact seams:

- Revolution u-winding: the stored profile curve over the two boundary
  vertices' authoritative v parameters.
- Linear-extrusion u-winding on a periodic profile: a line parallel to `D`.
- A doubly periodic revolution uses the winding axis to choose the profile
  seam or the swept orbit; an unsupported exact seam refuses with
  `swept_surface_seam_unsupported` without mutating topology.

## Evaluation, derivatives, normals, and projection

Both structs implement `ParametricSurface`. Revolution derivatives are the
axis cross the rotated radial vector for `S_u` and the rotated profile
derivative for `S_v`. Linear-extrusion derivatives are `C'(u)` and `D`.
Normals are the normalized cross product in parameter order. At a permitted
pole, the limiting profile derivative supplies the normal; a genuinely
singular patch returns a typed construction or evaluation error rather than a
fixed world-axis normal.

Projection reduces to a bounded one-dimensional solve after eliminating the
sweep parameter:

- Revolution minimizes squared distance in the `(axis coordinate, radial
  distance)` meridian, then chooses the sweep angle aligning radial vectors.
- Linear extrusion eliminates `v` by projecting `P - C(u)` onto `D`, then
  minimizes the remaining perpendicular distance over the profile parameter.

The solver uses the operation context's work budget, deterministic seed grid,
and safeguarded Newton refinement. It returns a qualified result with residual
and convergence status. `FaceSurface::project_point_checked` exposes the typed
result; the legacy `project_point -> Option` is only a compatibility adapter.
No new call site may replace projection failure with the input point, a domain
midpoint, or `(0, 0)`.

Principal curvatures are computed from exact first and second derivatives and
the shared fundamental-form routine. O2.1b differentially checks position,
partials, normal, and curvatures against an exact NURBS-lowered twin at domain
interior, seam-adjacent, reversed-span, large-coordinate, and pole-adjacent
samples.

## Delegate coverage

The new variants receive explicit arms in every `FaceSurface` delegate:

- `evaluate`, `normal`, `partial_u`, `partial_v`, and checked projection;
- `estimate_radius`, using the maximum profile distance from the revolution
  axis or the profile's finite-bound estimate for extrusion;
- `type_tag` (`"revolution"`, `"linear_extrusion"`), `is_planar`, and
  `is_analytic`;
- `u_period`, `v_period`, and conservative finite-patch bounds;
- exact NURBS lowering over caller-supplied finite parameter ranges.

`effective_plane_normal` remains plane-only. `as_analytic` remains the
quadric closed-form adapter. Their wildcard arms are intentional and named in
the audit.

## Exact NURBS lowering and recognition

Lowering is tensor-product exact over finite bounds:

- Revolution uses the existing nine-point rational quadratic rotation ring;
  weights are ring weight times profile weight.
- Linear extrusion uses a degree-1 sweep row with identical profile weights.
- Line, circle, ellipse, hyperbola, and parabola profiles first use exact
  finite-span rational forms. NURBS profiles retain their knots and weights.

Recognition in `geometry/src/convert/recognize_surface.rs` is proof-first:

1. Existing elementary recognition runs first, so planes, cylinders, cones,
   spheres, and tori keep their more specific types.
2. Linear extrusion recognizes the canonical tensor-product structure:
   degree one in the sweep axis, equal per-profile weights, and one constant
   displacement vector across every corresponding control point.
3. Revolution recognizes the canonical full-turn quadratic knot pattern,
   factorable ring/profile weights, one common axis, and circular control rings
   for every profile control point.
4. Every candidate is re-evaluated on a deterministic grid. Position and
   first-derivative residuals must fit the caller's scale-relative tolerance
   and orientation must agree. Ambiguity or any failed proof returns
   `NotRecognized` and leaves the NURBS unchanged.

The inverse conversion lives beside the existing surface-to-NURBS functions.
Round-trip tests cover native → NURBS → recognized native and the current STEP
lowering → recognized native. Recognition is not a substitute for preserving
the native type through copy, serialization, or STEP.

## Transform and copy semantics

Copy clones the carrier and all parameterization data exactly. Rigid motion,
reflection, and uniform scale transform the profile, axis, and extrusion
vector while preserving the native variant. Face reversal remains topology's
orientation authority.

For a general affine transform, preservation is allowed only when the
transformed construction is still the same class: revolution requires an
axis-preserving similarity on every radial plane; linear extrusion requires a
non-zero transformed `D` and an exactly transformable profile. Otherwise the
operation lowers the bounded face to exact NURBS before transforming. If a
finite exact lowering cannot be established, it refuses transactionally with
`swept_surface_transform_unsupported`.

## Tessellation, validation, and properties

O2.1c adds a structured band mesher. Boundary UV is unwrapped through the
period delegates; profile rows are adaptively sampled by chord and normal
deflection; revolution columns are sized by the largest radial distance in the
face's v range; extrusion rows use profile curvature. Boundary edge samples
are shared with the face grid. The generic NURBS mesher is an explicit,
disclosed fallback, never an implicit type erasure.

Finite validation checks the profile, axis/vector, evaluated derivatives, and
all face p-curves. Validation reports a degenerate metric or failed checked
projection with stable codes. The property integrator uses the same UV bounds,
period flags, and Gaussian integration used for existing curved faces. A
successful measure must be deflection-independent; tessellation is not the
volume oracle for a native swept face.

## Boolean and offset behavior

O2.1e adds disclosed pair dispatch. Initial exact pairs are
revolution×plane-through-axis, coaxial revolution×revolution, and
extrusion×plane parallel or normal to the sweep direction. Other supported
pairs lower a finite patch to the general NURBS marcher while retaining a
quality record that names the lowered pair. A pair with no bounded exact
lowering refuses with `swept_surface_pair_unsupported`; it does not enter mesh
fallback under an analytic claim.

Classifier, VF, and EF paths consume checked evaluate/project/normal delegates.
Same-domain and surface-equivalence tests receive structural native arms.
Offset may use the generic NURBS path only with disclosed lowering; routines
whose mathematics is explicitly plane/cylinder/cone remain narrow and refuse
or decline as their current contract specifies.

Any O2.1e boolean change runs the full workspace and `approx_census`; every
moved row is explained. Analytic inputs must remain low-face-count, typed, and
watertight, or refuse under `ExactOnly` without mutation.

## STEP, arena, and WASM contracts

O2.1d writes the native variants as `SURFACE_OF_REVOLUTION` and
`SURFACE_OF_LINEAR_EXTRUSION`, including the profile basis, `AXIS1_PLACEMENT`,
and the extrusion `VECTOR` magnitude. Import stops at the native variant after
the existing elementary collapses. Arena serialization adds tagged variants
under a versioned, backward-compatible reader; old documents remain
byte-stable when re-serialized without mutation.

Per R8, the capability is not complete at the Rust boundary. Direct and batch
WASM contracts must both cover STEP import/export, surface type query, point
projection, tessellation, and measurement for one revolution and one extrusion
fixture. They assert native tags, zero NURBS faces when the source had none,
stable typed failures on degenerate and budget-exhausted inputs, and unchanged
kernel state after refusal.

## Staging and exit gates

### O2.1b — math substrate

- Add `SweptCurve`, both surface structs, checked projection, periods,
  derivatives, curvatures, and exact finite NURBS lowering.
- Differential property tests cover supported profile variants, scale, seams,
  reversed spans, and both success/refusal sides.
- No topology variant changes in this stage.

### O2.1c — topology variants and consumers

- Add both `FaceSurface` variants and every compiler-required arm.
- Re-run the wildcard inventory and resolve every `ADD`/`DELEGATE` row in the
  companion audit; `NARROW` rows remain narrow with a code comment or an
  explicit typed-decline test where silent support would be plausible.
- Add structured tessellation, validation, properties, transform/copy, surface
  equality, and query coverage. Full workspace and WASM library suites pass.

### O2.1d — interchange

- Native STEP and arena round-trips preserve parameterization and type.
- A fixture containing both entities imports and re-exports with zero NURBS
  faces and byte-stable arena replay.
- Direct and batch WASM contract tests pass.

### O2.1e — boolean arms

- The named exact pairs are exact; general pairs are disclosed marcher paths
  or typed refusals on both operand orders.
- Native and WASM tests assert rollback on refusal, low face count, manifold
  topology, watertight tessellation, and independent property oracles.
- `approx_census` is run and every byte of movement is explained.

The O2.1 pillar closes only when real-corpus gauntlet runs show a reproducible
increase in native analytic faces, the zero-lowering round-trip fixture is
permanent, and the census diff is reviewed.

## Rejected alternatives

- **Store `EdgeCurve` directly.** Rejected because `Line` is not
  self-contained and because it would force the math layer to depend upward.
- **Keep exact NURBS lowering as the representation.** Geometry survives, but
  semantic type, periodicity, source construction, closed-form routing, and
  same-entity STEP export do not.
- **Treat every swept surface as a quadric analytic surface.** False for
  general profiles and would send unsupported pairs into incorrect closed
  forms.
- **Recognize from samples alone.** A sampled fit can upgrade a near-sweep to
  the wrong exact carrier. Structural proof plus residual verification is the
  acceptance rule.
- **Use mesh fallback for unsupported pairs.** It erases the exact carrier and
  violates the typed-or-exact contract.

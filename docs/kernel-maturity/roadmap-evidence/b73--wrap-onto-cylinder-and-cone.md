---
row: B73
---
Wrap solids (2026-11-17, PR #1847, merge
`b6b89017aef6f5e6370e0b207578523f04084b8b`, OpenZCAD M12; Tier: core):
`wrap::wrap_profile_solid` / direct and batch `wrapProfileOnCylinder` build the
solid between two radii, and `wrap::wrap_profile_on_cone` /
`wrapProfileOnCone` the solid between two normal offsets through the cone's
development, preserving arc length read along the profile plane. Inner and
outer faces are exact cylinders or cones; on a cylinder, edges along or across
the axis wall exact planes. Every other wrapped edge is a cubic B-spline
certified within 1e-7 of the wrap's scale by the Hermite remainder (the
fourth-derivative bound comes from the profile curve's derivative bounds, on a
cone through closed-form bounds of the homogeneous development map and Faà di
Bruno), splines are split at every interior knot, and each wall is the exact
ruled surface between an edge's inner and outer fits. Turning the reference
direction moves the wrap about the axis without scaling it. Tests:
`crates/operations/src/wrap/tests.rs` (a square to planes and cylinders exact
to 1e-12, an oblique parallelogram to certified helical walls, a disc and an
annulus, watertight meshes, a kinked spline wrapped span by span at its closed
form within 1e-6, a circle 1.0005 turns wide refused between samples, typed
refusals), `wrap/cone/tests.rs` (a developed annular sector within 1e-6 of its
closed form, a disc against a quadrature oracle, watertight meshes at 0.05 and
0.01, five typed refusals) and `crates/wasm/src/bindings/wrap_tests.rs` (both
bindings direct and through `executeBatch`; refusals leave the session
unchanged). Limits: `solid_volume` and `mass_properties` read a cone face with
a curved `(u, v)` trim through the face integrator's boundary chording, so a
disc wrapped on a cone reads 1.3e-4 low (a mesh at deflection 2e-4 converges
on the closed form; the test checks the volume only within that limit).
Rational curves other than circles and ellipses, profiles not parallel to the
axis, full-turn profiles, profiles reaching a cone's apex, offsets past an apex
and empty ranges are refused by name.

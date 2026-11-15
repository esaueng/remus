//! Exact moments of a cylinder face trimmed by exact curves, by Green's
//! theorem on its own boundary.
//!
//! The general trimmed path samples each trim edge into a 128-point
//! polygon in `(u, v)` and integrates the cells inside it. A curved trim
//! (the ellipses two equal crossing cylinders cut into each other) then
//! costs O(1/N²): a Steinmetz bicylinder r = 10 read 5333.2664 against
//! 16r³/3 = 5333.3333 (1.25e-5 low) although its B-rep is exact.
//!
//! Exact trims (lines, circles, ellipses) take this path, and so does a
//! fitted NURBS trim (the quartic section of unequal crossing cylinders)
//! whose every quadrature point lies on the cylinder within
//! [`NURBS_ON_SURFACE`] of its radius (and at least [`NURBS_ON_SURFACE_ABS`]). Each point is projected onto the
//! cylinder before it is integrated, so the face is measured on the trim the
//! B-rep stores, read on its own surface. A fitted trim further off the
//! surface than that has no boundary on the cylinder to integrate, and the
//! sampled path keeps measuring it.
//!
//! The sampled path cannot be trusted on such trims either: when two
//! unequal crossing cylinders cut a single closed loop (a drill r 7.3 at
//! 3 mm off the axis of a bar r 10, so the drill breaks out through the
//! bar's top), both trimmed walls read about 1.1 mm³ high and the total
//! moved with the bar's seam (2.16, 2.03 or 2.07 mm³ high, esaueng/OpenZCAD
//! K10). On its boundary the same faces read within 0.01 mm³.
//!
//! On a cylinder `P(u, v) = O + r ρ(u) + v a` every integrand the
//! [`Accumulator`] sums is a polynomial of degree at most three in `v`
//! times a function of `u`. With `F(u, v) = ∫₀ᵛ f(u, s) ds`,
//! `∬_D f du dv = −∮_{∂D} F du` (Green, the boundary run counter-clockwise
//! in `(u, v)`, which is how the kernel orients a face's wires on its own
//! surface). `F` is single-valued and periodic in `u`, so the identity also
//! holds for bands whose loops wind the axis, and a seam traversed both
//! ways cancels. Each edge is integrated by composite Gauss–Legendre on its
//! own parameter, `F` by three Gauss points in `s` (exact for cubics), and
//! `du/dt` is the edge velocity projected on `∂P/∂u`. No `u` unwrapping is
//! needed: `u` enters only through the periodic surface point.

use remus_math::nurbs::surface::DerivativeScratch;
use remus_math::quadrature::gauss_legendre_points;
use remus_math::surfaces::CylindricalSurface;
use remus_math::traits::{ParametricCurve, ParametricSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::Face;

use super::{Accumulator, FaceContribution, IntegrationRule};
use crate::CheckError;

/// Composite pieces per edge and Gauss points per piece. A conic trim's
/// integrand is a trigonometric polynomial of low order in its parameter (a
/// cubic on a line), resolved to round-off well before this.
const EDGE_PIECES: usize = 16;
const EDGE_ORDER: usize = 16;
/// Gauss points in the axial integration variable `s ∈ [0, 1]`.
const AXIAL_ORDER: usize = 3;
/// How far a fitted NURBS trim's quadrature points may lie off the cylinder
/// for the face to integrate on its boundary: the larger of
/// [`NURBS_ON_SURFACE_ABS`] and this fraction of the radius. Every point is
/// projected onto the cylinder before it is integrated, so the bound only
/// rejects trims that are not curves on this surface; the sampled path reads
/// the same projected trim. The kernel's crossing-cylinder sections sit up
/// to 1.4e-3 off a r 10 wall (1.4e-4 of the radius).
const NURBS_ON_SURFACE: f64 = 1e-3;
/// Absolute floor of the on-surface bound, in model units.
const NURBS_ON_SURFACE_ABS: f64 = 1e-3;

/// The edge's velocity `dP/dt` at `t` for an exact trim (line, circle,
/// ellipse); `None` for anything else.
fn velocity(curve: &EdgeCurve, t: f64, start: Point3, end: Point3) -> Option<Vec3> {
    match curve {
        EdgeCurve::Line => Some(end - start),
        EdgeCurve::Circle(c) => ParametricCurve::derivative_pair(c, t).map(|(d1, _)| d1),
        EdgeCurve::Ellipse(e) => ParametricCurve::derivative_pair(e, t).map(|(d1, _)| d1),
        // A fitted trim is read on the surface; `integrate` checks it lies
        // there (see the module docs).
        EdgeCurve::NurbsCurve(n) => ParametricCurve::derivative_pair(n, t).map(|(d1, _)| d1),
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => None,
    }
}

/// Equal quadrature pieces of `[lo, hi]`.
fn pieces(lo: f64, hi: f64) -> impl Iterator<Item = (f64, f64)> {
    let step = (hi - lo) / EDGE_PIECES as f64;
    (0..EDGE_PIECES).map(move |k| (step.mul_add(k as f64, lo), step.mul_add((k + 1) as f64, lo)))
}

/// The face's contribution by Green's theorem, with its vector area when the
/// boundary is exact.
///
/// The vector area is `None` for a face with a fitted NURBS trim. Read on
/// the cylinder, such a trim sits up to its fit error off the neighbouring
/// face's own edge, so this face's vector area no longer cancels against
/// its neighbours' to the closure certificate's tolerance (a filleted
/// off-seam cross drill closed to 3.2e-5 of its area). The caller keeps the
/// sampled reading's vector area then: it closes as it always has, and only
/// the volume comes from the boundary.
///
/// `None` when a boundary edge is not a line, circle, ellipse or on-surface
/// NURBS curve, has no stored domain, or the boundary does not
/// enclose positive area (an orientation outside the convention this relies
/// on): the caller keeps its sampled path then.
pub(super) fn integrate<const AREA_ONLY: bool>(
    topo: &Topology,
    face: &Face,
    cylinder: &CylindricalSurface,
    rule: &IntegrationRule<'_>,
    sign: f64,
) -> Result<Option<(FaceContribution, Option<Vec3>)>, CheckError> {
    let mut acc = Accumulator {
        volume_only: rule.volume_only,
        ..Accumulator::default()
    };
    let mut scratch = DerivativeScratch::new();
    let edge_points = gauss_legendre_points(EDGE_ORDER);
    let axial_points = gauss_legendre_points(AXIAL_ORDER);
    let Ok(axis) = cylinder.axis().normalize() else {
        return Ok(None);
    };
    let off_surface_limit = (NURBS_ON_SURFACE * cylinder.radius()).max(NURBS_ON_SURFACE_ABS);
    let mut fitted_trim = false;

    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            let (start, end) = (
                topo.vertex(edge.start())?.point(),
                topo.vertex(edge.end())?.point(),
            );
            let curve = edge.curve();
            let Ok((t0, t1)) = edge.strict_domain() else {
                return Ok(None);
            };
            let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
            // A reversed use runs the domain backwards.
            let direction = if oriented.is_forward() == (t0 <= t1) {
                1.0
            } else {
                -1.0
            };
            for (p0, p1) in pieces(lo, hi) {
                let half = (p1 - p0) / 2.0;
                let mid = p0 + half;
                for gp in edge_points {
                    let t = half.mul_add(gp.x, mid);
                    let point = curve.evaluate_with_endpoints(t, start, end);
                    let Some(dp_dt) = velocity(curve, t, start, end) else {
                        return Ok(None);
                    };
                    if matches!(curve, EdgeCurve::NurbsCurve(_)) {
                        fitted_trim = true;
                        let offset = point - cylinder.origin();
                        let radial = offset - axis * offset.dot(axis);
                        if (radial.length() - cylinder.radius()).abs() > off_surface_limit {
                            return Ok(None);
                        }
                    }
                    let (u, v) = cylinder.project_point(point);
                    let (_, du_vec, _, _, _) =
                        cylinder.span_hinted_point_and_partials_with_scratch(u, v, &mut scratch);
                    let du_dt = dp_dt.dot(du_vec) / du_vec.dot(du_vec);
                    // −∮ F du, F(u, v) = v ∫₀¹ f(u, v s) ds.
                    let line_weight = -(direction * gp.w * half) * du_dt * v;
                    for sp in axial_points {
                        let s = 0.5 * (sp.x + 1.0);
                        acc.add::<_, AREA_ONLY>(
                            cylinder,
                            u,
                            v * s,
                            line_weight * 0.5 * sp.w,
                            rule.reference,
                            &mut scratch,
                        );
                    }
                }
            }
        }
    }
    if !acc.area.is_finite() || acc.area <= 0.0 {
        return Ok(None);
    }
    let flux = (!fitted_trim).then(|| acc.flux(sign));
    Ok(Some((acc.finish(sign), flux)))
}

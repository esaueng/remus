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
//! [`NURBS_ON_SURFACE`] of its radius (and at least [`NURBS_ON_SURFACE_ABS`]).
//! Such a trim is read on the surface: each point is projected onto the
//! cylinder and its angular rate taken at the projected radius, so the face
//! is measured on the trim the B-rep stores. A fitted trim further off the
//! surface has no boundary on the cylinder to integrate, and the sampled
//! path keeps measuring it.
//!
//! Fitted trims are read here only for a moment (not area-only) reading,
//! and only when the caller does not collect the
//! vector area. The callers that do (the shell-closure and order-convergence
//! certificates of `solid_volume`) qualify the reading they are handed, and a
//! fitted trim read on the surface sits up to its fit error off the
//! neighbouring face's own edge, so its vector area would not cancel to their
//! tolerance. They keep the sampled reading and its checks unchanged.
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
/// Relative agreement a fitted trim's span-by-span and halved-span readings
/// must reach in every term (see `readings_agree`).
const FITTED_CONVERGENCE: f64 = 1e-9;

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

/// `[lo, hi]` cut at every distinct interior knot, each knot span then into
/// `per_span` equal pieces.
fn knot_pieces(knots: &[f64], lo: f64, hi: f64, per_span: usize) -> Vec<(f64, f64)> {
    let mut breaks = vec![lo];
    for &knot in knots {
        if knot > lo && knot < hi && breaks.last().is_none_or(|&last| knot > last) {
            breaks.push(knot);
        }
    }
    breaks.push(hi);
    breaks
        .windows(2)
        .flat_map(|w| pieces(w[0], w[1], per_span))
        .collect()
}

/// `count` equal quadrature pieces of `[lo, hi]`.
fn pieces(lo: f64, hi: f64, count: usize) -> impl Iterator<Item = (f64, f64)> {
    let step = (hi - lo) / count as f64;
    (0..count).map(move |k| (step.mul_add(k as f64, lo), step.mul_add((k + 1) as f64, lo)))
}

/// The face's contribution and vector area by Green's theorem.
///
/// `None` when a boundary edge is not a line, circle or ellipse (or, for a
/// caller that does not collect the vector area, an on-surface NURBS curve;
/// see the module docs), has no stored domain, or the boundary does not
/// enclose positive area (an orientation outside the convention this relies
/// on): the caller keeps its sampled path then.
pub(super) fn integrate<const AREA_ONLY: bool>(
    topo: &Topology,
    face: &Face,
    cylinder: &CylindricalSurface,
    rule: &IntegrationRule<'_>,
    sign: f64,
) -> Result<Option<(FaceContribution, Vec3)>, CheckError> {
    let fresh = || Accumulator {
        volume_only: rule.volume_only,
        ..Accumulator::default()
    };
    let mut acc = fresh();
    let Ok(axis) = cylinder.axis().normalize() else {
        return Ok(None);
    };
    let reading = Reading {
        cylinder,
        axis,
        reference: rule.reference,
        off_surface_limit: (NURBS_ON_SURFACE * cylinder.radius()).max(NURBS_ON_SURFACE_ABS),
    };
    // Fitted trims only for a moment reading no certificate qualifies (module
    // docs). An area-only reading keeps the sampled path: the area entry point
    // (`analytic_face_area`) already reduces fitted trims to the boundary.
    let fitted_trims_allowed = !AREA_ONLY && rule.flux.is_none();
    // Every term compared at its own dimension of this length: the radius,
    // plus how far the integration reference lies from the cylinder.
    let length_scale = cylinder.radius() + (cylinder.origin() - rule.reference).length();

    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            let span = EdgeSpan {
                curve: edge.curve(),
                start: topo.vertex(edge.start())?.point(),
                end: topo.vertex(edge.end())?.point(),
            };
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
            if !matches!(span.curve, EdgeCurve::NurbsCurve(_)) {
                let intervals: Vec<_> = pieces(lo, hi, EDGE_PIECES).collect();
                if !reading.edge::<AREA_ONLY>(&span, direction, &intervals, &mut acc) {
                    return Ok(None);
                }
                continue;
            }
            if !fitted_trims_allowed {
                return Ok(None);
            }
            // A fitted trim is a spline: the rule is exact-to-round-off only
            // inside one knot span, and a parameterization that crowds its
            // turn into a few spans is under-resolved by equal pieces. Read
            // it span by span, then with every span halved, and keep it only
            // where the two agree.
            let EdgeCurve::NurbsCurve(nurbs) = span.curve else {
                return Ok(None);
            };
            let (coarse_pieces, fine_pieces) = (
                knot_pieces(nurbs.knots(), lo, hi, 1),
                knot_pieces(nurbs.knots(), lo, hi, 2),
            );
            let (mut coarse, mut fine) = (fresh(), fresh());
            if !reading.edge::<AREA_ONLY>(&span, direction, &coarse_pieces, &mut coarse)
                || !reading.edge::<AREA_ONLY>(&span, direction, &fine_pieces, &mut fine)
            {
                return Ok(None);
            }
            let converged = readings_agree(&coarse, &fine, length_scale);
            if !converged {
                return Ok(None);
            }
            acc.absorb(&fine);
        }
    }
    if !acc.area.is_finite() || acc.area <= 0.0 {
        return Ok(None);
    }
    let flux = acc.flux(sign);
    Ok(Some((acc.finish(sign), flux)))
}

/// Whether two readings of one fitted edge agree in every term they carry
/// to [`FITTED_CONVERGENCE`], each against its own dimension of `length`:
/// area and vector area as length², volume and the surface-centroid sums as
/// length³, the first volume moments as length⁴ and the second as length⁵.
/// Comparing every term, not only area and volume, keeps a reading whose
/// higher moments are under-resolved from passing on a lucky cancellation.
fn readings_agree(coarse: &Accumulator, fine: &Accumulator, length: f64) -> bool {
    let close = |a: f64, b: f64, power: i32| {
        (a - b).abs() <= FITTED_CONVERGENCE * (b.abs() + length.powi(power))
    };
    close(coarse.area, fine.area, 2)
        && close(coarse.nx, fine.nx, 2)
        && close(coarse.ny, fine.ny, 2)
        && close(coarse.nz, fine.nz, 2)
        && close(coarse.vol, fine.vol, 3)
        && close(coarse.cx, fine.cx, 3)
        && close(coarse.cy, fine.cy, 3)
        && close(coarse.cz, fine.cz, 3)
        && close(coarse.mx, fine.mx, 4)
        && close(coarse.my, fine.my, 4)
        && close(coarse.mz, fine.mz, 4)
        && close(coarse.qxx, fine.qxx, 5)
        && close(coarse.qyy, fine.qyy, 5)
        && close(coarse.qzz, fine.qzz, 5)
        && close(coarse.qxy, fine.qxy, 5)
        && close(coarse.qxz, fine.qxz, 5)
        && close(coarse.qyz, fine.qyz, 5)
}

/// One boundary edge's curve and end points.
struct EdgeSpan<'a> {
    curve: &'a EdgeCurve,
    start: Point3,
    end: Point3,
}

/// What every edge of one face is read against.
struct Reading<'a> {
    cylinder: &'a CylindricalSurface,
    axis: Vec3,
    reference: Point3,
    off_surface_limit: f64,
}

impl Reading<'_> {
    /// Add `−∮ F du` along `span` over `intervals` (run in `direction`), one
    /// Gauss–Legendre rule per interval. `false` when the edge is not a curve
    /// this path reads, or a fitted point lies off the surface.
    fn edge<const AREA_ONLY: bool>(
        &self,
        span: &EdgeSpan<'_>,
        direction: f64,
        intervals: &[(f64, f64)],
        acc: &mut Accumulator,
    ) -> bool {
        let mut scratch = DerivativeScratch::new();
        let edge_points = gauss_legendre_points(EDGE_ORDER);
        let axial_points = gauss_legendre_points(AXIAL_ORDER);
        let cylinder = self.cylinder;
        for &(p0, p1) in intervals {
            let half = (p1 - p0) / 2.0;
            let mid = p0 + half;
            for gp in edge_points {
                let t = half.mul_add(gp.x, mid);
                let point = span.curve.evaluate_with_endpoints(t, span.start, span.end);
                let Some(dp_dt) = velocity(span.curve, t, span.start, span.end) else {
                    return false;
                };
                // The angular rate of the projected point: ∂P/∂u has the
                // radius's length, so a point at radial distance ρ moves in u
                // at r/ρ times `dp·∂P/∂u / |∂P/∂u|²`. Exact trims lie on the
                // surface (ρ = r) and keep the plain quotient.
                let rate_scale = if matches!(span.curve, EdgeCurve::NurbsCurve(_)) {
                    let offset = point - cylinder.origin();
                    let rho = (offset - self.axis * offset.dot(self.axis)).length();
                    if (rho - cylinder.radius()).abs() > self.off_surface_limit {
                        return false;
                    }
                    cylinder.radius() / rho
                } else {
                    1.0
                };
                let (u, v) = cylinder.project_point(point);
                let (_, du_vec, _, _, _) =
                    cylinder.span_hinted_point_and_partials_with_scratch(u, v, &mut scratch);
                let du_dt = rate_scale * dp_dt.dot(du_vec) / du_vec.dot(du_vec);
                // −∮ F du, F(u, v) = v ∫₀¹ f(u, v s) ds.
                let line_weight = -(direction * gp.w * half) * du_dt * v;
                for sp in axial_points {
                    let s = 0.5 * (sp.x + 1.0);
                    acc.add::<_, AREA_ONLY>(
                        cylinder,
                        u,
                        v * s,
                        line_weight * 0.5 * sp.w,
                        self.reference,
                        &mut scratch,
                    );
                }
            }
        }
        true
    }
}

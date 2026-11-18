//! Exact moments of a cone face trimmed by fitted curves on the cone, by
//! Green's theorem on its own boundary.
//!
//! The cone counterpart of [`super::cylinder_green`]. The general trimmed
//! path samples each trim edge into a polygon in `(u, v)` and integrates the
//! cells inside it, so a trim that is curved in `(u, v)` is chorded: a disc
//! wrapped onto a cone (`wrap_profile_on_cone`, whose rims are fitted NURBS
//! curves on two offset cones) read its volume 1.3e-4 low that way.
//!
//! Only faces with at least one fitted NURBS trim take this path, and only
//! for a moment (not area-only) reading whose caller does not collect the
//! vector area, the same policy the cylinder applies to fitted trims (see
//! its module docs). A face trimmed only by lines, circles and ellipses keeps
//! the sampled reading bit for bit, and so does every certificate reading of
//! `solid_volume`. The area entry point (`analytic_face_area`) already reduces
//! these trims to the boundary.
//!
//! On a cone `P(u, v) = A + v (cos a ρ(u) + sin a â)` the surface Jacobian
//! `∂P/∂u × ∂P/∂v` is `v` times a function of `u` and `P` is affine in `v`,
//! so every integrand the [`Accumulator`] sums is a polynomial of degree at
//! most four in `v` times a function of `u` (the second moments: `P³` times
//! the Jacobian). With `F(u, v) = ∫₀ᵛ f(u, s) ds`,
//! `∬_D f du dv = −∮_{∂D} F du` (Green, the boundary run counter-clockwise
//! in `(u, v)` about the outward normal, the kernel's wire convention).
//! `F` vanishes at the apex (`v = 0`), so a pointed face, whose `(u, v)`
//! domain runs down to the apex with no edge there, closes through it; `F`
//! is periodic in `u`, so a seam traversed both ways cancels. `F` takes
//! three Gauss points in `s` (exact for quintics), each edge composite
//! Gauss–Legendre on its own parameter.
//!
//! Each fitted point is read on the cone at its own longitude: its angular
//! rate is `dP/dt · t̂(u) / ρ` with `ρ` its own distance from the axis, and
//! its `v` the cone's projection. A point further than [`NURBS_ON_SURFACE`]
//! of its radius (and [`NURBS_ON_SURFACE_ABS`]) off the cone, at or beyond
//! the apex (`v ≤ 0`, where the projection does not invert the chart), or
//! within [`APEX_CLEARANCE`] of the axis declines the path. A fitted edge is
//! read span by span and again with every span halved, and kept only where
//! every term of the two readings agrees.

use remus_math::nurbs::surface::DerivativeScratch;
use remus_math::quadrature::gauss_legendre_points;
use remus_math::surfaces::ConicalSurface;
use remus_math::traits::ParametricSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::Face;

use super::cylinder_green::{
    AXIAL_ORDER, EDGE_ORDER, EDGE_PIECES, EdgeSpan, NURBS_ON_SURFACE, NURBS_ON_SURFACE_ABS,
    knot_pieces, pieces, readings_agree, velocity,
};
use super::{Accumulator, FaceContribution, IntegrationRule};
use crate::CheckError;

/// How close to the axis, relative to the face's length scale, a boundary
/// quadrature point may come before the face keeps the sampled path: near
/// the apex the angular rate is singular and the edge rule loses accuracy.
const APEX_CLEARANCE: f64 = 1e-6;

/// The face's contribution and vector area by Green's theorem.
///
/// `None` when the face has no fitted (NURBS) trim, when the caller collects
/// the vector area or reads area only, when a boundary edge is a hyperbola or
/// parabola or has no stored domain, when a boundary point leaves the cone
/// or nears its apex, when a fitted edge's two readings disagree, or when the
/// boundary does not enclose positive area: the caller keeps its sampled
/// path then.
pub(super) fn integrate<const AREA_ONLY: bool>(
    topo: &Topology,
    face: &Face,
    cone: &ConicalSurface,
    rule: &IntegrationRule<'_>,
    sign: f64,
) -> Result<Option<(FaceContribution, Vec3)>, CheckError> {
    // Fitted trims only, and only for a moment reading no certificate
    // qualifies (module docs).
    if AREA_ONLY || rule.flux.is_some() {
        return Ok(None);
    }
    let wires: Vec<_> = std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .collect();
    let mut fitted = false;
    let mut reach = 0.0_f64;
    for &wire_id in &wires {
        for oriented in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            fitted |= matches!(edge.curve(), EdgeCurve::NurbsCurve(_));
            for vertex in [edge.start(), edge.end()] {
                reach = reach.max((topo.vertex(vertex)?.point() - cone.apex()).length());
            }
        }
    }
    if !(fitted && reach.is_finite() && reach > 0.0) {
        return Ok(None);
    }
    let fresh = || Accumulator {
        volume_only: rule.volume_only,
        ..Accumulator::default()
    };
    let mut acc = fresh();
    // Every term compared at its own dimension of this length: how far the
    // boundary reaches from the apex, plus how far the integration reference
    // lies from it.
    let length_scale = reach + (cone.apex() - rule.reference).length();
    let reading = Reading {
        cone,
        reference: rule.reference,
        axis_clearance: APEX_CLEARANCE * length_scale,
    };

    for wire_id in wires {
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
            let EdgeCurve::NurbsCurve(nurbs) = span.curve else {
                let intervals: Vec<_> = pieces(lo, hi, EDGE_PIECES).collect();
                if !reading.edge::<AREA_ONLY>(&span, direction, &intervals, &mut acc) {
                    return Ok(None);
                }
                continue;
            };
            // As on the cylinder: span by span, then every span halved, and
            // kept only where the two agree.
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
            if !readings_agree(&coarse, &fine, length_scale) {
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

/// What every edge of one face is read against.
struct Reading<'a> {
    cone: &'a ConicalSurface,
    reference: Point3,
    axis_clearance: f64,
}

impl Reading<'_> {
    /// Add `−∮ F du` along `span` over `intervals` (run in `direction`), one
    /// Gauss–Legendre rule per interval. `false` when the edge is not a curve
    /// this path reads, or a point lies off the cone, at or beyond its apex,
    /// or too near its axis.
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
        let cone = self.cone;
        let axis = cone.axis();
        let (sin_a, cos_a) = cone.half_angle().sin_cos();
        let fitted = matches!(span.curve, EdgeCurve::NurbsCurve(_));
        for &(p0, p1) in intervals {
            let half = (p1 - p0) / 2.0;
            let mid = p0 + half;
            for gp in edge_points {
                let t = half.mul_add(gp.x, mid);
                let point = span.curve.evaluate_with_endpoints(t, span.start, span.end);
                let Some(dp_dt) = velocity(span.curve, t, span.start, span.end) else {
                    return false;
                };
                let offset = point - cone.apex();
                let height = offset.dot(axis);
                let rho = (offset - axis * height).length();
                if rho.partial_cmp(&self.axis_clearance) != Some(std::cmp::Ordering::Greater) {
                    return false;
                }
                let (u, v) = cone.project_point(point);
                if v.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
                    return false;
                }
                // Distance from the generator through the point's own
                // meridian.
                if fitted
                    && (rho * sin_a - height * cos_a).abs()
                        > (NURBS_ON_SURFACE * rho).max(NURBS_ON_SURFACE_ABS)
                {
                    return false;
                }
                let (_, du_vec, _, _, _) =
                    cone.span_hinted_point_and_partials_with_scratch(u, v, &mut scratch);
                // ∂P/∂u has the chart radius `v cos a` as its length; a point
                // at its own radius ρ moves in u at that radius over ρ times
                // the plain quotient. Exact trims keep the plain quotient.
                let rate_scale = if fitted { du_vec.length() / rho } else { 1.0 };
                let du_dt = rate_scale * dp_dt.dot(du_vec) / du_vec.dot(du_vec);
                // −∮ F du, F(u, v) = v ∫₀¹ f(u, v s) ds.
                let line_weight = -(direction * gp.w * half) * du_dt * v;
                for sp in axial_points {
                    let s = 0.5 * (sp.x + 1.0);
                    acc.add::<_, AREA_ONLY>(
                        cone,
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

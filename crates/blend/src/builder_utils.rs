//! Shared utilities for fillet and chamfer builders.
//!
//! Functions used by both [`FilletBuilder`](crate::fillet_builder::FilletBuilder)
//! and [`ChamferBuilder`](crate::chamfer_builder::ChamferBuilder) for creating
//! blend faces and sampling contact curves.

use brepkit_math::nurbs::curve::NurbsCurve;
use brepkit_math::traits::ParametricSurface;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::{Edge, EdgeCurve};
use brepkit_topology::face::{Face, FaceId, FaceSurface};
use brepkit_topology::vertex::{Vertex, VertexId};
use brepkit_topology::wire::{OrientedEdge, Wire, WireId};

use crate::BlendError;
use crate::stripe::Stripe;

/// Sample the start and end points of a NURBS curve.
#[must_use]
pub fn sample_nurbs_endpoints(curve: &NurbsCurve) -> Vec<Point3> {
    let (t0, t1) = curve.domain();
    vec![curve.evaluate(t0), curve.evaluate(t1)]
}

fn contact_geometry_matches(
    edge_curve: &EdgeCurve,
    edge_start: Point3,
    edge_end: Point3,
    want_curve: &NurbsCurve,
    tolerance: f64,
) -> bool {
    if matches!(edge_curve, EdgeCurve::Line) {
        let chord = edge_end - edge_start;
        let chord_len_sq = chord.dot(chord);
        if chord_len_sq <= f64::EPSILON {
            return want_curve
                .control_points()
                .iter()
                .all(|point| (*point - edge_start).length() <= tolerance);
        }

        return want_curve.control_points().iter().all(|point| {
            let offset = *point - edge_start;
            let parameter = offset.dot(chord) / chord_len_sq;
            let distance = (offset - chord * parameter).length();
            distance <= tolerance && parameter >= -tolerance && parameter <= 1.0 + tolerance
        });
    }

    let (t0, t1) = want_curve.domain();
    let want_mid = want_curve.evaluate((t0 + t1) * 0.5);
    let edge_mid = edge_curve.evaluate_with_endpoints(0.5, edge_start, edge_end);
    (edge_mid - want_mid).length() <= tolerance
}

/// Create a blend face from a stripe's surface and contact curves.
///
/// Builds a minimal quadrilateral wire from the four contact-curve endpoints
/// and associates the blend surface with it.
///
/// # Errors
///
/// Returns [`BlendError`] if wire or face construction fails.
/// [`create_blend_face`] that REUSES the trimmers' contact edges when they
/// span the same contacts. Minting fresh edges for curves the trimmed
/// neighbours already carry leaves two edge entities per contact — each used
/// by one face — opening the shell along every blend flank. A trimmer edge
/// is adopted (with its vertices) when its endpoints and geometry match the
/// stripe's contact curve within the weld band, in either orientation;
/// otherwise that side falls back to a fresh edge.
pub fn create_blend_face_with_contacts(
    topo: &mut Topology,
    stripe: &Stripe,
    contact1_edge: Option<brepkit_topology::edge::EdgeId>,
    contact2_edge: Option<brepkit_topology::edge::EdgeId>,
) -> Result<BlendFaceInfo, BlendError> {
    const WELD: f64 = 1e-5;
    let (t0_1, t1_1) = stripe.contact1.domain();
    let (t0_2, t1_2) = stripe.contact2.domain();

    let p1_start = stripe.contact1.evaluate(t0_1);
    let p1_end = stripe.contact1.evaluate(t1_1);
    let p2_start = stripe.contact2.evaluate(t0_2);
    let p2_end = stripe.contact2.evaluate(t1_2);

    // Adopt a trimmer contact edge when its endpoints match `(want_s, want_e)`
    // in either orientation: returns (edge, forward, start_vid, end_vid) in
    // the WIRE traversal direction.
    let adopt = |topo: &Topology,
                 eid: Option<brepkit_topology::edge::EdgeId>,
                 want_s: Point3,
                 want_e: Point3,
                 want_curve: &NurbsCurve|
     -> Option<(brepkit_topology::edge::EdgeId, bool, VertexId, VertexId)> {
        let eid = eid?;
        let e = topo.edge(eid).ok()?;
        let (sv, ev) = (e.start(), e.end());
        let sp = topo.vertex(sv).ok()?.point();
        let ep = topo.vertex(ev).ok()?.point();
        if !contact_geometry_matches(e.curve(), sp, ep, want_curve, WELD) {
            return None;
        }
        if (sp - want_s).length() <= WELD && (ep - want_e).length() <= WELD {
            Some((eid, true, sv, ev))
        } else if (sp - want_e).length() <= WELD && (ep - want_s).length() <= WELD {
            Some((eid, false, ev, sv))
        } else {
            None
        }
    };
    let adopt1 = adopt(topo, contact1_edge, p1_start, p1_end, &stripe.contact1);
    // Contact 2 traverses end -> start in the quad below.
    let adopt2 = adopt(topo, contact2_edge, p2_end, p2_start, &stripe.contact2);

    // Create/reuse vertices (snapshot then allocate).
    let (v1s, v1e) = adopt1.map_or_else(
        || {
            (
                topo.add_vertex(Vertex::new(p1_start, 1e-7)),
                topo.add_vertex(Vertex::new(p1_end, 1e-7)),
            )
        },
        |(_, _, s, e)| (s, e),
    );
    let (v2e, v2s) = adopt2.map_or_else(
        || {
            (
                topo.add_vertex(Vertex::new(p2_end, 1e-7)),
                topo.add_vertex(Vertex::new(p2_start, 1e-7)),
            )
        },
        |(_, _, s, e)| (s, e),
    );

    // Build quad: p1_start -> p1_end -> p2_end -> p2_start -> p1_start.
    // Use actual contact curves for e0 and e2 (the longitudinal edges along
    // the spine direction). Cross edges e1 and e3 are straight lines connecting
    // the two contact curves at the spine endpoints.
    let (e0, e0_fwd) = adopt1.map_or_else(
        || {
            (
                topo.add_edge(Edge::new(
                    v1s,
                    v1e,
                    EdgeCurve::NurbsCurve(stripe.contact1.clone()),
                )),
                true,
            )
        },
        |(eid, fwd, _, _)| (eid, fwd),
    );
    // Cross edges carry the true end cross-section arcs when the stripe has
    // sections: the fillet's end profile is a circular arc, and a straight
    // chord both misrepresents the surface boundary and can never be shared
    // with a notched end cap. The arc's plane normal comes from the two
    // contact endpoints and the section centre.
    let arc_curve =
        |sec: &crate::section::CircSection, a: Point3, b: Point3| -> Option<EdgeCurve> {
            let u = a - sec.center;
            let v = b - sec.center;
            let n = u.cross(v);
            let n = n.normalize().ok()?;
            let circle = brepkit_math::curves::Circle3D::new(sec.center, n, sec.radius).ok()?;
            Some(EdgeCurve::Circle(circle))
        };
    let end_curve = stripe
        .sections
        .last()
        .and_then(|sec| arc_curve(sec, p1_end, p2_end))
        .unwrap_or(EdgeCurve::Line);
    let start_curve = stripe
        .sections
        .first()
        .and_then(|sec| arc_curve(sec, p2_start, p1_start))
        .unwrap_or(EdgeCurve::Line);
    let e1 = topo.add_edge(Edge::new(v1e, v2e, end_curve));
    let (e2, e2_fwd) = adopt2.map_or_else(
        || {
            (
                topo.add_edge(Edge::new(
                    v2e,
                    v2s,
                    EdgeCurve::NurbsCurve(stripe.contact2.clone()),
                )),
                true,
            )
        },
        |(eid, fwd, _, _)| (eid, fwd),
    );
    let e3 = topo.add_edge(Edge::new(v2s, v1s, start_curve));

    let wire = Wire::new(
        vec![
            OrientedEdge::new(e0, e0_fwd),
            OrientedEdge::new(e1, true),
            OrientedEdge::new(e2, e2_fwd),
            OrientedEdge::new(e3, true),
        ],
        true,
    )?;
    let wire_id = topo.add_wire(wire);

    let face = Face::new(wire_id, Vec::new(), stripe.surface.clone());
    let face_id = topo.add_face(face);

    Ok(BlendFaceInfo {
        face: face_id,
        cross_end: (e1, v1e, v2e),
        cross_start: (e3, v2s, v1s),
    })
}

/// A created blend face plus its two cross edges (the end cross-section
/// arcs), each with its (from, to) vertices in the blend wire's traversal
/// direction — the handles the end-cap notch surgery needs to SHARE those
/// arcs instead of leaving both sides use-1.
pub struct BlendFaceInfo {
    /// The blend face.
    pub face: FaceId,
    /// Cross edge at the spine end: `(edge, from, to)`.
    pub cross_end: (brepkit_topology::edge::EdgeId, VertexId, VertexId),
    /// Cross edge at the spine start: `(edge, from, to)`.
    pub cross_start: (brepkit_topology::edge::EdgeId, VertexId, VertexId),
}

/// Replace a face's two-edge corner path `from -> corner -> to` with the
/// single cross-section arc `edge`, notching the fillet's end profile out of
/// an end cap so the cap and the blend share one edge entity. Both replaced
/// edges must be straight (the box corner sides); returns whether a
/// replacement happened.
pub fn notch_face_corner_with_arc(
    topo: &mut Topology,
    face_id: FaceId,
    arc: (brepkit_topology::edge::EdgeId, VertexId, VertexId),
) -> Result<Option<FaceId>, BlendError> {
    let (arc_eid, va, vb) = arc;
    let wire_id = topo.face(face_id)?.outer_wire();
    let oes = topo.wire(wire_id)?.edges().to_vec();
    let n = oes.len();
    if n < 3 {
        return Ok(None);
    }
    let ends = |oe: &OrientedEdge| -> Result<(VertexId, VertexId), BlendError> {
        let e = topo.edge(oe.edge())?;
        Ok((oe.oriented_start(e), oe.oriented_end(e)))
    };
    for i in 0..n {
        let j = (i + 1) % n;
        let (s0, e0) = ends(&oes[i])?;
        let (s1, e1) = ends(&oes[j])?;
        if e0 != s1 || e0 == va || e0 == vb {
            continue;
        }
        let fwd = s0 == va && e1 == vb;
        let rev = s0 == vb && e1 == va;
        if !(fwd || rev) {
            continue;
        }
        let both_straight = [oes[i].edge(), oes[j].edge()].iter().all(|&eid| {
            topo.edge(eid)
                .is_ok_and(|e| matches!(e.curve(), EdgeCurve::Line))
        });
        if !both_straight {
            continue;
        }
        let mut new_oes: Vec<OrientedEdge> = Vec::with_capacity(n - 1);
        for (k, oe) in oes.iter().enumerate() {
            if k == i {
                new_oes.push(OrientedEdge::new(arc_eid, fwd));
            } else if k != j {
                new_oes.push(*oe);
            }
        }
        let new_wire = topo.add_wire(Wire::new(new_oes, true)?);
        let (surface, reversed, inners) = {
            let f = topo.face(face_id)?;
            (
                f.surface().clone(),
                f.is_reversed(),
                f.inner_wires().to_vec(),
            )
        };
        let new_face = if reversed {
            Face::new_reversed(new_wire, inners, surface)
        } else {
            Face::new(new_wire, inners, surface)
        };
        let nf = topo.add_face(new_face);
        return Ok(Some(nf));
    }
    Ok(None)
}

/// Adapter that provides [`ParametricSurface`] for a `FaceSurface::Plane`.
///
/// Planes store only a normal and signed distance `d`, with no parametric
/// frame.  This adapter builds an orthonormal UV frame from the normal so
/// that the walking engine can evaluate, project, and differentiate the
/// plane surface uniformly.
pub struct PlaneAdapter {
    /// Origin point on the plane (the point closest to the world origin).
    pub origin: Point3,
    /// U-direction tangent (unit vector in the plane).
    pub u_dir: Vec3,
    /// V-direction tangent (unit vector in the plane, orthogonal to `u_dir`).
    pub v_dir: Vec3,
    /// Outward-facing unit normal.
    pub norm: Vec3,
}

impl PlaneAdapter {
    /// Build a `PlaneAdapter` from a plane normal and signed distance.
    ///
    /// The UV frame is constructed by choosing a non-parallel reference vector
    /// and computing the cross products.
    #[must_use]
    pub fn from_normal_and_d(normal: Vec3, d: f64) -> Self {
        let origin = Point3::new(normal.x() * d, normal.y() * d, normal.z() * d);

        // Pick a reference vector that is not parallel to the normal.
        let ref_vec = if normal.x().abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };

        let u_dir = normal
            .cross(ref_vec)
            .normalize()
            .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
        let v_dir = normal
            .cross(u_dir)
            .normalize()
            .unwrap_or(Vec3::new(0.0, 1.0, 0.0));

        Self {
            origin,
            u_dir,
            v_dir,
            norm: normal,
        }
    }
}

impl ParametricSurface for PlaneAdapter {
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.origin + self.u_dir * u + self.v_dir * v
    }

    fn normal(&self, _u: f64, _v: f64) -> Vec3 {
        self.norm
    }

    fn project_point(&self, point: Point3) -> (f64, f64) {
        let d = point - self.origin;
        (d.dot(self.u_dir), d.dot(self.v_dir))
    }

    fn partial_u(&self, _u: f64, _v: f64) -> Vec3 {
        self.u_dir
    }

    fn partial_v(&self, _u: f64, _v: f64) -> Vec3 {
        self.v_dir
    }
}

/// A [`ParametricSurface`] view that negates the wrapped surface's normal.
///
/// The walking engine's blend constraint places the rolling-ball centre on the
/// `+normal` side of each surface (`centre = p + r·normal`), so the surfaces
/// must present their **inward** (toward-material) normals. `PlaneAdapter`
/// flips a plane via its stored normal, but analytic/NURBS surfaces have an
/// intrinsic outward normal that can't be re-oriented in place — wrapping one
/// here flips it so a fillet against a curved neighbour solves the internal
/// (material-side) branch instead of the external common-tangent one.
pub struct FlippedNormalSurface<'a> {
    inner: &'a dyn ParametricSurface,
}

impl<'a> FlippedNormalSurface<'a> {
    /// Wrap a surface so its normal is negated.
    #[must_use]
    pub const fn new(inner: &'a dyn ParametricSurface) -> Self {
        Self { inner }
    }
}

impl ParametricSurface for FlippedNormalSurface<'_> {
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.inner.evaluate(u, v)
    }

    fn normal(&self, u: f64, v: f64) -> Vec3 {
        -self.inner.normal(u, v)
    }

    fn project_point(&self, point: Point3) -> (f64, f64) {
        self.inner.project_point(point)
    }

    fn partial_u(&self, u: f64, v: f64) -> Vec3 {
        self.inner.partial_u(u, v)
    }

    fn partial_v(&self, u: f64, v: f64) -> Vec3 {
        self.inner.partial_v(u, v)
    }
}

/// Extract a `&dyn ParametricSurface` from a `FaceSurface`, or build a
/// `PlaneAdapter` for plane faces.
///
/// Returns `Ok(adapter)` for planes and `Err(face_id)` for unsupported types.
/// For analytic and NURBS surfaces that already implement `ParametricSurface`,
/// the reference is extracted directly and the adapter is unused.
///
/// # Usage pattern
///
/// ```ignore
/// let mut adapter = None;
/// let surf: &dyn ParametricSurface = surface_ref_or_adapter(&face_surface, &mut adapter);
/// ```
#[must_use]
pub fn surface_ref_or_adapter<'a>(
    surface: &'a FaceSurface,
    adapter_slot: &'a mut Option<PlaneAdapter>,
) -> &'a dyn ParametricSurface {
    // For Plane faces, we need to populate the adapter_slot first,
    // then return a reference to it. For all other variants, we can
    // return a reference directly to the surface inside FaceSurface.
    if let FaceSurface::Plane { normal, d } = surface {
        let adapter = adapter_slot.insert(PlaneAdapter::from_normal_and_d(*normal, *d));
        return adapter as &dyn ParametricSurface;
    }
    match surface {
        FaceSurface::Plane { .. } => {
            // Already handled above; this arm is unreachable.
            adapter_slot.insert(PlaneAdapter::from_normal_and_d(
                Vec3::new(0.0, 0.0, 1.0),
                0.0,
            )) as &dyn ParametricSurface
        }
        FaceSurface::Cylinder(c) => c as &dyn ParametricSurface,
        FaceSurface::Cone(c) => c as &dyn ParametricSurface,
        FaceSurface::Sphere(s) => s as &dyn ParametricSurface,
        FaceSurface::Torus(t) => t as &dyn ParametricSurface,
        FaceSurface::Nurbs(n) => n as &dyn ParametricSurface,
    }
}

/// Project a point onto the infinite axis line through `origin` with unit
/// direction `axis`, returning the foot of the perpendicular.
#[must_use]
pub fn project_onto_axis(p: Point3, origin: Point3, axis: Vec3) -> Point3 {
    let d = p - origin;
    origin + axis * axis.dot(d)
}

/// Radial distance from a point to the axis line.
#[must_use]
pub fn radial_distance(p: Point3, origin: Point3, axis: Vec3) -> f64 {
    let d = p - origin;
    (d - axis * axis.dot(d)).length()
}

/// How far a wire reaches along the axis, either side of `origin`, as
/// `(min, max)` signed distances.
///
/// Used to check that a rim setback stays on the wall it is shortening, so an
/// answer that overstated the wall's extent would let the contact circle sit
/// off the end of it. Circles perpendicular to the axis and straight edges are
/// exact at their endpoints; anything else is sampled and then pulled IN by
/// half the largest sample spacing, since the axial coordinate is 1-Lipschitz
/// in 3D position.
pub fn wire_axial_range(
    topo: &Topology,
    wire: WireId,
    origin: Point3,
    axis: Vec3,
) -> Result<(f64, f64), BlendError> {
    const SAMPLES: usize = 64;
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for oe in topo.wire(wire)?.edges() {
        let e = topo.edge(oe.edge())?;
        let (sp, ep) = (
            topo.vertex(e.start())?.point(),
            topo.vertex(e.end())?.point(),
        );
        let s = |p: Point3| axis.dot(p - origin);
        lo = lo.min(s(sp)).min(s(ep));
        hi = hi.max(s(sp)).max(s(ep));
        let perpendicular_circle = matches!(e.curve(), EdgeCurve::Circle(c)
            if c.normal().cross(axis).length() < 1e-9);
        if matches!(e.curve(), EdgeCurve::Line) || perpendicular_circle {
            // Constant (circle) or monotone (line) along the axis: the
            // endpoints already bound it.
            continue;
        }
        let (t0, t1) = e.curve().domain_with_endpoints(sp, ep);
        let mut prev: Option<Point3> = None;
        let mut spacing: f64 = 0.0;
        let (mut c_lo, mut c_hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for k in 0..=SAMPLES {
            #[allow(clippy::cast_precision_loss)]
            let t = t0 + (t1 - t0) * (k as f64) / (SAMPLES as f64);
            let p = e.curve().evaluate_with_endpoints(t, sp, ep);
            if let Some(q) = prev {
                spacing = spacing.max((p - q).length());
            }
            prev = Some(p);
            c_lo = c_lo.min(s(p));
            c_hi = c_hi.max(s(p));
        }
        // Understate the reach rather than overstate it.
        lo = lo.min(c_lo + spacing * 0.5);
        hi = hi.max(c_hi - spacing * 0.5);
    }
    Ok((lo, hi))
}

/// The extremal radial distance from the axis line to a whole wire: the
/// minimum when `want_min`, otherwise the maximum.
///
/// This decides whether a rim fillet's moved contact circle still clears the
/// cap's other loops, so an answer that is optimistic by even a little would
/// admit a self-intersecting cap. Every value returned is therefore either
/// exact or erring the safe way (a smaller minimum, a larger maximum):
///
///   * straight edges — the radial distance along a segment is the norm of an
///     affine function, so it is convex: the maximum is at an endpoint and the
///     minimum is solved for in closed form.
///   * whole circles lying in a plane perpendicular to the axis — the usual
///     `|d ± ρ|`, where `d` is the centre's own radial distance.
///   * anything else — sampled, then widened by half the largest sample
///     spacing. Radial distance is 1-Lipschitz in 3D position, so the true
///     extremum cannot lie further than that from the sampled one. No
///     assumption about the curve is needed for this to hold.
pub fn wire_radial_extremum(
    topo: &Topology,
    wire: WireId,
    origin: Point3,
    axis: Vec3,
    want_min: bool,
) -> Result<f64, BlendError> {
    const SAMPLES: usize = 64;
    let pick = |acc: f64, v: f64| if want_min { acc.min(v) } else { acc.max(v) };
    let mut best = if want_min { f64::INFINITY } else { 0.0 };

    for oe in topo.wire(wire)?.edges() {
        let e = topo.edge(oe.edge())?;
        let (sp, ep) = (
            topo.vertex(e.start())?.point(),
            topo.vertex(e.end())?.point(),
        );
        best = pick(best, radial_distance(sp, origin, axis));
        best = pick(best, radial_distance(ep, origin, axis));

        match e.curve() {
            EdgeCurve::Line => {
                if !want_min {
                    // Convex along the segment: the endpoints already bound it.
                    continue;
                }
                let perp = |p: Point3| {
                    let d = p - origin;
                    d - axis * axis.dot(d)
                };
                let (pa, pb) = (perp(sp), perp(ep));
                let ab = pb - pa;
                let len_sq = ab.dot(ab);
                if len_sq > 0.0 {
                    let t = (-pa.dot(ab) / len_sq).clamp(0.0, 1.0);
                    best = best.min((pa + ab * t).length());
                }
            }
            EdgeCurve::Circle(c)
                if e.start() == e.end() && c.normal().cross(axis).length() < 1e-9 =>
            {
                // A whole circle in a plane perpendicular to the axis: the
                // radial distance sweeps the full interval about its centre.
                let d = radial_distance(c.center(), origin, axis);
                best = pick(
                    best,
                    if want_min {
                        (d - c.radius()).abs()
                    } else {
                        d + c.radius()
                    },
                );
            }
            curve => {
                let (t0, t1) = curve.domain_with_endpoints(sp, ep);
                let mut prev: Option<Point3> = None;
                let mut spacing: f64 = 0.0;
                for k in 0..=SAMPLES {
                    #[allow(clippy::cast_precision_loss)]
                    let t = t0 + (t1 - t0) * (k as f64) / (SAMPLES as f64);
                    let p = curve.evaluate_with_endpoints(t, sp, ep);
                    if let Some(q) = prev {
                        spacing = spacing.max((p - q).length());
                    }
                    prev = Some(p);
                    best = pick(best, radial_distance(p, origin, axis));
                }
                // Widen by the Lipschitz bound so the answer never claims more
                // clearance than the curve actually has.
                best = if want_min {
                    best - spacing * 0.5
                } else {
                    best + spacing * 0.5
                };
            }
        }
    }
    Ok(best)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::expect_used)]
    fn straight_contact_edge_does_not_match_curved_stripe_contact() {
        let start = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 0.0);
        let curved = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![start, Point3::new(0.5, 0.5, 0.0), end],
            vec![1.0, 1.0, 1.0],
        )
        .expect("valid quadratic NURBS");

        assert!(!contact_geometry_matches(
            &EdgeCurve::Line,
            start,
            end,
            &curved,
            1e-5,
        ));
    }

    #[test]
    #[allow(clippy::expect_used)]
    fn straight_contact_matches_nonuniform_linear_nurbs() {
        let start = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 0.0);
        let nonuniform = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 0.5, 1.0, 1.0],
            vec![start, Point3::new(0.9, 0.0, 0.0), end],
            vec![1.0, 1.0, 1.0],
        )
        .expect("valid linear NURBS");

        assert!(contact_geometry_matches(
            &EdgeCurve::Line,
            start,
            end,
            &nonuniform,
            1e-5,
        ));
    }
}

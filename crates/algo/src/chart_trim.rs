//! Boundary wires of an analytic curved face as outlines in the surface's
//! own `(u, v)` chart.
//!
//! A cylinder wall an earlier cut left with a HOLE (an inner wire that closes
//! in the chart) or a BAY (an outer wire that is not a chart rectangle) is no
//! longer described by a height band and an angular gap. The section-pave
//! pass ([`crate::pave_filler::section_paves`]) and the ray-cast classifier
//! both need the face's true region there; this module samples a wire once
//! and answers the two questions they ask of it: is a chart point inside the
//! outline, and how far is a model point from it.
//! Every chord carries a whole-interval deviation bound; NURBS bounds reuse
//! the rounding-inclusive Bernstein certificates in `remus_math`. When a
//! bound is unavailable the outline cannot authorize section exclusion.
//!
//! The outline is lifted CONTINUOUSLY along the wire, so a seam-cut wall's
//! outer wire (which runs its seam edge once at each end of the period)
//! closes into one polygon, and a hole straddling the chart seam stays one
//! polygon. A wire that winds a period (a band rim, a loop around a sphere
//! pole) does not close and has no outline.

use std::f64::consts::TAU;

use remus_math::context::OperationContext;
use remus_math::nurbs::{NurbsCurve, reuse::bound_curve_position};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::wire::WireId;

use crate::builder::classify_2d::point_in_polygon_2d;
use crate::periodic_chart::unwrap_near;

/// Samples per wire edge.
pub const SAMPLES_PER_EDGE: usize = 24;

/// Chart distance within which an outline sample counts as lying on a side
/// of the outline's bounding rectangle.
const RECTANGLE_EPS: f64 = 1e-7;

/// One wire of a curved face, sampled and lifted into the surface's chart.
#[derive(Debug, Clone)]
pub struct ChartLoop {
    /// Outline lifted continuously into `(u, v)`; closed implicitly.
    pub uv: Vec<Point2>,
    /// The same outline in model space.
    pub pts: Vec<Point3>,
    /// Lower chart bound of `uv`.
    pub lo: Point2,
    /// Upper chart bound of `uv`.
    pub hi: Point2,
    /// Whole-interval upper bound on the outline's chord error, in model
    /// units. Infinity means unavailable: no exclusion may rely on it.
    pub sagitta: f64,
}

/// Continuous enclosure about a parameter-matched polyline. For NURBS the
/// existing outward-rounded rational Bernstein certificate covers EVERY
/// common knot span, including spans between all the polyline's samples.
/// No sampled midpoint is evidence of a bound. An unavailable certificate
/// deliberately makes boundary proximity undecidable for all points.
fn polyline_error(curve: &EdgeCurve, parameters: &[f64], points: &[Point3]) -> Option<f64> {
    let step = parameters
        .windows(2)
        .map(|w| (w[1] - w[0]).abs())
        .fold(0.0, f64::max);
    let bound = match curve {
        EdgeCurve::Line => 0.0,
        EdgeCurve::Circle(c) if step <= TAU => {
            // Exact maximum distance to an arc chord, evaluated without
            // subtracting nearly equal numbers.
            2.0 * c.radius() * (step * 0.25).sin().powi(2)
        }
        EdgeCurve::Ellipse(e) => {
            // Linear interpolation remainder: max |c''| * step² / 8.
            e.semi_major().max(e.semi_minor()) * step.powi(2) / 8.0
        }
        EdgeCurve::NurbsCurve(n) => {
            let mut parameters = parameters.to_vec();
            let mut points = points.to_vec();
            // A stored trim can traverse the native domain backwards;
            // Bernstein comparison consumes an increasing interval.
            if parameters[0] > *parameters.last()? {
                parameters.reverse();
                points.reverse();
            }
            let mut knots = vec![parameters[0]];
            knots.extend_from_slice(&parameters);
            knots.push(*parameters.last()?);
            let weights = vec![1.0; points.len()];
            let polyline = NurbsCurve::new(1, knots, points, weights).ok()?;
            bound_curve_position(
                n,
                &polyline,
                (parameters[0], *parameters.last()?),
                1_000_000,
                &OperationContext::default(),
            )
            .ok()?
        }
        EdgeCurve::Circle(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => return None,
    };
    let scale = points
        .iter()
        .flat_map(|p| p.0)
        .fold(1.0_f64, |s, x| s.max(x.abs()));
    let bound = (bound + 1e-14 * scale).next_up();
    bound.is_finite().then_some(bound)
}

impl ChartLoop {
    /// Whether some periodic image of chart point `(u, v)` lies inside the
    /// outline (even-odd). `u` is always periodic; `v` only on a torus.
    pub fn contains(&self, (u, v): (f64, f64), v_periodic: bool) -> bool {
        let first_u = u + TAU * ((self.lo.x() - u) / TAU).ceil();
        let first_v = if v_periodic {
            v + TAU * ((self.lo.y() - v) / TAU).ceil()
        } else {
            v
        };
        // Whole periods between the first image and the outline's far side.
        let images = |first: f64, hi: f64, periodic: bool| -> i32 {
            if first > hi {
                0
            } else if periodic {
                #[allow(clippy::cast_possible_truncation)]
                let extra = ((hi - first) / TAU).floor() as i32;
                extra.saturating_add(1)
            } else {
                1
            }
        };
        let count_v = if v_periodic {
            images(first_v, self.hi.y(), true)
        } else {
            1
        };
        for i in 0..images(first_u, self.hi.x(), true) {
            for j in 0..count_v {
                let image = Point2::new(
                    f64::from(i).mul_add(TAU, first_u),
                    f64::from(j).mul_add(TAU, first_v),
                );
                if point_in_polygon_2d(image, &self.uv) {
                    return true;
                }
            }
        }
        false
    }

    /// Model-space distance from `p` to the outline.
    pub fn distance(&self, p: Point3) -> f64 {
        let n = self.pts.len();
        let mut best = f64::INFINITY;
        for i in 0..n {
            let (a, b) = (self.pts[i], self.pts[(i + 1) % n]);
            let ab = b - a;
            let len_sq = ab.length_squared();
            let foot = if len_sq > 0.0 {
                a + ab * ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0)
            } else {
                a
            };
            best = best.min((p - foot).length());
        }
        best
    }

    /// Whether the outline is its own bounding rectangle in the chart: every
    /// sample lies on one of the rectangle's sides.
    pub fn is_chart_rectangle(&self) -> bool {
        self.uv.iter().all(|q| {
            (q.x() - self.lo.x()).abs() <= RECTANGLE_EPS
                || (q.x() - self.hi.x()).abs() <= RECTANGLE_EPS
                || (q.y() - self.lo.y()).abs() <= RECTANGLE_EPS
                || (q.y() - self.hi.y()).abs() <= RECTANGLE_EPS
        })
    }
}

/// Whether `surface`'s chart is periodic in `v` as well as `u`. `None` for
/// surfaces this module does not chart (planes and NURBS).
pub fn chart_v_periodic(surface: &FaceSurface) -> Option<bool> {
    match surface {
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_) | FaceSurface::Sphere(_) => Some(false),
        FaceSurface::Torus(_) => Some(true),
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => None,
    }
}

/// Sample `wire_id` on `surface` and lift it into the chart. `None` when the
/// wire does not close there (it winds a period) or cannot be sampled.
pub fn chart_loop(
    topo: &Topology,
    surface: &FaceSurface,
    wire_id: WireId,
    _tol: Tolerance,
) -> Option<ChartLoop> {
    chart_v_periodic(surface)?;
    let wire = topo.wire(wire_id).ok()?;
    let mut pts: Vec<Point3> = Vec::new();
    let mut sagitta = 0.0_f64;
    let mut previous_end: Option<Point3> = None;
    let mut join_error = 0.0_f64;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        let sp = topo.vertex(edge.start()).ok()?.point();
        let ep = topo.vertex(edge.end()).ok()?.point();
        let (t0, t1) = edge.strict_domain().ok()?;
        #[allow(clippy::cast_precision_loss)]
        let n = SAMPLES_PER_EDGE as f64;
        #[allow(clippy::cast_precision_loss)]
        let parameters: Vec<f64> = (0..=SAMPLES_PER_EDGE)
            .map(|k| (t1 - t0).mul_add(k as f64 / n, t0))
            .collect();
        let mut samples: Vec<Point3> = parameters
            .iter()
            .map(|&t| edge.curve().evaluate_with_endpoints(t, sp, ep))
            .collect();
        sagitta = sagitta
            .max(polyline_error(edge.curve(), &parameters, &samples).unwrap_or(f64::INFINITY));
        if !oe.is_forward() {
            samples.reverse();
        }
        if let Some(end) = previous_end {
            join_error = join_error.max((samples[0] - end).length());
        }
        previous_end = samples.last().copied();
        // Keep every sample: removing even a short segment would require
        // bounding the deviation to the replacement chord too.
        pts.extend_from_slice(&samples[..SAMPLES_PER_EDGE]);
    }
    if let (Some(&first), Some(end)) = (pts.first(), previous_end) {
        join_error = join_error.max((first - end).length());
    }
    sagitta = (sagitta + join_error).next_up();
    if pts.len() < 3 {
        return None;
    }

    loop_from_points(surface, pts, sagitta)
}

/// Refine ONLY the section-exclusion outline. Classifier and boundary-riding
/// samples remain unchanged. Each accepted chord bounds its complete native
/// interval; knot spans are seeded explicitly, so parameter compression cannot
/// conceal an excursion. Work exhaustion or any unavailable bound means KEEP.
pub fn exclusion_loop(
    topo: &Topology,
    surface: &FaceSurface,
    wire_id: WireId,
    error: f64,
) -> Option<ChartLoop> {
    let wire = topo.wire(wire_id).ok()?;
    let mut pts: Vec<Point3> = Vec::new();
    let mut previous_end: Option<Point3> = None;
    let mut sagitta = 0.0_f64;
    let mut work = 0;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        let sp = topo.vertex(edge.start()).ok()?.point();
        let ep = topo.vertex(edge.end()).ok()?.point();
        let (t0, t1) = edge.strict_domain().ok()?;
        let (lo, hi) = (t0.min(t1), t0.max(t1));
        let mut breaks = vec![lo];
        if let EdgeCurve::NurbsCurve(n) = edge.curve() {
            for &knot in n.knots() {
                if knot > *breaks.last()? && knot < hi {
                    breaks.push(knot);
                }
            }
        }
        breaks.push(hi);
        let at = |t| edge.curve().evaluate_with_endpoints(t, sp, ep);
        let mut pending: Vec<_> = breaks.windows(2).rev().map(|w| (w[0], w[1])).collect();
        let mut samples = Vec::new();
        while let Some((a, b)) = pending.pop() {
            work += 1;
            if work > 32768 {
                return None;
            }
            let (pa, pb) = (at(a), at(b));
            let bound = polyline_error(edge.curve(), &[a, b], &[pa, pb])?;
            if bound <= error {
                samples.push(pa);
                sagitta = sagitta.max(bound);
            } else {
                let m = f64::midpoint(a, b);
                if m <= a || m >= b {
                    return None;
                }
                pending.extend([(m, b), (a, m)]);
            }
        }
        samples.push(at(hi));
        if (t0 > t1) == oe.is_forward() {
            samples.reverse();
        }
        if let Some(end) = previous_end {
            sagitta = (sagitta + (samples[0] - end).length()).next_up();
        }
        previous_end = samples.last().copied();
        pts.extend_from_slice(&samples[..samples.len() - 1]);
    }
    sagitta = (sagitta + (pts.first().copied()? - previous_end?).length()).next_up();
    if sagitta > error || pts.len() < 3 {
        return None;
    }
    loop_from_points(surface, pts, sagitta)
}

/// Construct the same continuous chart lift for a certified model polyline.
fn loop_from_points(surface: &FaceSurface, pts: Vec<Point3>, sagitta: f64) -> Option<ChartLoop> {
    let v_periodic = chart_v_periodic(surface)?;
    let mut uv: Vec<Point2> = Vec::with_capacity(pts.len());
    for &p in &pts {
        let (u, v) = surface.project_point(p)?;
        let q = match uv.last() {
            Some(prev) => Point2::new(
                unwrap_near(u, prev.x()),
                if v_periodic {
                    unwrap_near(v, prev.y())
                } else {
                    v
                },
            ),
            None => Point2::new(u, v),
        };
        uv.push(q);
    }
    // A loop that winds a period does not close in the chart: it is a band
    // rim (or surrounds a pole) and has no inside to test against.
    let (first, last) = (uv[0], uv[uv.len() - 1]);
    if (unwrap_near(first.x(), last.x()) - first.x()).abs() > 1e-9 {
        return None;
    }
    if v_periodic && (unwrap_near(first.y(), last.y()) - first.y()).abs() > 1e-9 {
        return None;
    }
    let (mut lo, mut hi) = (first, first);
    for q in &uv {
        lo = Point2::new(lo.x().min(q.x()), lo.y().min(q.y()));
        hi = Point2::new(hi.x().max(q.x()), hi.y().max(q.y()));
    }
    Some(ChartLoop {
        uv,
        pts,
        lo,
        hi,
        sagitta,
    })
}

/// Unit cylinder walls (axis z, `v ∈ [0, 4]`, seam at `u = 0`) trimmed by a
/// chart rectangle, shared by this module's tests and its two consumers'.
#[cfg(test)]
pub mod fixtures {
    #![allow(clippy::unwrap_used)]

    use std::f64::consts::TAU;

    use remus_math::curves::Circle3D;
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
    use remus_topology::face::{Face, FaceId, FaceSurface};
    use remus_topology::vertex::{Vertex, VertexId};
    use remus_topology::wire::{OrientedEdge, Wire, WireId};

    const TOL: f64 = 1e-7;

    /// How the wall is trimmed.
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum Trim {
        /// The plain seam-cut lateral.
        None,
        /// A hole `u ∈ [1, 2] × v ∈ [1, 3]`.
        Hole,
        /// A bay `u ∈ [1, 2] × v ∈ [3, 4]` notched into the top rim.
        Bay,
        /// An inner wire that is a whole ring at `v = 2`: a band rim.
        Ring,
    }

    /// The unit cylinder about the z axis.
    pub fn cylinder() -> CylindricalSurface {
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap()
    }

    /// The iso-`v` circle at height `z`, parameterised by the cylinder's `u`.
    pub fn ring(cyl: &CylindricalSurface, z: f64) -> Circle3D {
        let center = Point3::new(0.0, 0.0, z);
        Circle3D::new_with_ref(
            center,
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            cyl.evaluate(0.0, z) - center,
        )
        .unwrap()
    }

    fn arc(
        topo: &mut Topology,
        cyl: &CylindricalSurface,
        z: f64,
        (from, a): (VertexId, f64),
        (to, b): (VertexId, f64),
    ) -> EdgeId {
        let mut edge = Edge::new(from, to, EdgeCurve::Circle(ring(cyl, z)));
        edge.set_trim(Some((a, b)));
        topo.add_edge(edge)
    }

    fn rectangle_wire(
        topo: &mut Topology,
        cyl: &CylindricalSurface,
        (u0, u1): (f64, f64),
        (v0, v1): (f64, f64),
    ) -> WireId {
        let a = topo.add_vertex(Vertex::new(cyl.evaluate(u0, v0), TOL));
        let b = topo.add_vertex(Vertex::new(cyl.evaluate(u1, v0), TOL));
        let c = topo.add_vertex(Vertex::new(cyl.evaluate(u1, v1), TOL));
        let d = topo.add_vertex(Vertex::new(cyl.evaluate(u0, v1), TOL));
        let bottom = arc(topo, cyl, v0, (a, u0), (b, u1));
        let right = topo.add_edge(Edge::new(b, c, EdgeCurve::Line));
        let top = arc(topo, cyl, v1, (d, u0), (c, u1));
        let left = topo.add_edge(Edge::new(a, d, EdgeCurve::Line));
        topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(bottom, true),
                    OrientedEdge::new(right, true),
                    OrientedEdge::new(top, false),
                    OrientedEdge::new(left, false),
                ],
                true,
            )
            .unwrap(),
        )
    }

    /// The wall face trimmed as asked.
    pub fn wall(trim: Trim) -> (Topology, FaceId, CylindricalSurface) {
        let mut topo = Topology::new();
        let cyl = cylinder();
        let s0 = topo.add_vertex(Vertex::new(cyl.evaluate(0.0, 0.0), TOL));
        let s1 = topo.add_vertex(Vertex::new(cyl.evaluate(0.0, 4.0), TOL));
        let bottom = arc(&mut topo, &cyl, 0.0, (s0, 0.0), (s0, TAU));
        let seam = topo.add_edge(Edge::new(s0, s1, EdgeCurve::Line));
        let mut edges = vec![
            OrientedEdge::new(bottom, true),
            OrientedEdge::new(seam, true),
        ];
        if trim == Trim::Bay {
            let a = topo.add_vertex(Vertex::new(cyl.evaluate(1.0, 3.0), TOL));
            let b = topo.add_vertex(Vertex::new(cyl.evaluate(2.0, 3.0), TOL));
            let c = topo.add_vertex(Vertex::new(cyl.evaluate(2.0, 4.0), TOL));
            let d = topo.add_vertex(Vertex::new(cyl.evaluate(1.0, 4.0), TOL));
            let rim_far = arc(&mut topo, &cyl, 4.0, (c, 2.0), (s1, TAU));
            let right = topo.add_edge(Edge::new(b, c, EdgeCurve::Line));
            let floor = arc(&mut topo, &cyl, 3.0, (a, 1.0), (b, 2.0));
            let left = topo.add_edge(Edge::new(a, d, EdgeCurve::Line));
            let rim_near = arc(&mut topo, &cyl, 4.0, (s1, 0.0), (d, 1.0));
            edges.extend([
                OrientedEdge::new(rim_far, false),
                OrientedEdge::new(right, false),
                OrientedEdge::new(floor, false),
                OrientedEdge::new(left, true),
                OrientedEdge::new(rim_near, false),
            ]);
        } else {
            let top = arc(&mut topo, &cyl, 4.0, (s1, 0.0), (s1, TAU));
            edges.push(OrientedEdge::new(top, false));
        }
        edges.push(OrientedEdge::new(seam, false));
        let outer = topo.add_wire(Wire::new(edges, true).unwrap());
        let inner = match trim {
            Trim::Hole => vec![rectangle_wire(&mut topo, &cyl, (1.0, 2.0), (1.0, 3.0))],
            Trim::Ring => {
                let v = topo.add_vertex(Vertex::new(cyl.evaluate(0.0, 2.0), TOL));
                let ring = arc(&mut topo, &cyl, 2.0, (v, 0.0), (v, TAU));
                vec![topo.add_wire(Wire::new(vec![OrientedEdge::new(ring, true)], true).unwrap())]
            }
            Trim::None | Trim::Bay => Vec::new(),
        };
        let face = topo.add_face(Face::new(outer, inner, FaceSurface::Cylinder(cyl.clone())));
        (topo, face, cyl)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::fixtures::{Trim, wall};
    use super::*;

    fn outer(trim: Trim) -> ChartLoop {
        let (topo, face, _) = wall(trim);
        let face = topo.face(face).unwrap();
        chart_loop(&topo, face.surface(), face.outer_wire(), Tolerance::new()).unwrap()
    }

    #[test]
    fn a_seam_cut_outer_wire_closes_into_one_period_wide_rectangle() {
        let plain = outer(Trim::None);
        assert!(plain.is_chart_rectangle());
        assert!((plain.hi.x() - plain.lo.x() - TAU).abs() < 1e-12);
        assert!(plain.lo.y().abs() < 1e-12 && (plain.hi.y() - 4.0).abs() < 1e-12);
        // Rim circles are chorded into 24 segments; the seam ruling is not.
        let expected = 1.0 - (TAU / 48.0).cos();
        assert!(
            (plain.sagitta - expected).abs() < 1e-12,
            "{}",
            plain.sagitta
        );
    }

    #[test]
    fn a_bay_makes_the_outer_wire_something_other_than_a_rectangle() {
        let bayed = outer(Trim::Bay);
        assert!(!bayed.is_chart_rectangle());
        // Same bounds as the plain wall: the bay is inside them.
        assert!((bayed.hi.x() - bayed.lo.x() - TAU).abs() < 1e-12);
        assert!((bayed.hi.y() - 4.0).abs() < 1e-12);
        // On the wall, in the bay, above the rim; and one period away.
        assert!(bayed.contains((0.5, 3.5), false));
        assert!(!bayed.contains((1.5, 3.5), false));
        assert!(!bayed.contains((0.5, 4.5), false));
        assert!(bayed.contains((0.5 - TAU, 3.5), false));
        assert!(bayed.contains((0.5 + 3.0 * TAU, 3.5), false));
    }

    #[test]
    fn a_wire_that_winds_the_period_has_no_outline() {
        let (topo, face, _) = wall(Trim::Ring);
        let face = topo.face(face).unwrap();
        let ring = face.inner_wires()[0];
        assert!(chart_loop(&topo, face.surface(), ring, Tolerance::new()).is_none());
        // Nor does any wire of a plane or a NURBS face.
        assert!(
            chart_v_periodic(&FaceSurface::Plane {
                normal: remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            })
            .is_none()
        );
    }

    #[test]
    fn distance_is_measured_to_the_outline_in_model_space() {
        let (_, _, cyl) = wall(Trim::Bay);
        let bayed = outer(Trim::Bay);
        // From the bay's middle the two rulings at u = 1 and u = 2 are a
        // chord of half a radian away, nearer than the floor 0.5 below.
        let chord = 2.0 * 0.25_f64.sin();
        assert!((bayed.distance(cyl.evaluate(1.5, 3.5)) - chord).abs() < 1e-12);
        // Opposite the seam, at mid-height, the nearest outline point is the
        // bay's corner at (u, v) = (2, 3): a chord of π − 2 radians across
        // and 1 up, nearer than either rim 2 away.
        let far = cyl.evaluate(std::f64::consts::PI, 2.0);
        let corner = (2.0 * ((std::f64::consts::PI - 2.0) / 2.0).sin()).hypot(1.0);
        assert!((bayed.distance(far) - corner).abs() < 1e-12);
    }
}

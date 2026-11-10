//! The lifted `(u, v)` chart of a torus or cylinder face bounded by
//! iso-parameter curves and closed in at least one direction.
//!
//! A tube bounded by its two end circles and a seam (a swept or imported
//! pipe), a whole torus bounded by its two seam circles, or a cylinder
//! lateral bounded by its two rims and a seam line, lifts to an axis-aligned
//! rectangle in the `(u, v)` plane whose periodic sides are the two copies of
//! one seam edge. Membership of a point on the carrier is then exact: lift it
//! into the rectangle and compare with the bounded sides.
//!
//! The recognizer is deliberately narrow. Every boundary edge must be an
//! iso-parameter curve (a circle on a torus; a rim circle or a ruling line
//! on a cylinder) lying on one side of the rectangle along its whole span,
//! the lifted boundary must close, and at least one direction must span a
//! whole period. A fillet patch (bounded in both directions) or any other
//! trim declines, and its callers keep their established handling.

use std::f64::consts::TAU;

use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
use remus_math::vec::{Point2, Point3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

/// Chart distance below which a lifted boundary sample sits on a side.
pub const SIDE_EPS: f64 = 1e-7;
/// Samples per boundary edge when lifting the face boundary.
const BOUNDARY_SAMPLES: u32 = 32;

/// The carrier surface of a chart.
#[derive(Clone, Debug)]
pub enum ChartCarrier {
    /// A ring torus: both parameters are angles.
    Torus(ToroidalSurface),
    /// A cylinder: `u` is the angle, `v` the axial height.
    Cylinder(CylindricalSurface),
}

impl ChartCarrier {
    /// The carrier of a supported face surface.
    pub fn of(surface: &FaceSurface) -> Option<Self> {
        match surface {
            FaceSurface::Torus(t) if t.major_radius() > t.minor_radius() => {
                Some(Self::Torus(t.clone()))
            }
            FaceSurface::Cylinder(c) => Some(Self::Cylinder(c.clone())),
            FaceSurface::Torus(_)
            | FaceSurface::Plane { .. }
            | FaceSurface::Nurbs(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_) => None,
        }
    }

    /// Raw carrier parameters, `u` (and a torus's `v`) in `[0, 2π)`.
    pub fn project(&self, p: Point3) -> (f64, f64) {
        match self {
            Self::Torus(t) => t.project_point(p),
            Self::Cylinder(c) => c.project_point(p),
        }
    }

    /// The carrier point at `(u, v)`.
    pub fn evaluate(&self, u: f64, v: f64) -> Point3 {
        match self {
            Self::Torus(t) => t.evaluate(u, v),
            Self::Cylinder(c) => c.evaluate(u, v),
        }
    }

    /// Whether `v` is an angle (so it can be periodic).
    pub const fn v_angular(&self) -> bool {
        matches!(self, Self::Torus(_))
    }
}

/// Which side of the chart rectangle a boundary edge runs along.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Side {
    /// `u = u_lo`.
    Left,
    /// `u = u_hi`.
    Right,
    /// `v = v_lo`.
    Bottom,
    /// `v = v_hi`.
    Top,
}

/// The chart rectangle `[u_lo, u_hi] × [v_lo, v_hi]` of a recognized face.
#[derive(Clone, Debug)]
pub struct PeriodicChart {
    pub u_lo: f64,
    pub u_hi: f64,
    pub v_lo: f64,
    pub v_hi: f64,
    pub periodic_u: bool,
    pub periodic_v: bool,
    /// Whether `v` is an angle (a torus) rather than a length.
    pub v_angular: bool,
}

/// One boundary edge in wire order, as the lifter reads it.
pub struct BoundaryUse<'a> {
    pub curve: &'a EdgeCurve,
    /// Native parameters at the traversal start and end.
    pub t_start: f64,
    pub t_end: f64,
    /// Native start and end points, for curves evaluated from endpoints.
    pub native_start: Point3,
    pub native_end: Point3,
}

/// A boundary edge lifted into the chart.
pub struct LiftedBoundary {
    pub side: Side,
    /// Sampled chart polyline in traversal order.
    pub poly: Vec<Point2>,
}

impl PeriodicChart {
    /// Shift `x` by whole periods into `[lo, lo + 2π)` (periodic) or as close
    /// as possible to `[lo, hi]` (bounded).
    fn place(x: f64, lo: f64, hi: f64, periodic: bool) -> f64 {
        if periodic {
            return lo + (x - lo).rem_euclid(TAU);
        }
        let mid = 0.5 * (lo + hi);
        x + TAU * ((mid - x) / TAU).round()
    }

    /// Place carrier parameters `(u, v)` in the chart cell.
    pub fn place_uv(&self, (u, v): (f64, f64)) -> Point2 {
        Point2::new(
            Self::place(u, self.u_lo, self.u_hi, self.periodic_u),
            if self.v_angular {
                Self::place(v, self.v_lo, self.v_hi, self.periodic_v)
            } else {
                v
            },
        )
    }

    /// Whether a point already on the carrier lies on the face: its lifted
    /// parameters fall within the bounded sides, to `eps` in the chart.
    pub fn contains_carrier_point(&self, carrier: &ChartCarrier, p: Point3, eps: f64) -> bool {
        let q = self.place_uv(carrier.project(p));
        (self.periodic_u || (q.x() >= self.u_lo - eps && q.x() <= self.u_hi + eps))
            && (self.periodic_v || (q.y() >= self.v_lo - eps && q.y() <= self.v_hi + eps))
    }

    /// The single side a chart point lies on; `None` off the sides and at
    /// corners.
    pub fn side_of(&self, p: Point2) -> Option<Side> {
        let near = |a: f64, b: f64| (a - b).abs() <= 1e-6;
        let mut found = None;
        let mut count = 0;
        for (cond, side) in [
            (near(p.x(), self.u_lo), Side::Left),
            (near(p.x(), self.u_hi), Side::Right),
            (near(p.y(), self.v_lo), Side::Bottom),
            (near(p.y(), self.v_hi), Side::Top),
        ] {
            if cond {
                found = Some(side);
                count += 1;
            }
        }
        if count == 1 { found } else { None }
    }

    /// Whether a chart point lies in the closed rectangle, to `eps`.
    pub fn contains(&self, p: Point2, eps: f64) -> bool {
        p.x() >= self.u_lo - eps
            && p.x() <= self.u_hi + eps
            && p.y() >= self.v_lo - eps
            && p.y() <= self.v_hi + eps
    }
}

/// Continuous lift of a raw `[0, 2π)` angle next to a previous lifted value.
pub fn unwrap_near(raw: f64, previous: f64) -> f64 {
    raw + TAU * ((previous - raw) / TAU).round()
}

/// Lift sampled carrier points continuously, the first one at `first`.
pub fn lift_continuous(carrier: &ChartCarrier, pts: &[Point3], first: Point2) -> Vec<Point2> {
    let mut out = Vec::with_capacity(pts.len());
    let mut prev = first;
    for (i, &p) in pts.iter().enumerate() {
        let q = if i == 0 {
            first
        } else {
            let (u, v) = carrier.project(p);
            let v = if carrier.v_angular() {
                unwrap_near(v, prev.y())
            } else {
                v
            };
            Point2::new(unwrap_near(u, prev.x()), v)
        };
        out.push(q);
        prev = q;
    }
    out
}

/// Lift a face's outer wire and recognize its chart rectangle.
///
/// Returns the chart, the orientation sign of the lifted wire (`+1` counter-
/// clockwise in `(u, v)`) and each edge's side and polyline, or `None` when
/// the wire is outside the qualified family (see the module docs).
pub fn lift_boundary(
    carrier: &ChartCarrier,
    uses: &[BoundaryUse<'_>],
) -> Option<(PeriodicChart, f64, Vec<LiftedBoundary>)> {
    if uses.is_empty() {
        return None;
    }
    let mut polys: Vec<Vec<Point2>> = Vec::with_capacity(uses.len());
    let mut prev: Option<Point2> = None;
    for edge in uses {
        let supported = match carrier {
            ChartCarrier::Torus(_) => matches!(edge.curve, EdgeCurve::Circle(_)),
            ChartCarrier::Cylinder(_) => {
                matches!(edge.curve, EdgeCurve::Circle(_) | EdgeCurve::Line)
            }
        };
        if !supported {
            return None;
        }
        let (ta, tb) = (edge.t_start, edge.t_end);
        if !(ta.is_finite() && tb.is_finite()) || (tb - ta).abs() <= 1e-12 {
            return None;
        }
        let pts: Vec<Point3> = (0..=BOUNDARY_SAMPLES)
            .map(|k| {
                let t = ta + (tb - ta) * f64::from(k) / f64::from(BOUNDARY_SAMPLES);
                edge.curve
                    .evaluate_with_endpoints(t, edge.native_start, edge.native_end)
            })
            .collect();
        let (u, v) = carrier.project(pts[0]);
        let first = match prev {
            None => Point2::new(u, v),
            Some(p) => Point2::new(
                unwrap_near(u, p.x()),
                if carrier.v_angular() {
                    unwrap_near(v, p.y())
                } else {
                    v
                },
            ),
        };
        let poly = lift_continuous(carrier, &pts, first);
        prev = poly.last().copied();
        polys.push(poly);
    }
    let first = polys.first()?.first().copied()?;
    let last = polys.last()?.last().copied()?;
    if (first - last).length() > 1e-6 {
        return None;
    }
    let (mut u_lo, mut u_hi, mut v_lo, mut v_hi) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    let mut area2 = 0.0;
    for poly in &polys {
        for w in poly.windows(2) {
            area2 += w[0].x() * w[1].y() - w[1].x() * w[0].y();
        }
        for p in poly {
            u_lo = u_lo.min(p.x());
            u_hi = u_hi.max(p.x());
            v_lo = v_lo.min(p.y());
            v_hi = v_hi.max(p.y());
        }
    }
    let periodic_u = (u_hi - u_lo - TAU).abs() <= 1e-7;
    let periodic_v = carrier.v_angular() && (v_hi - v_lo - TAU).abs() <= 1e-7;
    if !(periodic_u || periodic_v) {
        return None;
    }
    // A bounded side pair must be a real interval short of a whole turn.
    let bounded_ok = |span: f64, angular: bool| span > 1e-6 && (!angular || span < TAU - 1e-6);
    if (!periodic_u && !bounded_ok(u_hi - u_lo, true))
        || (!periodic_v && !bounded_ok(v_hi - v_lo, carrier.v_angular()))
    {
        return None;
    }
    let chart = PeriodicChart {
        u_lo,
        u_hi,
        v_lo,
        v_hi,
        periodic_u,
        periodic_v,
        v_angular: carrier.v_angular(),
    };
    let mut lifted = Vec::with_capacity(polys.len());
    for poly in polys {
        let along =
            |f: fn(Point2) -> f64, x: f64| poly.iter().all(|&p| (f(p) - x).abs() <= SIDE_EPS);
        let side = if along(Point2::x, u_lo) {
            Side::Left
        } else if along(Point2::x, u_hi) {
            Side::Right
        } else if along(Point2::y, v_lo) {
            Side::Bottom
        } else if along(Point2::y, v_hi) {
            Side::Top
        } else {
            return None;
        };
        if (poly[0] - poly[poly.len() - 1]).length() <= 1e-9 {
            return None;
        }
        lifted.push(LiftedBoundary { side, poly });
    }
    if area2 == 0.0 {
        return None;
    }
    Some((chart, area2.signum(), lifted))
}

/// The chart of a stored torus face, read from its outer wire.
///
/// `None` for faces with holes, non-torus faces and faces outside the
/// qualified family.
pub fn face_chart(topo: &Topology, face_id: FaceId) -> Option<(ChartCarrier, PeriodicChart)> {
    let face = topo.face(face_id).ok()?;
    let carrier = ChartCarrier::of(face.surface())?;
    if !face.inner_wires().is_empty() {
        return None;
    }
    let wire = topo.wire(face.outer_wire()).ok()?;
    let mut curves = Vec::with_capacity(wire.edges().len());
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        let ns = topo.vertex(edge.start()).ok()?.point();
        let ne = topo.vertex(edge.end()).ok()?.point();
        let (d0, d1) = if matches!(edge.curve(), EdgeCurve::Line) {
            (0.0, 1.0)
        } else {
            edge.strict_domain().ok()?
        };
        let (ts, te) = if oe.is_forward() { (d0, d1) } else { (d1, d0) };
        curves.push((edge.curve(), ts, te, ns, ne));
    }
    let uses: Vec<BoundaryUse<'_>> = curves
        .iter()
        .map(
            |&(curve, t_start, t_end, native_start, native_end)| BoundaryUse {
                curve,
                t_start,
                t_end,
                native_start,
                native_end,
            },
        )
        .collect();
    let (chart, _, _) = lift_boundary(&carrier, &uses)?;
    Some((carrier, chart))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use remus_math::curves::Circle3D;
    use remus_math::vec::Vec3;

    fn torus() -> ToroidalSurface {
        ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 7.0, 3.0).unwrap()
    }

    /// The meridian circle at `u` and the parallel at `v`, each starting at
    /// the carrier point `(u, v)` and running with its parameter.
    fn meridian(t: &ToroidalSurface, u: f64) -> EdgeCurve {
        let radial = Vec3::new(u.cos(), u.sin(), 0.0);
        let centre = Point3::new(0.0, 0.0, 0.0) + radial * t.major_radius();
        EdgeCurve::Circle(
            Circle3D::new_with_ref(
                centre,
                radial.cross(Vec3::new(0.0, 0.0, 1.0)),
                t.minor_radius(),
                radial,
            )
            .unwrap(),
        )
    }

    fn parallel(t: &ToroidalSurface, v: f64) -> EdgeCurve {
        EdgeCurve::Circle(
            Circle3D::new_with_ref(
                Point3::new(0.0, 0.0, t.minor_radius() * v.sin()),
                Vec3::new(0.0, 0.0, 1.0),
                t.major_radius() + t.minor_radius() * v.cos(),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        )
    }

    fn uses(curves: &[(EdgeCurve, f64, f64)], p: Point3) -> Vec<BoundaryUse<'_>> {
        curves
            .iter()
            .map(|(curve, t_start, t_end)| BoundaryUse {
                curve,
                t_start: *t_start,
                t_end: *t_end,
                native_start: p,
                native_end: p,
            })
            .collect()
    }

    #[test]
    fn a_whole_torus_bounded_by_its_seams_is_periodic_both_ways() {
        let t = torus();
        let tau = std::f64::consts::TAU;
        let curves = [
            (meridian(&t, 0.0), 0.0, tau),
            (parallel(&t, 0.0), 0.0, tau),
            (meridian(&t, 0.0), tau, 0.0),
            (parallel(&t, 0.0), tau, 0.0),
        ];
        let carrier = ChartCarrier::Torus(t.clone());
        let (chart, orientation, sides) =
            lift_boundary(&carrier, &uses(&curves, t.evaluate(0.0, 0.0))).unwrap();
        assert!(chart.periodic_u && chart.periodic_v);
        assert!(orientation != 0.0);
        assert_eq!(sides.len(), 4);
        // Membership is the whole carrier.
        assert!(chart.contains_carrier_point(&carrier, t.evaluate(2.0, 4.0), 0.0));
    }

    #[test]
    fn a_tube_between_two_meridians_is_periodic_in_v_only() {
        let t = torus();
        let tau = std::f64::consts::TAU;
        let (u0, u1) = (0.5, 2.0);
        let curves = [
            (parallel(&t, std::f64::consts::PI), u0, u1),
            (
                meridian(&t, u1),
                std::f64::consts::PI,
                std::f64::consts::PI + tau,
            ),
            (parallel(&t, std::f64::consts::PI), u1, u0),
            (
                meridian(&t, u0),
                std::f64::consts::PI + tau,
                std::f64::consts::PI,
            ),
        ];
        let carrier = ChartCarrier::Torus(t.clone());
        let (chart, _, _) = lift_boundary(
            &carrier,
            &uses(&curves, t.evaluate(u0, std::f64::consts::PI)),
        )
        .unwrap();
        assert!(chart.periodic_v && !chart.periodic_u);
        assert!((chart.u_hi - chart.u_lo - (u1 - u0)).abs() < 1e-9);
        assert!(chart.contains_carrier_point(&carrier, t.evaluate(1.0, 0.3), 1e-9));
        assert!(!chart.contains_carrier_point(&carrier, t.evaluate(3.0, 0.3), 1e-9));
    }

    #[test]
    fn a_patch_bounded_both_ways_declines() {
        let t = torus();
        let (u0, u1, v0, v1) = (0.5, 2.0, 0.2, 1.0);
        let curves = [
            (parallel(&t, v0), u0, u1),
            (meridian(&t, u1), v0, v1),
            (parallel(&t, v1), u1, u0),
            (meridian(&t, u0), v1, v0),
        ];
        let carrier = ChartCarrier::Torus(t.clone());
        assert!(lift_boundary(&carrier, &uses(&curves, t.evaluate(u0, v0))).is_none());
    }
}

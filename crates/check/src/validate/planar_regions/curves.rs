//! Trim-local conservative bounds and bounded line/arc separation proofs.
use super::{CheckError, refuse};
use remus_geometry::bounds::curve::{
    circle_arc_bounds, ellipse_arc_bounds, hyperbola_arc_bounds, line_segment_bounds,
    nurbs_curve_bounds, parabola_arc_bounds,
};
use remus_math::{aabb::Aabb3, nurbs::knot_ops::curve_split, vec::Point3};
use remus_topology::{Topology, edge::EdgeCurve, wire::WireId};

pub(super) struct Span {
    curve: EdgeCurve,
    start: Point3,
    end: Point3,
    a: f64,
    b: f64,
}
impl Span {
    #[allow(clippy::float_cmp)] // Equal endpoints are an empty parameter interval.
    fn bounds(&self) -> Result<Aabb3, CheckError> {
        let (a, b) = (self.a.min(self.b), self.a.max(self.b));
        let bound = match &self.curve {
            EdgeCurve::Line => line_segment_bounds(self.at(a), self.at(b)),
            EdgeCurve::Circle(c) => circle_arc_bounds(c, a, b),
            EdgeCurve::Ellipse(c) => ellipse_arc_bounds(c, a, b),
            EdgeCurve::Hyperbola(c) => hyperbola_arc_bounds(c, a, b),
            EdgeCurve::Parabola(c) => parabola_arc_bounds(c, a, b),
            EdgeCurve::NurbsCurve(c) => {
                // Clip the control net to the actual trim before taking its hull.
                // Using the original carrier's active poles can include distant
                // parts of the curve and needlessly refuse separate holes.
                let (lo, hi) = c.domain();
                if a < lo || b > hi || a == b {
                    return Err(refuse());
                }
                let clipped = if b < hi {
                    curve_split(c, b)?.0
                } else {
                    c.clone()
                };
                let clipped = if a > lo {
                    curve_split(&clipped, a)?.1
                } else {
                    clipped
                };
                nurbs_curve_bounds(&clipped, a, b)
            }
        };
        if !bound.is_prunable() {
            return Err(refuse());
        }
        Ok(bound.aabb())
    }
    fn at(&self, t: f64) -> Point3 {
        self.curve.evaluate_with_endpoints(t, self.start, self.end)
    }
    fn part(&self, a: f64, b: f64) -> Self {
        Self {
            curve: self.curve.clone(),
            start: self.start,
            end: self.end,
            a,
            b,
        }
    }
}

pub(super) struct WireBounds {
    pub(super) bounds: Aabb3,
    pub(super) spans: Vec<Span>,
    pub(super) sample: Point3,
    pub(super) exact: bool,
}
#[allow(clippy::float_cmp)] // Exact empty parameter interval.
pub(super) fn read(
    topo: &Topology,
    wire: WireId,
    normal: remus_math::vec::Vec3,
    d: f64,
    eps: f64,
) -> Result<WireBounds, CheckError> {
    let wire = topo.wire(wire)?;
    if !wire.is_closed() || wire.edges().is_empty() || wire.edges().len() > 256 {
        return Err(refuse());
    }
    let mut spans = Vec::new();
    let mut bounds: Option<Aabb3> = None;
    let mut exact = true;
    for (oe, next) in wire.edges().iter().zip(wire.edges().iter().cycle().skip(1)) {
        let edge = topo.edge(oe.edge())?;
        if oe.oriented_end(edge) != next.oriented_start(topo.edge(next.edge())?) {
            return Err(refuse());
        }
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let (a, b) = edge.domain_with_endpoints(start, end);
        if !a.is_finite() || !b.is_finite() || a == b {
            return Err(refuse());
        }
        exact &= match edge.curve() {
            EdgeCurve::Line => true,
            EdgeCurve::Circle(c) => {
                if c.normal().cross(normal).length() > 1e-12
                    || (normal.dot(c.center() - Point3::new(0., 0., 0.)) - d).abs() > eps
                    || (c.evaluate(a) - start).length() > eps
                    || (c.evaluate(b) - end).length() > eps
                    || (b - a).abs() > std::f64::consts::TAU + 1e-12
                {
                    return Err(refuse());
                }
                true
            }
            _ => crate::distance::edge::is_linear_curve(edge.curve()),
        };
        for p in [start, end] {
            if p.0.iter().any(|v| !v.is_finite())
                || (normal.dot(p - Point3::new(0., 0., 0.)) - d).abs() > eps
            {
                return Err(refuse());
            }
        }
        let span = Span {
            curve: edge.curve().clone(),
            start,
            end,
            a,
            b,
        };
        let box_ = span.bounds()?;
        bounds = Some(bounds.map_or(box_, |old| old.union(box_)));
        spans.push(span);
    }
    let bounds = bounds.ok_or_else(refuse)?;
    let sample = spans[0].at(spans[0].a + (spans[0].b - spans[0].a) * 0.371);
    Ok(WireBounds {
        bounds,
        spans,
        sample,
        exact,
    })
}

// Interval subdivision proves separation on the native curves. No chord or
// sampling approximation can turn a crossing into a decided containment.
// Unresolved contacts and the deterministic work/depth limits refuse.
fn separated(
    a: &Span,
    b: &Span,
    eps: f64,
    depth: usize,
    work: &mut usize,
) -> Result<bool, CheckError> {
    *work += 1;
    if *work > 262_144 {
        return Err(refuse());
    }
    let ab = a.bounds()?.expanded(eps);
    let bb = b.bounds()?.expanded(eps);
    if !ab.intersects(bb) {
        return Ok(true);
    }
    if depth == 48 {
        return Ok(false);
    }
    let size = |box_: Aabb3| (box_.max - box_.min).length_squared();
    if size(ab) >= size(bb) {
        let mid = (a.a + a.b) * 0.5;
        Ok(separated(&a.part(a.a, mid), b, eps, depth + 1, work)?
            && separated(&a.part(mid, a.b), b, eps, depth + 1, work)?)
    } else {
        let mid = (b.a + b.b) * 0.5;
        Ok(separated(a, &b.part(b.a, mid), eps, depth + 1, work)?
            && separated(a, &b.part(mid, b.b), eps, depth + 1, work)?)
    }
}
pub(super) fn wires_separated(
    a: &WireBounds,
    b: &WireBounds,
    eps: f64,
    work: &mut usize,
) -> Result<bool, CheckError> {
    for x in &a.spans {
        for y in &b.spans {
            if !separated(x, y, eps, 0, work)? {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use remus_math::{
        curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D},
        nurbs::NurbsCurve,
        vec::Vec3,
    };

    fn span(curve: EdgeCurve, a: f64, b: f64) -> Span {
        let origin = Point3::new(0., 0., 0.);
        let start = curve.evaluate_with_endpoints(a, origin, origin);
        let end = curve.evaluate_with_endpoints(b, origin, origin);
        Span {
            curve,
            start,
            end,
            a,
            b,
        }
    }
    #[test]
    fn trimmed_bounds_cover_every_curve_type_in_both_directions() {
        let origin = Point3::new(0., 0., 0.);
        let z = Vec3::new(0., 0., 1.);
        let curves = [
            EdgeCurve::Circle(Circle3D::new(origin, z, 3.).unwrap()),
            EdgeCurve::Ellipse(Ellipse3D::new(origin, z, 4., 2.).unwrap()),
            EdgeCurve::Hyperbola(Hyperbola3D::new(origin, z, 3., 1.).unwrap()),
            EdgeCurve::Parabola(Parabola3D::new(origin, z, 2.).unwrap()),
            EdgeCurve::NurbsCurve(
                NurbsCurve::new(
                    2,
                    vec![0., 0., 0., 1., 1., 1.],
                    vec![
                        Point3::new(-100., 0., 0.),
                        origin,
                        Point3::new(100., 0., 0.),
                    ],
                    vec![1.; 3],
                )
                .unwrap(),
            ),
        ];
        for curve in curves {
            for (a, b) in [(0.45, 0.55), (0.55, 0.45)] {
                let s = span(curve.clone(), a, b);
                let bounds = s.bounds().unwrap().expanded(1e-10);
                for i in 0..=100 {
                    assert!(bounds.contains_point(s.at(a + (b - a) * i as f64 / 100.)));
                }
            }
        }
        let s = Span {
            curve: EdgeCurve::Line,
            start: origin,
            end: Point3::new(10., 0., 0.),
            a: 0.4,
            b: 0.6,
        };
        let bounds = s.bounds().unwrap();
        assert!((bounds.min.x() - 4.).abs() < 1e-10 && (bounds.max.x() - 6.).abs() < 1e-10);
    }
    #[test]
    fn nurbs_bounds_clip_the_control_net_to_the_trim() {
        let c = NurbsCurve::new(
            2,
            vec![0., 0., 0., 1., 1., 1.],
            vec![
                Point3::new(-100., 0., 0.),
                Point3::new(0., 0., 0.),
                Point3::new(100., 0., 0.),
            ],
            vec![1.; 3],
        )
        .unwrap();
        let bounds = span(EdgeCurve::NurbsCurve(c), 0.45, 0.55).bounds().unwrap();
        assert!(bounds.min.x() > -10.000_000_1 && bounds.max.x() < 10.000_000_1);
    }
    #[test]
    fn disjoint_arc_trims_do_not_use_the_full_circle_box() {
        let c = Circle3D::new(Point3::new(0., 0., 0.), Vec3::new(0., 0., 1.), 10.).unwrap();
        let a = span(EdgeCurve::Circle(c.clone()), 0.1, 0.2);
        let b = span(EdgeCurve::Circle(c), 3.1, 3.2);
        assert!(!a.bounds().unwrap().intersects(b.bounds().unwrap()));
        let mut work = 0;
        assert!(separated(&a, &b, 1e-7, 0, &mut work).unwrap());
        assert_eq!(work, 1);
    }
}

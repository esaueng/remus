//! Inner-wire nesting from trim-local bounds and exact line/circle ray events.
//!
//! Disjoint conservative boxes prove unnested pairs for every curve carrier.
//! Overlapping pairs require exact readers and a native-curve separation proof.
//! An unreadable outer wire never prevents an inner-vs-inner decision.
mod curves;
use crate::CheckError;
use remus_math::vec::{Point2, Vec2, Vec3};
use remus_topology::{
    Topology,
    edge::EdgeCurve,
    face::{FaceId, FaceSurface},
    wire::WireId,
};

/// Inner-wire depths and immediate parents, parallel to `Face::inner_wires`.
/// Depth zero is a hole; depth one is an island. `None` means the outer wire.
pub struct PlanarRegions {
    /// Number of enclosing inner wires.
    pub depths: Vec<usize>,
    /// Index of the nearest enclosing inner wire.
    pub parents: Vec<Option<usize>>,
}

enum Boundary {
    Polygon(Vec<Point2>),
    Circle(Point2, f64),
}
fn refuse() -> CheckError {
    CheckError::ValidationFailed("profile nesting requires separated simple boundaries, decidable overlapping line/circle arcs and the bounded work budget".into())
}
fn distance(p: Point2, a: Point2, b: Point2) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0., 1.);
    (p - (a + ab * t)).length()
}
fn segments_touch(a: Point2, b: Point2, c: Point2, d: Point2, eps: f64) -> bool {
    let opposite = |u: f64, v: f64| (u > 0. && v < 0.) || (u < 0. && v > 0.);
    (opposite(
        remus_math::predicates::orient2d(a, b, c),
        remus_math::predicates::orient2d(a, b, d),
    ) && opposite(
        remus_math::predicates::orient2d(c, d, a),
        remus_math::predicates::orient2d(c, d, b),
    )) || distance(a, c, d)
        .min(distance(b, c, d))
        .min(distance(c, a, b))
        .min(distance(d, a, b))
        <= eps
}
impl Boundary {
    fn sample(&self) -> Point2 {
        match self {
            Self::Polygon(p) => p[0],
            Self::Circle(c, r) => *c + Vec2::new(*r, 0.),
        }
    }
    fn contains(&self, p: Point2) -> bool {
        match self {
            Self::Circle(c, r) => (p - *c).length() < *r,
            Self::Polygon(points) => remus_math::predicates::winding_number(p, points) != 0,
        }
    }
    fn touches(&self, other: &Self, eps: f64) -> bool {
        match (self, other) {
            (Self::Circle(a, r), Self::Circle(b, s)) => {
                let d = (*a - *b).length();
                d <= r + s + eps && d >= (r - s).abs() - eps
            }
            (Self::Polygon(p), Self::Circle(c, r)) | (Self::Circle(c, r), Self::Polygon(p)) => {
                p.iter().zip(p.iter().cycle().skip(1)).any(|(&a, &b)| {
                    distance(*c, a, b) <= r + eps
                        && (a - *c).length().max((b - *c).length()) >= r - eps
                })
            }
            (Self::Polygon(p), Self::Polygon(q)) => {
                p.iter().zip(p.iter().cycle().skip(1)).any(|(&a, &b)| {
                    q.iter()
                        .zip(q.iter().cycle().skip(1))
                        .any(|(&c, &d)| segments_touch(a, b, c, d, eps))
                })
            }
        }
    }
}
fn read(
    topo: &Topology,
    wire: WireId,
    normal: Vec3,
    d: f64,
    eps: f64,
) -> Result<Boundary, CheckError> {
    let wire = topo.wire(wire)?;
    let axis = if normal.x().abs() < 0.8 {
        Vec3::new(1., 0., 0.)
    } else {
        Vec3::new(0., 1., 0.)
    };
    let u = normal.cross(axis).normalize()?;
    let v = normal.cross(u);
    let project = |p: remus_math::vec::Point3| {
        Point2::new(
            u.x() * p.x() + u.y() * p.y() + u.z() * p.z(),
            v.x() * p.x() + v.y() * p.y() + v.z() * p.z(),
        )
    };
    let in_plane = |p: remus_math::vec::Point3| {
        (normal.x() * p.x() + normal.y() * p.y() + normal.z() * p.z() - d).abs() <= eps
    };
    if !wire.is_closed() || wire.edges().is_empty() || wire.edges().len() > 256 {
        return Err(refuse());
    }
    if wire.edges().len() == 1 {
        let edge = topo.edge(wire.edges()[0].edge())?;
        if let EdgeCurve::Circle(circle) = edge.curve() {
            let seam = topo.vertex(edge.start())?.point();
            if edge.start() == edge.end()
                && edge.trim().is_none_or(|(a, b)| {
                    a.is_finite()
                        && b.is_finite()
                        && (b - a - std::f64::consts::TAU).abs() < 1e-12
                        && (circle.evaluate(a) - seam).length() <= eps
                })
                && in_plane(circle.center())
                && circle.normal().cross(normal).length() < 1e-12
                && circle.radius() > eps
                && ((seam - circle.center()).length() - circle.radius()).abs() <= eps
                && in_plane(seam)
            {
                return Ok(Boundary::Circle(project(circle.center()), circle.radius()));
            }
        }
        return Err(refuse());
    }
    let mut points = Vec::new();
    for (oe, next) in wire.edges().iter().zip(wire.edges().iter().cycle().skip(1)) {
        let edge = topo.edge(oe.edge())?;
        let next_edge = topo.edge(next.edge())?;
        let (start, end) = if oe.is_forward() {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        let next_start = if next.is_forward() {
            next_edge.start()
        } else {
            next_edge.end()
        };
        let p = topo.vertex(start)?.point();
        if !crate::distance::edge::is_linear_curve(edge.curve())
            || end != next_start
            || !in_plane(p)
            || !p.x().is_finite()
            || !p.y().is_finite()
            || !p.z().is_finite()
        {
            return Err(refuse());
        }
        points.push(project(p));
    }
    if points.len() < 3 {
        return Err(refuse());
    }
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        let prev = points[(i + points.len() - 1) % points.len()];
        if (a - b).length() <= eps || distance(prev, a, b) <= eps || distance(b, prev, a) <= eps {
            return Err(refuse());
        }
        for j in i + 2..points.len() {
            if i == 0 && j + 1 == points.len() {
                continue;
            }
            if segments_touch(a, b, points[j], points[(j + 1) % points.len()], eps) {
                return Err(refuse());
            }
        }
    }
    Ok(Boundary::Polygon(points))
}

/// Read inner-wire containment, independently of the outer wire's carriers.
///
/// Valid faces have inner wires inside their outer boundary. When that boundary
/// and an inner boundary are readable as polygons/full circles, also reject
/// invalid outside/contacting inputs. Unreadable outers do not gate nesting.
/// Pairwise disjoint trim-local boxes prove unnested wires of any curve type.
/// Overlapping boxes require exact line/circle-arc membership and separation.
/// At most 63 inner wires, 256 edges per wire and 262_144 separation steps.
///
/// # Errors
/// Refuses contacting, crossing, non-planar, over-budget or unresolved pairs,
/// and propagates missing topology handles.
pub fn planar_regions(topo: &Topology, face: FaceId) -> Result<PlanarRegions, CheckError> {
    use crate::distance::plane_trim::PlanarWire;
    let face = topo.face(face)?;
    let FaceSurface::Plane { normal, d } = face.surface() else {
        return Err(refuse());
    };
    if !normal.length().is_finite()
        || (normal.length() - 1.).abs() > 1e-10
        || !d.is_finite()
        || face.inner_wires().len() > 63
    {
        return Err(refuse());
    }
    let eps = remus_math::tolerance::Tolerance::new().linear;
    let outer = read(topo, face.outer_wire(), *normal, *d, eps).ok();
    let mut boundaries = Vec::new();
    let mut readers = Vec::new();
    let mut work = 0;
    for &wire in face.inner_wires() {
        let bounds = curves::read(topo, wire, *normal, *d, eps)?;
        // Retain structural validation of line polygons, including bow ties.
        let all_lines = topo.wire(wire)?.edges().iter().all(|oe| {
            topo.edge(oe.edge())
                .is_ok_and(|e| crate::distance::edge::is_linear_curve(e.curve()))
        });
        let boundary = if all_lines {
            Some(read(topo, wire, *normal, *d, eps)?)
        } else {
            read(topo, wire, *normal, *d, eps).ok()
        };
        if let (Some(outer), Some(inner)) = (&outer, &boundary)
            && (outer.touches(inner, eps) || !outer.contains(inner.sample()))
        {
            return Err(refuse());
        }
        readers.push(if bounds.exact {
            Some(PlanarWire::build(topo, wire, *normal)?)
        } else {
            None
        });
        boundaries.push(bounds);
    }
    let n = boundaries.len();
    let mut contains = vec![vec![false; n]; n];
    for i in 0..n {
        for j in i + 1..n {
            if !boundaries[i]
                .bounds
                .expanded(eps)
                .intersects(boundaries[j].bounds.expanded(eps))
            {
                continue;
            }
            let (Some(a), Some(b)) = (&readers[i], &readers[j]) else {
                return Err(refuse());
            };
            if !curves::wires_separated(&boundaries[i], &boundaries[j], eps, &mut work)? {
                return Err(refuse());
            }
            contains[i][j] = a.contains(boundaries[j].sample)?;
            contains[j][i] = b.contains(boundaries[i].sample)?;
            if contains[i][j] && contains[j][i] {
                return Err(refuse());
            }
        }
    }
    let depths: Vec<_> = (0..n)
        .map(|i| contains.iter().filter(|row| row[i]).count())
        .collect();
    let parents = (0..n)
        .map(|i| (0..n).find(|&j| contains[j][i] && depths[j] + 1 == depths[i]))
        .collect();
    Ok(PlanarRegions { depths, parents })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use remus_math::vec::Point3;
    use remus_topology::{builder::make_polygon_wire, face::Face};
    fn square(topo: &mut Topology, r: f64, cw: bool) -> WireId {
        let mut p = vec![
            Point3::new(-r, -r, 5.),
            Point3::new(r, -r, 5.),
            Point3::new(r, r, 5.),
            Point3::new(-r, r, 5.),
        ];
        if cw {
            p.reverse();
        }
        make_polygon_wire(topo, &p, 1e-7).unwrap()
    }
    fn face(topo: &mut Topology, wrong_island: bool) -> FaceId {
        let outer = square(topo, 10., false);
        let hole = square(topo, 5., true);
        let island = square(topo, 3., wrong_island);
        let bore = square(topo, 1., true);
        // Deliberately not sorted by depth.
        topo.add_face(Face::new(
            outer,
            vec![island, bore, hole],
            FaceSurface::Plane {
                normal: Vec3::new(0., 0., 1.),
                d: 5.,
            },
        ))
    }
    #[test]
    fn planar_regions_find_parents_independent_of_wire_order() {
        let mut topo = Topology::new();
        let face = face(&mut topo, false);
        let r = planar_regions(&topo, face).unwrap();
        assert_eq!(r.depths, vec![1, 2, 0]);
        assert_eq!(r.parents, vec![Some(2), Some(0), None]);
    }
    #[test]
    fn planar_orientation_accepts_alternating_depth_and_face_reversal() {
        let mut topo = Topology::new();
        let face = face(&mut topo, false);
        for reversed in [false, true] {
            topo.face_mut(face).unwrap().set_reversed(reversed);
            assert!(
                crate::validate::check_face_inner_wire_orientation(&topo, face)
                    .unwrap()
                    .is_empty()
            );
        }
    }
    #[test]
    fn planar_orientation_rejects_wrong_island_winding() {
        let mut topo = Topology::new();
        let face = face(&mut topo, true);
        let issues = crate::validate::check_face_inner_wire_orientation(&topo, face).unwrap();
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].severity, crate::validate::Severity::Error);
    }
    #[test]
    fn planar_regions_refuse_self_crossing_and_budget_exhaustion() {
        let mut topo = Topology::new();
        let outer = square(&mut topo, 10., false);
        let bow = make_polygon_wire(
            &mut topo,
            &[
                Point3::new(-2., -2., 5.),
                Point3::new(2., 2., 5.),
                Point3::new(-2., 2., 5.),
                Point3::new(2., -2., 5.),
            ],
            1e-7,
        )
        .unwrap();
        let id = topo.add_face(Face::new(
            outer,
            vec![bow],
            FaceSurface::Plane {
                normal: Vec3::new(0., 0., 1.),
                d: 5.,
            },
        ));
        assert!(planar_regions(&topo, id).is_err());
        let id = topo.add_face(Face::new(
            outer,
            vec![bow; 64],
            FaceSurface::Plane {
                normal: Vec3::new(0., 0., 1.),
                d: 5.,
            },
        ));
        assert!(planar_regions(&topo, id).is_err());
    }
}

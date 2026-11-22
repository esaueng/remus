//! Compare a rectangular plate cap with the polygonal region its mesh samples.
//! Only disjoint, interior, single-circle holes are eligible. A fan from the
//! center triangulates each convex sampled rim. Subtract their areas from the
//! sampled rectangle. All other configurations use the existing CDT probe.

use crate::tessellate::edge_sampling::sample_wire_positions;
use remus_math::vec::Point3;
use remus_topology::{
    Topology,
    edge::EdgeCurve,
    face::{FaceId, FaceSurface},
    wire::WireId,
};

const CLEARANCE: f64 = 1e-4;
const COORD_LIMIT: f64 = 1e4;

#[derive(Clone, Copy)]
struct Bounds {
    x0: f64,
    x1: f64,
    y0: f64,
    y1: f64,
}
impl Bounds {
    fn contains(self, h: Self) -> bool {
        h.x0 > self.x0 + CLEARANCE
            && h.x1 < self.x1 - CLEARANCE
            && h.y0 > self.y0 + CLEARANCE
            && h.y1 < self.y1 - CLEARANCE
    }
}

pub(super) fn sampled_area_agrees(
    topo: &Topology,
    face_id: FaceId,
    area: f64,
    probe: f64,
    budget: f64,
) -> bool {
    region_area(topo, face_id, probe).is_some_and(|(sampled, roundoff)| {
        budget.is_finite() && (area - sampled).abs() + roundoff < 0.5 * budget
    })
}

// Exact coordinate comparisons restrict this path to horizontal rectangles.
// The sampled XY differences below keep their signs and magnitude far from
// both floating-point roundoff and the CDT's vertex/constraint thresholds.
#[allow(clippy::float_cmp)]
fn region_area(topo: &Topology, face_id: FaceId, probe: f64) -> Option<(f64, f64)> {
    let face = topo.face(face_id).ok()?;
    let FaceSurface::Plane { normal, .. } = face.surface() else {
        return None;
    };
    if normal.x() != 0.
        || normal.y() != 0.
        || normal.z().abs() != 1.
        || !(2..=128).contains(&face.inner_wires().len())
    {
        return None;
    }
    let (outer, z) = rectangle(topo, face.outer_wire())?;
    let mut sampled = (outer.x1 - outer.x0) * (outer.y1 - outer.y0);
    let mut vertices = 4u32;
    let mut circles: Vec<(Point3, f64)> = Vec::with_capacity(face.inner_wires().len());
    for &wire_id in face.inner_wires() {
        let hole = topo.wire(wire_id).ok()?;
        let [oe] = hole.edges() else {
            return None;
        };
        let edge = topo.edge(oe.edge()).ok()?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            return None;
        };
        let (a, b) = edge.strict_domain().ok()?;
        let c = circle.center();
        let r = circle.radius();
        if !hole.is_closed()
            || !edge.is_closed()
            || (a - b).abs() != std::f64::consts::TAU
            || circle.normal().z().abs() != 1.
            || circle.normal().x() != 0.
            || circle.normal().y() != 0.
            || c.z() != z
            || !(0.01..=COORD_LIMIT).contains(&r)
        {
            return None;
        }
        let bounds = Bounds {
            x0: c.x() - r,
            x1: c.x() + r,
            y0: c.y() - r,
            y1: c.y() + r,
        };
        if !outer.contains(bounds)
            || circles.iter().any(|&(other, radius)| {
                let minimum = radius + r + CLEARANCE;
                (other - c).length_squared() <= minimum * minimum
            })
        {
            return None;
        }
        circles.push((c, r));
        let rim = sample_wire_positions(
            topo,
            hole,
            1e-10,
            probe,
            remus_math::chord::DEFAULT_ANGULAR_TOL,
        )
        .ok()?;
        if !(8..=128).contains(&rim.len()) {
            return None;
        }
        sampled -= fan_area(&rim, c, r, z)?;
        vertices += u32::try_from(rim.len()).ok()?;
    }
    let n = f64::from(vertices + 2 * u32::try_from(circles.len()).ok()?);
    // Each triangle's area is bounded by the box area. Allow error from
    // coordinate subtraction, products, norms and the ordered mesh sum.
    let roundoff = 128. * n * n * f64::EPSILON * COORD_LIMIT * COORD_LIMIT;
    (sampled > 0.).then_some((sampled, roundoff))
}

#[allow(clippy::float_cmp)]
fn rectangle(topo: &Topology, wire_id: WireId) -> Option<(Bounds, f64)> {
    let wire = topo.wire(wire_id).ok()?;
    if wire.edges().len() != 4 || !wire.is_closed() {
        return None;
    }
    let (mut starts, mut ends) = ([None; 4], [None; 4]);
    let mut points = [Point3::new(0., 0., 0.); 4];
    for (i, oe) in wire.edges().iter().enumerate() {
        let e = topo.edge(oe.edge()).ok()?;
        if !matches!(e.curve(), EdgeCurve::Line) {
            return None;
        }
        let (a, b) = if oe.is_forward() {
            (e.start(), e.end())
        } else {
            (e.end(), e.start())
        };
        starts[i] = Some(a);
        ends[i] = Some(b);
        points[i] = topo.vertex(a).ok()?.point();
    }
    let p = points[0];
    let q = points[2];
    let outer = Bounds {
        x0: p.x().min(q.x()),
        x1: p.x().max(q.x()),
        y0: p.y().min(q.y()),
        y1: p.y().max(q.y()),
    };
    let width = outer.x1 - outer.x0;
    let height = outer.y1 - outer.y0;
    if width <= CLEARANCE || height <= CLEARANCE {
        return None;
    }
    let mut corners = 0u8;
    for (i, &a) in points.iter().enumerate() {
        let j = (i + 1) % 4;
        let b = points[j];
        if ends[i] != starts[j]
            || a.z() != p.z()
            || !(a.x().abs() <= COORD_LIMIT
                && a.y().abs() <= COORD_LIMIT
                && a.z().abs() <= COORD_LIMIT)
            || ((a.x() == b.x()) == (a.y() == b.y()))
        {
            return None;
        }
        if !(a.x() == outer.x0 || a.x() == outer.x1) || !(a.y() == outer.y0 || a.y() == outer.y1) {
            return None;
        }
        corners |= 1 << (u8::from(a.x() == outer.x1) + 2 * u8::from(a.y() == outer.y1));
    }
    if corners != 15 {
        return None;
    }
    Some((outer, p.z()))
}

#[allow(clippy::float_cmp)]
fn fan_area(rim: &[Point3], c: Point3, r: f64, z: f64) -> Option<f64> {
    let mut twice_area = 0.;
    let mut sign = 0.;
    for (&a, &b) in rim.iter().zip(rim.iter().cycle().skip(1)) {
        let pa = a - c;
        let pb = b - c;
        let separated = (a - b).length() > CLEARANCE;
        let on_circle = (pa.length() - r).abs() < CLEARANCE / 64.;
        if a.z() != z || !separated || !on_circle {
            return None;
        }
        let cross = pa.x() * pb.y() - pa.y() * pb.x();
        if !cross.is_finite() || cross.abs() <= CLEARANCE * CLEARANCE {
            return None;
        }
        if sign != 0. && sign != cross.signum() {
            return None;
        }
        sign = cross.signum();
        twice_area += cross;
    }
    Some(twice_area.abs() * 0.5)
}

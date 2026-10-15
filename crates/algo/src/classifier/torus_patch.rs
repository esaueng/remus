//! Recognition of bounded rectangular torus patches.
use remus_math::tolerance::Tolerance;
use remus_topology::{Topology, face::FaceId};

/// Recognize a four-arc rectangular torus patch in unwrapped parameter space.
/// Returns each angular interval as (start, span).
pub fn rectangular_torus_domain(
    topo: &Topology,
    face_id: FaceId,
    tol: Tolerance,
) -> Option<((f64, f64), (f64, f64))> {
    use remus_math::vec::Point2;
    use remus_topology::{edge::EdgeCurve, face::FaceSurface};
    let face = topo.face(face_id).ok()?;
    let FaceSurface::Torus(torus) = face.surface() else {
        return None;
    };
    if !face.inner_wires().is_empty() {
        return None;
    }
    let edges = topo.wire(face.outer_wire()).ok()?.edges();
    if edges.len() != 4 {
        return None;
    }
    let angular_tol = tol.angular.max(tol.linear / torus.minor_radius());
    let mut polygon: Vec<Point2> = Vec::new();
    let mut previous: Option<Point2> = None;
    for oe in edges {
        let edge = topo.edge(oe.edge()).ok()?;
        if !matches!(edge.curve(), EdgeCurve::Circle(_)) {
            return None;
        }
        let (mut t0, mut t1) = edge.strict_domain().ok()?;
        if !oe.is_forward() {
            std::mem::swap(&mut t0, &mut t1);
        }
        let a = topo.vertex(edge.start()).ok()?.point();
        let b = topo.vertex(edge.end()).ok()?.point();
        let mut points = Vec::new();
        for k in 0..=8 {
            let p = edge.curve().evaluate_with_endpoints(
                (t1 - t0).mul_add(f64::from(k) / 8.0, t0),
                a,
                b,
            );
            let (mut u, mut v) = face.surface().project_point(p)?;
            let on_surface = face.surface().evaluate(u, v)?;
            if !u.is_finite()
                || !v.is_finite()
                || (on_surface - p).length()
                    > tol.linear.max(edge.tolerance().unwrap_or(tol.linear))
            {
                return None;
            }
            if let Some(last) = previous {
                let tau = std::f64::consts::TAU;
                u += tau * ((last.x() - u) / tau).round();
                v += tau * ((last.y() - v) / tau).round();
            }
            let uv = Point2::new(u, v);
            previous = Some(uv);
            points.push(uv);
        }
        let first = points[0];
        let constant_u = points
            .iter()
            .all(|p| (p.x() - first.x()).abs() <= angular_tol);
        let constant_v = points
            .iter()
            .all(|p| (p.y() - first.y()).abs() <= angular_tol);
        if constant_u == constant_v {
            return None;
        }
        polygon.extend(points);
    }
    let first = polygon.first()?;
    let last = polygon.last()?;
    if (*last - *first).length() > angular_tol {
        return None;
    }
    let u0 = polygon.iter().map(|p| p.x()).fold(f64::INFINITY, f64::min);
    let u1 = polygon
        .iter()
        .map(|p| p.x())
        .fold(f64::NEG_INFINITY, f64::max);
    let v0 = polygon.iter().map(|p| p.y()).fold(f64::INFINITY, f64::min);
    let v1 = polygon
        .iter()
        .map(|p| p.y())
        .fold(f64::NEG_INFINITY, f64::max);
    if u1 - u0 <= angular_tol || v1 - v0 <= angular_tol {
        return None;
    }
    let centre = Point2::new(f64::midpoint(u0, u1), f64::midpoint(v0, v1));
    if !crate::builder::classify_2d::point_in_polygon_2d(centre, &polygon) {
        return None;
    }
    Some(((u0, u1 - u0), (v0, v1 - v0)))
}

/// A local torus trim. The carrier is exact; this polygon approximates only
/// trim membership with adaptive sampling. `margin` includes the sampling
/// target and observed projection error; near-trim ray hits remain suspicious.
#[derive(Debug, Clone, PartialEq)]
pub struct TorusTrim {
    pub polygon: Vec<remus_math::vec::Point2>,
    pub centre: remus_math::vec::Point2,
    pub margin: f64,
}

impl TorusTrim {
    /// Bytes the polygon holds.
    pub const fn heap_bytes(&self) -> usize {
        self.polygon.len() * std::mem::size_of::<remus_math::vec::Point2>()
    }
}

/// [`trimmed_torus_polygon`] through this thread's ray-geometry memo when it
/// is on and the face has a key (see [`trim_key`]).
pub(super) fn trimmed_torus_polygon_memoized(
    topo: &Topology,
    face_id: FaceId,
    tol: Tolerance,
) -> Option<TorusTrim> {
    if crate::ray_geom_memo::enabled()
        && let Some(key) = trim_key(topo, face_id, tol)
    {
        return crate::ray_geom_memo::memoized(&key, || trimmed_torus_polygon(topo, face_id, tol));
    }
    trimmed_torus_polygon(topo, face_id, tol)
}

/// Leading word of a torus-trim key in the ray-geometry memo.
const TORUS_TRIM_KEY: u64 = 1;

/// The exact bits [`trimmed_torus_polygon`] reads: the tolerance, the torus,
/// then per outer-wire edge in traversal order its direction, domain,
/// endpoints, tolerance and curve. Counts and tags make the encoding
/// prefix-free. `None` when the builder would decline before reading an edge
/// (not a hole-free ring torus), for a hyperbola or parabola edge, or when a
/// lookup or domain fails (the builder then declines on its own).
fn trim_key(topo: &Topology, face_id: FaceId, tol: Tolerance) -> Option<Vec<u64>> {
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::{edge::EdgeCurve, face::FaceSurface};
    fn point(key: &mut Vec<u64>, p: Point3) {
        key.extend([p.x(), p.y(), p.z()].map(f64::to_bits));
    }
    fn vector(key: &mut Vec<u64>, v: Vec3) {
        key.extend([v.x(), v.y(), v.z()].map(f64::to_bits));
    }
    fn count(key: &mut Vec<u64>, n: usize) {
        key.push(n as u64);
    }
    let face = topo.face(face_id).ok()?;
    let FaceSurface::Torus(torus) = face.surface() else {
        return None;
    };
    if !face.inner_wires().is_empty() || torus.major_radius() <= torus.minor_radius() {
        return None;
    }
    let edges = topo.wire(face.outer_wire()).ok()?.edges();
    let mut key = Vec::with_capacity(16 + 32 * edges.len());
    key.push(TORUS_TRIM_KEY);
    key.extend([tol.linear, tol.angular, tol.relative].map(f64::to_bits));
    point(&mut key, torus.center());
    key.extend([torus.major_radius(), torus.minor_radius()].map(f64::to_bits));
    vector(&mut key, torus.x_axis());
    vector(&mut key, torus.y_axis());
    vector(&mut key, torus.z_axis());
    count(&mut key, edges.len());
    for oe in edges {
        let edge = topo.edge(oe.edge()).ok()?;
        let (t0, t1) = edge.strict_domain().ok()?;
        key.push(u64::from(oe.is_forward()));
        key.extend([t0, t1].map(f64::to_bits));
        point(&mut key, topo.vertex(edge.start()).ok()?.point());
        point(&mut key, topo.vertex(edge.end()).ok()?.point());
        match edge.tolerance() {
            Some(t) => key.extend([1, t.to_bits()]),
            None => key.push(0),
        }
        match edge.curve() {
            EdgeCurve::Line => key.push(0),
            EdgeCurve::Circle(c) => {
                key.push(1);
                point(&mut key, c.center());
                vector(&mut key, c.normal());
                key.push(c.radius().to_bits());
                vector(&mut key, c.u_axis());
                vector(&mut key, c.v_axis());
            }
            EdgeCurve::Ellipse(e) => {
                key.push(2);
                point(&mut key, e.center());
                vector(&mut key, e.normal());
                key.extend([e.semi_major(), e.semi_minor()].map(f64::to_bits));
                vector(&mut key, e.u_axis());
                vector(&mut key, e.v_axis());
            }
            EdgeCurve::NurbsCurve(n) => {
                key.push(3);
                count(&mut key, n.degree());
                count(&mut key, n.knots().len());
                key.extend(n.knots().iter().map(|k| k.to_bits()));
                count(&mut key, n.control_points().len());
                for &p in n.control_points() {
                    point(&mut key, p);
                }
                count(&mut key, n.weights().len());
                key.extend(n.weights().iter().map(|w| w.to_bits()));
            }
            EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => return None,
        }
    }
    Some(key)
}

/// Recognize a hole-free local patch (less than half a turn in each angle).
/// Adaptively project authoritative edge trims, rejecting non-finite curves,
/// non-closing loops, and exhausted sampling budgets. Unsupported patches keep
/// the existing classifier path.
pub(super) fn trimmed_torus_polygon(
    topo: &Topology,
    face_id: FaceId,
    tol: Tolerance,
) -> Option<TorusTrim> {
    use remus_math::vec::Point2;
    use remus_topology::face::FaceSurface;
    let face = topo.face(face_id).ok()?;
    let FaceSurface::Torus(torus) = face.surface() else {
        return None;
    };
    if !face.inner_wires().is_empty() || torus.major_radius() <= torus.minor_radius() {
        return None;
    }
    let metric = torus
        .minor_radius()
        .min(torus.major_radius() - torus.minor_radius());
    let margin = tol.angular.max(tol.linear / metric);
    let mut polygon: Vec<Point2> = Vec::new();
    let mut closure_margin = margin;
    let projection_margin = std::cell::Cell::new(margin);
    for oe in topo.wire(face.outer_wire()).ok()?.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        let (mut t0, mut t1) = edge.strict_domain().ok()?;
        if !oe.is_forward() {
            std::mem::swap(&mut t0, &mut t1);
        }
        let a = topo.vertex(edge.start()).ok()?.point();
        let b = topo.vertex(edge.end()).ok()?.point();
        let allowance = tol.linear.max(edge.tolerance().unwrap_or(tol.linear));
        closure_margin = closure_margin.max(allowance / metric);
        let evaluate = |t: f64, anchor: Point2| -> Option<Point2> {
            let p = edge.curve().evaluate_with_endpoints(t, a, b);
            let (u, v) = torus.project_point(p);
            let residual = (torus.evaluate(u, v) - p).length();
            if !u.is_finite() || !v.is_finite() || !residual.is_finite() {
                return None;
            }
            // Imported space curves can deviate from their supporting surface.
            // Account for that projection in classifier uncertainty only; never
            // change the edge tolerance or the geometry acceptance policy.
            projection_margin.set(projection_margin.get().max(residual / metric));
            Some(unwrap_uv(Point2::new(u, v), anchor))
        };
        let start = evaluate(t0, polygon.last().copied().unwrap_or(Point2::new(0.0, 0.0)))?;
        if let Some(last) = polygon.last()
            && (*last - start).length() > closure_margin.max(projection_margin.get())
        {
            return None;
        }
        polygon.push(start);
        // Seed short spans so periodic unwrapping never chooses a complementary
        // arc from a large endpoint-only jump.
        for k in 0..16 {
            let lo = (t1 - t0).mul_add(f64::from(k) / 16.0, t0);
            let hi = (t1 - t0).mul_add(f64::from(k + 1) / 16.0, t0);
            let left = *polygon.last()?;
            let right = evaluate(hi, left)?;
            sample_trim_segment(&evaluate, lo, hi, left, right, margin, 0, &mut polygon)?;
        }
    }
    if (*polygon.last()? - *polygon.first()?).length() > closure_margin.max(projection_margin.get())
    {
        return None;
    }
    let mut lo = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut hi = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in &polygon {
        lo = Point2::new(lo.x().min(p.x()), lo.y().min(p.y()));
        hi = Point2::new(hi.x().max(p.x()), hi.y().max(p.y()));
    }
    let span = hi - lo;
    if span.x() <= margin
        || span.y() <= margin
        || span.x() >= std::f64::consts::PI
        || span.y() >= std::f64::consts::PI
    {
        return None;
    }
    Some(TorusTrim {
        polygon,
        centre: Point2::new(f64::midpoint(lo.x(), hi.x()), f64::midpoint(lo.y(), hi.y())),
        margin: closure_margin.max(projection_margin.get()) + margin,
    })
}

fn unwrap_uv(
    p: remus_math::vec::Point2,
    anchor: remus_math::vec::Point2,
) -> remus_math::vec::Point2 {
    let tau = std::f64::consts::TAU;
    remus_math::vec::Point2::new(
        p.x() + tau * ((anchor.x() - p.x()) / tau).round(),
        p.y() + tau * ((anchor.y() - p.y()) / tau).round(),
    )
}

#[allow(clippy::too_many_arguments)]
fn sample_trim_segment(
    evaluate: &impl Fn(f64, remus_math::vec::Point2) -> Option<remus_math::vec::Point2>,
    lo: f64,
    hi: f64,
    a: remus_math::vec::Point2,
    b: remus_math::vec::Point2,
    margin: f64,
    depth: u8,
    polygon: &mut Vec<remus_math::vec::Point2>,
) -> Option<()> {
    if polygon.len() >= 16384 || depth >= 20 {
        return None;
    }
    let mut error = 0.0_f64;
    for fraction in [0.25, 0.5, 0.75] {
        let expected = a + (b - a) * fraction;
        let actual = evaluate((hi - lo).mul_add(fraction, lo), expected)?;
        error = error.max((actual - expected).length());
    }
    if error <= margin {
        polygon.push(b);
        return Some(());
    }
    let mid = f64::midpoint(lo, hi);
    let p = evaluate(mid, a + (b - a) * 0.5)?;
    sample_trim_segment(evaluate, lo, mid, a, p, margin, depth + 1, polygon)?;
    sample_trim_segment(evaluate, mid, hi, p, b, margin, depth + 1, polygon)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use remus_math::{
        curves::Circle3D,
        surfaces::ToroidalSurface,
        vec::{Point3, Vec3},
    };
    use remus_topology::{
        edge::{Edge, EdgeCurve},
        face::{Face, FaceSurface},
        vertex::Vertex,
        wire::{OrientedEdge, Wire},
    };

    #[test]
    fn projected_trim_preserves_split_arcs_winding_and_periodic_seams() {
        for scale in [0.01, 1.0, 100.0] {
            for (u0, v0) in [(0.3, 0.4), (5.8, 5.9)] {
                let u1 = u0 + 1.2;
                let v1 = v0 + 1.0;
                let um = f64::midpoint(u0, u1);
                let t = ToroidalSurface::with_axis(
                    Point3::new(2.0, -3.0, 4.0),
                    12.0 * scale,
                    3.0 * scale,
                    Vec3::new(1.0, 2.0, 3.0),
                )
                .unwrap();
                let major = |v: f64| {
                    Circle3D::new_with_ref(
                        t.center() + t.z_axis() * (t.minor_radius() * v.sin()),
                        t.z_axis(),
                        t.major_radius() + t.minor_radius() * v.cos(),
                        t.x_axis(),
                    )
                    .unwrap()
                };
                let minor = |u: f64| {
                    let radial = t.x_axis() * u.cos() + t.y_axis() * u.sin();
                    Circle3D::new_with_ref(
                        t.center() + radial * t.major_radius(),
                        radial.cross(t.z_axis()),
                        t.minor_radius(),
                        radial,
                    )
                    .unwrap()
                };
                let mut topo = Topology::new();
                let vertices = [(u0, v0), (um, v0), (u1, v0), (u1, v1), (u0, v1)]
                    .map(|(u, v)| topo.add_vertex(Vertex::new(t.evaluate(u, v), 1e-7)));
                let curves = [
                    (major(v0), (u0, um)),
                    (major(v0), (um, u1)),
                    (minor(u1), (v0, v1)),
                    (major(v1), (u1, u0)),
                    (minor(u0), (v1, v0)),
                ];
                let edges: Vec<_> = curves
                    .into_iter()
                    .enumerate()
                    .map(|(i, (c, domain))| {
                        let mut e =
                            Edge::new(vertices[i], vertices[(i + 1) % 5], EdgeCurve::Circle(c));
                        e.set_trim(Some(domain));
                        OrientedEdge::new(topo.add_edge(e), true)
                    })
                    .collect();
                let reverse: Vec<_> = edges
                    .iter()
                    .rev()
                    .map(|e| OrientedEdge::new(e.edge(), false))
                    .collect();
                for winding in [edges, reverse] {
                    let wire = topo.add_wire(Wire::new(winding, true).unwrap());
                    let face =
                        topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(t.clone())));
                    assert!(rectangular_torus_domain(&topo, face, Tolerance::default()).is_none());
                    let trim = trimmed_torus_polygon(&topo, face, Tolerance::default()).unwrap();
                    let middle = unwrap_uv(
                        remus_math::vec::Point2::new(um, f64::midpoint(v0, v1)),
                        trim.centre,
                    );
                    assert!(crate::builder::classify_2d::point_in_polygon_2d(
                        middle,
                        &trim.polygon
                    ));
                    assert!(
                        crate::builder::classify_2d::distance_to_polygon_boundary(
                            middle,
                            &trim.polygon
                        ) > 0.49
                    );
                    let outside = unwrap_uv(
                        remus_math::vec::Point2::new(u1 + 0.1, v0 + 0.5),
                        trim.centre,
                    );
                    assert!(!crate::builder::classify_2d::point_in_polygon_2d(
                        outside,
                        &trim.polygon
                    ));
                    let holed =
                        topo.add_face(Face::new(wire, vec![wire], FaceSurface::Torus(t.clone())));
                    assert!(trimmed_torus_polygon(&topo, holed, Tolerance::default()).is_none());
                }
            }
        }
    }

    /// How [`patch`] bounds the first half of its bottom rim.
    #[derive(Clone, Copy)]
    enum Rim {
        Circle,
        /// A rational quadratic NURBS arc.
        Nurbs,
        /// A straight chord, the one curve that reads its endpoints.
        Chord,
    }

    /// The one input in which a [`patch`] differs from the plain one.
    #[derive(Clone, Copy)]
    enum Tweak {
        None,
        /// The second edge used backwards, nothing else changed.
        FlippedUse,
        /// A stored tolerance on the third edge.
        EdgeTolerance,
        /// The vertex between the two rim halves moved by this much.
        MovedVertex(f64),
    }

    /// A five-edge patch `[u0, u0 + du] x [v0, v0 + 1.0]` whose bottom rim
    /// is split in two, its first half bounded as `rim` says. None is a
    /// four-arc rectangle, so all take the sampled trim.
    fn patch(
        topo: &mut Topology,
        t: &ToroidalSurface,
        (u0, v0, du): (f64, f64, f64),
        rim: Rim,
        tweak: Tweak,
    ) -> FaceId {
        use remus_math::nurbs::curve::NurbsCurve;
        let (u1, v1) = (u0 + du, v0 + 1.0);
        let um = f64::midpoint(u0, u1);
        let major = |v: f64| {
            Circle3D::new_with_ref(
                t.center() + t.z_axis() * (t.minor_radius() * v.sin()),
                t.z_axis(),
                t.major_radius() + t.minor_radius() * v.cos(),
                t.x_axis(),
            )
            .unwrap()
        };
        let minor = |u: f64| {
            let radial = t.x_axis() * u.cos() + t.y_axis() * u.sin();
            Circle3D::new_with_ref(
                t.center() + radial * t.major_radius(),
                radial.cross(t.z_axis()),
                t.minor_radius(),
                radial,
            )
            .unwrap()
        };
        let nudge = match tweak {
            Tweak::MovedVertex(by) => t.z_axis() * by,
            Tweak::None | Tweak::FlippedUse | Tweak::EdgeTolerance => Vec3::new(0.0, 0.0, 0.0),
        };
        let mut points =
            [(u0, v0), (um, v0), (u1, v0), (u1, v1), (u0, v1)].map(|(u, v)| t.evaluate(u, v));
        points[1] = points[1] + nudge;
        let vertices = points.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
        let bottom = major(v0);
        let first = match rim {
            Rim::Nurbs => {
                let half = 0.5 * (um - u0);
                let mid = u0 + half;
                let apex = bottom.center()
                    + (bottom.u_axis() * mid.cos() + bottom.v_axis() * mid.sin())
                        * (bottom.radius() / half.cos());
                let curve = NurbsCurve::new(
                    2,
                    vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    vec![bottom.evaluate(u0), apex, bottom.evaluate(um)],
                    vec![1.0, half.cos(), 1.0],
                )
                .unwrap();
                (EdgeCurve::NurbsCurve(curve), (0.0, 1.0))
            }
            Rim::Chord => (EdgeCurve::Line, (0.0, 1.0)),
            Rim::Circle => (EdgeCurve::Circle(bottom.clone()), (u0, um)),
        };
        let curves = [
            first,
            (EdgeCurve::Circle(bottom), (um, u1)),
            (EdgeCurve::Circle(minor(u1)), (v0, v1)),
            (EdgeCurve::Circle(major(v1)), (u1, u0)),
            (EdgeCurve::Circle(minor(u0)), (v1, v0)),
        ];
        let edges: Vec<_> = curves
            .into_iter()
            .enumerate()
            .map(|(i, (c, domain))| {
                let mut e = Edge::new(vertices[i], vertices[(i + 1) % 5], c);
                e.set_trim(Some(domain));
                if i == 2 && matches!(tweak, Tweak::EdgeTolerance) {
                    e.set_tolerance(Some(1e-4)).unwrap();
                }
                let forward = !(i == 1 && matches!(tweak, Tweak::FlippedUse));
                OrientedEdge::new(topo.add_edge(e), forward)
            })
            .collect();
        let wire = topo.add_wire(Wire::new(edges, true).unwrap());
        topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(t.clone())))
    }

    fn torus(major: f64, minor: f64) -> ToroidalSurface {
        ToroidalSurface::with_axis(
            Point3::new(2.0, -3.0, 4.0),
            major,
            minor,
            Vec3::new(1.0, 2.0, 3.0),
        )
        .unwrap()
    }

    /// Patches on several carriers, circle- and NURBS-bordered, plus one that
    /// differs from another by one ulp of its minor radius and one that the
    /// trim builder declines (more than half a turn in u).
    fn memo_corpus(topo: &mut Topology) -> Vec<FaceId> {
        let mut faces = Vec::new();
        for (major, minor) in [(12.0, 3.0), (0.12, 0.03), (1200.0, 300.0)] {
            let t = torus(major, minor);
            for anchor in [(0.3, 0.4, 1.2), (5.8, 5.9, 1.2)] {
                for rim in [Rim::Circle, Rim::Nurbs] {
                    faces.push(patch(topo, &t, anchor, rim, Tweak::None));
                }
            }
        }
        let t = torus(12.0, 3.0_f64.next_up());
        faces.push(patch(topo, &t, (0.3, 0.4, 1.2), Rim::Nurbs, Tweak::None));
        let t = torus(12.0, 3.0);
        faces.push(patch(topo, &t, (0.3, 0.4, 3.3), Rim::Circle, Tweak::None));
        faces
    }

    fn compute_all(topo: &Topology, faces: &[FaceId]) -> Vec<String> {
        faces
            .iter()
            // `Debug` prints every f64 exactly.
            .map(|&f| {
                format!(
                    "{:?}",
                    trimmed_torus_polygon_memoized(topo, f, Tolerance::default())
                )
            })
            .collect()
    }

    #[test]
    fn memoized_trims_are_bit_identical_to_computed_ones() {
        use crate::ray_geom_memo::{
            enable_thread_ray_geom_memo, set_thread_ray_geom_memo_capacity,
            thread_ray_geom_memo_stats,
        };
        let mut topo = Topology::new();
        let faces = memo_corpus(&mut topo);
        set_thread_ray_geom_memo_capacity(0);
        let computed = compute_all(&topo, &faces);
        let direct: Vec<String> = faces
            .iter()
            .map(|&f| {
                format!(
                    "{:?}",
                    trimmed_torus_polygon(&topo, f, Tolerance::default())
                )
            })
            .collect();
        enable_thread_ray_geom_memo();
        let stored = compute_all(&topo, &faces);
        let recalled = compute_all(&topo, &faces);
        let stats = thread_ray_geom_memo_stats();
        set_thread_ray_geom_memo_capacity(0);

        assert_eq!(computed, direct);
        assert_eq!(computed, stored);
        assert_eq!(computed, recalled);
        assert!(computed.iter().any(|t| t.starts_with("Some")));
        assert_eq!(computed.last().map(String::as_str), Some("None"));
        // Every face has its own key, the ulp-larger torus included (a
        // collision would answer it from the memo while storing).
        let n = faces.len() as u64;
        assert_eq!((stats.misses, stats.hits), (n, n), "{stats:?}");
    }

    #[test]
    fn trim_memo_is_bounded_and_off_by_default() {
        use crate::ray_geom_memo::{
            enable_thread_ray_geom_memo, set_thread_ray_geom_memo_bytes,
            set_thread_ray_geom_memo_capacity, thread_ray_geom_memo_stats,
        };
        let mut topo = Topology::new();
        let faces = memo_corpus(&mut topo);
        set_thread_ray_geom_memo_capacity(0);
        let _ = compute_all(&topo, &faces);
        assert_eq!(
            thread_ray_geom_memo_stats().len,
            0,
            "a disabled memo stored trims"
        );
        set_thread_ray_geom_memo_capacity(3);
        let _ = compute_all(&topo, &faces);
        assert_eq!(thread_ray_geom_memo_stats().len, 3);
        set_thread_ray_geom_memo_capacity(1);
        assert_eq!(thread_ray_geom_memo_stats().len, 1);
        set_thread_ray_geom_memo_capacity(0);
        assert_eq!(thread_ray_geom_memo_stats().bytes, 0);

        // The byte budget evicts oldest first and never holds more.
        enable_thread_ray_geom_memo();
        let _ = compute_all(&topo, &faces);
        let full = thread_ray_geom_memo_stats();
        assert_eq!(full.len, faces.len());
        let budget = full.bytes / 2;
        set_thread_ray_geom_memo_bytes(budget);
        let half = thread_ray_geom_memo_stats();
        assert!(
            half.bytes <= budget && half.len < full.len && half.len > 0,
            "{half:?}"
        );
        let _ = compute_all(&topo, &faces);
        assert!(thread_ray_geom_memo_stats().bytes <= budget);

        // A trim heavier than the whole budget is not stored and evicts
        // nothing already held.
        set_thread_ray_geom_memo_bytes(crate::ray_geom_memo::DEFAULT_RAY_GEOM_MEMO_BYTES);
        set_thread_ray_geom_memo_capacity(0);
        enable_thread_ray_geom_memo();
        let weights: Vec<usize> = faces
            .iter()
            .map(|&f| {
                let before = thread_ray_geom_memo_stats().bytes;
                let _ = trimmed_torus_polygon_memoized(&topo, f, Tolerance::default());
                thread_ray_geom_memo_stats().bytes - before
            })
            .collect();
        let heaviest = weights.iter().copied().max().unwrap();
        let (light, &light_weight) = weights
            .iter()
            .enumerate()
            .find(|(_, w)| **w < heaviest)
            .unwrap();
        let heavy = weights.iter().position(|&w| w == heaviest).unwrap();
        set_thread_ray_geom_memo_capacity(0);
        enable_thread_ray_geom_memo();
        set_thread_ray_geom_memo_bytes(heaviest - 1);
        let _ = trimmed_torus_polygon_memoized(&topo, faces[light], Tolerance::default());
        let _ = trimmed_torus_polygon_memoized(&topo, faces[heavy], Tolerance::default());
        let held = thread_ray_geom_memo_stats();
        assert_eq!((held.len, held.bytes), (1, light_weight), "{held:?}");

        set_thread_ray_geom_memo_bytes(crate::ray_geom_memo::DEFAULT_RAY_GEOM_MEMO_BYTES);
        set_thread_ray_geom_memo_capacity(0);
    }

    /// Faces that differ from another in one input the trim builder reads (an
    /// edge's use, an edge tolerance, a vertex position, each classifier
    /// tolerance) get keys of their own, so the memo, holding the other face,
    /// misses on them and computes their own trim.
    #[test]
    fn trim_keys_tell_apart_faces_differing_in_one_input() {
        use crate::ray_geom_memo::{
            enable_thread_ray_geom_memo, set_thread_ray_geom_memo_capacity,
            thread_ray_geom_memo_stats,
        };
        let mut topo = Topology::new();
        let t = torus(12.0, 3.0);
        let anchor = (0.3, 0.4, 1.2);
        let base = Tolerance::default();
        let plain = patch(&mut topo, &t, anchor, Rim::Circle, Tweak::None);
        let chord = patch(&mut topo, &t, anchor, Rim::Chord, Tweak::None);
        let cases = [
            (
                "edge use",
                patch(&mut topo, &t, anchor, Rim::Circle, Tweak::FlippedUse),
                base,
                plain,
            ),
            (
                "edge tolerance",
                patch(&mut topo, &t, anchor, Rim::Circle, Tweak::EdgeTolerance),
                base,
                plain,
            ),
            (
                "vertex position",
                patch(&mut topo, &t, anchor, Rim::Chord, Tweak::MovedVertex(1e-3)),
                base,
                chord,
            ),
            (
                "linear tolerance",
                plain,
                Tolerance {
                    linear: base.linear * 2.0,
                    ..base
                },
                plain,
            ),
            (
                "angular tolerance",
                plain,
                // Above `linear / metric`, so the builder's margin follows it.
                Tolerance {
                    angular: 1e-6,
                    ..base
                },
                plain,
            ),
            (
                "relative tolerance",
                plain,
                Tolerance {
                    relative: base.relative * 2.0,
                    ..base
                },
                plain,
            ),
        ];
        let mut changed_trims = 0;
        for (what, face, tol, other) in cases {
            let (key, other_key) = (trim_key(&topo, face, tol), trim_key(&topo, other, base));
            assert!(key.is_some() && other_key.is_some(), "{what}: no key");
            assert_ne!(key, other_key, "{what}: same key");
            let direct = format!("{:?}", trimmed_torus_polygon(&topo, face, tol));
            let other_direct = format!("{:?}", trimmed_torus_polygon(&topo, other, base));
            changed_trims += usize::from(direct != other_direct);
            set_thread_ray_geom_memo_capacity(0);
            enable_thread_ray_geom_memo();
            let _ = trimmed_torus_polygon_memoized(&topo, other, base);
            let before = thread_ray_geom_memo_stats();
            let memoized = format!("{:?}", trimmed_torus_polygon_memoized(&topo, face, tol));
            let after = thread_ray_geom_memo_stats();
            set_thread_ray_geom_memo_capacity(0);
            assert_eq!(memoized, direct, "{what}: memo answered with another trim");
            assert_eq!(
                (
                    after.misses - before.misses,
                    after.hits - before.hits,
                    after.len
                ),
                (1, 0, 2),
                "{what}: {after:?}"
            );
        }
        // Every input but the unread relative tolerance changes the trim, so
        // a key without it would return a wrong one.
        assert_eq!(changed_trims, cases.len() - 1);
    }

    /// The byte count covers every heap byte the memo holds for an entry:
    /// both copies of its key (map and eviction queue) and the trim polygon.
    #[test]
    fn trim_memo_counts_both_copies_of_its_keys() {
        use crate::ray_geom_memo::{
            enable_thread_ray_geom_memo, set_thread_ray_geom_memo_capacity,
            thread_ray_geom_memo_stats,
        };
        let mut topo = Topology::new();
        let faces = memo_corpus(&mut topo);
        let tol = Tolerance::default();
        for &face in &faces {
            set_thread_ray_geom_memo_capacity(0);
            enable_thread_ray_geom_memo();
            let trim = trimmed_torus_polygon_memoized(&topo, face, tol);
            let bytes = thread_ray_geom_memo_stats().bytes;
            set_thread_ray_geom_memo_capacity(0);
            let key = trim_key(&topo, face, tol).unwrap();
            let held = 2 * key.len() * std::mem::size_of::<u64>()
                + trim.as_ref().map_or(0, |t| {
                    t.polygon.len() * std::mem::size_of::<remus_math::vec::Point2>()
                });
            assert!(
                (held..=held + 256).contains(&bytes),
                "{bytes} bytes counted for {held} held"
            );
        }
    }
}

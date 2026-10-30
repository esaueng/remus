//! Recognition of bounded rectangular torus patches.
use remus_math::tolerance::Tolerance;
use remus_topology::{Topology, face::FaceId};

/// Recognize a rectangular torus patch in unwrapped parameter space.
/// Returns each angular interval as (start, span).
///
/// Every boundary edge must be an iso-parametric circle (constant `u` or
/// constant `v`). Consecutive edges on the same iso-line form one side, so a
/// rim split into arcs still bounds a rectangle: STEP writers commonly split
/// a full-revolution fillet's rim where a neighbouring face's seam meets it.
/// The merged boundary must have exactly four alternating sides.
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
    if edges.len() < 4 {
        return None;
    }
    let angular_tol = tol.angular.max(tol.linear / torus.minor_radius());
    let mut polygon: Vec<Point2> = Vec::new();
    let mut previous: Option<Point2> = None;
    // One entry per side: whether it holds `u` (rather than `v`) constant,
    // and the unwrapped value it holds.
    let mut sides: Vec<(bool, f64)> = Vec::with_capacity(4);
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
        let side = (constant_u, if constant_u { first.x() } else { first.y() });
        match sides.last() {
            Some(&(along_u, value))
                if along_u == side.0 && (value - side.1).abs() <= angular_tol => {}
            _ => sides.push(side),
        }
        polygon.extend(points);
    }
    let first = polygon.first()?;
    let last = polygon.last()?;
    if (*last - *first).length() > angular_tol {
        return None;
    }
    // The wire may start partway along a side; its first and last runs then
    // lie on the same iso-line. The polygon closes, so their values agree.
    if sides.len() > 1
        && let (Some(&(first_u, first_value)), Some(&(last_u, last_value))) =
            (sides.first(), sides.last())
        && first_u == last_u
        && (first_value - last_value).abs() <= angular_tol
    {
        sides.pop();
    }
    if sides.len() != 4
        || sides
            .iter()
            .zip(sides.iter().skip(1))
            .any(|(a, b)| a.0 == b.0)
    {
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

/// One finite raw projection on the current immutable edge and torus. This
/// value never crosses an outer-wire edge boundary or escapes this trim call.
/// The parameter key is bit-exact; UV unwrapping is deliberately not cached.
#[derive(Clone, Copy)]
struct RawTrimSample {
    parameter_bits: u64,
    uv: remus_math::vec::Point2,
    residual: f64,
}

impl RawTrimSample {
    fn matching(self, parameter: f64) -> Option<Self> {
        (self.parameter_bits == parameter.to_bits()).then_some(self)
    }
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
        let evaluate = |t: f64,
                        anchor: Point2,
                        previous: Option<RawTrimSample>|
         -> Option<(Point2, RawTrimSample)> {
            #[cfg(test)]
            tests::note_evaluation(t, anchor);
            let sample = previous
                .and_then(|sample| sample.matching(t))
                .unwrap_or_else(|| {
                    #[cfg(test)]
                    tests::note_raw_evaluation();
                    let p = edge.curve().evaluate_with_endpoints(t, a, b);
                    let (u, v) = torus.project_point(p);
                    let residual = (torus.evaluate(u, v) - p).length();
                    RawTrimSample {
                        parameter_bits: t.to_bits(),
                        uv: Point2::new(u, v),
                        residual,
                    }
                });
            let (u, v) = (sample.uv.x(), sample.uv.y());
            let residual = sample.residual;
            if !u.is_finite() || !v.is_finite() || !residual.is_finite() {
                return None;
            }
            // Imported space curves can deviate from their supporting surface.
            // Account for that projection in classifier uncertainty only; never
            // change the edge tolerance or the geometry acceptance policy.
            projection_margin.set(projection_margin.get().max(residual / metric));
            Some((unwrap_uv(Point2::new(u, v), anchor), sample))
        };
        let (start, _) = evaluate(
            t0,
            polygon.last().copied().unwrap_or(Point2::new(0.0, 0.0)),
            None,
        )?;
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
            let (right, _) = evaluate(hi, left, None)?;
            sample_trim_segment(
                &evaluate,
                lo,
                hi,
                left,
                right,
                margin,
                0,
                &mut polygon,
                None,
            )?;
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
    evaluate: &impl Fn(
        f64,
        remus_math::vec::Point2,
        Option<RawTrimSample>,
    ) -> Option<(remus_math::vec::Point2, RawTrimSample)>,
    lo: f64,
    hi: f64,
    a: remus_math::vec::Point2,
    b: remus_math::vec::Point2,
    margin: f64,
    depth: u8,
    polygon: &mut Vec<remus_math::vec::Point2>,
    previous_middle: Option<RawTrimSample>,
) -> Option<()> {
    if polygon.len() >= 16384 || depth >= 20 {
        return None;
    }
    let mut probes = [None; 3];
    let mut error = 0.0_f64;
    for (index, fraction) in [0.25, 0.5, 0.75].into_iter().enumerate() {
        let expected = a + (b - a) * fraction;
        let previous = if index == 1 { previous_middle } else { None };
        let (actual, raw) = evaluate((hi - lo).mul_add(fraction, lo), expected, previous)?;
        probes[index] = Some(raw);
        error = error.max((actual - expected).length());
    }
    if error <= margin {
        polygon.push(b);
        return Some(());
    }
    let mid = f64::midpoint(lo, hi);
    let (p, _) = evaluate(mid, a + (b - a) * 0.5, probes[1])?;
    // The parent's quarter probes are candidate middle probes of the children.
    // Rounding can make the parameters differ; evaluate checks their exact bits.
    // Each call still applies its current anchor and updates projection margin.
    sample_trim_segment(
        evaluate,
        lo,
        mid,
        a,
        p,
        margin,
        depth + 1,
        polygon,
        probes[0],
    )?;
    sample_trim_segment(
        evaluate,
        mid,
        hi,
        p,
        b,
        margin,
        depth + 1,
        polygon,
        probes[2],
    )
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]
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

    /// A full-revolution fillet band as STEP writers emit it: one rim split
    /// into two half-circle arcs, the other rim closed, and the seam used
    /// twice. The merged boundary is the whole `u` period by the band's `v`.
    #[test]
    fn full_revolution_band_with_a_split_rim_is_rectangular() {
        use std::f64::consts::{PI, TAU};
        let (v0, v1) = (2.0, 3.4);
        let t = ToroidalSurface::with_axis(
            Point3::new(0.0, 0.0, 30.0),
            20.5,
            4.0,
            Vec3::new(0.0, 0.0, 1.0),
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
        let seam = {
            let radial = t.x_axis();
            Circle3D::new_with_ref(
                t.center() + radial * t.major_radius(),
                radial.cross(t.z_axis()),
                t.minor_radius(),
                radial,
            )
            .unwrap()
        };
        let mut topo = Topology::new();
        let [low_start, low_half, high] = [(0.0, v0), (PI, v0), (0.0, v1)]
            .map(|(u, v)| topo.add_vertex(Vertex::new(t.evaluate(u, v), 1e-7)));
        let mut edge = |start, end, circle, domain| {
            let mut e = Edge::new(start, end, EdgeCurve::Circle(circle));
            e.set_trim(Some(domain));
            topo.add_edge(e)
        };
        let low_a = edge(low_start, low_half, major(v0), (0.0, PI));
        let low_b = edge(low_half, low_start, major(v0), (PI, TAU));
        let seam = edge(low_start, high, seam, (v0, v1));
        let high_rim = edge(high, high, major(v1), (0.0, TAU));
        let forward = vec![
            OrientedEdge::new(low_a, true),
            OrientedEdge::new(low_b, true),
            OrientedEdge::new(seam, true),
            OrientedEdge::new(high_rim, false),
            OrientedEdge::new(seam, false),
        ];
        // Start the wire partway along the split rim as well.
        let rotated = [&forward[1..], &forward[..1]].concat();
        for winding in [forward, rotated] {
            let wire = topo.add_wire(Wire::new(winding, true).unwrap());
            let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(t.clone())));
            let ((_, du), (dv0, dv)) =
                rectangular_torus_domain(&topo, face, Tolerance::default()).unwrap();
            assert!((du - TAU).abs() < 1e-9, "u span {du}");
            assert!((dv0.rem_euclid(TAU) - v0).abs() < 1e-9, "v start {dv0}");
            assert!((dv - (v1 - v0)).abs() < 1e-9, "v span {dv}");
        }
    }

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
                    // The split side is one side: the patch is still a
                    // rectangle in (u, v).
                    let ((du0, du), (dv0, dv)) =
                        rectangular_torus_domain(&topo, face, Tolerance::default()).unwrap();
                    let start = unwrap_uv(
                        remus_math::vec::Point2::new(du0, dv0),
                        remus_math::vec::Point2::new(u0, v0),
                    );
                    assert!((start.x() - u0).abs() < 1e-9, "u0 {du0} vs {u0}");
                    assert!((start.y() - v0).abs() < 1e-9, "v0 {dv0} vs {v0}");
                    assert!((du - (u1 - u0)).abs() < 1e-9 && (dv - (v1 - v0)).abs() < 1e-9);
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
    #[derive(Default)]
    struct EvaluationLog {
        logical: Vec<[u64; 3]>,
        raw: usize,
    }

    thread_local! {
        static EVALUATION_LOG: std::cell::RefCell<Option<EvaluationLog>> = const { std::cell::RefCell::new(None) };
    }

    pub(super) fn note_evaluation(t: f64, anchor: remus_math::vec::Point2) {
        EVALUATION_LOG.with(|log| {
            if let Some(log) = log.borrow_mut().as_mut() {
                log.logical
                    .push([t.to_bits(), anchor.x().to_bits(), anchor.y().to_bits()]);
            }
        });
    }

    pub(super) fn note_raw_evaluation() {
        EVALUATION_LOG.with(|log| {
            if let Some(log) = log.borrow_mut().as_mut() {
                log.raw += 1;
            }
        });
    }

    fn logged<T>(run: impl FnOnce() -> T) -> (T, EvaluationLog) {
        EVALUATION_LOG.with(|log| *log.borrow_mut() = Some(EvaluationLog::default()));
        let value = run();
        let log = EVALUATION_LOG.with(|log| log.borrow_mut().take().unwrap());
        (value, log)
    }

    fn trim_bits(trim: TorusTrim) -> (Vec<[u64; 2]>, [u64; 2], u64) {
        (
            trim.polygon
                .iter()
                .map(|p| [p.x().to_bits(), p.y().to_bits()])
                .collect(),
            [trim.centre.x().to_bits(), trim.centre.y().to_bits()],
            trim.margin.to_bits(),
        )
    }

    #[test]
    fn carried_raw_samples_match_original_trim_bits_and_logical_order() {
        let mut topo = Topology::new();
        let faces = memo_corpus(&mut topo);
        let mut saved = 0;
        for face in faces {
            let (actual, actual_log) =
                logged(|| trimmed_torus_polygon(&topo, face, Tolerance::default()));
            let (reference, reference_log) =
                logged(|| reference_trimmed_torus_polygon(&topo, face, Tolerance::default()));
            assert_eq!(actual.map(trim_bits), reference.map(trim_bits));
            assert_eq!(
                actual_log.logical, reference_log.logical,
                "parameter/anchor/order changed"
            );
            assert!(actual_log.raw <= reference_log.raw);
            saved += reference_log.raw - actual_log.raw;
        }
        assert!(
            saved > 0,
            "actual curve/projection evaluations were not reused"
        );
    }

    fn sampler_pair(lo: f64, hi: f64, margin: f64) -> (usize, usize) {
        use remus_math::vec::Point2;
        use std::cell::{Cell, RefCell};
        let actual_raw = Cell::new(0);
        let actual_log = RefCell::new(Vec::new());
        let actual = |t: f64, anchor: Point2, previous: Option<RawTrimSample>| {
            actual_log
                .borrow_mut()
                .push([t.to_bits(), anchor.x().to_bits(), anchor.y().to_bits()]);
            let sample = previous
                .and_then(|sample| sample.matching(t))
                .unwrap_or_else(|| {
                    actual_raw.set(actual_raw.get() + 1);
                    RawTrimSample {
                        parameter_bits: t.to_bits(),
                        uv: Point2::new(t, t * t),
                        residual: 0.0,
                    }
                });
            Some((sample.uv, sample))
        };
        let reference_raw = Cell::new(0);
        let reference_log = RefCell::new(Vec::new());
        let reference = |t: f64, anchor: Point2| {
            reference_raw.set(reference_raw.get() + 1);
            reference_log.borrow_mut().push([
                t.to_bits(),
                anchor.x().to_bits(),
                anchor.y().to_bits(),
            ]);
            Some(Point2::new(t, t * t))
        };
        let a = Point2::new(lo, lo * lo);
        let b = Point2::new(hi, hi * hi);
        let mut actual_polygon = vec![a];
        let mut reference_polygon = vec![a];
        let actual_result =
            sample_trim_segment(&actual, lo, hi, a, b, margin, 0, &mut actual_polygon, None);
        let reference_result = reference_sample_trim_segment(
            &reference,
            lo,
            hi,
            a,
            b,
            margin,
            0,
            &mut reference_polygon,
        );
        assert_eq!(actual_result, reference_result);
        let bits = |points: Vec<Point2>| {
            points
                .into_iter()
                .map(|p| [p.x().to_bits(), p.y().to_bits()])
                .collect::<Vec<_>>()
        };
        assert_eq!(bits(actual_polygon), bits(reference_polygon));
        assert_eq!(actual_log.into_inner(), reference_log.into_inner());
        (actual_raw.get(), reference_raw.get())
    }

    #[test]
    fn carried_parent_probes_have_deterministic_raw_work_reduction() {
        let (actual, reference) = sampler_pair(0.0, 1.0, 1e-5);
        assert!(
            actual > 0 && actual * 5 <= reference * 4,
            "expected at least20% fewer raw evaluations, got {actual}/{reference}"
        );
        // This parabola has256 dyadic leaves: 511 nodes. Original raw work is
        // 3*511+255=1788; carrying parent probes needs3+2*510=1023.
        assert_eq!((actual, reference), (1023, 1788));
    }

    #[test]
    fn carried_probes_match_original_for_non_dyadic_reversed_and_close_domains() {
        for (lo, hi, margin) in [
            (0.1, 0.9, 1e-5),
            (1.0, 1e16, 1e30),
            (-2.0, 3.0, 1e-4),
            (270.0, 180.0, 1e-3),
            (1e-150, 2e-150, 1e-310),
            (-1e150, 1e150, 1e290),
            (1.0, 1.0 + 1e-8, 1e-8),
            (-0.0, 0.0, 1e-5),
        ] {
            sampler_pair(lo, hi, margin);
        }
    }

    #[test]
    fn raw_parameter_match_is_bit_exact_and_unwrap_stays_anchor_dependent() {
        use remus_math::vec::Point2;
        let sample = RawTrimSample {
            parameter_bits: 0.0_f64.to_bits(),
            uv: Point2::new(0.1, 0.2),
            residual: 0.0,
        };
        assert!(sample.matching(0.0).is_some());
        assert!(sample.matching(-0.0).is_none());
        assert!(sample.matching(f64::from_bits(1)).is_none());
        let first = unwrap_uv(sample.uv, Point2::new(0.0, 0.0));
        let second = unwrap_uv(
            sample.uv,
            Point2::new(std::f64::consts::TAU, std::f64::consts::TAU),
        );
        assert_ne!(first.x().to_bits(), second.x().to_bits());
        assert_ne!(first.y().to_bits(), second.y().to_bits());
    }

    #[test]
    fn carried_sampler_preserves_both_hard_refusals_before_evaluation() {
        use remus_math::vec::Point2;
        let p = Point2::new(0.0, 0.0);
        for (depth, length) in [(20, 0), (0, 16384)] {
            let mut actual = vec![p; length];
            let mut reference = actual.clone();
            let forbidden_actual =
                |_: f64, _: Point2, _: Option<RawTrimSample>| -> Option<(Point2, RawTrimSample)> {
                    panic!("budget must refuse before evaluation")
                };
            let forbidden_reference = |_: f64, _: Point2| -> Option<Point2> {
                panic!("budget must refuse before evaluation")
            };
            assert!(
                sample_trim_segment(
                    &forbidden_actual,
                    0.0,
                    1.0,
                    p,
                    p,
                    1e-7,
                    depth,
                    &mut actual,
                    None
                )
                .is_none()
            );
            assert!(
                reference_sample_trim_segment(
                    &forbidden_reference,
                    0.0,
                    1.0,
                    p,
                    p,
                    1e-7,
                    depth,
                    &mut reference
                )
                .is_none()
            );
            assert_eq!(actual.len(), reference.len());
        }
    }

    #[test]
    fn raw_match_declines_non_dyadic_and_overflowing_midpoint_mismatches() {
        use remus_math::vec::Point2;
        for (lo, hi) in [(1.0_f64, 1e16_f64), (-f64::MAX, f64::MAX)] {
            let probe = (hi - lo).mul_add(0.5, lo);
            let midpoint = f64::midpoint(lo, hi);
            assert_ne!(probe.to_bits(), midpoint.to_bits());
            let sample = RawTrimSample {
                parameter_bits: probe.to_bits(),
                uv: Point2::new(0.1, 0.2),
                residual: 0.0,
            };
            assert!(sample.matching(midpoint).is_none());
        }
    }

    #[test]
    fn carried_sampler_preserves_partial_polygon_and_margin_on_budget_refusal() {
        use remus_math::vec::Point2;
        use std::cell::Cell;
        let candidate_margin = Cell::new(0.0_f64);
        let reference_margin = Cell::new(0.0_f64);
        let candidate = |t: f64, _: Point2, previous: Option<RawTrimSample>| {
            let sample = previous
                .and_then(|sample| sample.matching(t))
                .unwrap_or_else(|| RawTrimSample {
                    parameter_bits: t.to_bits(),
                    uv: Point2::new(t, t * t),
                    residual: t.abs(),
                });
            candidate_margin.set(candidate_margin.get().max(sample.residual));
            Some((sample.uv, sample))
        };
        let reference = |t: f64, _: Point2| {
            reference_margin.set(reference_margin.get().max(t.abs()));
            Some(Point2::new(t, t * t))
        };
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 1.0);
        let mut candidate_polygon = vec![a; 16380];
        let mut reference_polygon = candidate_polygon.clone();
        assert!(
            sample_trim_segment(
                &candidate,
                0.0,
                1.0,
                a,
                b,
                1e-12,
                0,
                &mut candidate_polygon,
                None
            )
            .is_none()
        );
        assert!(
            reference_sample_trim_segment(
                &reference,
                0.0,
                1.0,
                a,
                b,
                1e-12,
                0,
                &mut reference_polygon
            )
            .is_none()
        );
        let bits = |points: Vec<Point2>| {
            points
                .into_iter()
                .map(|p| [p.x().to_bits(), p.y().to_bits()])
                .collect::<Vec<_>>()
        };
        assert_eq!(candidate_polygon.len(), 16384);
        assert_eq!(bits(candidate_polygon), bits(reference_polygon));
        assert_eq!(
            candidate_margin.get().to_bits(),
            reference_margin.get().to_bits()
        );
    }

    #[test]
    #[allow(clippy::print_stderr)]
    fn carried_samples_match_original_on_actual_holder_torus_edges() {
        // Existing public/licensed full31 fixture, not a new imported asset.
        // Deserialize only the four measured-path torus boundaries; no I/O
        // crate dependency, healing, geometry replacement or normalization.
        let document: serde_json::Value = serde_json::from_str(include_str!(
            "../../../../scripts/perf/parameter-rebuild/fixtures/base.arena.json"
        ))
        .unwrap();
        let mut topo = Topology::new();
        let vertices: Vec<_> = document["vertices"]
            .as_array()
            .unwrap()
            .iter()
            .map(|record| {
                let point: Point3 = serde_json::from_value(record["point"].clone()).unwrap();
                topo.add_vertex(Vertex::new(point, record["tolerance"].as_f64().unwrap()))
            })
            .collect();
        let mut total_saved = 0;
        for index in [63, 65, 74, 88] {
            let record = &document["faces"][index];
            assert!(record["inner_wires"].as_array().unwrap().is_empty());
            let surface: ToroidalSurface =
                serde_json::from_value(record["surface"]["Torus"].clone()).unwrap();
            let wire = &document["wires"][record["outer_wire"].as_u64().unwrap() as usize];
            let edges = wire["edges"]
                .as_array()
                .unwrap()
                .iter()
                .map(|use_record| {
                    let edge = &document["edges"][use_record["edge"].as_u64().unwrap() as usize];
                    let curve = if let Some(circle) = edge["curve"].get("Circle") {
                        EdgeCurve::Circle(serde_json::from_value(circle.clone()).unwrap())
                    } else {
                        EdgeCurve::NurbsCurve(
                            serde_json::from_value(edge["curve"]["NurbsCurve"].clone()).unwrap(),
                        )
                    };
                    let mut made = Edge::new(
                        vertices[edge["start"].as_u64().unwrap() as usize],
                        vertices[edge["end"].as_u64().unwrap() as usize],
                        curve,
                    );
                    made.set_trim(Some((
                        edge["trim"][0].as_f64().unwrap(),
                        edge["trim"][1].as_f64().unwrap(),
                    )));
                    if let Some(tolerance) = edge["tolerance"].as_f64() {
                        made.set_tolerance(Some(tolerance)).unwrap();
                    }
                    OrientedEdge::new(
                        topo.add_edge(made),
                        use_record["forward"].as_bool().unwrap(),
                    )
                })
                .collect();
            let wire = topo.add_wire(Wire::new(edges, true).unwrap());
            let mut face = Face::new(wire, vec![], FaceSurface::Torus(surface));
            face.set_reversed(record["reversed"].as_bool().unwrap());
            let face = topo.add_face(face);
            assert!(rectangular_torus_domain(&topo, face, Tolerance::default()).is_none());
            let (candidate, candidate_log) =
                logged(|| trimmed_torus_polygon(&topo, face, Tolerance::default()));
            let (reference, reference_log) =
                logged(|| reference_trimmed_torus_polygon(&topo, face, Tolerance::default()));
            assert!(
                candidate.is_some(),
                "actual holder torus face{index} unexpectedly declined"
            );
            assert_eq!(
                candidate.map(trim_bits),
                reference.map(trim_bits),
                "holder torus face{index}"
            );
            assert_eq!(
                candidate_log.logical, reference_log.logical,
                "holder face{index} logical order"
            );
            assert!(
                candidate_log.raw < reference_log.raw,
                "holder face{index}: no raw-work saving"
            );
            eprintln!(
                "holder torus face{index}: raw evaluations {} -> {}",
                reference_log.raw, candidate_log.raw
            );
            total_saved += reference_log.raw - candidate_log.raw;
        }
        assert!(total_saved > 0);
    }

    // Literal original production algorithms retained as an independent oracle.
    fn reference_trimmed_torus_polygon(
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
                note_evaluation(t, anchor);
                note_raw_evaluation();
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
                reference_sample_trim_segment(
                    &evaluate,
                    lo,
                    hi,
                    left,
                    right,
                    margin,
                    0,
                    &mut polygon,
                )?;
            }
        }
        if (*polygon.last()? - *polygon.first()?).length()
            > closure_margin.max(projection_margin.get())
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

    #[allow(clippy::too_many_arguments)]
    fn reference_sample_trim_segment(
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
        reference_sample_trim_segment(evaluate, lo, mid, a, p, margin, depth + 1, polygon)?;
        reference_sample_trim_segment(evaluate, mid, hi, p, b, margin, depth + 1, polygon)
    }
}

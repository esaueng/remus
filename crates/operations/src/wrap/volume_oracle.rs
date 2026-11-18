//! An independent divergence-theorem volume of a wrapped solid's stored
//! B-rep (tests).
//!
//! `V = (1/3) Σ ∬ (P − R) · n dA` about the shell's vertex-box centre `R`
//! (the reference `solid_volume` sums about), each face read on its own
//! terms:
//!
//! - a cylinder or cone face in its own polar chart, reduced to its boundary
//!   in closed form: on a cylinder `(P − R)·n dA = r (r + (O − R)·ρ(θ)) dθ dz`,
//!   so `∬ = −∮ z r (r + (O − R)·ρ(θ)) dθ`; on a cone `(P − R)·n = (A − R)·n(θ)`
//!   and `dA = s cos α dθ ds`, so `∬ = −cos α ∮ (s²/2) (A − R)·n(θ) dθ`. The
//!   loop's orientation is read off its own enclosed area. A boundary point
//!   off the surface (a fitted rim, by its fit error) is read where the
//!   surface's own projection puts it, at its angle and axial height (on a
//!   cone `s = h / sin α`, as `ConicalSurface::project_point` reads it): the
//!   stored B-rep's face is the region that projection bounds;
//! - a NURBS face (a wall) over its whole parameter rectangle, by composite
//!   Gauss–Legendre in both directions, every knot span split.
//!
//! Nothing here goes through the kernel's face integrator: it shares only
//! curve and surface evaluation with it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::quadrature::gauss_legendre_points;
use remus_math::traits::ParametricCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// Gauss points per piece.
const ORDER: usize = 16;
/// Pieces per knot span of a NURBS edge or surface direction.
const PER_SPAN: usize = 16;
/// Pieces of a line, circle or ellipse edge.
const EXACT_PIECES: usize = 128;

/// The stored B-rep's enclosed volume (see the module docs).
pub(super) fn stored_brep_volume(topo: &Topology, solid: SolidId) -> f64 {
    let faces = solid_faces(topo, solid).unwrap();
    let reference = vertex_box_centre(topo, &faces);
    let total: f64 = faces
        .iter()
        .map(|&face| face_volume(topo, face, reference))
        .sum();
    total.abs()
}

/// The centre of the box of `faces`' vertices: the reference `solid_volume`
/// sums a shell's faces about.
pub(super) fn vertex_box_centre(topo: &Topology, faces: &[remus_topology::face::FaceId]) -> Point3 {
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for &face in faces {
        let face = topo.face(face).unwrap();
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oriented in topo.wire(wire).unwrap().edges() {
                let edge = topo.edge(oriented.edge()).unwrap();
                for vertex in [edge.start(), edge.end()] {
                    let p = topo.vertex(vertex).unwrap().point();
                    for k in 0..3 {
                        lo[k] = lo[k].min(p.0[k]);
                        hi[k] = hi[k].max(p.0[k]);
                    }
                }
            }
        }
    }
    Point3::new(
        f64::midpoint(lo[0], hi[0]),
        f64::midpoint(lo[1], hi[1]),
        f64::midpoint(lo[2], hi[2]),
    )
}

/// `[lo, hi]` cut at every distinct interior knot, each piece then into
/// `per_span` equal parts.
fn knot_pieces(knots: &[f64], lo: f64, hi: f64, per_span: usize) -> Vec<(f64, f64)> {
    let mut breaks = vec![lo];
    for &knot in knots {
        if knot > lo && knot < hi && knot > *breaks.last().unwrap() {
            breaks.push(knot);
        }
    }
    breaks.push(hi);
    let mut out = Vec::new();
    for w in breaks.windows(2) {
        let step = (w[1] - w[0]) / per_span as f64;
        for k in 0..per_span {
            out.push((w[0] + step * k as f64, w[0] + step * (k + 1) as f64));
        }
    }
    out
}

/// Gauss points `(t, weight)` over `[t0, t1]` (either order: a reversed
/// interval carries negative weights) for `edge`.
fn edge_points(edge: &Edge) -> Vec<(f64, f64)> {
    let (t0, t1) = edge.strict_domain().unwrap();
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    let pieces = match edge.curve() {
        EdgeCurve::NurbsCurve(n) => knot_pieces(n.knots(), lo, hi, PER_SPAN),
        EdgeCurve::Line | EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => {
            knot_pieces(&[], lo, hi, EXACT_PIECES)
        }
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => panic!("no wrapped edge is a conic"),
    };
    let sense = if t0 <= t1 { 1.0 } else { -1.0 };
    let gauss = gauss_legendre_points(ORDER);
    pieces
        .into_iter()
        .flat_map(|(a, b)| {
            gauss.iter().map(move |gp| {
                let half = (b - a) / 2.0;
                (a + half + half * gp.x, sense * half * gp.w)
            })
        })
        .collect()
}

/// The edge's point and velocity at `t`.
fn point_velocity(edge: &Edge, start: Point3, end: Point3, t: f64) -> (Point3, Vec3) {
    let point = edge.curve().evaluate_with_endpoints(t, start, end);
    let velocity = match edge.curve() {
        EdgeCurve::Line => end - start,
        EdgeCurve::Circle(c) => ParametricCurve::derivative_pair(c, t).unwrap().0,
        EdgeCurve::Ellipse(e) => ParametricCurve::derivative_pair(e, t).unwrap().0,
        EdgeCurve::NurbsCurve(n) => ParametricCurve::derivative_pair(n, t).unwrap().0,
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => panic!("no wrapped edge is a conic"),
    };
    (point, velocity)
}

/// `Σ` over the face's boundary of `w · f(P, dP/dt)`, each edge run the way
/// its wire uses it.
fn boundary_sum(
    topo: &Topology,
    face: &remus_topology::face::Face,
    mut f: impl FnMut(Point3, Vec3) -> [f64; 2],
) -> [f64; 2] {
    let mut sum = [0.0; 2];
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire).unwrap().edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            let (start, end) = (
                topo.vertex(edge.start()).unwrap().point(),
                topo.vertex(edge.end()).unwrap().point(),
            );
            let use_sense = if oriented.is_forward() { 1.0 } else { -1.0 };
            for (t, w) in edge_points(edge) {
                let (p, dp) = point_velocity(edge, start, end, t);
                let [a, b] = f(p, dp);
                sum[0] += use_sense * w * a;
                sum[1] += use_sense * w * b;
            }
        }
    }
    sum
}

/// One face's `(1/3) ∬ (P − R)·n dA` about `reference`, outward by its stored
/// orientation.
pub(super) fn face_volume(
    topo: &Topology,
    face_id: remus_topology::face::FaceId,
    reference: Point3,
) -> f64 {
    let face = topo.face(face_id).unwrap();
    let outward = if face.is_reversed() { -1.0 } else { 1.0 };
    match face.surface() {
        FaceSurface::Cylinder(c) => {
            let (o, a, x, y, r) = (
                c.origin(),
                c.axis().normalize().unwrap(),
                c.x_axis(),
                c.y_axis(),
                c.radius(),
            );
            // [−∮ z dθ, −∮ z r (r + (O − R)·ρ) dθ]
            let [area, moment] = boundary_sum(topo, face, |p, dp| {
                let d = p - o;
                let (px, py) = (d.dot(x), d.dot(y));
                let dtheta = (px * dp.dot(y) - py * dp.dot(x)) / (px * px + py * py);
                let theta = py.atan2(px);
                let rho = x * theta.cos() + y * theta.sin();
                let z = d.dot(a);
                [
                    -z * dtheta,
                    -z * r * (r + (o - reference).dot(rho)) * dtheta,
                ]
            });
            outward * area.signum() * moment / 3.0
        }
        FaceSurface::Cone(c) => {
            let (apex, a, x, y, alpha) = (
                c.apex(),
                c.axis().normalize().unwrap(),
                c.x_axis(),
                c.y_axis(),
                c.half_angle(),
            );
            let (sin_a, cos_a) = alpha.sin_cos();
            // [−∮ (s²/2) dθ, −∮ (s²/2) (A − R)·n dθ]
            let [area, moment] = boundary_sum(topo, face, |p, dp| {
                let d = p - apex;
                let (px, py) = (d.dot(x), d.dot(y));
                let dtheta = (px * dp.dot(y) - py * dp.dot(x)) / (px * px + py * py);
                let theta = py.atan2(px);
                let rho = x * theta.cos() + y * theta.sin();
                let normal = rho * sin_a - a * cos_a;
                let s = d.dot(a) / sin_a;
                let half_s2 = 0.5 * s * s;
                [
                    -half_s2 * dtheta,
                    -half_s2 * (apex - reference).dot(normal) * dtheta,
                ]
            });
            outward * area.signum() * cos_a * moment / 3.0
        }
        FaceSurface::Nurbs(s) => {
            let ((u0, u1), (v0, v1)) = (s.domain_u(), s.domain_v());
            let gauss = gauss_legendre_points(ORDER);
            let mut total = 0.0;
            for (ua, ub) in knot_pieces(s.knots_u(), u0, u1, PER_SPAN) {
                for (va, vb) in knot_pieces(s.knots_v(), v0, v1, PER_SPAN) {
                    let (hu, hv) = ((ub - ua) / 2.0, (vb - va) / 2.0);
                    for gu in gauss {
                        for gv in gauss {
                            let (u, v) = (ua + hu + hu * gu.x, va + hv + hv * gv.x);
                            let d = s.derivatives(u, v, 1);
                            let p = s.evaluate(u, v);
                            let n = d[1][0].cross(d[0][1]);
                            total += gu.w * gv.w * hu * hv * (p - reference).dot(n);
                        }
                    }
                }
            }
            outward * total / 3.0
        }
        FaceSurface::Plane { .. } | FaceSurface::Sphere(_) | FaceSurface::Torus(_) => {
            panic!("the oracle reads cylinder, cone and NURBS faces only")
        }
    }
}

//! Regressions for the certified torus broad phase and minor-angle band,
//! run through the box the broad phase actually builds
//! (`compute_face_bbox`) and the band the marcher actually receives
//! (`face_v_range`).
//!
//! The fixtures are exact: every boundary edge lies on the carrier (to
//! rounding). The bump edge is the rational curve with `tan(u/2)` linear and
//! `tan(v/2)` quadratic in its parameter, a degree-6 rational Bezier with
//! positive weights, then re-stored with uneven knots or reweighted, which
//! changes its parameter speed and nothing else. Sampled boundaries stop well
//! short of the ridge these faces cross; a fixed-sample enclosure (64 per
//! edge plus midpoint bulge) under-bounded every one of them.
#![allow(clippy::unwrap_used, clippy::cast_precision_loss, clippy::float_cmp)]

use super::*;
use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const R: f64 = 10.0;
const RM: f64 = 2.0;
const PI: f64 = std::f64::consts::PI;

/// A torus frame: centre and orthonormal right-handed axes.
#[derive(Clone, Copy)]
struct Frame {
    c: Point3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
}

impl Frame {
    fn axis_z() -> Self {
        Self {
            c: Point3::new(0.0, 0.0, 0.0),
            x: Vec3::new(1.0, 0.0, 0.0),
            y: Vec3::new(0.0, 1.0, 0.0),
            z: Vec3::new(0.0, 0.0, 1.0),
        }
    }

    /// Tilted 0.6 rad about x and translated (the reviewer's variant).
    fn tilted() -> Self {
        let (s, c) = 0.6_f64.sin_cos();
        Self {
            c: Point3::new(3.0, -7.0, 11.0),
            x: Vec3::new(1.0, 0.0, 0.0),
            y: Vec3::new(0.0, c, s),
            z: Vec3::new(0.0, -s, c),
        }
    }

    fn world(&self, l: [f64; 3]) -> Point3 {
        self.c + self.x * l[0] + self.y * l[1] + self.z * l[2]
    }

    fn torus(&self) -> ToroidalSurface {
        ToroidalSurface::with_axis_and_ref_dir(self.c, R, RM, self.z, self.x).unwrap()
    }

    fn radial(&self, u: f64) -> Vec3 {
        self.x * u.cos() + self.y * u.sin()
    }
}

/// Power-basis coefficients to Bernstein coefficients of degree `n`.
fn to_bernstein(p: &[f64], n: usize) -> Vec<f64> {
    let binom = |n: usize, k: usize| -> f64 {
        (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64)
    };
    (0..=n)
        .map(|i| {
            (0..=i)
                .map(|j| p.get(j).copied().unwrap_or(0.0) * binom(i, j) / binom(n, j))
                .sum()
        })
        .collect()
}

fn poly_mul(p: &[f64], q: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; p.len() + q.len() - 1];
    for (i, a) in p.iter().enumerate() {
        for (j, b) in q.iter().enumerate() {
            out[i + j] += a * b;
        }
    }
    out
}

fn poly_add(p: &[f64], q: &[f64]) -> Vec<f64> {
    (0..p.len().max(q.len()))
        .map(|i| p.get(i).copied().unwrap_or(0.0) + q.get(i).copied().unwrap_or(0.0))
        .collect()
}

fn scaled(c: f64, p: &[f64]) -> Vec<f64> {
    p.iter().map(|x| c * x).collect()
}

/// Homogeneous `[w·P, w]` control net of the exact on-torus bump from
/// `(u_from, v_end)` to `(u_to, v_end)` peaking at `v_peak` halfway.
fn bump_net(f: &Frame, u_from: f64, u_to: f64, v_end: f64, v_peak: f64) -> Vec<[f64; 4]> {
    let (a0, a1) = ((u_from / 2.0).tan(), (u_to / 2.0).tan());
    let (b0, bp) = ((v_end / 2.0).tan(), (v_peak / 2.0).tan());
    let a = [a0, a1 - a0];
    let b = [b0, 4.0 * (bp - b0), -4.0 * (bp - b0)];
    let one = [1.0];
    let (a2, b2) = (poly_mul(&a, &a), poly_mul(&b, &b));
    let w = poly_mul(&poly_add(&one, &a2), &poly_add(&one, &b2));
    let radial = poly_add(
        &scaled(R, &poly_add(&one, &b2)),
        &scaled(RM, &poly_add(&one, &scaled(-1.0, &b2))),
    );
    let x = poly_mul(&radial, &poly_add(&one, &scaled(-1.0, &a2)));
    let y = poly_mul(&radial, &scaled(2.0, &a));
    let z = scaled(2.0 * RM, &poly_mul(&b, &poly_add(&one, &a2)));
    let [w, x, y, z] = [w, x, y, z].map(|p| to_bernstein(&p, 6));
    (0..=6)
        .map(|i| {
            assert!(w[i] > 0.0);
            let p = f.world([x[i] / w[i], y[i] / w[i], z[i] / w[i]]);
            [p.x() * w[i], p.y() * w[i], p.z() * w[i], w[i]]
        })
        .collect()
}

/// De Casteljau split of a homogeneous Bezier net at `s`.
fn split_at(net: &[[f64; 4]], s: f64) -> (Vec<[f64; 4]>, Vec<[f64; 4]>) {
    let n = net.len() - 1;
    let mut work = net.to_vec();
    let (mut left, mut right) = (vec![work[0]], vec![work[n]]);
    for r in 1..=n {
        for i in 0..=n - r {
            for k in 0..4 {
                work[i][k] = (1.0 - s) * work[i][k] + s * work[i + 1][k];
            }
        }
        left.push(work[0]);
        right.push(work[n - r]);
    }
    right.reverse();
    (left, right)
}

/// A degree-6 NURBS made of `segments` (each a Bezier net, sharing end
/// points) on the knot breaks `breaks` (interior knots of full multiplicity).
fn piecewise(segments: &[Vec<[f64; 4]>], breaks: &[f64]) -> NurbsCurve {
    let mut knots = vec![0.0; 7];
    for &b in breaks {
        knots.extend([b; 6]);
    }
    knots.extend([1.0; 7]);
    let mut net: Vec<[f64; 4]> = segments[0].clone();
    for seg in &segments[1..] {
        net.extend_from_slice(&seg[1..]);
    }
    curve_from_net(6, knots, &net)
}

fn curve_from_net(degree: usize, knots: Vec<f64>, net: &[[f64; 4]]) -> NurbsCurve {
    NurbsCurve::new(
        degree,
        knots,
        net.iter()
            .map(|h| Point3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3]))
            .collect(),
        net.iter().map(|h| h[3]).collect(),
    )
    .unwrap()
}

/// Analytic meridian circle at `u` (parameter = `v`).
fn meridian(f: &Frame, u: f64) -> Circle3D {
    let radial = f.radial(u);
    let c = Circle3D::new_with_ref(f.c + radial * R, radial.cross(f.z), RM, radial).unwrap();
    for t in [0.3, 1.7, 4.0] {
        assert!((c.evaluate(t) - f.torus().evaluate(u, t)).length() < 1e-12);
    }
    c
}

/// Analytic parallel circle at `v` (parameter = `u`).
fn parallel(f: &Frame, v: f64) -> Circle3D {
    let c = Circle3D::new_with_ref(f.c + f.z * (RM * v.sin()), f.z, RM.mul_add(v.cos(), R), f.x)
        .unwrap();
    for t in [0.3, 1.7, 4.0] {
        assert!((c.evaluate(t) - f.torus().evaluate(t, v)).length() < 1e-12);
    }
    c
}

/// One boundary edge of a fixture: its curve, the parameter range it runs
/// (in traversal order; a descending range is stored ascending and used
/// reversed), and its end points.
struct Side {
    curve: EdgeCurve,
    from: f64,
    to: f64,
}

fn eval(curve: &EdgeCurve, t: f64) -> Point3 {
    curve.evaluate_with_endpoints(t, Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0))
}

/// A face on `torus` bounded by `sides` (a closed loop, in order). Shared
/// corners become shared vertices; a side used twice (a doubled seam) is
/// passed as `Reuse(k, forward)`.
enum Spec {
    New(Side),
    Reuse(usize, bool),
}

fn build_face(torus: &ToroidalSurface, specs: Vec<Spec>, reversed: bool) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let mut vertices: Vec<(Point3, remus_topology::vertex::VertexId)> = Vec::new();
    let mut vertex = |topo: &mut Topology, p: Point3| {
        if let Some(&(_, id)) = vertices.iter().find(|(q, _)| (*q - p).length() < 1e-9) {
            return id;
        }
        let id = topo.add_vertex(Vertex::new(p, 1e-7));
        vertices.push((p, id));
        id
    };
    let mut edges = Vec::new();
    let mut oriented = Vec::new();
    for spec in specs {
        match spec {
            Spec::New(side) => {
                let (t0, t1, forward) = if side.from <= side.to {
                    (side.from, side.to, true)
                } else {
                    (side.to, side.from, false)
                };
                let (p0, p1) = (eval(&side.curve, t0), eval(&side.curve, t1));
                let (a, b) = (vertex(&mut topo, p0), vertex(&mut topo, p1));
                let mut edge = Edge::new(a, b, side.curve);
                edge.set_trim(Some((t0, t1)));
                let id = topo.add_edge(edge);
                edges.push(id);
                oriented.push(OrientedEdge::new(id, forward));
            }
            Spec::Reuse(k, forward) => oriented.push(OrientedEdge::new(edges[k], forward)),
        }
    }
    let wire = topo.add_wire(Wire::new(oriented, true).unwrap());
    let surface = FaceSurface::Torus(torus.clone());
    let face = if reversed {
        Face::new_reversed(wire, Vec::new(), surface)
    } else {
        Face::new(wire, Vec::new(), surface)
    };
    let id = topo.add_face(face);
    (topo, id)
}

/// The bump face: `v` from `v_bot` up to the bump edge, `u ∈ [ua, ub]`.
/// Counter-clockwise in `(u, v)`: bottom parallel, right meridian, the bump
/// back from `ub` to `ua`, left meridian down.
fn bump_face(f: &Frame, top: NurbsCurve, v_end: f64, reversed: bool) -> (Topology, FaceId) {
    let (ua, ub, v_bot) = (0.7, 1.2, 1.0);
    let specs = vec![
        Spec::New(Side {
            curve: EdgeCurve::Circle(parallel(f, v_bot)),
            from: ua,
            to: ub,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Circle(meridian(f, ub)),
            from: v_bot,
            to: v_end,
        }),
        Spec::New(Side {
            curve: EdgeCurve::NurbsCurve(top),
            from: 0.0,
            to: 1.0,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Circle(meridian(f, ua)),
            from: v_end,
            to: v_bot,
        }),
    ];
    build_face(&f.torus(), specs, reversed)
}

/// Dense box of the true bump face, sampled in the exact `(tan u/2, tan v/2)`
/// chart that defines it.
fn true_bump_box(f: &Frame, v_end: f64, v_peak: f64, n: usize) -> Aabb3 {
    let torus = f.torus();
    let (a0, a1) = ((1.2_f64 / 2.0).tan(), (0.7_f64 / 2.0).tan());
    let (b0, bp) = ((v_end / 2.0).tan(), (v_peak / 2.0).tan());
    let mut points = Vec::new();
    for i in 0..=n {
        let s = i as f64 / n as f64;
        let u = 2.0 * (a1 - a0).mul_add(s, a0).atan();
        let v_top = 2.0 * (4.0 * (bp - b0) * s).mul_add(1.0 - s, b0).atan();
        for j in 0..=n {
            points.push(torus.evaluate(u, (v_top - 1.0).mul_add(j as f64 / n as f64, 1.0)));
        }
    }
    Aabb3::try_from_points(points).unwrap()
}

/// Dense box of the `(u, v)` hull patch `[0.7, 1.2] x [1.0, v_peak]` the
/// certified enclosure is tight against (the face itself is smaller: its
/// top is the bump, not the parallel at `v_peak`).
fn hull_patch_box(f: &Frame, v_peak: f64, n: usize) -> Aabb3 {
    let torus = f.torus();
    let mut points = Vec::new();
    for i in 0..=n {
        for j in 0..=n {
            points.push(torus.evaluate(
                0.5_f64.mul_add(i as f64 / n as f64, 0.7),
                (v_peak - 1.0).mul_add(j as f64 / n as f64, 1.0),
            ));
        }
    }
    Aabb3::try_from_points(points).unwrap()
}

fn contains(outer: &Aabb3, inner: &Aabb3, slack: f64) -> bool {
    let o = outer.expanded(slack);
    o.contains_point(inner.min) && o.contains_point(inner.max)
}

/// Every sampled boundary point the broad phase reads, nine per edge.
fn boundary_samples_top(topo: &Topology, face: FaceId, axis: Vec3) -> f64 {
    let mut top = f64::NEG_INFINITY;
    for eid in remus_topology::explorer::face_edges(topo, face).unwrap() {
        let edge = topo.edge(eid).unwrap();
        let (t0, t1) = edge.strict_domain().unwrap();
        let (s, e) = (
            topo.vertex(edge.start()).unwrap().point(),
            topo.vertex(edge.end()).unwrap().point(),
        );
        for i in 0..=8 {
            let p = edge.curve().evaluate_with_endpoints(
                (t1 - t0).mul_add(f64::from(i) / 8.0, t0),
                s,
                e,
            );
            top = top.max(Vec3::new(p.x(), p.y(), p.z()).dot(axis));
        }
    }
    top
}

/// The reviewer's variants of the same bump, as stored curves.
fn variants(f: &Frame, v_peak: f64) -> Vec<(&'static str, NurbsCurve)> {
    let net = bump_net(f, 1.2, 0.7, 1.2, v_peak);
    let bezier = curve_from_net(6, [[0.0; 7], [1.0; 7]].concat(), &net);
    // Tail-compressed: split at s = 0.05, the remaining 95% of the curve
    // (its whole peak) stored on the last 0.2% of the parameter.
    let (head, tail) = split_at(&net, 0.05);
    let compressed_tail = piecewise(&[head, tail], &[0.998]);
    // Middle-compressed: s in [0.3, 0.7] on a span of width 5e-4.
    let (first, rest) = split_at(&net, 0.3);
    let (middle, last) = split_at(&rest, (0.7 - 0.3) / 0.7);
    let a = 0.5 + 1.0 / 1024.0;
    let compressed_middle = piecewise(&[first, middle, last], &[a, a + 5e-4]);
    // Positive-weight (Moebius) reparametrization: w_i -> w_i * mu^i.
    let reweight = |mu: f64| {
        let net: Vec<[f64; 4]> = net
            .iter()
            .enumerate()
            .map(|(i, h)| h.map(|c| c * mu.powi(i32::try_from(i).unwrap())))
            .collect();
        curve_from_net(6, [[0.0; 7], [1.0; 7]].concat(), &net)
    };
    vec![
        ("bezier", bezier),
        ("tail-compressed knots", compressed_tail),
        ("middle-compressed knots", compressed_middle),
        ("reweighted mu=0.01", reweight(0.01)),
        ("reweighted mu=1e-3", reweight(1e-3)),
        ("reweighted mu=1e-9", reweight(1e-9)),
        ("reweighted mu=1e-12", reweight(1e-12)),
    ]
}

#[test]
fn compressed_on_torus_nurbs_trims_reach_the_ridge_through_the_broad_phase() {
    let tol = Tolerance::default();
    for (frame_name, f) in [("axis z", Frame::axis_z()), ("tilted", Frame::tilted())] {
        for v_peak in [1.6, 2.4] {
            let truth = true_bump_box(&f, 1.2, v_peak, 400);
            let hull = hull_patch_box(&f, v_peak, 400);
            for (name, curve) in variants(&f, v_peak) {
                // The stored curve is the same exact on-torus point set.
                let torus = f.torus();
                let (d0, d1) = curve.domain();
                for i in 0..=200 {
                    let p = curve.evaluate((d1 - d0).mul_add(f64::from(i) / 200.0, d0));
                    let (u, v) = torus.project_point(p);
                    assert!((torus.evaluate(u, v) - p).length() < 1e-11, "{name}");
                }
                let (topo, face) = bump_face(&f, curve, 1.2, false);
                let bbox = super::super::compute_face_bbox(&topo, face, tol).unwrap();
                let what = format!("{frame_name}, peak {v_peak}, {name}");
                assert!(
                    contains(&bbox, &truth, 1e-9),
                    "{what}: {bbox:?} misses {truth:?}"
                );
                assert!(
                    contains(&hull, &bbox, 1e-4),
                    "{what}: {bbox:?} looser than the hull patch {hull:?}"
                );
                if frame_name == "axis z" {
                    // The ridge z = r lies inside the face; the box reaches
                    // it although the sampled boundary stops well short.
                    assert!(bbox.max.z() >= RM - 1e-12, "{what}: {}", bbox.max.z());
                    if name == "tail-compressed knots" && v_peak < 2.0 {
                        // The reviewer's trap: every boundary sample the
                        // broad phase reads stops below the crossing plane.
                        let sampled = boundary_samples_top(&topo, face, f.z);
                        assert!(sampled < 1.95, "{what}: boundary samples reach {sampled}");
                    }
                    let plane = Aabb3 {
                        min: Point3::new(-20.0, -20.0, 1.95),
                        max: Point3::new(20.0, 20.0, 1.95),
                    };
                    assert!(bbox.expanded(tol.linear).intersects(plane), "{what}");
                }
                // The band the marcher receives holds the face's v range.
                let band = super::super::face_v_range(
                    &topo,
                    face,
                    topo.face(face).unwrap().surface(),
                    tol,
                )
                .unwrap()
                .unwrap();
                assert!(
                    band.0 <= 1.0 + 1e-12 && band.1 >= v_peak - 1e-12,
                    "{what}: {band:?}"
                );
                assert!(
                    band.0 >= 1.0 - 1e-6 && band.1 <= v_peak + 1e-6,
                    "{what}: loose {band:?}"
                );
            }
        }
    }
}

#[test]
fn certified_lift_is_independent_of_parameter_speed() {
    // The same point set stored with different knots and weights lifts to
    // the same hull, to the refinement tightness.
    let f = Frame::tilted();
    let carrier = Carrier::new(&f.torus()).unwrap();
    let mut hulls = Vec::new();
    for (name, curve) in variants(&f, 1.6) {
        let (topo, face) = bump_face(&f, curve, 1.2, false);
        let outer = topo.face(face).unwrap().outer_wire();
        let lift = lift::lift_loop(&topo, outer, &carrier).unwrap().unwrap();
        assert_eq!(lift.winding, (0, 0), "{name}");
        assert!(lift.area > lift.area_error, "{name}: {lift:?}");
        hulls.push((name, lift.u, lift.v));
    }
    let (_, u0, v0) = hulls[0];
    for (name, u, v) in hulls {
        for (a, b) in [(u0.0, u.0), (u0.1, u.1), (v0.0, v.0), (v0.1, v.1)] {
            assert!((a - b).abs() < 1e-8, "{name}: {u:?} {v:?} vs {u0:?} {v0:?}");
        }
    }
}

#[test]
fn a_common_projective_weight_scale_preserves_the_lift() {
    let f = Frame::tilted();
    let carrier = Carrier::new(&f.torus()).unwrap();
    let net = bump_net(&f, 1.2, 0.7, 1.2, 2.4);
    let mut hulls = Vec::new();
    for scale in [1.0, 1e-160, 1e160] {
        let scaled: Vec<[f64; 4]> = net.iter().map(|h| h.map(|c| c * scale)).collect();
        let curve = curve_from_net(6, [[0.0; 7], [1.0; 7]].concat(), &scaled);
        let (topo, face) = bump_face(&f, curve, 1.2, false);
        let outer = topo.face(face).unwrap().outer_wire();
        let lift = lift::lift_loop(&topo, outer, &carrier).unwrap();
        assert!(
            lift.is_some(),
            "common weight scale {scale:e} lost its certificate"
        );
        let lift = lift.unwrap();
        hulls.push((scale, lift.u, lift.v));
    }
    let (_, u0, v0) = hulls[0];
    for (scale, u, v) in hulls {
        for (a, b) in [(u0.0, u.0), (u0.1, u.1), (v0.0, v.0), (v0.1, v.1)] {
            assert!((a - b).abs() < 1e-8, "weight scale {scale:e}: {u:?} {v:?}");
        }
    }
}

/// Exact rational quadratic full circle in four quarters on uneven double
/// knots: almost all of its parameter range sweeps its last quarter.
fn uneven_full_circle(center: Point3, ua: Vec3, va: Vec3, radius: f64) -> NurbsCurve {
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let at = |k: usize, scale: f64| {
        let angle = k as f64 * PI / 4.0;
        center + (ua * angle.cos() + va * angle.sin()) * (radius * scale)
    };
    let points: Vec<Point3> = (0..9)
        .map(|k| at(k, if k % 2 == 1 { 2.0_f64.sqrt() } else { 1.0 }))
        .collect();
    let weights: Vec<f64> = (0..9).map(|k| if k % 2 == 1 { h } else { 1.0 }).collect();
    let knots = vec![
        0.0, 0.0, 0.0, 0.001, 0.001, 0.002, 0.002, 0.003, 0.003, 1.0, 1.0, 1.0,
    ];
    NurbsCurve::new(2, knots, points, weights).unwrap()
}

#[test]
fn full_minor_circle_on_uneven_knots_proves_the_whole_tube() {
    // A tube segment u in [0.4, 1.3] bounded by two whole minor circles
    // stored as NURBS with uneven knots, joined by a doubled seam. 64
    // uniform samples of such a circle put all but the first in its last
    // quarter, and a gap heuristic read the band as [3pi/2, 2pi].
    let tol = Tolerance::default();
    let f = Frame::tilted();
    let torus = f.torus();
    let (u0, u1) = (0.4, 1.3);
    let circle = |u: f64| {
        let radial = f.radial(u);
        uneven_full_circle(f.c + radial * R, radial, f.z, RM)
    };
    for (k, t) in [(0usize, 0.0005), (1, 0.0015), (2, 0.0025), (3, 0.5)] {
        let p = circle(u0).evaluate(t);
        let (_, v) = torus.project_point(p);
        assert!(
            (v - (k as f64 + 0.5) * PI / 2.0).abs() < 0.3,
            "quarter {k}: v = {v}"
        );
    }
    let specs = vec![
        Spec::New(Side {
            curve: EdgeCurve::Circle(parallel(&f, 0.0)),
            from: u0,
            to: u1,
        }),
        Spec::New(Side {
            curve: EdgeCurve::NurbsCurve(circle(u1)),
            from: 0.0,
            to: 1.0,
        }),
        Spec::Reuse(0, false),
        Spec::New(Side {
            curve: EdgeCurve::NurbsCurve(circle(u0)),
            from: 1.0,
            to: 0.0,
        }),
    ];
    let (topo, face) = build_face(&torus, specs, false);
    let surface = topo.face(face).unwrap().surface().clone();
    let band = super::super::face_v_range(&topo, face, &surface, tol).unwrap();
    assert_eq!(band, Some((0.0, TAU)));
    // The broad phase holds the whole tube segment, and only that.
    let bbox = super::super::compute_face_bbox(&topo, face, tol).unwrap();
    let mut truth = None::<Aabb3>;
    for i in 0..=300 {
        for j in 0..=300 {
            let p = torus.evaluate(
                (u1 - u0).mul_add(f64::from(i) / 300.0, u0),
                TAU * f64::from(j) / 300.0,
            );
            let b = Aabb3 { min: p, max: p };
            truth = Some(truth.map_or(b, |t| t.union(b)));
        }
    }
    let truth = truth.unwrap();
    assert!(contains(&bbox, &truth, 1e-9), "{bbox:?} misses {truth:?}");
    assert!(
        contains(&truth, &bbox, 1e-3),
        "{bbox:?} looser than {truth:?}"
    );
}

/// An analytic `(u, v)` rectangle face, counter-clockwise.
fn rectangle_face(
    f: &Frame,
    (u0, u1): (f64, f64),
    (v0, v1): (f64, f64),
    reversed: bool,
) -> (Topology, FaceId) {
    let specs = vec![
        Spec::New(Side {
            curve: EdgeCurve::Circle(parallel(f, v0)),
            from: u0,
            to: u1,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Circle(meridian(f, u1)),
            from: v0,
            to: v1,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Circle(parallel(f, v1)),
            from: u1,
            to: u0,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Circle(meridian(f, u0)),
            from: v1,
            to: v0,
        }),
    ];
    build_face(&f.torus(), specs, reversed)
}

#[test]
fn partial_bands_are_lifted_and_carried_into_the_projection_range() {
    let tol = Tolerance::default();
    let f = Frame::tilted();
    let band = |v: (f64, f64)| {
        let (topo, face) = rectangle_face(&f, (0.2, 0.6), v, false);
        let surface = topo.face(face).unwrap().surface().clone();
        super::super::face_v_range(&topo, face, &surface, tol).unwrap()
    };
    // Inside the projection range: the exact band, to refinement tightness.
    let (lo, hi) = band((0.3, 0.9)).unwrap();
    assert!(
        (lo - 0.3).abs() < 1e-9 && (hi - 0.9).abs() < 1e-9,
        "{lo} {hi}"
    );
    // Stored past 2pi: carried down a turn.
    let (lo, hi) = band((TAU + 0.3, TAU + 0.9)).unwrap();
    assert!(
        (lo - 0.3).abs() < 1e-9 && (hi - 0.9).abs() < 1e-9,
        "{lo} {hi}"
    );
    // Ending on the wrap: clamped to it, not mistaken for a straddle.
    let (lo, hi) = band((1.5 * PI, TAU)).unwrap();
    assert!((lo - 1.5 * PI).abs() < 1e-9 && hi == TAU, "{lo} {hi}");
    let (lo, hi) = band((0.0, 0.5)).unwrap();
    assert!(lo == 0.0 && (hi - 0.5).abs() < 1e-9, "{lo} {hi}");
    // Straddling the wrap: no single [v0, v1] in [0, 2pi) holds both
    // pieces, so the band is the whole tube (raw min/max would have dropped
    // the face's own middle).
    assert_eq!(band((-0.3, 0.4)), Some((0.0, TAU)));
}

/// The analytic rectangle of [`rectangle_face`] wound clockwise in `(u, v)`.
fn clockwise_rectangle_face(
    f: &Frame,
    (u0, u1): (f64, f64),
    (v0, v1): (f64, f64),
) -> (Topology, FaceId) {
    let specs = vec![
        Spec::New(Side {
            curve: EdgeCurve::Circle(meridian(f, u0)),
            from: v0,
            to: v1,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Circle(parallel(f, v1)),
            from: u0,
            to: u1,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Circle(meridian(f, u1)),
            from: v1,
            to: v0,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Circle(parallel(f, v0)),
            from: u1,
            to: u0,
        }),
    ];
    build_face(&f.torus(), specs, false)
}

#[test]
fn opposite_winding_takes_the_whole_carrier_and_reversal_does_not() {
    // The stored outer wire runs counter-clockwise about the stored surface
    // normal whatever the face's `reversed` flag (the kernel contract; the
    // hammer holder's reversed fillets are stored so). A loop wound the
    // other way may bound the torus minus the disc, which the rectangle and
    // hull bounds would under-bound: it takes the whole carrier and the
    // whole tube.
    let tol = Tolerance::default();
    for f in [Frame::axis_z(), Frame::tilted()] {
        let whole = f.torus().aabb();
        let (topo, face) = clockwise_rectangle_face(&f, (0.2, 0.6), (0.3, 0.9));
        assert!(crate::classifier::rectangular_torus_domain(&topo, face, tol).is_some());
        let bbox = super::super::compute_face_bbox(&topo, face, tol).unwrap();
        assert!(contains(&bbox, &whole, 1e-9), "{bbox:?}");
        let surface = topo.face(face).unwrap().surface().clone();
        assert_eq!(
            super::super::face_v_range(&topo, face, &surface, tol).unwrap(),
            Some((0.0, TAU))
        );
        // The same loops on reversed faces are discs: tight boxes and bands.
        let (topo, face) = rectangle_face(&f, (0.2, 0.6), (0.3, 0.9), true);
        let bbox = super::super::compute_face_bbox(&topo, face, tol).unwrap();
        assert!(!contains(&bbox, &whole, 0.0), "{bbox:?}");
        let surface = topo.face(face).unwrap().surface().clone();
        let (lo, hi) = super::super::face_v_range(&topo, face, &surface, tol)
            .unwrap()
            .unwrap();
        assert!(
            (lo - 0.3).abs() < 1e-9 && (hi - 0.9).abs() < 1e-9,
            "{lo} {hi}"
        );
        let curve = variants(&f, 1.6).swap_remove(1).1;
        let (topo, face) = bump_face(&f, curve, 1.2, true);
        let bbox = super::super::compute_face_bbox(&topo, face, tol).unwrap();
        assert!(!contains(&bbox, &whole, 0.0), "{bbox:?}");
        assert!(contains(&bbox, &true_bump_box(&f, 1.2, 1.6, 200), 1e-9));
    }
}

#[test]
fn windings_and_unsupported_carriers_fall_back_to_the_whole_carrier() {
    let tol = Tolerance::default();
    let f = Frame::axis_z();
    let torus = f.torus();
    let whole = torus.aabb();
    // A whole parallel as the only boundary winds once in u: the face is a
    // band on one side of it or the other, not a disc.
    let specs = vec![Spec::New(Side {
        curve: EdgeCurve::Circle(parallel(&f, 0.8)),
        from: 0.0,
        to: TAU,
    })];
    let (topo, face) = build_face(&torus, specs, false);
    let carrier = Carrier::new(&torus).unwrap();
    let lift = lift::lift_loop(&topo, topo.face(face).unwrap().outer_wire(), &carrier)
        .unwrap()
        .unwrap();
    assert_eq!(lift.winding, (1, 0));
    let bbox = super::super::compute_face_bbox(&topo, face, tol).unwrap();
    assert!(contains(&bbox, &whole, 1e-9));
    // A whole meridian winds once in v: the face covers the whole tube.
    let specs = vec![Spec::New(Side {
        curve: EdgeCurve::Circle(meridian(&f, 0.8)),
        from: 0.0,
        to: TAU,
    })];
    let (topo, face) = build_face(&torus, specs, false);
    let lift = lift::lift_loop(&topo, topo.face(face).unwrap().outer_wire(), &carrier)
        .unwrap()
        .unwrap();
    assert_eq!(lift.winding, (0, 1));
    let surface = topo.face(face).unwrap().surface().clone();
    assert_eq!(
        super::super::face_v_range(&topo, face, &surface, tol).unwrap(),
        Some((0.0, TAU))
    );
    // An ellipse side is not certified: the whole carrier, and the whole
    // tube for the band.
    let ellipse = Ellipse3D::new(
        Point3::new(0.0, 0.0, RM * 1.3_f64.sin()),
        Vec3::new(0.0, 0.0, 1.0),
        RM.mul_add(1.3_f64.cos(), R),
        RM.mul_add(1.3_f64.cos(), R),
    )
    .unwrap();
    let specs = vec![
        Spec::New(Side {
            curve: EdgeCurve::Circle(parallel(&f, 0.3)),
            from: 0.2,
            to: 0.6,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Circle(meridian(&f, 0.6)),
            from: 0.3,
            to: 1.3,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Ellipse(ellipse),
            from: 0.6,
            to: 0.2,
        }),
        Spec::New(Side {
            curve: EdgeCurve::Circle(meridian(&f, 0.2)),
            from: 1.3,
            to: 0.3,
        }),
    ];
    let (topo, face) = build_face(&torus, specs, false);
    assert!(
        lift::lift_loop(&topo, topo.face(face).unwrap().outer_wire(), &carrier)
            .unwrap()
            .is_none()
    );
    let bbox = super::super::compute_face_bbox(&topo, face, tol).unwrap();
    assert!(contains(&bbox, &whole, 1e-9));
    let surface = topo.face(face).unwrap().surface().clone();
    assert_eq!(
        super::super::face_v_range(&topo, face, &surface, tol).unwrap(),
        Some((0.0, TAU))
    );
}

#[test]
fn seam_crossing_disc_lifts_continuously() {
    // A rectangle straddling both seams (u and v cross 0 = 2pi) is a disc:
    // the lift is continuous across the projection's wrap, the box is the
    // patch's, and the band is the whole tube (it straddles v = 0).
    let tol = Tolerance::default();
    let f = Frame::tilted();
    let torus = f.torus();
    let (topo, face) = rectangle_face(&f, (-0.25, 0.35), (-0.4, 0.5), false);
    let carrier = Carrier::new(&torus).unwrap();
    let lift = lift::lift_loop(&topo, topo.face(face).unwrap().outer_wire(), &carrier)
        .unwrap()
        .unwrap();
    assert_eq!(lift.winding, (0, 0));
    assert!(lift.area > lift.area_error && (lift.area - 0.6 * 0.9).abs() < 1e-6);
    assert!((lift.u.1 - lift.u.0 - 0.6).abs() < 1e-8, "{lift:?}");
    assert!((lift.v.1 - lift.v.0 - 0.9).abs() < 1e-8, "{lift:?}");
    let bbox = super::super::compute_face_bbox(&topo, face, tol).unwrap();
    assert!(!contains(&bbox, &torus.aabb(), 0.0));
    let surface = topo.face(face).unwrap().surface().clone();
    assert_eq!(
        super::super::face_v_range(&topo, face, &surface, tol).unwrap(),
        Some((0.0, TAU))
    );
}

#[test]
fn random_on_torus_trims_are_never_under_bounded() {
    // Random bump faces in random frames, peaks on either side of the
    // ridge, random reparametrizations: the broad-phase box holds a dense
    // sampling of the true face and stays within the sampling sag of the
    // `(u, v)` hull patch it certifies.
    let tol = Tolerance::default();
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut next = move || {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        (state.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1_u64 << 53) as f64
    };
    for case in 0..24 {
        let (sa, ca) = (next() * TAU).sin_cos();
        let (sb, cb) = (next() * TAU).sin_cos();
        // Rotation about x by a, then about z by b.
        let x = Vec3::new(cb, sb, 0.0);
        let y = Vec3::new(-sb * ca, cb * ca, sa);
        let z = x.cross(y);
        let f = Frame {
            c: Point3::new(
                next() * 40.0 - 20.0,
                next() * 40.0 - 20.0,
                next() * 40.0 - 20.0,
            ),
            x,
            y,
            z,
        };
        let v_peak = 1.25 + next() * 1.6;
        let net = bump_net(&f, 1.2, 0.7, 1.2, v_peak);
        let s = 0.02 + next() * 0.9;
        let (head, tail) = split_at(&net, s);
        let k = 0.01 + next() * 0.98;
        let curve = piecewise(&[head, tail], &[k]);
        let (topo, face) = bump_face(&f, curve, 1.2, false);
        let bbox = super::super::compute_face_bbox(&topo, face, tol).unwrap();
        let truth = true_bump_box(&f, 1.2, v_peak, 300);
        let hull = hull_patch_box(&f, v_peak, 300);
        assert!(
            contains(&bbox, &truth, 1e-9),
            "case {case}: {bbox:?} misses {truth:?}"
        );
        assert!(
            contains(&hull, &bbox, 1e-3),
            "case {case}: {bbox:?} looser than {hull:?}"
        );
    }
}

#[test]
fn wrap_ending_patch_extent_reads_its_band_periodically_and_its_u_window() {
    // A fillet-like patch u in [pi/2, pi], v in [3pi/2, 2pi] (lifted
    // [-pi/2, 0]). Its rim at v = 2pi projects to just above 0, and a plane
    // parallel to the torus axis can cut the carrier on two parallels, one
    // inside the band and one outside it (v = 5.94 and 3.48 for the hammer
    // holder's fillets). The extent must keep the first, drop the second, and
    // drop the in-band parallel where it runs past the patch in u.
    let tol = Tolerance::default();
    let f = Frame::tilted();
    let torus = f.torus();
    let (topo, face) = rectangle_face(&f, (PI / 2.0, PI), (-PI / 2.0, 0.0), false);
    let surface = topo.face(face).unwrap().surface().clone();
    let band = super::super::face_v_range(&topo, face, &surface, tol).unwrap();
    let (lo, hi) = band.unwrap();
    assert!((lo - 1.5 * PI).abs() < 1e-9 && hi == TAU, "{band:?}");
    let gap = torus_u_gap(&topo, face, &torus).unwrap().unwrap();
    assert!(
        (gap.0 - PI).abs() < 1e-9 && (gap.1 - (PI / 2.0 + TAU)).abs() < 1e-9,
        "{gap:?}"
    );
    let extent = super::super::FaceExtent::new(&topo, face, &surface, band, tol)
        .unwrap()
        .unwrap();
    let (u_in, u_out) = (0.75 * PI, 1.5 * PI);
    // On the rim v = 2pi, nudged across the wrap.
    for v in [TAU - 1e-12, 1e-12] {
        assert!(extent.contains(torus.evaluate(u_in, v)), "rim at {v}");
    }
    assert!(extent.contains_strict(torus.evaluate(u_in, 5.94), 0.0));
    assert!(!extent.contains(torus.evaluate(u_in, 3.48)));
    assert!(!extent.contains(torus.evaluate(u_out, 5.94)));
    assert!(!extent.contains_strict(torus.evaluate(u_out, 5.94), 0.0));
    // The membership helper itself.
    let periodic = super::super::torus_v_in_band;
    assert!(periodic(1e-12, (1.5 * PI - 0.01, TAU + 0.01)));
    assert!(periodic(5.94, (1.5 * PI, TAU)));
    assert!(!periodic(3.48, (1.5 * PI - 0.01, TAU + 0.01)));
    assert!(periodic(6.2, (-0.3, 0.4)) && !periodic(1.0, (-0.3, 0.4)));
    assert!(periodic(3.0, (0.0, TAU)) && !periodic(3.0, (1.0, 0.5)));
    // No window where the face does not certify.
    let (topo, face) = clockwise_rectangle_face(&f, (PI / 2.0, PI), (-PI / 2.0, 0.0));
    assert!(torus_u_gap(&topo, face, &torus).unwrap().is_none());
}

#[test]
fn recognized_rectangle_must_hold_the_certified_hull() {
    let pi = PI;
    // Same window, shifted by turns, wrapping, and a full turn.
    assert!(window_holds((0.2, 0.4), (0.2, 0.6), 1e-9));
    assert!(window_holds((0.2, 0.4), (0.2 + TAU, 0.6 + TAU), 1e-9));
    assert!(window_holds((pi - 0.1, 0.3), (-pi - 0.1, -pi + 0.2), 1e-9));
    assert!(window_holds((5.0, TAU), (1.0, 3.0), 0.0));
    // A rectangle narrower than the hull (a mis-unwrapped recognizer) or
    // beside it is refused.
    assert!(!window_holds((0.2, 0.4), (0.15, 0.6), 1e-9));
    assert!(!window_holds((0.2, 0.4), (0.2, 0.7), 1e-9));
    assert!(!window_holds((0.2, 0.4), (1.2, 1.4), 1e-9));
    assert!(window_holds((0.2, 0.4), (0.2 - 1e-10, 0.6 + 1e-10), 1e-9));
}

/// F2 regressions: band provenance, not width, decides the whole-tube paths.
mod band_proof;

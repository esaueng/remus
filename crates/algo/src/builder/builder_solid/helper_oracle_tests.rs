//! Independent-oracle tests for builder-solid helpers: conic support
//! identity, closed-pair traversal, and revolved-face wire winding.
//!
//! Winding is judged by the Newell vector area of the sampled loop against
//! the carrier's hand-derived outward normal, not by the UV-area routine the
//! repair itself uses.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface};
use remus_topology::edge::Edge;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::Wire;

use super::{
    EdgeCurve, EdgeId, Face, FaceId, FaceSurface, OrientedEdge, Point3, SelectedFace, Topology,
    Vec3, WireId, closed_pair_traversal_flipped, conics_share_support, orient_revolved_face_wires,
};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

// ── conics_share_support ────────────────────────────────────────────────

#[test]
fn conics_share_support_only_for_the_same_carrier_curve() {
    let tol = 1e-7;
    let c = Point3::new(0.4, -1.2, 3.0);
    let circle = |center: Point3, normal: Vec3, r: f64| {
        EdgeCurve::Circle(Circle3D::new(center, normal, r).unwrap())
    };
    let base = circle(c, Z, 2.0);
    // Same support, either traversal direction.
    assert!(conics_share_support(&base, &circle(c, Z, 2.0), tol));
    assert!(conics_share_support(&base, &circle(c, -Z, 2.0), tol));
    assert!(conics_share_support(
        &base,
        &circle(c + Vec3::new(0.5 * tol, 0.0, 0.0), Z, 2.0 + 0.5 * tol),
        tol
    ));
    // Different circles.
    assert!(!conics_share_support(
        &base,
        &circle(c + Vec3::new(0.0, 0.0, 0.1), Z, 2.0),
        tol
    ));
    assert!(!conics_share_support(
        &base,
        &circle(c + Vec3::new(3.0 * tol, 0.0, 0.0), Z, 2.0),
        tol
    ));
    assert!(!conics_share_support(
        &base,
        &circle(c, Z, 2.0 + 3.0 * tol),
        tol
    ));
    assert!(!conics_share_support(
        &base,
        &circle(c, Vec3::new(0.0, 0.01, 1.0), 2.0),
        tol
    ));
    // Different curve kinds never share.
    assert!(!conics_share_support(&base, &EdgeCurve::Line, tol));
    assert!(!conics_share_support(&EdgeCurve::Line, &base, tol));

    let ellipse =
        |normal: Vec3, a: f64, b: f64| EdgeCurve::Ellipse(Ellipse3D::new(c, normal, a, b).unwrap());
    let e = ellipse(Z, 3.0, 1.5);
    assert!(conics_share_support(&e, &ellipse(Z, 3.0, 1.5), tol));
    assert!(conics_share_support(&e, &ellipse(-Z, 3.0, 1.5), tol));
    assert!(!conics_share_support(&e, &ellipse(Z, 3.0, 1.4), tol));
    assert!(!conics_share_support(&e, &base, tol));
    assert!(!conics_share_support(&base, &e, tol));
}

// ── closed_pair_traversal_flipped ───────────────────────────────────────

fn closed_edge(
    topo: &mut Topology,
    seam: VertexId,
    curve: EdgeCurve,
    domain: (f64, f64),
) -> EdgeId {
    let mut edge = Edge::with_tolerance(seam, seam, curve, Some(1e-7));
    edge.set_trim(Some(domain));
    topo.add_edge(edge)
}

/// The circle through `seam` about `center` with `normal`, as a closed edge
/// whose domain starts at the seam.
fn ring(
    topo: &mut Topology,
    seam_vertex: VertexId,
    seam: Point3,
    center: Point3,
    normal: Vec3,
) -> EdgeId {
    let circle = Circle3D::new(center, normal, (seam - center).length()).unwrap();
    let t = circle.project(seam);
    closed_edge(topo, seam_vertex, EdgeCurve::Circle(circle), (t, t + TAU))
}

/// A rational quadratic full circle in the z = `center.z` plane, starting at
/// angle 0 and running counter-clockwise (`ccw`) or clockwise about +z.
fn nurbs_ring(center: Point3, r: f64, ccw: bool) -> NurbsCurve {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let s = if ccw { 1.0 } else { -1.0 };
    let dirs = [
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (-1.0, 1.0),
        (-1.0, 0.0),
        (-1.0, -1.0),
        (0.0, -1.0),
        (1.0, -1.0),
        (1.0, 0.0),
    ];
    let points = dirs
        .iter()
        .map(|&(x, y)| center + Vec3::new(r * x, s * r * y, 0.0))
        .collect();
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0];
    NurbsCurve::new(2, knots, points, vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0]).unwrap()
}

#[test]
fn closed_pair_traversal_is_flipped_exactly_when_the_rings_run_opposite() {
    let center = Point3::new(1.0, 2.0, -0.5);
    let r = 1.5;
    let seam = center + Vec3::new(r, 0.0, 0.0);
    let mut topo = Topology::new();
    let v = topo.add_vertex(Vertex::new(seam, 1e-7));
    // A ring about +z runs counter-clockwise seen from +z; about −z, clockwise.
    let ccw = ring(&mut topo, v, seam, center, Z);
    let ccw_again = ring(&mut topo, v, seam, center, Z);
    let cw = ring(&mut topo, v, seam, center, -Z);
    assert_eq!(
        closed_pair_traversal_flipped(&topo, ccw, ccw_again),
        Some(false)
    );
    assert_eq!(closed_pair_traversal_flipped(&topo, ccw, cw), Some(true));
    assert_eq!(closed_pair_traversal_flipped(&topo, cw, ccw), Some(true));

    // Mixed carriers: an ellipse degenerate to the same circle, and a NURBS
    // copy in each direction.
    let ellipse = Ellipse3D::new(center, -Z, r, r).unwrap();
    let t = ellipse.project(seam);
    let ellipse_cw = closed_edge(&mut topo, v, EdgeCurve::Ellipse(ellipse), (t, t + TAU));
    assert_eq!(
        closed_pair_traversal_flipped(&topo, ccw, ellipse_cw),
        Some(true)
    );
    assert_eq!(
        closed_pair_traversal_flipped(&topo, cw, ellipse_cw),
        Some(false)
    );
    let nurbs_ccw = closed_edge(
        &mut topo,
        v,
        EdgeCurve::NurbsCurve(nurbs_ring(center, r, true)),
        (0.0, 4.0),
    );
    let nurbs_cw = closed_edge(
        &mut topo,
        v,
        EdgeCurve::NurbsCurve(nurbs_ring(center, r, false)),
        (0.0, 4.0),
    );
    assert_eq!(
        closed_pair_traversal_flipped(&topo, ccw, nurbs_ccw),
        Some(false)
    );
    assert_eq!(
        closed_pair_traversal_flipped(&topo, ccw, nurbs_cw),
        Some(true)
    );
    assert_eq!(
        closed_pair_traversal_flipped(&topo, nurbs_cw, cw),
        Some(false)
    );

    // No tangent to compare: a line, or a ring that misses the seam.
    let line = topo.add_edge(Edge::new(v, v, EdgeCurve::Line));
    assert_eq!(closed_pair_traversal_flipped(&topo, ccw, line), None);
    let elsewhere = Circle3D::new(center + Vec3::new(0.0, 0.0, 1.0), Z, r).unwrap();
    let off = closed_edge(&mut topo, v, EdgeCurve::Circle(elsewhere), (0.0, TAU));
    assert_eq!(closed_pair_traversal_flipped(&topo, ccw, off), None);
}

// ── orient_revolved_face_wires ──────────────────────────────────────────

/// A point on a surface of revolution about the z axis through `apex`:
/// radius `radius(z)` at height `z` above it, angle `theta`.
struct Revolved {
    apex: Point3,
    radius: fn(f64) -> f64,
    surface: FaceSurface,
    /// Hand-derived outward normal at `(theta, z)`.
    outward: fn(f64, f64) -> Vec3,
}

impl Revolved {
    fn cylinder() -> Self {
        let apex = Point3::new(0.3, -0.7, 0.0);
        Self {
            apex,
            radius: |_| 2.0,
            surface: FaceSurface::Cylinder(CylindricalSurface::new(apex, Z, 2.0).unwrap()),
            outward: |theta, _| Vec3::new(theta.cos(), theta.sin(), 0.0),
        }
    }

    /// A cone opening upward from `apex`; its generator rises at 60° from
    /// the base plane, so the radius at height `z` is `z / tan 60°`.
    fn cone() -> Self {
        let apex = Point3::new(0.3, -0.7, 0.0);
        let a = std::f64::consts::FRAC_PI_3;
        Self {
            apex,
            radius: |z| z / std::f64::consts::FRAC_PI_3.tan(),
            surface: FaceSurface::Cone(ConicalSurface::new(apex, Z, a).unwrap()),
            // Perpendicular to the generator, away from the axis.
            outward: |theta, _| {
                let (s, c) = std::f64::consts::FRAC_PI_3.sin_cos();
                Vec3::new(theta.cos() * s, theta.sin() * s, -c)
            },
        }
    }

    fn at(&self, theta: f64, z: f64) -> Point3 {
        let r = (self.radius)(z);
        self.apex + Vec3::new(r * theta.cos(), r * theta.sin(), z)
    }

    fn parallel(&self, z: f64) -> Circle3D {
        Circle3D::new(self.apex + Vec3::new(0.0, 0.0, z), Z, (self.radius)(z)).unwrap()
    }

    /// A θ ∈ [θ0, θ1], z ∈ [z0, z1] patch loop, counter-clockwise in
    /// (θ, z) when `ccw` (which is outward-facing on both carriers).
    fn patch_loop(&self, topo: &mut Topology, th: (f64, f64), zs: (f64, f64), ccw: bool) -> WireId {
        let v00 = topo.add_vertex(Vertex::new(self.at(th.0, zs.0), 1e-7));
        let v10 = topo.add_vertex(Vertex::new(self.at(th.1, zs.0), 1e-7));
        let v11 = topo.add_vertex(Vertex::new(self.at(th.1, zs.1), 1e-7));
        let v01 = topo.add_vertex(Vertex::new(self.at(th.0, zs.1), 1e-7));
        let arc = |topo: &mut Topology, z: f64, s: VertexId, e: VertexId| {
            let circle = self.parallel(z);
            let t0 = circle.project(self.at(th.0, z));
            let mut edge = Edge::with_tolerance(s, e, EdgeCurve::Circle(circle), Some(1e-7));
            edge.set_trim(Some((t0, t0 + (th.1 - th.0))));
            topo.add_edge(edge)
        };
        let bottom = arc(topo, zs.0, v00, v10);
        let top = arc(topo, zs.1, v01, v11);
        let right = topo.add_edge(Edge::new(v10, v11, EdgeCurve::Line));
        let left = topo.add_edge(Edge::new(v01, v00, EdgeCurve::Line));
        let mut edges = vec![
            OrientedEdge::new(bottom, true),
            OrientedEdge::new(right, true),
            OrientedEdge::new(top, false),
            OrientedEdge::new(left, true),
        ];
        if !ccw {
            edges = edges
                .into_iter()
                .rev()
                .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
                .collect();
        }
        topo.add_wire(Wire::new(edges, true).unwrap())
    }

    /// Sign of the loop's Newell area along the outward normal at its
    /// center `(theta, z)`: positive = counter-clockwise seen from outside.
    fn winding(&self, topo: &Topology, wire: WireId, theta: f64, z: f64) -> f64 {
        let mut points = Vec::new();
        for oe in topo.wire(wire).unwrap().edges() {
            let edge = topo.edge(oe.edge()).unwrap();
            let (t0, t1) = edge.strict_domain().unwrap();
            let s = topo.vertex(edge.start()).unwrap().point();
            let e = topo.vertex(edge.end()).unwrap().point();
            for i in 0..16 {
                let f = f64::from(i) / 16.0;
                let f = if oe.is_forward() { f } else { 1.0 - f };
                points.push(
                    edge.curve()
                        .evaluate_with_endpoints(t0 + (t1 - t0) * f, s, e),
                );
            }
        }
        let mut area = Vec3::new(0.0, 0.0, 0.0);
        for i in 0..points.len() {
            let (p, q) = (points[i], points[(i + 1) % points.len()]);
            let (p, q) = (p - self.apex, q - self.apex);
            area += p.cross(q);
        }
        area.dot((self.outward)(theta, z))
    }
}

fn select(face: FaceId) -> [SelectedFace; 1] {
    [SelectedFace {
        face_id: face,
        source_face: face,
        reversed: false,
    }]
}

#[test]
fn revolved_outer_loops_are_rewound_to_face_outward() {
    for (label, carrier) in [
        ("cylinder", Revolved::cylinder()),
        ("cone", Revolved::cone()),
    ] {
        for ccw in [true, false] {
            let mut topo = Topology::new();
            let outer = carrier.patch_loop(&mut topo, (0.4, 1.9), (0.5, 2.0), ccw);
            let face = topo.add_face(Face::new(outer, vec![], carrier.surface.clone()));
            assert_eq!(
                carrier.winding(&topo, outer, 1.15, 1.25) > 0.0,
                ccw,
                "fixture self-check"
            );
            orient_revolved_face_wires(&mut topo, &select(face)).unwrap();
            let outer = topo.face(face).unwrap().outer_wire();
            assert!(
                carrier.winding(&topo, outer, 1.15, 1.25) > 0.0,
                "{label} outer loop built ccw={ccw} must run counter-clockwise outside"
            );
        }
    }
}

#[test]
fn revolved_holes_are_rewound_against_their_outer_loop() {
    // A cone patch with one hole, and a cylinder patch with two (a single
    // cylinder hole takes a different arm). Every hole must run clockwise
    // seen from outside, whatever winding the splitter handed over.
    for (label, carrier, holes) in [
        ("cone", Revolved::cone(), vec![((0.7, 0.9), (0.8, 1.1))]),
        (
            "cylinder",
            Revolved::cylinder(),
            vec![((0.6, 0.9), (0.8, 1.2)), ((1.2, 1.6), (1.4, 1.8))],
        ),
    ] {
        for (outer_ccw, hole_ccw) in [(true, true), (true, false), (false, true), (false, false)] {
            let mut topo = Topology::new();
            let outer = carrier.patch_loop(&mut topo, (0.4, 1.9), (0.5, 2.0), outer_ccw);
            let inner: Vec<_> = holes
                .iter()
                .enumerate()
                .map(|(i, &(th, zs))| {
                    // Alternate the hole windings on multi-hole faces.
                    carrier.patch_loop(&mut topo, th, zs, hole_ccw ^ (i % 2 == 1))
                })
                .collect();
            let face = topo.add_face(Face::new(outer, inner, carrier.surface.clone()));
            orient_revolved_face_wires(&mut topo, &select(face)).unwrap();
            let face_data = topo.face(face).unwrap();
            assert!(carrier.winding(&topo, face_data.outer_wire(), 1.15, 1.25) > 0.0);
            for (&wire, &(th, zs)) in face_data.inner_wires().iter().zip(&holes) {
                let (theta, z) = (0.5 * (th.0 + th.1), 0.5 * (zs.0 + zs.1));
                assert!(
                    carrier.winding(&topo, wire, theta, z) < 0.0,
                    "{label}: hole at θ {th:?} (outer ccw={outer_ccw}, hole ccw={hole_ccw}) \
                     must run clockwise seen from outside"
                );
            }
        }
    }
}

#[test]
fn a_hole_in_a_pointed_cone_whose_rim_has_no_uv_area_is_still_rewound() {
    // A pointed cone lateral bounded only by its full base rim: the outer
    // loop wraps the axis, so it has no contractible UV area. Its hole must
    // still end up clockwise seen from outside.
    let carrier = Revolved::cone();
    for hole_ccw in [true, false] {
        for rim_normal in [Z, -Z] {
            let mut topo = Topology::new();
            let rim = Circle3D::new(
                carrier.apex + Vec3::new(0.0, 0.0, 2.0),
                rim_normal,
                (carrier.radius)(2.0),
            )
            .unwrap();
            let seam = rim.evaluate(0.0);
            let v = topo.add_vertex(Vertex::new(seam, 1e-7));
            let mut edge = Edge::with_tolerance(v, v, EdgeCurve::Circle(rim), Some(1e-7));
            edge.set_trim(Some((0.0, TAU)));
            let rim_edge = topo.add_edge(edge);
            let outer =
                topo.add_wire(Wire::new(vec![OrientedEdge::new(rim_edge, true)], true).unwrap());
            let hole = carrier.patch_loop(&mut topo, (0.7, 1.3), (0.8, 1.4), hole_ccw);
            let face = topo.add_face(Face::new(outer, vec![hole], carrier.surface.clone()));
            orient_revolved_face_wires(&mut topo, &select(face)).unwrap();
            let hole = topo.face(face).unwrap().inner_wires()[0];
            assert!(
                carrier.winding(&topo, hole, 1.0, 1.1) < 0.0,
                "pointed-cone hole built ccw={hole_ccw}, rim normal {rim_normal:?}"
            );
        }
    }
}

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
    CapPlane, EdgeCurve, EdgeId, Face, FaceId, FaceSurface, OrientedEdge, Point3, SelectedFace,
    Topology, Vec3, WireId, analytic_cap_loop_area, cap_partial_overlap_free_loops,
    closed_pair_traversal_flipped, conics_share_support, measure_revolved_wire_uv_area,
    orient_revolved_face_wires, revolved_wire_uv_area,
};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

// A sector's area is angle/TAU times the complete disc/ellipse area. This
// oracle does not sum the line integrals used by the cap writer.
fn cap_sector(
    scale: f64,
    placed: bool,
    ellipse: bool,
    span: f64,
    reversed_carrier: bool,
) -> (Topology, Vec<OrientedEdge>, CapPlane, f64) {
    use remus_math::mat::Mat4;
    let placement = if placed {
        Mat4::translation(11.0 * scale, -7.0 * scale, 5.0 * scale)
            * Mat4::rotation_x(0.37)
            * Mat4::rotation_z(0.23)
    } else {
        Mat4::identity()
    };
    let direction = |v: Vec3| {
        Vec3::new(
            placement.0[0][0] * v.x() + placement.0[0][1] * v.y() + placement.0[0][2] * v.z(),
            placement.0[1][0] * v.x() + placement.0[1][1] * v.y() + placement.0[1][2] * v.z(),
            placement.0[2][0] * v.x() + placement.0[2][1] * v.y() + placement.0[2][2] * v.z(),
        )
    };
    let center = placement.mul_point(Point3::new(0.0, 0.0, 0.0));
    let normal = direction(Z);
    let carrier_sign = if reversed_carrier { -1.0 } else { 1.0 };
    let u = direction(Vec3::new(1.0, 0.0, 0.0));
    let v = direction(Vec3::new(0.0, carrier_sign, 0.0));
    let (a, b) = if ellipse { (3.0, 1.5) } else { (2.0, 2.0) };
    let curve = if ellipse {
        EdgeCurve::Ellipse(
            Ellipse3D::with_axes(center, normal * carrier_sign, a * scale, b * scale, u, v)
                .unwrap(),
        )
    } else {
        EdgeCurve::Circle(
            Circle3D::new_with_ref(center, normal * carrier_sign, a * scale, u).unwrap(),
        )
    };
    let mut topo = Topology::new();
    let start = topo.add_vertex(Vertex::new(
        curve.evaluate_with_endpoints(0.0, center, center),
        1e-7,
    ));
    let full_turn = (span.abs() - TAU).abs() < 1e-12;
    let end = if full_turn {
        start
    } else {
        topo.add_vertex(Vertex::new(
            curve.evaluate_with_endpoints(span, center, center),
            1e-7,
        ))
    };
    let mut edge = Edge::new(start, end, curve);
    edge.set_trim(Some((0.0, span)));
    let arc = topo.add_edge(edge);
    let mut edges = vec![OrientedEdge::new(arc, true)];
    if !full_turn {
        let middle = topo.add_vertex(Vertex::new(center, 1e-7));
        let first = topo.add_edge(Edge::new(end, middle, EdgeCurve::Line));
        let last = topo.add_edge(Edge::new(middle, start, EdgeCurve::Line));
        edges.extend([
            OrientedEdge::new(first, true),
            OrientedEdge::new(last, true),
        ]);
    }
    let cap = CapPlane {
        normal,
        d: normal.dot(center - Point3::new(0.0, 0.0, 0.0)),
        out_normal: normal,
    };
    (
        topo,
        edges,
        cap,
        0.5 * a * b * scale * scale * span * carrier_sign,
    )
}

#[test]
fn analytic_cap_area_recovers_the_major_sector_hidden_by_endpoint_chords() {
    let (topo, edges, cap, expected) = cap_sector(1.0, false, false, TAU * 0.75, false);
    let vertices: Vec<_> = edges
        .iter()
        .map(|usage| {
            topo.vertex(topo.edge(usage.edge()).unwrap().start())
                .unwrap()
                .point()
        })
        .collect();
    assert!(super::newell_normal(&vertices).dot(Z) < 0.0);
    let area = analytic_cap_loop_area(&topo, &edges, cap).unwrap().unwrap();
    assert!((area - expected).abs() < 1e-12);
    assert!(area > 0.0);
}

#[test]
fn analytic_cap_area_matches_sector_fractions_under_native_and_coedge_reversal() {
    for scale in [1e-3, 1.0, 1e3] {
        for placed in [false, true] {
            for ellipse in [false, true] {
                for reversed_carrier in [false, true] {
                    for span in [TAU * 0.25, TAU * 0.75, TAU, -TAU * 0.25, -TAU] {
                        let (topo, edges, cap, expected) =
                            cap_sector(scale, placed, ellipse, span, reversed_carrier);
                        for reverse in [false, true] {
                            let mut traversed = edges.clone();
                            if reverse {
                                traversed.reverse();
                                traversed = traversed
                                    .into_iter()
                                    .map(|edge| OrientedEdge::new(edge.edge(), !edge.is_forward()))
                                    .collect();
                            }
                            let actual = analytic_cap_loop_area(&topo, &traversed, cap)
                                .unwrap()
                                .unwrap();
                            let expected = if reverse { -expected } else { expected };
                            assert!(
                                (actual - expected).abs() < expected.abs() * 1e-12,
                                "scale={scale} placed={placed} ellipse={ellipse} span={span} carrier_reverse={reversed_carrier} reverse={reverse}: {actual} vs {expected}"
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn analytic_cap_area_declines_missing_authority_and_unsupported_or_unresolved_geometry() {
    let (mut topo, edges, cap, _) = cap_sector(1.0, false, false, TAU * 0.75, false);
    let arc = edges[0].edge();
    topo.edge_mut(arc).unwrap().set_trim(None);
    assert!(
        analytic_cap_loop_area(&topo, &edges, cap)
            .unwrap()
            .is_none()
    );
    topo.edge_mut(arc)
        .unwrap()
        .set_trim(Some((0.0, TAU * 0.75)));
    let start = topo.edge(arc).unwrap().start();
    let point = topo.vertex(start).unwrap().point();
    topo.vertex_mut(start)
        .unwrap()
        .set_point(point + Vec3::new(0.001, 0.0, 0.0));
    assert!(
        analytic_cap_loop_area(&topo, &edges, cap)
            .unwrap()
            .is_none()
    );

    let (mut topo, edges, cap, _) = cap_sector(1.0, false, false, TAU * 0.75, false);
    let arc = topo.edge(edges[0].edge()).unwrap();
    let (start, end) = (arc.start(), arc.end());
    let nurbs = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            topo.vertex(start).unwrap().point(),
            topo.vertex(end).unwrap().point(),
        ],
        vec![1.0, 1.0],
    )
    .unwrap();
    topo.edge_mut(edges[0].edge())
        .unwrap()
        .set_curve(EdgeCurve::NurbsCurve(nurbs));
    topo.edge_mut(edges[0].edge())
        .unwrap()
        .set_trim(Some((0.0, 1.0)));
    assert!(
        analytic_cap_loop_area(&topo, &edges, cap)
            .unwrap()
            .is_none()
    );

    let (topo, edges, cap, _) = cap_sector(1.0, false, false, TAU, false);
    let cancelling = vec![edges[0], OrientedEdge::new(edges[0].edge(), false)];
    assert!(
        analytic_cap_loop_area(&topo, &cancelling, cap)
            .unwrap()
            .is_none()
    );
    let mut nonplanar = cap;
    nonplanar.normal = Vec3::new(1.0, 0.0, 0.0);
    assert!(
        analytic_cap_loop_area(&topo, &edges, nonplanar)
            .unwrap()
            .is_none()
    );
    let mut nonfinite = cap;
    nonfinite.out_normal = Vec3::new(f64::NAN, 0.0, 1.0);
    assert!(
        analytic_cap_loop_area(&topo, &edges, nonfinite)
            .unwrap()
            .is_none()
    );
    let (mut topo, edges, cap, _) = cap_sector(1.0, false, false, TAU * 0.75, false);
    let middle = topo.edge(edges[1].edge()).unwrap().end();
    let point = topo.vertex(middle).unwrap().point();
    topo.vertex_mut(middle)
        .unwrap()
        .set_point(point + Vec3::new(0.0, 0.0, 0.001));
    assert!(
        analytic_cap_loop_area(&topo, &edges, cap)
            .unwrap()
            .is_none()
    );
}

#[test]
fn analytic_cap_area_declines_a_quantized_but_disconnected_line_junction() {
    let (mut topo, edges, cap, _) = cap_sector(1.0, false, false, TAU * 0.75, false);
    let middle = topo.edge(edges[1].edge()).unwrap().end();
    let point = topo.vertex(middle).unwrap().point();
    let duplicate = topo.add_vertex(Vertex::new(point + Vec3::new(1e-9, 0.0, 0.0), 1e-7));
    topo.edge_mut(edges[1].edge()).unwrap().set_end(duplicate);
    assert!(
        analytic_cap_loop_area(&topo, &edges, cap)
            .unwrap()
            .is_none()
    );
}

#[test]
fn analytic_cap_area_declines_native_detail_lost_in_a_distant_frame() {
    // The exact loop has area about -5e6. Evaluating both native points in
    // this local frame first rounds their small arc delta to zero and can
    // falsely report +theta/2. Unresolved endpoint authority must decline.
    let mut topo = Topology::new();
    let circle =
        Circle3D::new_with_ref(Point3::new(0.0, 0.0, 0.0), Z, 1.0, Vec3::new(1.0, 0.0, 0.0))
            .unwrap();
    let far = topo.add_vertex(Vertex::new(Point3::new(1e12, 1e12, 0.0), 1e-7));
    let a = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let b = topo.add_vertex(Vertex::new(circle.evaluate(1e-5), 1e-7));
    let first = topo.add_edge(Edge::new(far, a, EdgeCurve::Line));
    let mut arc = Edge::new(a, b, EdgeCurve::Circle(circle));
    arc.set_trim(Some((0.0, 1e-5)));
    let arc = topo.add_edge(arc);
    let last = topo.add_edge(Edge::new(b, far, EdgeCurve::Line));
    let edges = [
        OrientedEdge::new(first, true),
        OrientedEdge::new(arc, true),
        OrientedEdge::new(last, true),
    ];
    let cap = CapPlane {
        normal: Z,
        d: 0.0,
        out_normal: Z,
    };
    assert!(
        analytic_cap_loop_area(&topo, &edges, cap)
            .unwrap()
            .is_none()
    );

    // A tiny body translated so far that all stored endpoints collapse is
    // likewise unsupported; global native evaluation cannot grant authority.
    let (mut topo, edges, _, _) = cap_sector(1e-3, false, false, TAU * 0.75, false);
    let shift = Vec3::new(1e16, 1e16, 1e16);
    let vertices: Vec<_> = edges
        .iter()
        .flat_map(|usage| {
            let edge = topo.edge(usage.edge()).unwrap();
            [edge.start(), edge.end()]
        })
        .collect::<std::collections::BTreeSet<_>>()
        .into_iter()
        .map(|id| (id, topo.vertex(id).unwrap().point()))
        .collect();
    for (id, point) in vertices {
        topo.vertex_mut(id).unwrap().set_point(point + shift);
    }
    let arc = edges[0].edge();
    topo.edge_mut(arc).unwrap().set_curve(EdgeCurve::Circle(
        Circle3D::new_with_ref(
            Point3::new(1e16, 1e16, 1e16),
            Z,
            2e-3,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    ));
    let cap = CapPlane {
        normal: Z,
        d: 1e16,
        out_normal: Z,
    };
    assert!(
        analytic_cap_loop_area(&topo, &edges, cap)
            .unwrap()
            .is_none()
    );
}

#[test]
fn cap_area_interval_invalid_values_never_decide_a_sign() {
    use super::cap_area_math::I;
    for invalid in [
        I { lo: 2.0, hi: 1.0 },
        I::point(f64::NAN),
        I::point(f64::INFINITY),
    ] {
        assert!(!invalid.abs().finite());
        assert!(!invalid.mul(I::point(0.0)).finite());
        assert!(!invalid.add(I::point(1.0)).finite());
    }
    for parameter in [0.0, TAU * 0.25, TAU * 0.5, TAU, -TAU] {
        let (sine, cosine) = super::cap_area_math::sin_cos(parameter).unwrap();
        assert!(sine.lo <= parameter.sin() && sine.hi >= parameter.sin());
        assert!(cosine.lo <= parameter.cos() && cosine.hi >= parameter.cos());
    }
    assert!(super::cap_area_math::sin_cos(1e10).is_none());
}

#[test]
fn generated_major_sector_caps_oppose_every_incident_wall_coedge() {
    for reverse_faces in [false, true] {
        let mut topo = Topology::new();
        let circle = |height| {
            Circle3D::new_with_ref(
                Point3::new(0.0, 0.0, height),
                Z,
                2.0,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap()
        };
        let bottom = circle(0.0);
        let top = circle(1.0);
        let a = topo.add_vertex(Vertex::new(bottom.evaluate(0.0), 1e-7));
        let b = topo.add_vertex(Vertex::new(bottom.evaluate(TAU * 0.75), 1e-7));
        let o = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let aa = topo.add_vertex(Vertex::new(top.evaluate(0.0), 1e-7));
        let bb = topo.add_vertex(Vertex::new(top.evaluate(TAU * 0.75), 1e-7));
        let oo = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 1.0), 1e-7));
        let bottom_arc = closed_edge(&mut topo, a, EdgeCurve::Circle(bottom), (0.0, TAU * 0.75));
        topo.edge_mut(bottom_arc).unwrap().set_end(b);
        let top_arc = closed_edge(&mut topo, aa, EdgeCurve::Circle(top), (0.0, TAU * 0.75));
        topo.edge_mut(top_arc).unwrap().set_end(bb);
        let bo = topo.add_edge(Edge::new(b, o, EdgeCurve::Line));
        let oa = topo.add_edge(Edge::new(o, a, EdgeCurve::Line));
        let bboo = topo.add_edge(Edge::new(bb, oo, EdgeCurve::Line));
        let ooaa = topo.add_edge(Edge::new(oo, aa, EdgeCurve::Line));
        let aaaa = topo.add_edge(Edge::new(a, aa, EdgeCurve::Line));
        let bbbb = topo.add_edge(Edge::new(b, bb, EdgeCurve::Line));
        let oooo = topo.add_edge(Edge::new(o, oo, EdgeCurve::Line));
        let mut face = |edges: Vec<(EdgeId, bool)>, surface| {
            let wire = topo.add_wire(
                Wire::new(
                    edges
                        .into_iter()
                        .map(|(edge, forward)| OrientedEdge::new(edge, forward))
                        .collect(),
                    true,
                )
                .unwrap(),
            );
            topo.add_face(Face::new(wire, vec![], surface))
        };
        let mut faces = vec![
            face(
                vec![
                    (bottom_arc, true),
                    (bbbb, true),
                    (top_arc, false),
                    (aaaa, false),
                ],
                FaceSurface::Cylinder(
                    CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Z, 2.0).unwrap(),
                ),
            ),
            face(
                vec![(bo, true), (oooo, true), (bboo, false), (bbbb, false)],
                FaceSurface::Plane {
                    normal: Vec3::new(1.0, 0.0, 0.0),
                    d: 0.0,
                },
            ),
            face(
                vec![(oa, true), (aaaa, true), (ooaa, false), (oooo, false)],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, -1.0, 0.0),
                    d: 0.0,
                },
            ),
        ];
        if reverse_faces {
            faces.reverse();
        }
        let mut sources = faces.iter().copied().map(Some).collect();
        cap_partial_overlap_free_loops(
            &mut topo,
            &mut faces,
            &mut sources,
            &[
                CapPlane {
                    normal: Z,
                    d: 0.0,
                    out_normal: -Z,
                },
                CapPlane {
                    normal: Z,
                    d: 1.0,
                    out_normal: Z,
                },
            ],
        )
        .unwrap();
        assert_eq!(faces.len(), 5);
        let mut uses = std::collections::BTreeMap::<usize, Vec<bool>>::new();
        for id in faces {
            let face = topo.face(id).unwrap();
            for oriented in topo.wire(face.outer_wire()).unwrap().edges() {
                uses.entry(oriented.edge().index())
                    .or_default()
                    .push(oriented.is_forward() != face.is_reversed());
            }
        }
        assert!(
            uses.values()
                .all(|uses| uses.len() == 2 && uses[0] != uses[1]),
            "{uses:?}"
        );
    }
}

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

// ── wire area memo ──────────────────────────────────────────────────────

/// Patch loops on a cylinder and a cone, both windings, built into a fresh
/// topology each call (so equal content arrives under new entity ids).
fn area_fixtures() -> (Topology, Vec<(WireId, FaceSurface)>) {
    let mut topo = Topology::new();
    let mut wires = Vec::new();
    for carrier in [Revolved::cylinder(), Revolved::cone()] {
        for (th, zs) in [((0.4, 1.9), (0.5, 2.0)), ((-2.0, 3.5), (1.0, 1.5))] {
            for ccw in [true, false] {
                let wire = carrier.patch_loop(&mut topo, th, zs, ccw);
                wires.push((wire, carrier.surface.clone()));
            }
        }
    }
    (topo, wires)
}

fn area_bits(topo: &Topology, wires: &[(WireId, FaceSurface)]) -> Vec<Option<u64>> {
    wires
        .iter()
        .map(|(wire, surface)| {
            revolved_wire_uv_area(topo, *wire, surface)
                .unwrap()
                .map(f64::to_bits)
        })
        .collect()
}

#[test]
fn memoized_wire_areas_are_bit_identical_and_keyed_on_content() {
    use crate::wire_area_memo::{
        enable_thread_wire_area_memo, set_thread_wire_area_memo_capacity,
        thread_wire_area_memo_stats,
    };
    let (topo, wires) = area_fixtures();
    let measured: Vec<Option<u64>> = wires
        .iter()
        .map(|(wire, surface)| {
            measure_revolved_wire_uv_area(&topo, *wire, surface)
                .unwrap()
                .map(f64::to_bits)
        })
        .collect();
    assert!(
        measured.iter().all(Option::is_some),
        "fixtures must have area"
    );

    set_thread_wire_area_memo_capacity(0);
    let off = area_bits(&topo, &wires);
    enable_thread_wire_area_memo();
    let stored = area_bits(&topo, &wires);
    // The same content under new ids is a hit.
    let (fresh_topo, fresh_wires) = area_fixtures();
    let recalled = area_bits(&fresh_topo, &fresh_wires);
    let stats = thread_wire_area_memo_stats();
    set_thread_wire_area_memo_capacity(0);

    assert_eq!(off, measured);
    assert_eq!(stored, measured);
    assert_eq!(recalled, measured);
    // Each loop and its reverse have their own key: one miss each while
    // storing, then one hit each.
    let n = wires.len() as u64;
    assert_eq!((stats.misses, stats.hits), (n, n), "{stats:?}");
}

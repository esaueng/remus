#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;
use remus_math::{aabb::Aabb3, mat::Mat4};
use remus_topology::{
    face::Face,
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};

fn direction(m: Mat4, v: Vec3) -> Vec3 {
    Vec3::new(
        m.0[0][0] * v.x() + m.0[0][1] * v.y() + m.0[0][2] * v.z(),
        m.0[1][0] * v.x() + m.0[1][1] * v.y() + m.0[1][2] * v.z(),
        m.0[2][0] * v.x() + m.0[2][1] * v.y() + m.0[2][2] * v.z(),
    )
}

fn circle_edge(topo: &mut Topology, circle: Circle3D) -> remus_topology::edge::EdgeId {
    let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, TAU)));
    topo.add_edge(edge)
}

fn disc(topo: &mut Topology, circle: Circle3D) -> FaceId {
    let normal = circle.normal();
    let d = normal.dot(position_vector(circle.center()));
    let edge = circle_edge(topo, circle);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d }))
}

fn wall(topo: &mut Topology, cyl: CylindricalSurface, height: f64) -> FaceId {
    let bottom = circle_edge(
        topo,
        Circle3D::new_with_ref(cyl.origin(), cyl.axis(), cyl.radius(), cyl.x_axis()).unwrap(),
    );
    let top = circle_edge(
        topo,
        Circle3D::new_with_ref(
            cyl.origin() + cyl.axis() * height,
            cyl.axis(),
            cyl.radius(),
            cyl.x_axis(),
        )
        .unwrap(),
    );
    let seam = topo.add_edge(Edge::new(
        topo.edge(bottom).unwrap().start(),
        topo.edge(top).unwrap().start(),
        EdgeCurve::Line,
    ));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(bottom, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(top, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(cyl)))
}

fn attach_owned_caps(topo: &mut Topology, lateral: FaceId) -> Vec<FaceId> {
    let wire = topo.wire(topo.face(lateral).unwrap().outer_wire()).unwrap();
    // `wall` emits the two rim uses at these fixed fixture positions.
    let rims = [wire.edges()[0].edge(), wire.edges()[2].edge()];
    rims.into_iter()
        .map(|rim| {
            let EdgeCurve::Circle(circle) = topo.edge(rim).unwrap().curve() else {
                unreachable!()
            };
            let normal = circle.normal();
            let d = normal.dot(position_vector(circle.center()));
            let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
            topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d }))
        })
        .collect()
}

fn early_fixture(scale: f64, placement: Mat4, offset: f64) -> (Topology, [FaceId; 2]) {
    let mut topo = Topology::new();
    let first = CylindricalSurface::new(
        placement.mul_point(Point3::new(0.0, 0.0, -2.0 * scale)),
        direction(placement, Vec3::new(0.0, 0.0, 1.0)),
        2.0 * scale,
    )
    .unwrap();
    let axis = Vec3::new(0.0, 3.0_f64.sqrt() * 0.5, 0.5);
    let second = CylindricalSurface::new(
        placement.mul_point(Point3::new(
            offset * scale,
            -4.0 * axis.y() * scale,
            -2.0 * scale,
        )),
        direction(placement, axis),
        scale,
    )
    .unwrap();
    let first_id = wall(&mut topo, first, 4.0 * scale);
    let second_id = wall(&mut topo, second, 8.0 * scale);
    // Deliberately create the cap faces after both lateral faces. The event
    // pool must be available before the normal FF order reaches these caps.
    attach_owned_caps(&mut topo, first_id);
    attach_owned_caps(&mut topo, second_id);
    (topo, [first_id, second_id])
}

#[test]
fn early_owned_cap_events_match_independent_cartesian_roots() {
    // x²+y²=4 and x²+(.5y-sqrt(3)/2*z)²=1 at z=±2.
    // The admissible y root is ±2*(sqrt(21)-sqrt(3))/3; the
    // other quadratic root is outside the stock circle.
    let y = 2.0 * (21.0_f64.sqrt() - 3.0_f64.sqrt()) / 3.0;
    let x = (4.0 - y * y).sqrt();
    for scale in [1e-3, 1.0, 1e3] {
        for placed in [false, true] {
            let placement = if placed {
                Mat4::translation(11.0 * scale, -7.0 * scale, 5.0 * scale)
                    * Mat4::rotation_x(0.37)
                    * Mat4::rotation_z(0.23)
            } else {
                Mat4::identity()
            };
            let (topo, ids) = early_fixture(scale, placement, 0.0);
            let points =
                early_cylinder_cap_junctions(&topo, ids[0], ids[1], &OperationContext::default())
                    .unwrap()
                    .unwrap();
            assert_eq!(points.len(), 4, "scale={scale} placed={placed}");
            for sx in [-1.0, 1.0] {
                for sz in [-1.0, 1.0] {
                    let expected = placement.mul_point(Point3::new(
                        sx * x * scale,
                        sz * y * scale,
                        sz * 2.0 * scale,
                    ));
                    assert_eq!(
                        points
                            .iter()
                            .filter(|&&point| (point - expected).length() < 1e-8)
                            .count(),
                        1,
                        "scale={scale} placed={placed} expected={expected:?} points={points:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn early_cap_pool_requires_unique_shared_rim_ownership() {
    let context = OperationContext::default();
    let (mut topo, ids) = early_fixture(1.0, Mat4::identity(), 0.0);
    let rim = topo
        .wire(topo.face(ids[0]).unwrap().outer_wire())
        .unwrap()
        .edges()[0]
        .edge();
    let EdgeCurve::Circle(circle) = topo.edge(rim).unwrap().curve() else {
        unreachable!()
    };
    let normal = circle.normal();
    let d = normal.dot(position_vector(circle.center()));
    let duplicate = circle.clone();
    // A coincident disc carrying a different edge cannot supply authority or
    // create ambiguity for the real shared-edge cap.
    disc(&mut topo, duplicate);
    assert_eq!(
        early_cylinder_cap_junctions(&topo, ids[0], ids[1], &context)
            .unwrap()
            .unwrap()
            .len(),
        4
    );
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
    topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d }));
    assert!(
        early_cylinder_cap_junctions(&topo, ids[0], ids[1], &context)
            .unwrap()
            .is_none()
    );

    let (mut missing, missing_ids) = early_fixture(1.0, Mat4::identity(), 0.0);
    let cap = owned_caps(&missing, missing_ids[0], &context)
        .unwrap()
        .unwrap()[0];
    let original = closed_disc(&missing, cap, context.tolerance).unwrap();
    let foreign = circle_edge(&mut missing, original);
    let replacement =
        missing.add_wire(Wire::new(vec![OrientedEdge::new(foreign, true)], true).unwrap());
    missing
        .set_face_boundary_wires(cap, replacement, vec![])
        .unwrap();
    assert!(
        early_cylinder_cap_junctions(&missing, missing_ids[0], missing_ids[1], &context)
            .unwrap()
            .is_none()
    );
}

#[test]
fn early_owned_cap_pool_certifies_disjoint_full_cylinders() {
    let (topo, ids) = early_fixture(1.0, Mat4::identity(), 10.0);
    assert!(
        early_cylinder_cap_junctions(&topo, ids[0], ids[1], &OperationContext::default())
            .unwrap()
            .unwrap()
            .is_empty()
    );
}

#[test]
fn f3_large_placed_cap_event_keeps_default_absolute_tolerance() {
    let scale = 1e3;
    let placement = Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale)
        * Mat4::rotation_x(0.37)
        * Mat4::rotation_z(0.23);
    let tool_placement = placement
        * Mat4::translation(4.0 * scale, -2.0 * scale, -0.5 * scale)
        * Mat4::rotation_x(PI / 6.0);
    let transformed = |matrix: Mat4, radius| {
        let base =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), radius)
                .unwrap();
        CylindricalSurface::with_ref_dir(
            matrix.mul_point(base.origin()),
            direction(matrix, base.axis()),
            radius,
            direction(matrix, base.x_axis()),
        )
        .unwrap()
    };
    let mut topo = Topology::new();
    let stock = wall(&mut topo, transformed(placement, 2.0 * scale), 3.5 * scale);
    let tool = wall(
        &mut topo,
        transformed(tool_placement, 5.5 * scale),
        1.5 * scale,
    );
    attach_owned_caps(&mut topo, stock);
    attach_owned_caps(&mut topo, tool);
    let context = OperationContext::default();
    assert_eq!(context.tolerance.linear.to_bits(), 1e-7_f64.to_bits());
    let points = early_cylinder_cap_junctions(&topo, stock, tool, &context)
        .unwrap()
        .unwrap();
    // Independent 90-digit decimal solution: eliminate x with
    // x=(-10.25-y²/4+sqrt(3)*(sqrt(3)+.25)*y+(sqrt(3)+.25)²)/8
    // and solve x²+y²=4. The other stock-bottom root has tool v<0.
    let expected = placement.mul_point(Point3::new(
        -1.443_898_073_929_639 * scale,
        -1.383_892_464_067_305_5 * scale,
        0.0,
    ));
    assert_eq!(
        points
            .iter()
            .filter(|&&point| (point - expected).length() <= 1e-8)
            .count(),
        1,
        "expected={expected:?} points={points:?}"
    );
}

#[test]
fn coefficient_floor_requires_a_resolved_physical_image() {
    // This coefficient interval has one monotone root family. Its arithmetic
    // floor cannot meet the requested parameter target, but its complete
    // image on a small ellipse is still tightly resolved in physical space.
    let polynomial = [
        I {
            lo: -1e-12,
            hi: 1e-12,
        },
        I::point(1.0),
        I::point(0.0),
        I::point(0.0),
        I::point(0.0),
    ];
    let intervals = roots(&polynomial, 1e-16).unwrap();
    assert_eq!(intervals.len(), 1);
    assert!(intervals[0].lo <= -1e-12 && intervals[0].hi >= 1e-12);
    assert!(intervals[0].hi - intervals[0].lo > 1e-16);
    let ellipse = Ellipse3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        2.0,
        1.0,
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let chart = Chart {
        phase: 0.0,
        a: vector(Vec3::new(2.0, 0.0, 0.0)),
        b: vector(Vec3::new(0.0, 1.0, 0.0)),
        events: [polynomial; 3],
    };
    assert!(check_root_image(&chart, &ellipse, intervals[0], 0.0, 1e-10).is_ok());
    assert!(matches!(
        check_root_image(&chart, &ellipse, intervals[0], 0.0, 1e-13),
        Err(AlgoError::FaceSplitFailed(_))
    ));
}

fn raw(ellipse: Ellipse3D) -> RawCurve {
    let center = ellipse.center();
    let a = ellipse.u_axis() * ellipse.semi_major();
    let b = ellipse.v_axis() * ellipse.semi_minor();
    let bbox = Aabb3::from_points([
        center + a + b,
        center + a - b,
        center - a + b,
        center - a - b,
    ]);
    let start = ellipse.evaluate(0.0);
    RawCurve {
        curve: EdgeCurve::Ellipse(ellipse),
        bbox,
        t_range: (0.0, TAU),
        p_start: start,
        p_end: start,
    }
}

fn fixture(
    scale: f64,
    placement: Mat4,
    radius: f64,
    height: f64,
    flipped: bool,
) -> (Topology, FaceId, FaceId, RawCurve, Tolerance) {
    let mut topo = Topology::new();
    let axis = Vec3::new(0.0, 3.0_f64.sqrt() / 2.0, 0.5);
    let center = placement.mul_point(Point3::new(0.0, 0.0, 0.0));
    let cap = disc(
        &mut topo,
        Circle3D::new(
            center,
            direction(placement, Vec3::new(0.0, 0.0, 1.0)),
            radius * scale,
        )
        .unwrap(),
    );
    let cyl = CylindricalSurface::new(
        placement.mul_point(Point3::new(
            0.0,
            -axis.y() * height * scale / 2.0,
            -axis.z() * height * scale / 2.0,
        )),
        direction(placement, axis),
        scale,
    )
    .unwrap();
    let lateral = wall(&mut topo, cyl, height * scale);
    let sign = if flipped { -1.0 } else { 1.0 };
    let ellipse = Ellipse3D::with_axes(
        center,
        direction(placement, Vec3::new(0.0, 0.0, -1.0)),
        2.0 * scale,
        scale,
        direction(placement, Vec3::new(0.0, sign, 0.0)),
        direction(placement, Vec3::new(sign, 0.0, 0.0)),
    )
    .unwrap();
    (
        topo,
        cap,
        lateral,
        raw(ellipse),
        Tolerance {
            linear: 1e-7 * scale,
            ..Tolerance::new()
        },
    )
}

fn clip(
    topo: &Topology,
    cap: FaceId,
    lateral: FaceId,
    raw: &RawCurve,
    tol: Tolerance,
) -> Result<Option<Vec<RawCurve>>, AlgoError> {
    clip_closed_ellipse_to_circular_cap(
        topo,
        cap,
        lateral,
        topo.face(cap).unwrap().surface(),
        topo.face(lateral).unwrap().surface(),
        raw,
        tol,
        &mut JunctionRegistry::default(),
    )
}

fn intervals(arcs: &[RawCurve]) -> Vec<(f64, f64)> {
    let mut out = arcs
        .iter()
        .map(|arc| {
            let start = arc.t_range.0.rem_euclid(TAU);
            (start, start + arc.t_range.1 - arc.t_range.0)
        })
        .collect::<Vec<_>>();
    out.sort_by(|a, b| a.0.total_cmp(&b.0));
    out
}

#[test]
fn four_simple_polynomial_roots_are_all_isolated() {
    let p = [0.015_625, 0.0, -0.3125, 0.0, 1.0].map(I::point);
    let mut found = roots(&p, 1e-11).unwrap();
    found.sort_by(|a, b| a.lo.total_cmp(&b.lo));
    assert_eq!(found.len(), 4);
    for (root, want) in found.iter().zip([-0.5, -0.25, 0.25, 0.5]) {
        assert!(root.lo <= want && root.hi >= want, "{root:?} misses {want}");
    }
    assert!(
        roots(&[0.1, 0.0, 1.0, 0.0, 0.0].map(I::point), 1e-11)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn invalid_interval_arithmetic_propagates_unknown_without_a_finite_certificate() {
    for bad in [
        I {
            lo: f64::NAN,
            hi: 1.0,
        },
        I {
            lo: 1.0,
            hi: f64::NAN,
        },
        I { lo: 2.0, hi: 1.0 },
        I {
            lo: 0.0,
            hi: f64::INFINITY,
        },
    ] {
        assert!(!bad.finite());
        for result in [
            bad.add(I::point(1.0)),
            bad.mul(I::point(1.0)),
            bad.neg(),
            bad.abs(),
            I::point(1.0).div_positive(bad),
        ] {
            assert!(!result.finite());
            assert!(result.lo.is_infinite() && result.lo.is_sign_negative());
            assert!(result.hi.is_infinite() && result.hi.is_sign_positive());
        }
    }
    for result in [
        I::point(f64::MAX).add(I::point(f64::MAX)),
        I::point(f64::MAX).scale(2.0),
        I::point(1.0).div_positive(I::point(0.0)),
        I::point(1.0).div_positive(I { lo: -1.0, hi: 1.0 }),
    ] {
        assert!(!result.finite());
        assert!(result.lo.is_sign_negative() && result.hi.is_sign_positive());
    }
    let valid = I { lo: -2.0, hi: 3.0 }.mul(I::point(2.0));
    assert!(valid.finite() && valid.lo <= -4.0 && valid.hi >= 6.0);
}

#[test]
fn multiple_roots_refuse_instead_of_certifying_empty() {
    for p in [[0.0, 0.0, 1.0, 0.0, 0.0], [0.0625, 0.0, -0.5, 0.0, 1.0]] {
        assert!(roots(&p.map(I::point), 1e-11).is_err());
    }
}

#[test]
fn carrier_residual_encloses_closed_form_radial_translation_at_each_scale() {
    for radius in [1e-3, 1.0, 1e3] {
        let cylinder =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), radius)
                .unwrap();
        for fraction in [0.25, 1.0] {
            let linear = 1e-7 * radius;
            let offset = linear * fraction;
            // Radial projection is exactly the radius-r circle translated
            // by offset along x: F(t) = offset² + 2r·offset·sin(t).
            let ellipse = Ellipse3D::with_axes(
                Point3::new(offset, 0.0, 0.0),
                Vec3::new(0.0, -3.0_f64.sqrt() / 2.0, 0.5),
                2.0 * radius,
                radius,
                Vec3::new(0.0, 0.5, 3.0_f64.sqrt() / 2.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap();
            let bound = ellipse_cylinder_residual(&ellipse, &cylinder).unwrap();
            let expected = offset.mul_add(offset, 2.0 * radius * offset);
            assert!(bound.lo <= expected.next_up() && bound.hi >= expected.next_down());
            assert!(bound.hi <= expected + 4096.0 * f64::EPSILON * radius * radius);
            let limit = I::point(linear).mul(I::point(radius));
            assert_eq!(bound.hi <= limit.lo, fraction < 0.5);
        }
    }
}

#[test]
fn plane_residual_encloses_offset_and_retains_roundoff_uncertainty() {
    let normal = Vec3::new(0.0, 0.0, 1.0);
    let make = |center| {
        Ellipse3D::with_axes(
            center,
            normal,
            2.0,
            1.0,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap()
    };
    let offset = 1.0 / 33_554_432.0;
    let bound = ellipse_plane_residual(&make(Point3::new(0.0, 0.0, offset)), normal, 0.0).unwrap();
    assert!(bound.lo <= offset && bound.hi >= offset);
    assert!(bound.hi < 1e-7);

    // Even exact cancellation of large stored coordinates must retain the
    // interval's arithmetic uncertainty. A bound wider than the qualification
    // threshold declines rather than presenting a rounded zero as proof.
    let large = 1_099_511_627_776.0;
    let bound = ellipse_plane_residual(&make(Point3::new(0.0, 0.0, large)), normal, large).unwrap();
    assert!(bound.hi > 1e-7);
}

#[test]
fn algebraic_circle_ellipse_windows_survive_scale_placement_and_parameter_half_turn() {
    let poses = [
        Mat4::identity(),
        Mat4::translation(11.0, -7.0, 5.0) * Mat4::rotation_x(0.37) * Mat4::rotation_z(-0.21),
    ];
    for scale in [1e-3, 1.0, 1e3] {
        for pose in poses {
            for flipped in [false, true] {
                let (topo, cap, lateral, raw, tol) =
                    fixture(scale, pose, 2.5_f64.sqrt(), 10.0, flipped);
                let arcs = clip(&topo, cap, lateral, &raw, tol).unwrap().unwrap();
                let got = intervals(&arcs);
                assert_eq!(
                    got.len(),
                    4,
                    "each rim-to-rim lens needs an interior vertex: {got:?}"
                );
                for ((start, end), (a, b)) in got.iter().zip([
                    (PI / 4.0, PI / 2.0),
                    (PI / 2.0, 3.0 * PI / 4.0),
                    (5.0 * PI / 4.0, 3.0 * PI / 2.0),
                    (3.0 * PI / 2.0, 7.0 * PI / 4.0),
                ]) {
                    assert!(
                        (start - a).abs() < 1e-8 && (end - b).abs() < 1e-8,
                        "{got:?}"
                    );
                }
                let mut rim_endpoints = 0;
                let mut interior_endpoints = 0;
                for arc in arcs {
                    assert!(matches!(arc.curve, EdgeCurve::Ellipse(_)));
                    for point in [arc.p_start, arc.p_end] {
                        let disc = closed_disc(&topo, cap, tol).unwrap();
                        let radius = (point - disc.center()).length();
                        if (radius - disc.radius()).abs() < tol.linear {
                            rim_endpoints += 1;
                        } else {
                            // Both exact child endpoints at the shared
                            // midpoint lie on the ellipse's minor axis,
                            // strictly inside the circle, never on its rim.
                            assert!((radius - scale).abs() < tol.linear);
                            interior_endpoints += 1;
                        }
                    }
                }
                assert_eq!((rim_endpoints, interior_endpoints), (4, 4));
            }
        }
    }
}

#[test]
fn cylinder_axial_events_bound_every_material_interval() {
    let (topo, cap, lateral, raw, tol) = fixture(1.0, Mat4::identity(), 2.5_f64.sqrt(), 1.0, false);
    let arcs = clip(&topo, cap, lateral, &raw, tol).unwrap().unwrap();
    let got = intervals(&arcs);
    let a = (0.5 / 3.0_f64.sqrt()).acos();
    assert_eq!(got.len(), 2, "{got:?}");
    for ((start, end), (lo, hi)) in got.iter().zip([(a, PI - a), (PI + a, TAU - a)]) {
        assert!(
            (start - lo).abs() < 1e-8 && (end - hi).abs() < 1e-8,
            "{got:?}"
        );
    }
}

#[test]
fn disjoint_and_fully_contained_sections_have_certified_distinct_outcomes() {
    let (topo, cap, lateral, raw, tol) = fixture(1.0, Mat4::identity(), 0.5, 10.0, false);
    assert!(
        clip(&topo, cap, lateral, &raw, tol)
            .unwrap()
            .unwrap()
            .is_empty()
    );
    let (topo, cap, lateral, raw, tol) = fixture(1.0, Mat4::identity(), 3.0, 10.0, false);
    let kept = clip(&topo, cap, lateral, &raw, tol).unwrap().unwrap();
    assert_eq!(kept.len(), 1);
    assert_eq!(kept[0].t_range, raw.t_range);
    let (topo, cap, lateral, raw, tol) = fixture(1.0, Mat4::identity(), 1.0, 10.0, false);
    assert!(
        clip(&topo, cap, lateral, &raw, tol).is_err(),
        "tangent contact must remain explicit"
    );
}

#[test]
fn missing_partial_and_off_carrier_boundary_authority_decline() {
    for trim in [None, Some((0.0, PI))] {
        let (mut topo, cap, lateral, raw, tol) = fixture(1.0, Mat4::identity(), 2.0, 10.0, false);
        let wire = topo.face(cap).unwrap().outer_wire();
        let edge = topo.wire(wire).unwrap().edges()[0].edge();
        topo.edge_mut(edge).unwrap().set_trim(trim);
        assert!(closed_disc(&topo, cap, tol).is_none());
        assert!(clip(&topo, cap, lateral, &raw, tol).unwrap().is_none());
    }
    let (mut topo, cap, lateral, raw, tol) = fixture(1.0, Mat4::identity(), 2.0, 10.0, false);
    let wire = topo.face(lateral).unwrap().outer_wire();
    let edge = topo.wire(wire).unwrap().edges()[0].edge();
    let vertex = topo.edge(edge).unwrap().start();
    let old = topo.vertex(vertex).unwrap().point();
    topo.vertex_mut(vertex)
        .unwrap()
        .set_point(old + Vec3::new(0.0, 0.0, 0.1));
    assert!(clip(&topo, cap, lateral, &raw, tol).unwrap().is_none());
}

#[test]
fn f3_bottom_cap_retains_the_real_window_between_all_coarse_samples() {
    for scale in [1e-3, 1.0, 1e3] {
        let mut topo = Topology::new();
        let tol = Tolerance {
            linear: 1e-7 * scale,
            ..Tolerance::new()
        };
        let cap = disc(
            &mut topo,
            Circle3D::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                2.0 * scale,
            )
            .unwrap(),
        );
        let cyl = CylindricalSurface::new(
            Point3::new(4.0 * scale, -2.0 * scale, -0.5 * scale),
            Vec3::new(0.0, -0.5, 3.0_f64.sqrt() / 2.0),
            5.5 * scale,
        )
        .unwrap();
        let lateral = wall(&mut topo, cyl, 1.5 * scale);
        let raw = raw(Ellipse3D::with_axes(
            Point3::new(4.0 * scale, (-2.0 - 0.5 / 3.0_f64.sqrt()) * scale, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            11.0 / 3.0_f64.sqrt() * scale,
            5.5 * scale,
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
        )
        .unwrap());
        let arcs = clip(&topo, cap, lateral, &raw, tol).unwrap().unwrap();
        let got = intervals(&arcs);
        assert_eq!(got.len(), 1, "{got:?}");
        // Independent equations: x=4-5.5sin(t), y=-2-.5/sqrt(3)-11/sqrt(3)cos(t).
        // The first endpoint is x²+y²=4; the second is the tool's v=0 plane.
        let (start, end) = got[0];
        let point = |t: f64| {
            (
                4.0 - 5.5 * t.sin(),
                -2.0 - 0.5 / 3.0_f64.sqrt() - 11.0 / 3.0_f64.sqrt() * t.cos(),
            )
        };
        let (x, y) = point(start);
        assert!((x * x + y * y - 4.0).abs() < 1e-8);
        let (_, y) = point(end);
        assert!((-0.566_987_298_107_780_6 - 0.5 * y).abs() < 1e-8);
        assert!(start < 1.74 && end > 1.74 && end - start < 0.04);
    }
}

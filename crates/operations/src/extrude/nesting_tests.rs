//! Closed-form material/void oracles for disconnected profile islands.
#![allow(clippy::unwrap_used, clippy::expect_used)]
use super::*;
use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::nurbs::NurbsCurve;
use remus_topology::builder::{make_circle_edge, make_polygon_wire};

fn rectangle(topo: &mut Topology, bounds: [f64; 4], scale: f64, cw: bool) -> WireId {
    let [x0, x1, y0, y1] = bounds;
    let mut points = vec![
        Point3::new(x0 * scale, y0 * scale, 5.0 * scale),
        Point3::new(x1 * scale, y0 * scale, 5.0 * scale),
        Point3::new(x1 * scale, y1 * scale, 5.0 * scale),
        Point3::new(x0 * scale, y1 * scale, 5.0 * scale),
    ];
    if cw {
        points.reverse();
    }
    make_polygon_wire(topo, &points, 1e-9 * scale).unwrap()
}
fn profile(topo: &mut Topology, kind: usize, scale: f64, reverse: bool) -> FaceId {
    let outer = rectangle(topo, [-20., 20., -20., 20.], scale, false);
    let inner = if kind == 2 {
        [(4.0, false), (1.0, true)]
            .into_iter()
            .map(|(r, forward)| {
                let edge = make_circle_edge(
                    topo,
                    Point3::new(-3. * scale, 0., 5. * scale),
                    Vec3::new(0., 0., 1.),
                    r * scale,
                    1e-9 * scale,
                )
                .unwrap();
                topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, forward)], true).unwrap())
            })
            .collect()
    } else {
        let mut wires = vec![
            rectangle(topo, [-7., 1., -4., 4.], scale, true),
            rectangle(topo, [-3., -1., 0., 2.], scale, false),
        ];
        if kind == 1 {
            wires.push(rectangle(topo, [-2.75, -2.25, 0.5, 1.], scale, true));
        }
        wires
    };
    let mut face = Face::new(
        outer,
        inner,
        FaceSurface::Plane {
            normal: Vec3::new(0., 0., 1.),
            d: 5. * scale,
        },
    );
    if reverse {
        face.set_reversed(true);
    }
    topo.add_face(face)
}
fn verify(kind: usize, scale: f64, reverse: bool) {
    let mut topo = Topology::new();
    let face = profile(&mut topo, kind, scale, reverse);
    let sign = if reverse { -1. } else { 1. };
    let solid = extrude(&mut topo, face, Vec3::new(0., 0., sign), 2. * scale).unwrap();
    let (area, mx, my) = match kind {
        0 => (1540., 184., 4.),
        1 => (1539.75, 184.625, 3.8125),
        _ => (
            1600. - 15. * std::f64::consts::PI,
            45. * std::f64::consts::PI,
            0.,
        ),
    };
    for deflection in [1e-5 * scale, 2e-6 * scale] {
        let volume = crate::measure::solid_volume(&topo, solid, deflection).unwrap();
        assert!(
            (volume / (2. * area * scale.powi(3)) - 1.).abs() < 1e-6,
            "volume {volume}, expected {}",
            2. * area * scale.powi(3)
        );
    }
    let centroid = crate::measure::solid_center_of_mass(&topo, solid, 1e-5 * scale).unwrap();
    let expected = Point3::new(mx / area * scale, my / area * scale, (5. + sign) * scale);
    assert!(
        (centroid - expected).length() < 1e-6 * scale,
        "centroid {centroid:?}, expected {expected:?}"
    );
    for fid in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
        assert!(
            matches!(
                topo.face(fid).unwrap().surface(),
                FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
            ),
            "an exact line/circle sweep keeps analytic carriers"
        );
    }
    let report = crate::validate::validate_solid(&topo, solid).unwrap();
    assert!(report.issues.is_empty(), "{:?}", report.issues);
    let mesh = crate::tessellate::tessellate_solid(&topo, solid, 1e-3 * scale).unwrap();
    assert_eq!(crate::tessellate::boundary_edge_count(&mesh), 0);
    assert_eq!(crate::tessellate::non_manifold_edge_count(&mesh), 0);
    let probes = if kind == 2 {
        vec![(-3., 0., true), (-5., 0., false)]
    } else {
        vec![(-1.5, 1.5, true), (-5., 0., false)]
    };
    for (x, y, inside) in probes
        .into_iter()
        .chain((kind == 1).then_some((-2.5, 0.75, false)))
    {
        let actual = classify_point(
            &topo,
            solid,
            Point3::new(x * scale, y * scale, (5. + sign) * scale),
            &ClassifyOptions::default(),
        )
        .unwrap();
        assert_eq!(
            actual,
            if inside {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            }
        );
    }
}
macro_rules! cases {
    ($a:ident,$b:ident,$c:ident,$kind:expr,$reverse:expr) => {
        #[test]
        fn $a() {
            verify($kind, 1e-3, $reverse);
        }
        #[test]
        fn $b() {
            verify($kind, 1., $reverse);
        }
        #[test]
        fn $c() {
            verify($kind, 1e3, $reverse);
        }
    };
}
cases!(
    nested_reproducer_small,
    nested_reproducer_unit,
    nested_reproducer_large,
    0,
    false
);
cases!(
    nested_three_levels_small,
    nested_three_levels_unit,
    nested_three_levels_large,
    1,
    false
);
cases!(
    nested_circles_small,
    nested_circles_unit,
    nested_circles_large,
    2,
    false
);
cases!(
    nested_reversed_small,
    nested_reversed_unit,
    nested_reversed_large,
    0,
    true
);
cases!(
    nested_three_reversed_small,
    nested_three_reversed_unit,
    nested_three_reversed_large,
    1,
    true
);
cases!(
    nested_circles_reversed_small,
    nested_circles_reversed_unit,
    nested_circles_reversed_large,
    2,
    true
);

#[test]
fn nested_other_sweeps_refuse_before_allocating() {
    for kind in 0..3 {
        let mut topo = Topology::new();
        let face = profile(&mut topo, kind, 1., false);
        let before = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_solids(),
        );
        let axis = Vec3::new(0., 0., 1.);
        let taper =
            taper::tapered_extrude_impl(&mut topo, face, Point3::new(0., 0., 20.), axis, 2.);
        assert!(matches!(
            taper,
            Err(crate::OperationsError::Unsupported { .. })
        ));
        let twist = twist::twisted_extrude_impl(
            &mut topo,
            face,
            Point3::new(0., 0., 5.),
            axis,
            2.,
            0.1,
            1e-7,
            256,
        );
        assert!(matches!(
            twist,
            Err(crate::OperationsError::Unsupported { .. })
        ));
        for angle in [1., std::f64::consts::TAU] {
            let revolve = crate::revolve::revolve(
                &mut topo,
                face,
                Point3::new(-30., 0., 5.),
                Vec3::new(0., 1., 0.),
                angle,
            );
            assert!(matches!(
                revolve,
                Err(crate::OperationsError::Unsupported { .. })
            ));
        }
        assert_eq!(
            before,
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_solids()
            )
        );
    }
}

#[test]
fn nested_profiles_refuse_contacts_crossings_and_unknown_curves_atomically() {
    for bounds in [[-7., -5., 0., 2.], [-8., -6., 0., 2.], [-30., -25., 0., 2.]] {
        let mut topo = Topology::new();
        let outer = rectangle(&mut topo, [-20., 20., -20., 20.], 1., false);
        let hole = rectangle(&mut topo, [-7., 1., -4., 4.], 1., true);
        let island = rectangle(&mut topo, bounds, 1., false);
        let face = topo.add_face(Face::new(
            outer,
            vec![hole, island],
            FaceSurface::Plane {
                normal: Vec3::new(0., 0., 1.),
                d: 5.,
            },
        ));
        let before = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
        );
        assert!(matches!(
            extrude(&mut topo, face, Vec3::new(0., 0., 1.), 2.),
            Err(crate::OperationsError::Unsupported { .. })
        ));
        assert_eq!(
            before,
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces()
            )
        );
    }
    let mut topo = Topology::new();
    let face = profile(&mut topo, 0, 1., false);
    let edge = topo
        .wire(topo.face(face).unwrap().inner_wires()[1])
        .unwrap()
        .edges()[0]
        .edge();
    let a = topo
        .vertex(topo.edge(edge).unwrap().start())
        .unwrap()
        .point();
    let b = topo.vertex(topo.edge(edge).unwrap().end()).unwrap().point();
    topo.edge_mut(edge)
        .unwrap()
        .set_curve(EdgeCurve::NurbsCurve(
            NurbsCurve::new(
                2,
                vec![0., 0., 0., 1., 1., 1.],
                vec![a, a + (b - a) * 0.5 + Vec3::new(0., 0.2, 0.), b],
                vec![1.; 3],
            )
            .unwrap(),
        ));
    assert!(matches!(
        extrude(&mut topo, face, Vec3::new(0., 0., 1.), 2.),
        Err(crate::OperationsError::Unsupported { .. })
    ));
}

#[test]
fn nested_entity_evolution_accounts_for_all_island_caps() {
    let mut topo = Topology::new();
    let face = profile(&mut topo, 1, 1., false);
    let result = extrude_with_entity_evolution(&mut topo, face, Vec3::new(0., 0., 1.), 2.).unwrap();
    assert!(
        (crate::measure::solid_volume(&topo, result.solid, 1e-5).unwrap() - 3079.5).abs() < 1e-8
    );
}

#[test]
fn nested_input_orientation_uses_parity() {
    for kind in 0..3 {
        let mut topo = Topology::new();
        let face = profile(&mut topo, kind, 1., false);
        assert!(
            remus_check::validate::check_face_inner_wire_orientation(&topo, face)
                .unwrap()
                .is_empty()
        );
    }
}

#[test]
fn nested_guard_preserves_separate_unnested_spline_wires() {
    let mut topo = Topology::new();
    let outer = rectangle(&mut topo, [-20., 20., -20., 20.], 1., false);
    let a = rectangle(&mut topo, [-8., -6., 0., 2.], 1., true);
    let b = rectangle(&mut topo, [6., 8., 0., 2.], 1., true);
    for wire in [a, b] {
        let edges = topo.wire(wire).unwrap().edges().to_vec();
        for oe in edges {
            let edge = topo.edge(oe.edge()).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            let curve = NurbsCurve::new(1, vec![0., 0., 1., 1.], vec![a, b], vec![1.; 2]).unwrap();
            topo.edge_mut(oe.edge())
                .unwrap()
                .set_curve(EdgeCurve::NurbsCurve(curve));
        }
    }
    let face = topo.add_face(Face::new(
        outer,
        vec![a, b],
        FaceSurface::Plane {
            normal: Vec3::new(0., 0., 1.),
            d: 5.,
        },
    ));
    let solid = extrude(&mut topo, face, Vec3::new(0., 0., 1.), 2.).unwrap();
    assert!((crate::measure::solid_volume(&topo, solid, 1e-5).unwrap() - 3184.).abs() < 1e-8);
    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .issues
            .is_empty()
    );
}

#[test]
fn nested_circle_contacts_crossings_and_in_plane_sweeps_refuse() {
    for (x, r) in [(0., 1.), (0.5, 1.)] {
        let mut topo = Topology::new();
        let face = profile(&mut topo, 2, 1., false);
        let island = topo.face(face).unwrap().inner_wires()[1];
        let edge = topo.wire(island).unwrap().edges()[0].edge();
        let seam = topo.edge(edge).unwrap().start();
        let circle =
            remus_math::curves::Circle3D::new(Point3::new(x, 0., 5.), Vec3::new(0., 0., 1.), r)
                .unwrap();
        topo.vertex_mut(seam)
            .unwrap()
            .set_point(circle.evaluate(0.));
        topo.edge_mut(edge)
            .unwrap()
            .set_curve(EdgeCurve::Circle(circle));
        assert!(matches!(
            extrude(&mut topo, face, Vec3::new(0., 0., 1.), 2.),
            Err(crate::OperationsError::Unsupported { .. })
        ));
    }
    let mut topo = Topology::new();
    let face = profile(&mut topo, 0, 1., false);
    assert!(matches!(
        extrude(&mut topo, face, Vec3::new(1., 0., 0.), 2.),
        Err(crate::OperationsError::Unsupported { .. })
    ));
}

#[test]
fn nested_negative_distance_agrees_with_negative_direction() {
    let mut topo = Topology::new();
    let face = profile(&mut topo, 0, 1., false);
    let solid = extrude(&mut topo, face, Vec3::new(0., 0., 1.), -2.).unwrap();
    assert!((crate::measure::solid_volume(&topo, solid, 1e-5).unwrap() - 3080.).abs() < 1e-8);
    let c = crate::measure::solid_center_of_mass(&topo, solid, 1e-5).unwrap();
    assert!((c - Point3::new(184. / 1540., 4. / 1540., 4.)).length() < 1e-8);
    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .issues
            .is_empty()
    );
}

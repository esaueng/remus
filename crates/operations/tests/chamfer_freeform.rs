//! Genuine spline support chamfers through the public operation.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::{
    mat::Mat4,
    nurbs::curve::NurbsCurve,
    vec::{Point3, Vec3},
};
use remus_operations::{blend_ops::chamfer_v2, extrude::extrude, measure::solid_volume};
use remus_topology::{
    Topology,
    edge::{Edge, EdgeCurve},
    explorer::{solid_edges, solid_faces},
    face::FaceSurface,
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};

fn spline_prism(topo: &mut Topology) -> remus_topology::solid::SolidId {
    spline_prism_variant(topo, false, false, 1.0)
}

fn spline_prism_variant(
    topo: &mut Topology,
    second_spline: bool,
    reverse: bool,
    weight: f64,
) -> remus_topology::solid::SolidId {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
    ];
    let roof = quadratic(points[2], Point3::new(5.0, 13.0, 0.0), points[3], weight);
    let side = if second_spline {
        EdgeCurve::NurbsCurve(quadratic(
            points[3],
            Point3::new(-2.0, 5.0, 0.0),
            points[0],
            weight,
        ))
    } else {
        EdgeCurve::Line
    };
    prism(
        topo,
        points,
        vec![
            EdgeCurve::Line,
            EdgeCurve::Line,
            EdgeCurve::NurbsCurve(roof),
            side,
        ],
        reverse,
    )
}

fn quadratic(a: Point3, b: Point3, c: Point3, weight: f64) -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![a, b, c],
        vec![1.0, weight, 1.0],
    )
    .unwrap()
}

fn prism(
    topo: &mut Topology,
    points: Vec<Point3>,
    curves: Vec<EdgeCurve>,
    reverse: bool,
) -> remus_topology::solid::SolidId {
    prism_with_bore(topo, points, curves, reverse, None)
}

fn prism_with_bore(
    topo: &mut Topology,
    points: Vec<Point3>,
    curves: Vec<EdgeCurve>,
    reverse: bool,
    bore: Option<Point3>,
) -> remus_topology::solid::SolidId {
    let vertices = points
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
        .collect::<Vec<_>>();
    let edges = curves
        .into_iter()
        .enumerate()
        .map(|(i, curve)| {
            let domain = match &curve {
                EdgeCurve::NurbsCurve(c) => c.domain(),
                _ => (0.0, 1.0),
            };
            let backwards = reverse && matches!(curve, EdgeCurve::NurbsCurve(_));
            let next = (i + 1) % points.len();
            let (start, end, curve) = if backwards {
                let EdgeCurve::NurbsCurve(c) = curve else {
                    unreachable!()
                };
                (
                    vertices[next],
                    vertices[i],
                    EdgeCurve::NurbsCurve(c.reversed()),
                )
            } else {
                (vertices[i], vertices[next], curve)
            };
            let mut edge = Edge::new(start, end, curve);
            edge.set_trim(Some(domain));
            OrientedEdge::new(topo.add_edge(edge), !backwards)
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let mut face = remus_topology::builder::make_planar_face_from_wire(topo, wire).unwrap();
    if let Some(center) = bore {
        let circle =
            remus_math::curves::Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 0.1).unwrap();
        let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let mut edge = Edge::new(seam, seam, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, std::f64::consts::TAU)));
        let edge = topo.add_edge(edge);
        let hole = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, false)], true).unwrap());
        face = topo.add_face(remus_topology::face::Face::new(
            wire,
            vec![hole],
            topo.face(face).unwrap().surface().clone(),
        ));
    }
    extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 20.0).unwrap()
}

#[test]
fn spline_chamfer_preserves_a_clear_bore_and_refuses_an_intersecting_one() {
    use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
    for (center, clear) in [
        (Point3::new(5.0, 5.0, 0.0), true),
        (Point3::new(0.3, 9.7, 0.0), false),
    ] {
        let mut topo = Topology::new();
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
        ];
        let roof = quadratic(points[2], Point3::new(5.0, 13.0, 0.0), points[3], 1.0);
        let solid = prism_with_bore(
            &mut topo,
            points,
            vec![
                EdgeCurve::Line,
                EdgeCurve::Line,
                EdgeCurve::NurbsCurve(roof),
                EdgeCurve::Line,
            ],
            false,
            Some(center),
        );
        assert!(
            remus_operations::validate::validate_solid(&topo, solid)
                .unwrap()
                .is_valid()
        );
        let edge = target(&topo, solid);
        let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
        let slots = topo.allocated_slot_count();
        let result = chamfer_v2(&mut topo, solid, &[edge], 1.0, 1.0);
        if clear {
            let result = result.unwrap();
            check(&topo, result.solid);
            assert_eq!(
                classify_point(
                    &topo,
                    result.solid,
                    center + Vec3::new(0.0, 0.0, 10.0),
                    &ClassifyOptions::default()
                )
                .unwrap(),
                PointClassification::Outside
            );
            capture(&topo, solid, result.solid, "spline-clear-bore");
        } else {
            assert!(result.is_err(), "a chamfer crossing the bore must refuse");
            assert_eq!(topo.allocated_slot_count(), slots);
        }
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
            before
        );
    }
}

fn target(topo: &Topology, solid: remus_topology::solid::SolidId) -> remus_topology::edge::EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let e = topo.edge(e).unwrap();
            [e.start(), e.end()].into_iter().all(|v| {
                let p = topo.vertex(v).unwrap().point();
                p.x().abs() < 1e-8 && (p.y() - 10.0).abs() < 1e-8
            })
        })
        .unwrap()
}

fn check(topo: &Topology, solid: remus_topology::solid::SolidId) {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{report:?}");
    let adj = topo.build_adjacency(solid).unwrap();
    assert!(adj.is_manifold() && adj.boundary_edges().is_empty());
    let pc =
        remus_topology::validation::validate_solid_pcurve_contracts(topo, solid, 1e-7, 0).unwrap();
    assert!(
        pc.validated_uses >= 18,
        "non-vacuous native pcurve coverage: {pc:?}"
    );
}

fn capture(
    topo: &Topology,
    input: remus_topology::solid::SolidId,
    result: remus_topology::solid::SolidId,
    label: &str,
) {
    if let Ok(directory) = std::env::var("REMUS_CHAMFER_EVIDENCE_DIR") {
        let directory = std::path::Path::new(&directory);
        std::fs::create_dir_all(directory).unwrap();
        for (suffix, solid) in [("input", input), ("result", result)] {
            let step = remus_io::step::writer::write_step(topo, &[solid]).unwrap();
            std::fs::write(directory.join(format!("{label}-{suffix}.step")), step).unwrap();
        }
    }
}

#[test]
fn genuine_spline_support_gets_a_watertight_chamfer() {
    let _ = env_logger::try_init();
    let mut topo = Topology::new();
    let solid = spline_prism(&mut topo);
    let surfaces = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
        .count();
    assert_eq!(
        surfaces, 1,
        "fixture must have a genuinely curved spline wall"
    );
    let edge = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let e = topo.edge(e).unwrap();
            [e.start(), e.end()].into_iter().all(|v| {
                let p = topo.vertex(v).unwrap().point();
                p.x().abs() < 1e-8 && (p.y() - 10.0).abs() < 1e-8
            })
        })
        .unwrap();
    let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
    let result = chamfer_v2(&mut topo, solid, &[edge], 1.0, 1.0).expect("freeform chamfer");
    assert_ne!(result.solid, solid);
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
        before
    );
    let report = remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
    assert!(report.is_valid(), "{report:?}");
    let adj = topo.build_adjacency(result.solid).unwrap();
    assert!(adj.is_manifold() && adj.boundary_edges().is_empty());
    assert!(
        solid_volume(&topo, result.solid, 0.001).unwrap()
            < solid_volume(&topo, solid, 0.001).unwrap() - 1.0
    );
    let volume = solid_volume(&topo, result.solid, 0.001).unwrap();
    assert!(
        (volume - 2_191.093_119_880_501).abs() < 0.01,
        "independent reference volume: {volume}"
    );
    check(&topo, result.solid);
    capture(&topo, solid, result.solid, "quadratic");
}

#[test]
fn cubic_multispan_profile_keeps_its_exact_carrier() {
    let mut topo = Topology::new();
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
    ];
    let curve = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.4, 0.7, 1.0, 1.0, 1.0, 1.0],
        vec![
            points[2],
            Point3::new(9.0, 13.0, 0.0),
            Point3::new(7.0, 8.0, 0.0),
            Point3::new(5.0, 14.0, 0.0),
            Point3::new(2.0, 8.0, 0.0),
            points[3],
        ],
        vec![1.0, 0.8, 1.3, 0.7, 1.5, 1.0],
    )
    .unwrap();
    let solid = prism(
        &mut topo,
        points,
        vec![
            EdgeCurve::Line,
            EdgeCurve::Line,
            EdgeCurve::NurbsCurve(curve),
            EdgeCurve::Line,
        ],
        false,
    );
    let edge = target(&topo, solid);
    let result = chamfer_v2(&mut topo, solid, &[edge], 0.5, 0.5).unwrap();
    check(&topo, result.solid);
    let surfaces = solid_faces(&topo, result.solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
        .collect::<Vec<_>>();
    assert_eq!(surfaces.len(), 1);
    let source = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
        .unwrap();
    let (FaceSurface::Nurbs(result_support), FaceSurface::Nurbs(original_support)) = (
        topo.face(surfaces[0]).unwrap().surface(),
        topo.face(source).unwrap().surface(),
    ) else {
        unreachable!()
    };
    assert_eq!(result_support, original_support);
    capture(&topo, solid, result.solid, "cubic-multispan");
}

#[test]
fn concave_spline_pair_adds_only_the_requested_corner() {
    use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
    let mut topo = Topology::new();
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 4.0, 0.0),
        Point3::new(4.0, 4.0, 0.0),
        Point3::new(4.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
    ];
    let incoming = quadratic(points[2], Point3::new(7.0, 3.0, 0.0), points[3], 1.0);
    let outgoing = quadratic(points[3], Point3::new(3.0, 7.0, 0.0), points[4], 1.0);
    let solid = prism(
        &mut topo,
        points,
        vec![
            EdgeCurve::Line,
            EdgeCurve::Line,
            EdgeCurve::NurbsCurve(incoming),
            EdgeCurve::NurbsCurve(outgoing),
            EdgeCurve::Line,
            EdgeCurve::Line,
        ],
        false,
    );
    let edge = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let e = topo.edge(e).unwrap();
            [e.start(), e.end()].into_iter().all(|v| {
                let p = topo.vertex(v).unwrap().point();
                (p.x() - 4.0).abs() < 1e-8 && (p.y() - 4.0).abs() < 1e-8
            })
        })
        .unwrap();
    let result = chamfer_v2(&mut topo, solid, &[edge], 1.0, 1.0).unwrap();
    check(&topo, result.solid);
    assert!(
        solid_volume(&topo, result.solid, 0.001).unwrap()
            > solid_volume(&topo, solid, 0.001).unwrap() + 1.0
    );
    let p = Point3::new(4.1, 4.1, 10.0);
    assert_eq!(
        classify_point(&topo, solid, p, &ClassifyOptions::default()).unwrap(),
        PointClassification::Outside
    );
    assert_eq!(
        classify_point(&topo, result.solid, p, &ClassifyOptions::default()).unwrap(),
        PointClassification::Inside
    );
    capture(&topo, solid, result.solid, "concave");
}

#[test]
fn all_longitudinal_edges_of_a_spline_prism_chamfer_together() {
    let mut topo = Topology::new();
    let solid = spline_prism_variant(&mut topo, true, false, 0.7);
    let edges = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let e = topo.edge(e).unwrap();
            let a = topo.vertex(e.start()).unwrap().point();
            let b = topo.vertex(e.end()).unwrap().point();
            (a.x() - b.x()).abs() < 1e-8 && (a.y() - b.y()).abs() < 1e-8
        })
        .collect::<Vec<_>>();
    assert_eq!(edges.len(), 4);
    let result = chamfer_v2(&mut topo, solid, &edges, 0.5, 0.5).unwrap();
    check(&topo, result.solid);
    assert_eq!(result.succeeded.len(), 4);
    assert_eq!(solid_faces(&topo, result.solid).unwrap().len(), 10);
    capture(&topo, solid, result.solid, "all-longitudinal");
}

#[test]
fn rational_spline_pairs_preserve_supports_under_placement_and_asymmetric_setbacks() {
    for second in [false, true] {
        for reverse in [false, true] {
            for weight in [0.7, 1.0, 1.6] {
                for scale in [0.01, 1.0, 100.0] {
                    for (d1, d2) in [(0.1, 0.1), (1.0, 0.6)] {
                        let mut topo = Topology::new();
                        let solid = spline_prism_variant(&mut topo, second, reverse, weight);
                        let edge = target(&topo, solid);
                        let matrix = Mat4::translation(17.0, -23.0, 31.0)
                            * Mat4::rotation_z(0.37)
                            * Mat4::rotation_x(0.23)
                            * Mat4::scale(scale, scale, scale);
                        remus_operations::transform::transform_solid(&mut topo, solid, &matrix)
                            .unwrap();
                        let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
                        let result=chamfer_v2(&mut topo,solid,&[edge],d1*scale,d2*scale)
                            .unwrap_or_else(|e|panic!("pair={second}, reverse={reverse}, weight={weight}, scale={scale}, setbacks={d1}/{d2}: {e}"));
                        check(&topo, result.solid);
                        assert_eq!(
                            remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
                            before
                        );
                        assert_eq!(solid_faces(&topo, result.solid).unwrap().len(), 7);
                        let origins = result.face_origins.unwrap();
                        assert_eq!(origins.survived.len(), 6);
                        assert_eq!(origins.created.len(), 1);
                        assert!(origins.created_unattributed.is_empty());
                    }
                }
            }
        }
    }
}

#[test]
fn exhausted_spline_support_refuses_transactionally() {
    for distance in [10.0, 20.0, 100.0] {
        let mut topo = Topology::new();
        let solid = spline_prism(&mut topo);
        let edge = target(&topo, solid);
        let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
        let counts = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_faces(),
            topo.num_solids(),
        );
        assert!(chamfer_v2(&mut topo, solid, &[edge], distance, distance).is_err());
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
            before
        );
        assert_eq!(
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_faces(),
                topo.num_solids()
            ),
            counts
        );
    }
}

#[test]
fn reversed_spline_carrier_keeps_its_native_parameter_direction() {
    let mut topo = Topology::new();
    let solid = spline_prism_variant(&mut topo, false, true, 0.7);
    let edge = target(&topo, solid);
    let matrix = Mat4::translation(17.0, -23.0, 31.0)
        * Mat4::rotation_z(0.37)
        * Mat4::rotation_x(0.23)
        * Mat4::scale(0.01, 0.01, 0.01);
    remus_operations::transform::transform_solid(&mut topo, solid, &matrix).unwrap();
    let result = chamfer_v2(&mut topo, solid, &[edge], 0.001, 0.001).unwrap();
    check(&topo, result.solid);
}

#[test]
fn curved_spline_rim_gets_a_complete_ruled_chamfer() {
    use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
    let _ = env_logger::try_init();
    let mut topo = Topology::new();
    let solid = curved_rim_prism(&mut topo, false, 1.0);
    let edge = curved_rim(&topo, solid, true);
    let result =
        chamfer_v2(&mut topo, solid, &[edge], 0.5, 0.5).expect("curved freeform rim chamfer");
    check(&topo, result.solid);
    assert_eq!(solid_faces(&topo, result.solid).unwrap().len(), 7);
    let EdgeCurve::NurbsCurve(guide) = topo.edge(edge).unwrap().curve() else {
        unreachable!()
    };
    let p = guide.evaluate(0.5);
    let inward = Vec3::new(0.0, 0.0, 1.0).cross(guide.tangent(0.5).unwrap());
    for (fraction, expected) in [
        (0.2, PointClassification::Outside),
        (0.8, PointClassification::Inside),
    ] {
        let q = p + inward * (0.5 * fraction) - Vec3::new(0.0, 0.0, 0.5 * fraction);
        assert_eq!(
            classify_point(&topo, solid, q, &ClassifyOptions::default()).unwrap(),
            PointClassification::Inside
        );
        assert_eq!(
            classify_point(&topo, result.solid, q, &ClassifyOptions::default()).unwrap(),
            expected
        );
    }
    assert!(
        solid_volume(&topo, result.solid, 0.001).unwrap()
            < solid_volume(&topo, solid, 0.001).unwrap() - 0.1
    );
    capture(&topo, solid, result.solid, "curved-rim");
}

fn curved_rim_prism(
    topo: &mut Topology,
    reverse: bool,
    weight: f64,
) -> remus_topology::solid::SolidId {
    curved_rim_prism_with_bore(topo, reverse, weight, None)
}

fn curved_rim_prism_with_bore(
    topo: &mut Topology,
    reverse: bool,
    weight: f64,
    bore: Option<Point3>,
) -> remus_topology::solid::SolidId {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
    ];
    let roof = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.4, 0.7, 1.0, 1.0, 1.0, 1.0],
        vec![
            points[2],
            Point3::new(8.0, 10.0, 0.0),
            Point3::new(7.0, 14.0, 0.0),
            Point3::new(3.0, 14.0, 0.0),
            Point3::new(2.0, 10.0, 0.0),
            points[3],
        ],
        vec![1.0, weight, weight, weight, weight, 1.0],
    )
    .unwrap();
    prism_with_bore(
        topo,
        points,
        vec![
            EdgeCurve::Line,
            EdgeCurve::Line,
            EdgeCurve::NurbsCurve(roof),
            EdgeCurve::Line,
        ],
        reverse,
        bore,
    )
}

fn curved_rim(
    topo: &Topology,
    solid: remus_topology::solid::SolidId,
    top: bool,
) -> remus_topology::edge::EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let e = topo.edge(e).unwrap();
            matches!(e.curve(), EdgeCurve::NurbsCurve(_))
                && [e.start(), e.end()].into_iter().all(|v| {
                    (topo.vertex(v).unwrap().point().z() - if top { 20.0 } else { 0.0 }).abs()
                        < 1e-8
                })
        })
        .unwrap()
}

#[test]
fn curved_rims_preserve_rational_placement_and_setback_contracts() {
    let _ = env_logger::try_init();
    for (top, reverse, weight, scale, d1, d2) in [
        (true, false, 1.0, 0.01, 0.5, 0.3),
        (false, true, 0.7, 1.0, 0.3, 0.5),
        (true, true, 1.6, 10.0, 0.5, 0.3),
        (false, false, 1.0, 100.0, 0.3, 0.5),
    ] {
        let mut topo = Topology::new();
        let solid = curved_rim_prism(&mut topo, reverse, weight);
        let edge = curved_rim(&topo, solid, top);
        let matrix = Mat4::translation(17.0, -23.0, 31.0)
            * Mat4::rotation_z(0.37)
            * Mat4::rotation_x(0.23)
            * Mat4::scale(scale, scale, scale);
        remus_operations::transform::transform_solid(&mut topo, solid, &matrix).unwrap();
        let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
        let result =
            chamfer_v2(&mut topo, solid, &[edge], d1 * scale, d2 * scale).unwrap_or_else(|e| {
                panic!("top={top}, reverse={reverse}, weight={weight}, scale={scale}: {e}")
            });
        check(&topo, result.solid);
        assert_eq!(solid_faces(&topo, result.solid).unwrap().len(), 7);
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
            before
        );
        let origins = result.face_origins.unwrap();
        assert_eq!(origins.survived.len(), 6);
        assert_eq!(origins.created.len(), 1);
    }
}

#[test]
fn both_freeform_rims_can_be_chamfered_together() {
    let mut topo = Topology::new();
    let solid = curved_rim_prism(&mut topo, false, 1.0);
    let edges = [
        curved_rim(&topo, solid, false),
        curved_rim(&topo, solid, true),
    ];
    let result = chamfer_v2(&mut topo, solid, &edges, 0.3, 0.3).unwrap();
    check(&topo, result.solid);
    assert_eq!(solid_faces(&topo, result.solid).unwrap().len(), 8);
}

#[test]
fn curved_rim_keeps_a_clear_bore_and_refuses_an_intersecting_mouth() {
    for clear in [true, false] {
        let mut topo = Topology::new();
        let plain = curved_rim_prism(&mut topo, false, 1.0);
        let guide = topo.edge(curved_rim(&topo, plain, true)).unwrap().curve();
        let EdgeCurve::NurbsCurve(guide) = guide else {
            unreachable!()
        };
        let near = guide.evaluate(0.5)
            + Vec3::new(0.0, 0.0, 1.0).cross(guide.tangent(0.5).unwrap()) * 0.25;
        let center = if clear {
            Point3::new(5.0, 5.0, 0.0)
        } else {
            Point3::new(near.x(), near.y(), 0.0)
        };
        let solid = curved_rim_prism_with_bore(&mut topo, false, 1.0, Some(center));
        assert!(
            remus_operations::validate::validate_solid(&topo, solid)
                .unwrap()
                .is_valid()
        );
        let edge = curved_rim(&topo, solid, true);
        let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
        let slots = topo.allocated_slot_count();
        let result = chamfer_v2(&mut topo, solid, &[edge], 0.5, 0.5);
        if clear {
            let result = result.unwrap();
            check(&topo, result.solid);
            capture(&topo, solid, result.solid, "curved-rim-clear-bore");
        } else {
            assert!(result.is_err());
            assert_eq!(topo.allocated_slot_count(), slots);
        }
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
            before
        );
    }
}

#[test]
fn exhausted_curved_rim_refuses_without_mutation() {
    let mut topo = Topology::new();
    let solid = curved_rim_prism(&mut topo, false, 1.0);
    let edge = curved_rim(&topo, solid, true);
    let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
    let slots = topo.allocated_slot_count();
    assert!(chamfer_v2(&mut topo, solid, &[edge], 20.0, 20.0).is_err());
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
        before
    );
    assert_eq!(topo.allocated_slot_count(), slots);
}

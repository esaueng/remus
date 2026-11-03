//! Analytic chamfers after earlier fillets and boolean cuts.
//! Tier: core. Exact carriers and the ordinary solid/material gates remain
//! authoritative; these cases may not be satisfied by polygonizing a round.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_operations::{blend_ops::chamfer_v2, measure::solid_volume};
use remus_topology::{
    Topology,
    edge::{EdgeCurve, EdgeId},
    explorer::{solid_edges, solid_faces},
    face::FaceSurface,
    solid::SolidId,
};

fn check(topo: &Topology, solid: SolidId) {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{report:?}");
    assert!(solid_volume(topo, solid, 0.001).unwrap() > 0.0);
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    remus_topology::validation::validate_shell_closed(shell, topo).unwrap();
    remus_topology::validation::validate_shell_manifold(shell, topo).unwrap();
}

fn filleted_block() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 30.0, 24.0, 20.0).unwrap();
    let edge = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|e| {
            let e = topo.edge(*e).unwrap();
            let a = topo.vertex(e.start()).unwrap().point();
            let b = topo.vertex(e.end()).unwrap().point();
            (a.x() - 30.0).abs() < 1e-9
                && a.y().abs() < 1e-9
                && (b.x() - 30.0).abs() < 1e-9
                && b.y().abs() < 1e-9
        })
        .unwrap();
    let rounded = remus_operations::blend_ops::fillet_cascade(&mut topo, solid, &[edge], 3.0)
        .unwrap()
        .solid;
    (topo, rounded)
}

fn sharp(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    remus_operations::query::filter_filletable_edges(
        topo,
        solid,
        &solid_edges(topo, solid).unwrap(),
    )
    .unwrap()
}

#[test]
fn complete_boss_chamfer_retrims_its_inner_footprint_and_mixed_corners() {
    let mut topo = Topology::new();
    let plate = remus_operations::primitives::make_box(&mut topo, 30.0, 30.0, 6.0).unwrap();
    let post = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 18.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        post,
        &Mat4::translation(10.0, 10.0, 6.0),
    )
    .unwrap();
    let solid = remus_operations::boolean::boolean(
        &mut topo,
        remus_operations::boolean::BooleanOp::Fuse,
        plate,
        post,
    )
    .unwrap();
    let edges = sharp(&topo, solid);
    assert_eq!(edges.len(), 24);
    let adjacency = topo.build_adjacency(solid).unwrap();
    for distance in [0.1, 1.0] {
        for &edge in &edges {
            let pair = adjacency.faces_for_edge(edge);
            let frame = remus_blend::query::edge_material_frames_at(&topo, edge, pair[0], pair[1])
                .unwrap()
                .unwrap();
            let data = topo.edge(edge).unwrap();
            let length = (topo.vertex(data.end()).unwrap().point()
                - topo.vertex(data.start()).unwrap().point())
            .length();
            let expected = 7200.0
                + if frame.concave().unwrap() { 0.5 } else { -0.5 } * distance * distance * length;
            let mut trial = topo.clone();
            let result = chamfer_v2(&mut trial, solid, &[edge], distance, distance).unwrap();
            check(&trial, result.solid);
            assert!((solid_volume(&trial, result.solid, 0.001).unwrap() - expected).abs() < 1e-6);
        }
    }
    for (distance, expected) in [(0.1, 7_198.326_666_666_665), (1.0, 7_038.666_666_666_667)] {
        for placed in [false, true] {
            let mut trial = topo.clone();
            if placed {
                remus_operations::transform::transform_solid(
                    &mut trial,
                    solid,
                    &(Mat4::translation(17.0, -23.0, 31.0)
                        * Mat4::rotation_z(0.37)
                        * Mat4::rotation_x(0.23)),
                )
                .unwrap();
            }
            let result = chamfer_v2(&mut trial, solid, &edges, distance, distance).unwrap();
            check(&trial, result.solid);
            assert_eq!(solid_faces(&trial, result.solid).unwrap().len(), 51);
            let actual = solid_volume(&trial, result.solid, 0.001).unwrap();
            assert!(
                (actual - expected).abs() < 1e-6,
                "C{distance}: expected {expected}, got {actual}"
            );
        }
    }
    let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
    // An uncertified mixed-corner cap must not be replaced with independent
    // finite wedges that happen to pass structural validity.
    assert!(chamfer_v2(&mut topo, solid, &edges, 0.1, 0.2).is_err());
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
        before
    );
}

#[test]
fn every_sharp_edge_of_a_filleted_block_chamfers_without_flattening_the_round() {
    let (topo, solid) = filleted_block();
    let edges = sharp(&topo, solid);
    assert_eq!(edges.len(), 13);
    for distance in [0.1, 1.0] {
        for index in 0..=edges.len() {
            let selection = if index == edges.len() {
                edges.clone()
            } else {
                vec![edges[index]]
            };
            let mut trial = topo.clone();
            let r = chamfer_v2(&mut trial, solid, &selection, distance, distance)
                .unwrap_or_else(|e| panic!("edge {index}, distance {distance}: {e}"));
            check(&trial, r.solid);
            assert!(solid_faces(&trial,r.solid).unwrap().iter().any(|f|matches!(trial.face(*f).unwrap().surface(),FaceSurface::Cylinder(c) if (c.radius()-3.0).abs()<1e-9)));
            assert!(
                solid_volume(&trial, r.solid, 0.001).unwrap()
                    < solid_volume(&topo, solid, 0.001).unwrap()
            );
        }
    }
}

#[test]
fn every_notch_edge_and_the_complete_selection_build() {
    let _ = env_logger::builder().is_test(true).try_init();
    let mut topo = Topology::new();
    let base = remus_operations::primitives::make_box(&mut topo, 30.0, 30.0, 20.0).unwrap();
    let tool = remus_operations::primitives::make_box(&mut topo, 15.0, 15.0, 20.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        tool,
        &Mat4::translation(20.0, 20.0, 0.0),
    )
    .unwrap();
    let solid = remus_operations::boolean::boolean(
        &mut topo,
        remus_operations::boolean::BooleanOp::Cut,
        base,
        tool,
    )
    .unwrap();
    let edges = sharp(&topo, solid);
    assert_eq!(edges.len(), 18);
    let mut failures = Vec::new();
    for distance in [0.1, 1.0] {
        for index in 0..=edges.len() {
            let selection = if index == edges.len() {
                edges.clone()
            } else {
                vec![edges[index]]
            };
            let mut trial = topo.clone();
            match chamfer_v2(&mut trial, solid, &selection, distance, distance) {
                Ok(r) => {
                    check(&trial, r.solid);
                    if index == edges.len() {
                        assert_eq!(solid_faces(&trial, r.solid).unwrap().len(), 38);
                        let expected = if distance.to_bits() == 1.0_f64.to_bits() {
                            15_847.333_333_333_332
                        } else {
                            15_998.407_333_333_34
                        };
                        let actual = solid_volume(&trial, r.solid, 0.001).unwrap();
                        assert!(
                            (actual - expected).abs() < 1e-6,
                            "C{distance}: expected {expected}, got {actual}"
                        );
                    }
                }
                Err(error) => {
                    let positions: Vec<_> = selection
                        .iter()
                        .map(|e| {
                            let e = topo.edge(*e).unwrap();
                            (
                                topo.vertex(e.start()).unwrap().point(),
                                topo.vertex(e.end()).unwrap().point(),
                            )
                        })
                        .collect();
                    failures.push(format!(
                        "notch edge {index}, distance {distance}, {positions:?}: {error}"
                    ));
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn both_frustum_rims_chamfer() {
    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_cone(&mut topo, 10.0, 5.0, 20.0).unwrap();
    let edges = sharp(&topo, solid);
    assert_eq!(edges.len(), 2);
    for &edge in &edges {
        assert!(matches!(
            topo.edge(edge).unwrap().curve(),
            EdgeCurve::Circle(_)
        ));
        let mut trial = topo.clone();
        let r = chamfer_v2(&mut trial, solid, &[edge], 1.0, 1.0).unwrap();
        check(&trial, r.solid);
    }
    let result = chamfer_v2(&mut topo, solid, &edges, 1.0, 1.0).unwrap();
    check(&topo, result.solid);
}

#[test]
fn holed_plate_all_edges_and_each_rim_build() {
    let mut topo = Topology::new();
    let plate = remus_operations::primitives::make_box(&mut topo, 30.0, 30.0, 20.0).unwrap();
    let tool = remus_operations::primitives::make_cylinder(&mut topo, 5.0, 20.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        tool,
        &Mat4::translation(15.0, 15.0, 0.0),
    )
    .unwrap();
    let solid = remus_operations::boolean::boolean(
        &mut topo,
        remus_operations::boolean::BooleanOp::Cut,
        plate,
        tool,
    )
    .unwrap();
    let edges = sharp(&topo, solid);
    assert_eq!(edges.len(), 14);
    for distance in [0.1, 1.0] {
        for index in 0..=edges.len() {
            let selection = if index == edges.len() {
                edges.clone()
            } else {
                vec![edges[index]]
            };
            let mut trial = topo.clone();
            let result = chamfer_v2(&mut trial, solid, &selection, distance, distance)
                .unwrap_or_else(|error| panic!("plate edge {index}, C{distance}: {error}"));
            check(&trial, result.solid);
        }
    }
}

fn twice_rounded_block() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let mut solid = remus_operations::primitives::make_box(&mut topo, 32.0, 28.0, 24.0).unwrap();
    for (radius, pass) in [(8.0, 0), (3.0, 1)] {
        let selected: Vec<_> = solid_edges(&topo, solid)
            .unwrap()
            .into_iter()
            .filter(|&edge| {
                let edge = topo.edge(edge).unwrap();
                let points = [
                    topo.vertex(edge.start()).unwrap().point(),
                    topo.vertex(edge.end()).unwrap().point(),
                ];
                let at = |a: f64, b: f64| (a - b).abs() < 1e-8;
                points.iter().all(|p| {
                    if pass == 0 {
                        at(p.x(), 32.0) && at(p.y(), 0.0)
                    } else {
                        (at(p.y(), 28.0) && at(p.z(), 24.0))
                            || (at(p.x(), 32.0) && at(p.y(), 28.0))
                            || (at(p.x(), 32.0) && at(p.z(), 24.0))
                    }
                })
            })
            .collect();
        assert_eq!(selected.len(), if pass == 0 { 1 } else { 3 });
        solid = remus_operations::blend_ops::fillet_cascade(&mut topo, solid, &selected, radius)
            .unwrap()
            .solid;
    }
    (topo, solid)
}

#[test]
fn every_sharp_edge_of_a_twice_rounded_block_and_all_modes_build() {
    let _ = env_logger::builder().is_test(true).try_init();
    let (topo, solid) = twice_rounded_block();
    let edges = sharp(&topo, solid);
    assert_eq!(solid_faces(&topo, solid).unwrap().len(), 13);
    assert_eq!(edges.len(), 11);
    let mut failures = Vec::new();
    for (d1, d2) in [(0.1, 0.1), (1.0, 1.0), (1.0, 0.5)] {
        for index in 0..=edges.len() {
            let selection = if index == edges.len() {
                edges.clone()
            } else {
                vec![edges[index]]
            };
            let mut trial = topo.clone();
            match chamfer_v2(&mut trial, solid, &selection, d1, d2) {
                Ok(result) => {
                    check(&trial, result.solid);
                    if index == edges.len() && d1.to_bits() == d2.to_bits() {
                        let expected = if d1.to_bits() == 1.0_f64.to_bits() {
                            20_871.540_710_428_697
                        } else {
                            20_962.570_811_165_03
                        };
                        // The general trimmed-face quadrature has a small
                        // numerical residual. Carrier setbacks below are exact;
                        // the STEP volumes were independently checked in OCCT.
                        let volume = solid_volume(&trial, result.solid, 0.001).unwrap();
                        assert!(
                            (volume - expected).abs() < 0.01,
                            "C{d1}: expected {expected}, got {volume}"
                        );
                        assert_eq!(solid_faces(&trial, result.solid).unwrap().len(), 26);
                        let cones: Vec<_> = solid_faces(&trial, result.solid)
                            .unwrap()
                            .into_iter()
                            .filter_map(|face| match trial.face(face).unwrap().surface() {
                                FaceSurface::Cone(cone) => Some(cone),
                                _ => None,
                            })
                            .collect();
                        assert_eq!(cones.len(), 4);
                        let large = cones
                            .iter()
                            .find(|cone| {
                                (cone.apex().x() - 24.0).abs() < 1e-8
                                    && (cone.apex().y() - 8.0).abs() < 1e-8
                            })
                            .unwrap();
                        let axial = (remus_math::vec::Point3::new(24.0, 8.0, 0.0) - large.apex())
                            .dot(large.axis());
                        assert!((axial * large.half_angle().tan() - (8.0 - d1)).abs() < 1e-8);
                    }
                }
                Err(error) => failures.push(format!("edge {index}, {d1}/{d2}: {error}")),
            }
        }
    }
    let selected = edges
        .iter()
        .copied()
        .find(|&edge| {
            let e = topo.edge(edge).unwrap();
            [e.start(), e.end()].iter().all(|&v| {
                let p = topo.vertex(v).unwrap().point();
                p.x().abs() < 1e-8 && (p.y() - 28.0).abs() < 1e-8
            })
        })
        .unwrap();
    let mut trial = topo;
    let angled = remus_operations::blend_ops::chamfer_distance_angle(
        &mut trial,
        solid,
        &[selected],
        1.0,
        std::f64::consts::FRAC_PI_6,
    )
    .unwrap();
    check(&trial, angled.solid);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn steep_chamfers_refuse_setbacks_beyond_the_original_support() {
    for placed in [false, true] {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 30.0, 18.0, 24.0).unwrap();
        if placed {
            remus_operations::transform::transform_solid(
                &mut topo,
                solid,
                &(Mat4::translation(17.0, -23.0, 31.0)
                    * Mat4::rotation_z(0.37)
                    * Mat4::rotation_x(0.23)),
            )
            .unwrap();
        }
        let edge = solid_edges(&topo, solid).unwrap()[0];
        let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
        let counts = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_faces(),
            topo.num_solids(),
        );
        let angle = 80.0_f64.to_radians();
        for distance in [10.0, 20.0] {
            assert!(
                remus_operations::blend_ops::chamfer_distance_angle(
                    &mut topo,
                    solid,
                    &[edge],
                    distance,
                    angle
                )
                .is_err()
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
            assert_eq!(
                remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
                before
            );
        }
        let accepted = remus_operations::blend_ops::chamfer_distance_angle(
            &mut topo,
            solid,
            &[edge],
            1.0,
            angle,
        )
        .unwrap();
        check(&topo, accepted.solid);
        let expected = 30.0 * 18.0 * 24.0 - 0.5 * angle.tan() * 30.0;
        assert!((solid_volume(&topo, accepted.solid, 0.001).unwrap() - expected).abs() < 1e-6);
    }
}

#[test]
fn rejected_chamfers_restore_source_geometry_and_all_live_entities() {
    let (mut topo, solid) = filleted_block();
    let sharp = sharp(&topo, solid);
    let smooth: Vec<_> = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|edge| !sharp.contains(edge))
        .collect();
    assert!(!smooth.is_empty());
    let other = remus_operations::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let foreign = solid_edges(&topo, other).unwrap()[0];
    let live = |t: &Topology| {
        (
            t.num_vertices(),
            t.num_edges(),
            t.num_wires(),
            t.num_faces(),
            t.num_shells(),
            t.num_solids(),
            t.num_loops(),
            t.num_coedges(),
        )
    };
    let before = remus_io::arena_io::serialize_solids(&topo, &[solid, other]).unwrap();
    let counts = live(&topo);
    for (selection, d1, d2) in [
        (smooth, 1.0, 1.0),
        (vec![sharp[0], foreign], 1.0, 1.0),
        (vec![sharp[0]], 1000.0, 1000.0),
        (sharp.clone(), 1000.0, 1000.0),
    ] {
        assert!(chamfer_v2(&mut topo, solid, &selection, d1, d2).is_err());
        assert_eq!(live(&topo), counts);
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[solid, other]).unwrap(),
            before
        );
    }
}

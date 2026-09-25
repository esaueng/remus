//! Public-path trim witnesses; carrier endpoints and parameter units are not edge extent.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::validate::{
    CheckId, EntityRef, Severity, ValidateOptions, ValidationReport, validate_solid,
    validate_wire_body,
};
use remus_math::curves2d::{Curve2D, Line2D, NurbsCurve2D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::test_utils::make_unit_cube_manifold;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn linear_curve(points: Vec<Point3>, breaks: &[f64]) -> NurbsCurve {
    let mut knots = vec![breaks[0]];
    knots.extend_from_slice(breaks);
    knots.push(*breaks.last().unwrap());
    let weights = vec![1.0; points.len()];
    NurbsCurve::new(1, knots, points, weights).unwrap()
}

fn cube_edge(scale: f64, placed: bool, span: f64) -> (Topology, SolidId, EdgeId) {
    let mut topo = Topology::new();
    let solid = make_unit_cube_manifold(&mut topo);
    let vertices: Vec<_> = topo
        .vertices()
        .iter()
        .map(|(id, v)| (id, v.point()))
        .collect();
    for (id, p) in vertices {
        let p = if placed {
            Point3::new(3.0 - p.y(), -2.0 + p.x(), 5.0 + p.z())
        } else {
            p
        };
        topo.vertex_mut(id).unwrap().set_point(Point3::new(
            p.x() * scale,
            p.y() * scale,
            p.z() * scale,
        ));
    }
    let faces: Vec<_> = topo
        .faces()
        .iter()
        .map(|(id, f)| (id, f.surface().clone()))
        .collect();
    for (id, surface) in faces {
        let remus_topology::face::FaceSurface::Plane { normal, d } = surface else {
            panic!("cube plane");
        };
        let normal = if placed {
            remus_math::vec::Vec3::new(-normal.y(), normal.x(), normal.z())
        } else {
            normal
        };
        let offset = if placed {
            normal.dot(remus_math::vec::Vec3::new(3.0, -2.0, 5.0))
        } else {
            0.0
        };
        topo.face_mut(id)
            .unwrap()
            .set_surface(remus_topology::face::FaceSurface::Plane {
                normal,
                d: (d + offset) * scale,
            });
    }
    let edge = topo.edge_id_from_index(0).unwrap();
    let data = topo.edge(edge).unwrap();
    let a = topo.vertex(data.start()).unwrap().point();
    let b = topo.vertex(data.end()).unwrap().point();
    assert!(((b - a).length() - scale).abs() < 1e-10 * scale);
    let curve = linear_curve(vec![a, b], &[0.0, span]);
    let data = topo.edge_mut(edge).unwrap();
    data.set_curve(EdgeCurve::NurbsCurve(curve));
    data.set_trim(Some((0.0, span)));
    (topo, solid, edge)
}

fn has(report: &ValidationReport, check: CheckId, edge: EdgeId) -> bool {
    report.issues.iter().any(|issue| {
        issue.check == check && matches!(issue.entity, EntityRef::Edge(id) if id == edge)
    })
}

#[test]
fn physical_cube_is_valid_under_equivalent_parameterizations_and_placements() {
    for scale in [1e-3, 1.0, 1e3] {
        for placed in [false, true] {
            for span in [1e-9, 1.0, 1e6] {
                let (topo, solid, edge) = cube_edge(scale, placed, span);
                let options = ValidateOptions {
                    tolerance_scale: scale,
                    ..Default::default()
                };
                let report = validate_solid(&topo, solid, &options).unwrap();
                assert!(
                    report.is_valid(),
                    "scale={scale}, span={span}: {:?}",
                    report.issues
                );
                assert!(!has(&report, CheckId::EdgeRangeValid, edge));
            }
        }
    }
}

#[test]
fn invalid_stored_trim_is_already_refused_without_a_stored_pcurve() {
    for trim in [(0.0, 0.0), (-1.0, 1.0), (0.0, 2.0)] {
        let (mut topo, solid, edge) = cube_edge(1.0, false, 1.0);
        topo.edge_mut(edge).unwrap().set_trim(Some(trim));
        let error = validate_solid(&topo, solid, &ValidateOptions::default()).unwrap_err();
        assert!(
            error.to_string().contains("invalid parameter range"),
            "{error}"
        );
    }
}

fn closed_curve_report(
    curve: NurbsCurve,
    trim: (f64, f64),
    scale: f64,
) -> (ValidationReport, EdgeId) {
    let mut topo = Topology::new();
    let v = topo.add_vertex(Vertex::new(curve.evaluate(trim.0), scale * 1e-7));
    let mut data = Edge::new(v, v, EdgeCurve::NurbsCurve(curve));
    data.set_trim(Some(trim));
    let edge = topo.add_edge(data);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let options = ValidateOptions {
        tolerance_scale: scale,
        ..Default::default()
    };
    (validate_wire_body(&topo, wire, &options).unwrap(), edge)
}

#[test]
fn degenerate_subtrim_does_not_inherit_carrier_length() {
    for scale in [1e-3, 1.0, 1e3] {
        for parameter_scale in [1e-6, 1.0, 1e6] {
            for reversed in [false, true] {
                let p = Point3::new(3.0 * scale, -2.0 * scale, 5.0 * scale);
                let curve = linear_curve(
                    vec![p, p, Point3::new(13.0 * scale, p.y(), p.z())],
                    &[0.0, parameter_scale, 100.0 * parameter_scale],
                );
                let trim = if reversed {
                    (parameter_scale, 0.0)
                } else {
                    (0.0, parameter_scale)
                };
                let (report, edge) = closed_curve_report(curve, trim, scale);
                assert!(
                    has(&report, CheckId::EdgeDegenerate, edge),
                    "{:?}",
                    report.issues
                );
                let issue = report
                    .issues
                    .iter()
                    .find(|i| i.check == CheckId::EdgeDegenerate)
                    .unwrap();
                assert_eq!(issue.severity, Severity::Warning);
                assert!(issue.deviation.unwrap() < 1e-7 * scale);
            }
        }
    }
}

#[test]
fn closed_excursion_between_uniform_samples_is_not_degenerate() {
    for span in [1e-6, 1.0, 1e6] {
        let p = Point3::new(0.0, 0.0, 0.0);
        let curve = linear_curve(
            vec![p, Point3::new(1.0, 0.0, 0.0), p, p],
            &[0.0, 0.01 * span, 0.02 * span, span],
        );
        // Independent polyline oracle: the closed curve travels one millimetre out and back.
        let (report, edge) = closed_curve_report(curve, (0.0, span), 1.0);
        assert!(
            !has(&report, CheckId::EdgeDegenerate, edge),
            "{:?}",
            report.issues
        );
        assert!(report.is_valid(), "{:?}", report.issues);
    }
}

#[test]
fn long_parameter_interval_with_tiny_physical_loop_keeps_warning() {
    for scale in [1e-3, 1.0, 1e3] {
        for span in [1.0, 1e6] {
            let p = Point3::new(0.0, 0.0, 0.0);
            let curve = linear_curve(
                vec![p, Point3::new(1e-10 * scale, 0.0, 0.0), p],
                &[0.0, span / 2.0, span],
            );
            let (report, edge) = closed_curve_report(curve, (0.0, span), scale);
            assert!(has(&report, CheckId::EdgeDegenerate, edge));
            assert!(!has(&report, CheckId::EdgeRangeValid, edge));
        }
    }
}

#[test]
fn small_trim_inside_large_carrier_is_valid_without_vertex_warning() {
    // The vertices sit at the trim ends, 40 carrier-lengths from either
    // carrier end. `check_vertex_on_curve` now measures against the edge's
    // trim ends (it used to measure against the carrier's knot-span ends and
    // warn here; this witness pinned that handoff until the fix).
    for scale in [1e-3, 1.0, 1e3] {
        let (mut topo, solid, edge) = cube_edge(scale, true, 1.0);
        let data = topo.edge(edge).unwrap();
        let a = topo.vertex(data.start()).unwrap().point();
        let b = topo.vertex(data.end()).unwrap().point();
        let delta = b - a;
        let curve = linear_curve(vec![a - delta * 40.0, a + delta * 60.0], &[0.0, 100.0]);
        assert!((curve.evaluate(40.0) - a).length() < 1e-9 * scale);
        assert!((curve.evaluate(41.0) - b).length() < 1e-9 * scale);
        let data = topo.edge_mut(edge).unwrap();
        data.set_curve(EdgeCurve::NurbsCurve(curve));
        data.set_trim(Some((40.0, 41.0)));
        for tolerance_scale in [1.0, scale] {
            let report = validate_solid(
                &topo,
                solid,
                &ValidateOptions {
                    tolerance_scale,
                    ..Default::default()
                },
            )
            .unwrap();
            assert!(report.is_valid(), "{:?}", report.issues);
            assert!(!has(&report, CheckId::EdgeRangeValid, edge));
            assert!(!has(&report, CheckId::EdgeDegenerate, edge));
            assert!(
                !report
                    .issues
                    .iter()
                    .any(|issue| issue.check == CheckId::VertexOnCurve),
                "{:?}",
                report.issues
            );
        }
    }
}

#[test]
fn reversed_trim_and_reversed_use_preserve_open_curve_extent() {
    for span in [1e-9, 1.0, 1e6] {
        for forward in [false, true] {
            let mut topo = Topology::new();
            let a = Point3::new(0.0, 0.0, 0.0);
            let b = Point3::new(1.0, 0.0, 0.0);
            let va = topo.add_vertex(Vertex::new(a, 1e-7));
            let vb = topo.add_vertex(Vertex::new(b, 1e-7));
            let mut data = Edge::new(
                vb,
                va,
                EdgeCurve::NurbsCurve(linear_curve(vec![a, b], &[0.0, span])),
            );
            data.set_trim(Some((span, 0.0)));
            let edge = topo.add_edge(data);
            let wire =
                topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, forward)], false).unwrap());
            let report = validate_wire_body(&topo, wire, &ValidateOptions::default()).unwrap();
            assert!(report.issues.is_empty(), "{:?}", report.issues);
        }
    }
}

fn seam_fixture(scale: f64, placed: bool) -> (Topology, EdgeId, FaceId, SolidId) {
    let mut topo = Topology::new();
    let origin = if placed {
        Point3::new(3.0 * scale, -2.0 * scale, 5.0 * scale)
    } else {
        Point3::new(0.0, 0.0, 0.0)
    };
    let axis = if placed {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    let cylinder =
        CylindricalSurface::with_ref_dir(origin, axis, scale, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let a = topo.add_vertex(Vertex::new(cylinder.evaluate(0.0, 0.0), scale * 1e-7));
    let b = topo.add_vertex(Vertex::new(cylinder.evaluate(0.0, scale), scale * 1e-7));
    let edge = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(edge, true),
                OrientedEdge::new(edge, false),
            ],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(cylinder)));
    for forward in [true, false] {
        let (u, v, direction) = if forward {
            (0.0, 0.0, 1.0)
        } else {
            (std::f64::consts::TAU, scale, -1.0)
        };
        topo.set_pcurve_oriented(
            edge,
            face,
            forward,
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(u, v), Vec2::new(0.0, direction)).unwrap()),
                0.0,
                scale,
            ),
        )
        .unwrap();
    }
    // This shell is only a traversal witness, not a claimed closed cylinder.
    let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    (topo, edge, face, solid)
}

#[test]
fn both_periodic_seam_uses_are_checked_at_physical_tolerances() {
    for scale in [1e-3, 1.0, 1e3] {
        for placed in [false, true] {
            let (mut topo, edge, face, solid) = seam_fixture(scale, placed);
            for tolerance_scale in [1.0, scale] {
                let options = ValidateOptions {
                    tolerance_scale,
                    ..Default::default()
                };
                let report = validate_solid(&topo, solid, &options).unwrap();
                assert!(
                    !has(&report, CheckId::EdgeSameParameter, edge),
                    "{:?}",
                    report.issues
                );
            }
            topo.set_pcurve_oriented(
                edge,
                face,
                false,
                PCurve::new(
                    Curve2D::Line(
                        Line2D::new(
                            Point2::new(std::f64::consts::PI, scale),
                            Vec2::new(0.0, -1.0),
                        )
                        .unwrap(),
                    ),
                    0.0,
                    scale,
                ),
            )
            .unwrap();
            let options = ValidateOptions {
                tolerance_scale: scale,
                ..Default::default()
            };
            let report = validate_solid(&topo, solid, &options).unwrap();
            assert!(has(&report, CheckId::EdgeSameParameter, edge));
        }
    }
}

#[test]
fn interior_pcurve_excursion_with_matching_endpoints_remains_refused_in_cavity() {
    let (mut topo, edge, face, seam_solid) = seam_fixture(1.0, false);
    let cavity = topo.solid(seam_solid).unwrap().outer_shell();
    let outer = make_unit_cube_manifold(&mut topo);
    topo.solid_mut(outer).unwrap().add_inner_shell(cavity);
    let bowed = NurbsCurve2D::new(
        1,
        vec![0.0, 0.0, 0.101, 0.105, 0.109, 1.0, 1.0],
        vec![
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 0.101),
            Point2::new(0.3, 0.105),
            Point2::new(0.0, 0.109),
            Point2::new(0.0, 1.0),
        ],
        vec![1.0; 5],
    )
    .unwrap();
    let pcurve = PCurve::new(Curve2D::Nurbs(bowed), 0.0, 1.0);
    let uv = pcurve.evaluate(0.105);
    // Independent cylinder chord oracle: matching endpoints hide a 0.3-radian excursion.
    assert!(2.0 * (uv.x() / 2.0).sin().abs() > 0.29);
    topo.set_pcurve_oriented(edge, face, true, pcurve).unwrap();
    let endpoints = remus_topology::validation::check_same_range_strict(&topo, edge, face, true)
        .unwrap()
        .unwrap();
    assert!(endpoints < 1e-12);
    let report = validate_solid(&topo, outer, &ValidateOptions::default()).unwrap();
    assert!(
        report
            .issues
            .iter()
            .any(|i| i.check == CheckId::EdgeSameParameter
                && i.description.contains("same_parameter_proof_unavailable")
                && matches!(i.entity, EntityRef::Edge(id) if id == edge))
    );
    assert!(!report.is_valid());
}

#[test]
fn stored_nurbs_curve_use_is_typed_unproven_even_with_exact_endpoints() {
    let (mut topo, edge, face, solid) = seam_fixture(1.0, false);
    let curve = linear_curve(
        vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0)],
        &[0.0, 1.0],
    );
    let data = topo.edge_mut(edge).unwrap();
    data.set_curve(EdgeCurve::NurbsCurve(curve));
    data.set_trim(Some((0.0, 1.0)));
    for forward in [true, false] {
        let endpoints =
            remus_topology::validation::check_same_range_strict(&topo, edge, face, forward)
                .unwrap()
                .unwrap();
        assert!(endpoints < 1e-12);
    }
    let report = validate_solid(&topo, solid, &ValidateOptions::default()).unwrap();
    let refusals = report
        .issues
        .iter()
        .filter(|i| {
            i.check == CheckId::EdgeSameParameter
                && i.description.contains("same_parameter_proof_unavailable")
        })
        .count();
    assert_eq!(refusals, 2);
    assert!(!report.is_valid());
}

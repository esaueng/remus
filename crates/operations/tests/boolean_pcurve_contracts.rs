//! Non-vacuous strict pcurve validation over a public boolean result.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::{PI, TAU};

use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::vec::{Point2, Vec2};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::validate;
use remus_topology::Topology;
use remus_topology::TopologyError;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::solid::SolidId;
use remus_topology::validation::{
    CurveUseValidationError, validate_boundary_authority, validate_same_parameter_strict,
    validate_same_range_strict, validate_solid_pcurve_contracts,
};

fn cylinder_seam(topo: &Topology, solid: SolidId) -> (EdgeId, FaceId, f64, f64, f64) {
    let mut match_ = None;
    for face_id in solid_faces(topo, solid).expect("solid faces") {
        let face = topo.face(face_id).expect("face");
        let FaceSurface::Cylinder(surface) = face.surface() else {
            continue;
        };
        let wire = topo.wire(face.outer_wire()).expect("lateral wire");
        for oriented in wire.edges() {
            if !oriented.is_forward() {
                continue;
            }
            if !matches!(
                topo.edge(oriented.edge()).expect("edge").curve(),
                EdgeCurve::Line
            ) {
                continue;
            }
            let uses: Vec<_> = wire
                .edges()
                .iter()
                .filter(|use_| use_.edge() == oriented.edge())
                .map(remus_topology::OrientedEdge::is_forward)
                .collect();
            if uses == [true, false] {
                let edge = topo.edge(oriented.edge()).expect("seam edge");
                let start = topo.vertex(edge.start()).expect("seam start").point();
                let end = topo.vertex(edge.end()).expect("seam end").point();
                let (u_start, v_start) = surface.project_point(start);
                let (u_end, v_end) = surface.project_point(end);
                assert!(
                    (u_end - u_start)
                        .rem_euclid(TAU)
                        .min((u_start - u_end).rem_euclid(TAU))
                        < 1e-12,
                    "seam endpoints must share one periodic-u branch"
                );
                assert!(
                    match_.is_none(),
                    "fixture must contain exactly one cylinder seam"
                );
                match_ = Some((oriented.edge(), face_id, u_start, v_start, v_end));
            }
        }
    }
    match_.expect("cylinder seam")
}

fn seam_pcurve(u: f64, v_start: f64, v_end: f64) -> PCurve {
    let span = (v_end - v_start).abs();
    let direction = if v_end > v_start { 1.0 } else { -1.0 };
    PCurve::new(
        Curve2D::Line(
            Line2D::new(Point2::new(u, v_start), Vec2::new(0.0, direction))
                .expect("non-zero seam direction"),
        ),
        0.0,
        span,
    )
}

fn attach_exact_seam_pcurves(topo: &mut Topology, solid: SolidId) -> (EdgeId, FaceId) {
    let (edge, face, u, v_start, v_end) = cylinder_seam(topo, solid);
    topo.set_pcurve_oriented(edge, face, true, seam_pcurve(u, v_start, v_end))
        .unwrap();
    topo.set_pcurve_oriented(edge, face, false, seam_pcurve(u + TAU, v_end, v_start))
        .unwrap();
    (edge, face)
}

#[test]
fn boolean_output_runs_non_vacuous_oriented_same_parameter_and_same_range_contracts() {
    const RADIUS: f64 = 2.0;
    const HEIGHT: f64 = 3.0;
    const TOLERANCE: f64 = 1e-7;

    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, RADIUS, HEIGHT).expect("cylinder");
    let (input_seam, input_face) = attach_exact_seam_pcurves(&mut topo, cylinder);

    for forward in [true, false] {
        validate_same_parameter_strict(&topo, input_seam, input_face, forward, TOLERANCE, 32)
            .expect("input SameParameter contract");
        validate_same_range_strict(&topo, input_seam, input_face, forward, TOLERANCE)
            .expect("input SameRange contract");
    }

    let result = boolean(&mut topo, BooleanOp::Intersect, cylinder, cylinder)
        .expect("identity intersection");
    assert_ne!(
        result, cylinder,
        "boolean result must be independent topology"
    );

    let summary = validate_solid_pcurve_contracts(&topo, result, TOLERANCE, 32)
        .expect("strict boolean-output pcurve contracts");
    assert_eq!(summary.boundary_uses, 6);
    assert_eq!(summary.stored_pcurves, 2, "gate must not pass vacuously");
    assert_eq!(summary.validated_uses, 2, "both seam branches are required");
    let boundary = validate_boundary_authority(&topo).expect("whole-topology boundary authority");
    assert_eq!(boundary.faces, topo.num_faces());
    assert_eq!(boundary.loops, topo.num_loops());
    assert_eq!(boundary.coedges, topo.num_coedges());
    assert_eq!(boundary.seam_edges, 2, "input and result cylinder seams");
    assert_eq!(boundary.stored_seam_branches, 4);

    let faces = solid_faces(&topo, result).expect("result faces");
    let plane_count = faces
        .iter()
        .filter(|&&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Plane { .. }
            )
        })
        .count();
    let cylinder_count = faces
        .iter()
        .filter(|&&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .count();
    assert_eq!((plane_count, cylinder_count), (2, 1));
    let report = validate::validate_solid(&topo, result).expect("result validation");
    assert!(
        report.is_valid(),
        "boolean result issues: {:?}",
        report.issues
    );
    let volume = solid_volume(&topo, result, 0.01).expect("result volume");
    let expected_volume = PI * RADIUS * RADIUS * HEIGHT;
    assert!(
        (volume - expected_volume).abs() <= expected_volume * 1e-9,
        "identity intersection volume {volume} != {expected_volume}"
    );

    let (result_seam, result_face, u, v_start, v_end) = cylinder_seam(&topo, result);
    assert!(
        topo.pcurve_oriented(result_seam, result_face, true)
            .is_some()
    );
    assert!(
        topo.pcurve_oriented(result_seam, result_face, false)
            .is_some()
    );
    topo.set_pcurve_oriented(
        result_seam,
        result_face,
        false,
        seam_pcurve(u + TAU + 0.2, v_end, v_start),
    )
    .unwrap();
    validate_same_parameter_strict(&topo, result_seam, result_face, true, TOLERANCE, 32)
        .expect("forward seam branch remains valid");
    assert!(matches!(
        validate_solid_pcurve_contracts(&topo, result, TOLERANCE, 32),
        Err(CurveUseValidationError::Topology(
            TopologyError::SameParameterExceeded { .. }
        ))
    ));
}

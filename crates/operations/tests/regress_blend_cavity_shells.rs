//! Blends and shell on a solid with a cavity.
//!
//! The rolling-ball fillet, the flat chamfer, the variable and v1 fillets and
//! `shell` rebuild the outer shell alone. On a hollow solid they used to return
//! the rebuilt outer shell with the cavity deleted: the result validated and
//! its volume was off only by the cavity's 8 mm³. Those engines now refuse a
//! solid with cavity shells; the v2 fillet routes it to the walking builder,
//! which carries the cavity through when the band stays clear of it and
//! refuses otherwise.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, missing_docs)]
#![allow(deprecated)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::blend_ops::{chamfer_v2, fillet_cascade, fillet_v2};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::fillet::{FilletRadiusLaw, fillet_rolling_ball, fillet_variable};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_operations::validate;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_edges;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

const R: f64 = 2.0;
const CAVITY: f64 = 8.0;

/// A 20³ box with a sealed 2³ cavity at `corner`.
fn hollow_box(topo: &mut Topology, corner: f64) -> SolidId {
    hollow_box_at(topo, [corner; 3])
}

/// A 20³ box with a sealed 2³ cavity whose minimum corner is `corner`.
fn hollow_box_at(topo: &mut Topology, corner: [f64; 3]) -> SolidId {
    let blank = make_box(topo, 20.0, 20.0, 20.0).unwrap();
    let tool = make_box(topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(
        topo,
        tool,
        &Mat4::translation(corner[0], corner[1], corner[2]),
    )
    .unwrap();
    let solid = boolean(topo, BooleanOp::Cut, blank, tool).unwrap();
    assert_eq!(
        topo.solid(solid).unwrap().inner_shells().len(),
        1,
        "premise: hollow"
    );
    solid
}

fn near(p: Point3, q: [f64; 3]) -> bool {
    (p.x() - q[0]).abs() < 1e-9 && (p.y() - q[1]).abs() < 1e-9 && (p.z() - q[2]).abs() < 1e-9
}

/// The edge from (0, 0, 20) to (20, 0, 20).
fn top_front_edge(topo: &Topology, solid: SolidId) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let edge = topo.edge(e).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            (near(a, [0.0, 0.0, 20.0]) && near(b, [20.0, 0.0, 20.0]))
                || (near(b, [0.0, 0.0, 20.0]) && near(a, [20.0, 0.0, 20.0]))
        })
        .expect("top front edge")
}

/// The z = 20 face.
fn top_face(topo: &Topology, solid: SolidId) -> FaceId {
    remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            let face = topo.face(f).unwrap();
            face.effective_plane_normal().is_some_and(|n| n.z() > 0.9)
                && topo
                    .wire(face.outer_wire())
                    .unwrap()
                    .edges()
                    .iter()
                    .all(|oe| {
                        let edge = topo.edge(oe.edge()).unwrap();
                        (topo.vertex(edge.start()).unwrap().point().z() - 20.0).abs() < 1e-9
                    })
        })
        .expect("top face")
}

fn fillet_volume() -> f64 {
    8000.0 - CAVITY - (R * R - PI * R * R / 4.0) * 20.0
}

/// The result keeps its cavity shell, validates, and measures `expected`.
fn assert_keeps_cavity(topo: &Topology, label: &str, solid: SolidId, expected: f64) {
    assert_eq!(
        topo.solid(solid).unwrap().inner_shells().len(),
        1,
        "{label}: cavity shell dropped"
    );
    let report = validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:#?}", report.issues);
    let volume = solid_volume(topo, solid, 1e-4).unwrap();
    assert!(
        (volume - expected).abs() < 1e-3,
        "{label}: volume {volume}, expected {expected}"
    );
}

/// The engine refuses with a typed `Unsupported` and leaves the arena as it was.
fn assert_refuses_unchanged(
    label: &str,
    run: impl FnOnce(&mut Topology, SolidId) -> Result<(), OperationsError>,
) {
    let mut topo = Topology::new();
    let solid = hollow_box(&mut topo, 9.0);
    let before = (
        topo.num_faces(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_pcurves(),
    );
    let error = run(&mut topo, solid).expect_err(label);
    assert!(
        matches!(error, OperationsError::Unsupported { .. }),
        "{label}: expected Unsupported, got {error}"
    );
    let after = (
        topo.num_faces(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_pcurves(),
    );
    assert_eq!(before, after, "{label}: refusal left entities behind");
}

#[test]
fn fillet_cascade_keeps_a_cavity() {
    let mut topo = Topology::new();
    let solid = hollow_box(&mut topo, 9.0);
    let edge = top_front_edge(&topo, solid);
    let result = fillet_cascade(&mut topo, solid, &[edge], R).unwrap();
    assert_keeps_cavity(&topo, "fillet_cascade", result.solid, fillet_volume());
}

#[test]
fn fillet_v2_keeps_a_cavity() {
    let mut topo = Topology::new();
    let solid = hollow_box(&mut topo, 9.0);
    let edge = top_front_edge(&topo, solid);
    let result = fillet_v2(&mut topo, solid, &[edge], R).unwrap();
    assert_keeps_cavity(&topo, "fillet_v2", result.solid, fillet_volume());
}

/// A cavity one unit under both faces of the filleted edge: an r = 4 band
/// removes material down to 4 - 4/sqrt(2) = 1.17 from each face along the
/// bisector, through the cavity's corner. The walking builder would carry the
/// cavity shell through unchanged, so the result would overlap it.
#[test]
fn walking_fillet_refuses_a_cavity_within_reach() {
    for (label, run) in [
        (
            "fillet_v2",
            (|topo, solid, edge| fillet_v2(topo, solid, &[edge], 4.0).map(drop))
                as fn(&mut Topology, SolidId, EdgeId) -> Result<(), OperationsError>,
        ),
        ("fillet_cascade", |topo, solid, edge| {
            fillet_cascade(topo, solid, &[edge], 4.0).map(drop)
        }),
    ] {
        let mut topo = Topology::new();
        let solid = hollow_box_at(&mut topo, [9.0, 1.0, 17.0]);
        let edge = top_front_edge(&topo, solid);
        let before = (topo.num_faces(), topo.num_loops(), topo.num_coedges());
        let error = run(&mut topo, solid, edge).expect_err(label);
        assert!(
            matches!(error, OperationsError::Unsupported { .. }),
            "{label}: expected Unsupported, got {error}"
        );
        assert_eq!(
            (topo.num_faces(), topo.num_loops(), topo.num_coedges()),
            before,
            "{label}: refusal left entities behind"
        );
    }
}

#[test]
fn outer_shell_engines_refuse_a_cavity() {
    assert_refuses_unchanged("fillet_rolling_ball", |topo, solid| {
        let edge = top_front_edge(topo, solid);
        fillet_rolling_ball(topo, solid, &[edge], R).map(drop)
    });
    assert_refuses_unchanged("fillet_variable", |topo, solid| {
        let edge = top_front_edge(topo, solid);
        fillet_variable(topo, solid, &[(edge, FilletRadiusLaw::Constant(R))]).map(drop)
    });
    assert_refuses_unchanged("fillet (v1)", |topo, solid| {
        let edge = top_front_edge(topo, solid);
        remus_operations::fillet::fillet(topo, solid, &[edge], R).map(drop)
    });
    assert_refuses_unchanged("chamfer", |topo, solid| {
        let edge = top_front_edge(topo, solid);
        remus_operations::chamfer::chamfer(topo, solid, &[edge], R).map(drop)
    });
    // The walking chamfer cannot trim a planar box edge either, so a hollow
    // planar-line chamfer stays a refusal rather than a cavity-less result.
    assert_refuses_unchanged("chamfer_v2", |topo, solid| {
        let edge = top_front_edge(topo, solid);
        chamfer_v2(topo, solid, &[edge], R, R).map(drop)
    });
}

#[test]
fn shell_refuses_a_cavity() {
    // The cavity at [1, 3]³ lies inside the 4-thick wall the shell would build.
    let mut topo = Topology::new();
    let solid = hollow_box(&mut topo, 1.0);
    let top = top_face(&topo, solid);
    let before = (
        topo.num_faces(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_pcurves(),
    );
    let error = remus_operations::shell_op::shell(&mut topo, solid, 4.0, &[top]).unwrap_err();
    assert!(
        matches!(error, OperationsError::Unsupported { .. }),
        "expected Unsupported, got {error}"
    );
    let after = (
        topo.num_faces(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_pcurves(),
    );
    assert_eq!(before, after);
}

#[test]
fn solid_without_a_cavity_keeps_the_planar_engines() {
    // Control: the refusal is cavity-gated, not a change to the planar routes.
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let edge = top_front_edge(&topo, solid);
    let result = fillet_v2(&mut topo, solid, &[edge], R).unwrap();
    assert_eq!(
        result.engine,
        remus_operations::blend_ops::BlendEngine::RollingBall
    );
    let volume = solid_volume(&topo, result.solid, 1e-4).unwrap();
    assert!(
        (volume - (fillet_volume() + CAVITY)).abs() < 1e-3,
        "{volume}"
    );
}

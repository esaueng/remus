//! Qualification evidence for feature recognition's declared feature set.
//!
//! Axes covered (see `docs/kernel-maturity/stabilization-plan.md`, item A4):
//! feature type (through-hole, blind hole, pocket, chamfer) × geometry
//! (planar and cylindrical walls) × noise (post-boolean topology), with
//! precision pins (a plain body yields no features), recall pins on
//! constructed ground truth, and determinism across rebuilds.
//!
//! "Not recognized" is a first-class outcome: the declared set is what these
//! tests pin, and anything outside it is absence of a claim, not an error.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::chamfer::chamfer;
use remus_operations::feature_recognition::{Feature, recognize_features};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::solid::SolidId;

const DEFLECTION: f64 = 0.05;

fn holes(features: &[Feature]) -> Vec<(usize, Option<f64>, bool)> {
    features
        .iter()
        .filter_map(|f| match f {
            Feature::Hole {
                faces,
                diameter,
                through,
            } => Some((faces.len(), *diameter, *through)),
            _ => None,
        })
        .collect()
}

fn pockets(features: &[Feature]) -> usize {
    features
        .iter()
        .filter(|f| matches!(f, Feature::Pocket { .. }))
        .count()
}

fn chamfers(features: &[Feature]) -> usize {
    features
        .iter()
        .filter(|f| matches!(f, Feature::Chamfer { .. }))
        .count()
}

/// Precision: a plain box and a plain cylinder carry no features, and the
/// recognizer must not invent any.
#[test]
fn plain_bodies_yield_no_features() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 2.0, 1.5, 1.0).unwrap();
    let features = recognize_features(&topo, cube, DEFLECTION).unwrap();
    assert!(
        features.is_empty(),
        "plain box must yield no features, got {features:?}"
    );

    let cyl = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    let features = recognize_features(&topo, cyl, DEFLECTION).unwrap();
    assert!(
        features.is_empty(),
        "plain cylinder must yield no features, got {features:?}"
    );
}

/// Recall: a drilled through-hole is recognized as a through hole with the
/// drill's diameter.
#[test]
fn cylindrical_through_hole_recognized() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 2.0, 2.0, 1.0).unwrap();
    let drill = make_cylinder(&mut topo, 0.3, 2.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(1.0, 1.0, -0.5)).unwrap();
    let bored = boolean(&mut topo, BooleanOp::Cut, cube, drill).unwrap();

    let features = recognize_features(&topo, bored, DEFLECTION).unwrap();
    let hs = holes(&features);
    assert_eq!(hs.len(), 1, "expected exactly one hole, got {features:?}");
    let (_, diameter, through) = hs[0];
    assert!(through, "the drill goes through");
    let d = diameter.expect("cylindrical hole should carry a diameter");
    assert!((d - 0.6).abs() < 1e-6, "expected diameter 0.6, got {d}");
}

/// Recall: a blind cylindrical hole is recognized and marked not-through.
#[test]
fn blind_hole_recognized() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 2.0, 2.0, 1.0).unwrap();
    let drill = make_cylinder(&mut topo, 0.3, 0.6).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(1.0, 1.0, 0.4)).unwrap();
    let bored = boolean(&mut topo, BooleanOp::Cut, cube, drill).unwrap();

    let features = recognize_features(&topo, bored, DEFLECTION).unwrap();
    let hs = holes(&features);
    assert_eq!(hs.len(), 1, "expected exactly one hole, got {features:?}");
    assert!(!hs[0].2, "a blind hole is not through");
}

/// Recall: a rectangular pocket is recognized.
#[test]
fn rectangular_pocket_recognized() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 2.0, 2.0, 1.0).unwrap();
    let tool = make_box(&mut topo, 0.6, 0.8, 0.5).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(0.7, 0.6, 0.5)).unwrap();
    let pocketed = boolean(&mut topo, BooleanOp::Cut, cube, tool).unwrap();

    let features = recognize_features(&topo, pocketed, DEFLECTION).unwrap();
    assert!(
        pockets(&features) >= 1,
        "expected a pocket, got {features:?}"
    );
}

/// Recall: a chamfered box edge is recognized as a chamfer.
#[test]
fn chamfer_recognized() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let target = top_x_edge(&topo, cube);
    let chamfered = chamfer(&mut topo, cube, &[target], 0.15).unwrap();

    let features = recognize_features(&topo, chamfered, DEFLECTION).unwrap();
    assert!(
        chamfers(&features) >= 1,
        "expected a chamfer, got {features:?}"
    );
}

/// Determinism: recognition output is identical across rebuilds of the same
/// body — same feature kinds in the same order.
#[test]
fn recognition_is_deterministic() {
    let run = || {
        let mut topo = Topology::new();
        let cube = make_box(&mut topo, 3.0, 2.0, 1.0).unwrap();
        let d1 = make_cylinder(&mut topo, 0.2, 2.0).unwrap();
        transform_solid(&mut topo, d1, &Mat4::translation(0.7, 1.0, -0.5)).unwrap();
        let d2 = make_cylinder(&mut topo, 0.2, 2.0).unwrap();
        transform_solid(&mut topo, d2, &Mat4::translation(2.3, 1.0, -0.5)).unwrap();
        let b1 = boolean(&mut topo, BooleanOp::Cut, cube, d1).unwrap();
        let b2 = boolean(&mut topo, BooleanOp::Cut, b1, d2).unwrap();
        let features = recognize_features(&topo, b2, DEFLECTION).unwrap();
        features
            .iter()
            .map(|f| match f {
                Feature::Hole {
                    faces,
                    diameter,
                    through,
                } => format!(
                    "hole:{}:{:.6}:{}",
                    faces.len(),
                    diameter.unwrap_or(-1.0),
                    through
                ),
                Feature::Chamfer { angle, .. } => format!("chamfer:{angle:.6}"),
                Feature::FilletLike { area, .. } => format!("fillet:{area:.6}"),
                Feature::Pocket { walls, .. } => format!("pocket:{}", walls.len()),
                Feature::Pattern {
                    count,
                    pattern_type,
                    ..
                } => format!("pattern:{count}:{pattern_type:?}"),
                other => format!("{other:?}"),
            })
            .collect::<Vec<_>>()
    };
    let a = run();
    let b = run();
    assert_eq!(a, b, "recognition must be deterministic");
    assert_eq!(
        a.iter().filter(|s| s.starts_with("hole:")).count(),
        2,
        "two drilled holes expected, got {a:?}"
    );
}

/// The top +X edge of a unit box anchored at the origin.
fn top_x_edge(topo: &Topology, cube: SolidId) -> EdgeId {
    let s = topo.solid(cube).unwrap();
    for &fid in topo.shell(s.outer_shell()).unwrap().faces() {
        let f = topo.face(fid).unwrap();
        for oe in topo.wire(f.outer_wire()).unwrap().edges() {
            let e = topo.edge(oe.edge()).unwrap();
            let a = topo.vertex(e.start()).unwrap().point();
            let b = topo.vertex(e.end()).unwrap().point();
            let on = |p: Point3| (p.x() - 1.0).abs() < 1e-9 && (p.z() - 1.0).abs() < 1e-9;
            if on(a) && on(b) {
                return oe.edge();
            }
        }
    }
    panic!("edge not found");
}

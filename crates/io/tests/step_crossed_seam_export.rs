//! Overlapping-cylinder unions export a STEP seam outside every hole
//! (OpenZCAD #1243).
//!
//! Cylinder A (r 8, h 16, base (0, 0, -8)) united with cylinder B
//! (r 6.56, h 13.12, base (6, 1, -6.06)) keeps A's lateral face as one band
//! with a window where B passes through it. A's seam half-plane (`u = 0`,
//! the +X side) lies inside that window, so the writer used to emit A's
//! seam LINE straight through the face's own hole. Remus re-read the file
//! correctly, but an independent reader treated it as a solid of the right volume that
//! classified points inside B as outside (A then B) and meshed it with open
//! and non-manifold edges, for both Union orders and after reload.
//!
//! The writer now moves such a seam into the band's widest free angular gap
//! on a private copy. These tests pin the internal crossing the fix is for,
//! the exported layout, the exact round trip of both orders and of the
//! reloaded export, independently produced inputs, and that the
//! source arena is not touched by export.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::{PI, TAU};

use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const REFERENCE_A: &str = include_str!("data/reference_cylinder_r8_h16.step");
const REFERENCE_B: &str = include_str!("data/reference_cylinder_r6_56_h13_12.step");

/// Strictly inside B and outside A: the issue's classification witnesses.
const B_WITNESSES: [[f64; 3]; 2] = [[10.23672, -2.992, 0.336], [8.2424, -4.32, 0.16]];

fn native_inputs(topo: &mut Topology) -> (SolidId, SolidId) {
    let a = make_cylinder(topo, 8.0, 16.0).unwrap();
    transform_solid(topo, a, &Mat4::translation(0.0, 0.0, -8.0)).unwrap();
    let b = make_cylinder(topo, 6.56, 13.12).unwrap();
    transform_solid(topo, b, &Mat4::translation(6.0, 1.0, -6.06)).unwrap();
    (a, b)
}

fn reference_inputs(topo: &mut Topology) -> (SolidId, SolidId) {
    let a = read_step(REFERENCE_A, topo).unwrap();
    let b = read_step(REFERENCE_B, topo).unwrap();
    assert_eq!((a.len(), b.len()), (1, 1));
    (a[0], b[0])
}

/// Angular separation between each cylindrical band's seam and the
/// unwrapped footprints of its inner wires; negative when a seam crosses.
fn seam_clearances(topo: &Topology, solid: SolidId) -> Vec<f64> {
    let mut clearances = Vec::new();
    for face_id in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        let FaceSurface::Cylinder(cylinder) = face.surface() else {
            continue;
        };
        if face.inner_wires().is_empty() {
            continue;
        }
        let outer = topo.wire(face.outer_wire()).unwrap();
        let Some(seam) = outer.edges().iter().find(|oriented| {
            outer
                .edges()
                .iter()
                .filter(|other| other.edge() == oriented.edge())
                .count()
                == 2
        }) else {
            continue;
        };
        let seam_edge = topo.edge(seam.edge()).unwrap();
        let seam_angle = cylinder
            .project_point(topo.vertex(seam_edge.start()).unwrap().point())
            .0;
        for &wire in face.inner_wires() {
            let mut angles = Vec::new();
            for oriented in topo.wire(wire).unwrap().edges() {
                let edge = topo.edge(oriented.edge()).unwrap();
                let start = topo.vertex(edge.start()).unwrap().point();
                let end = topo.vertex(edge.end()).unwrap().point();
                let (t0, t1) = match edge.curve() {
                    EdgeCurve::Line => (0.0, 1.0),
                    _ => edge.strict_domain().unwrap(),
                };
                for step in 0..=256 {
                    let fraction = f64::from(step) / 256.0;
                    let point =
                        edge.curve()
                            .evaluate_with_endpoints(t0 + (t1 - t0) * fraction, start, end);
                    angles.push(cylinder.project_point(point).0);
                }
            }
            let mut unwrapped = angles[0];
            let (mut lo, mut hi) = (unwrapped, unwrapped);
            for pair in angles.windows(2) {
                unwrapped += (pair[1] - pair[0] + PI).rem_euclid(TAU) - PI;
                lo = lo.min(unwrapped);
                hi = hi.max(unwrapped);
            }
            assert!(hi - lo < TAU, "an inner wire winds around the axis");
            // Distance from the seam to the arc [lo, hi] on the circle.
            let offset = (seam_angle - lo).rem_euclid(TAU);
            clearances.push(if offset <= hi - lo {
                -offset.min(hi - lo - offset)
            } else {
                (offset - (hi - lo)).min(TAU - offset)
            });
        }
    }
    clearances
}

fn assert_union(topo: &Topology, solid: SolidId, volume: f64, label: &str) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    validate_shell_closed(
        topo.shell(topo.solid(solid).unwrap().outer_shell())
            .unwrap(),
        topo,
    )
    .unwrap();
    let measured = solid_volume(topo, solid, 0.005).unwrap();
    assert!(
        (measured - volume).abs() <= volume * 1e-9,
        "{label}: volume {measured} vs {volume}"
    );
    assert!(
        (measured - 3_947.686_54).abs() < 1e-5,
        "{label}: {measured}"
    );
    for witness in B_WITNESSES {
        let point = Point3::new(witness[0], witness[1], witness[2]);
        assert_eq!(
            classify_point(topo, solid, point, 0.01, 1e-7).unwrap(),
            PointClassification::Inside,
            "{label}: {point:?}"
        );
    }
    for outside in [
        [0.0, 0.0, 8.5],
        [13.0, 1.0, 0.0],
        [10.0, 1.0, 7.5],
        [-8.5, 0.0, 0.0],
    ] {
        let point = Point3::new(outside[0], outside[1], outside[2]);
        assert_eq!(
            classify_point(topo, solid, point, 0.01, 1e-7).unwrap(),
            PointClassification::Outside,
            "{label}: {point:?}"
        );
    }
}

fn solid_points(topo: &Topology, solid: SolidId) -> Vec<Point3> {
    let mut points = Vec::new();
    for face in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face).unwrap();
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oriented in topo.wire(wire).unwrap().edges() {
                let edge = topo.edge(oriented.edge()).unwrap();
                for vertex in [edge.start(), edge.end()] {
                    points.push(topo.vertex(vertex).unwrap().point());
                }
            }
        }
    }
    points
}

fn reimport(step: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(step, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    (topo, solids[0])
}

fn check_order(mut topo: Topology, a: SolidId, b: SolidId, a_first: bool, label: &str) {
    let union = if a_first {
        boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap()
    } else {
        boolean(&mut topo, BooleanOp::Fuse, b, a).unwrap()
    };
    let volume = solid_volume(&topo, union, 0.005).unwrap();
    assert_union(&topo, union, volume, label);
    // The kernel's own band keeps A's seam inside the window: the layout
    // this export fix exists for.
    assert!(
        seam_clearances(&topo, union).iter().any(|&c| c < 0.0),
        "{label}: the union no longer crosses its seam; retarget this test"
    );

    let before = format!("{:?}", solid_points(&topo, union));
    let written = write_step(&topo, &[union]).unwrap();
    assert_eq!(
        before,
        format!("{:?}", solid_points(&topo, union)),
        "{label}: export must not move the source arena's seam"
    );
    assert_eq!(
        written,
        write_step(&topo, &[union]).unwrap(),
        "{label}: deterministic"
    );

    let (exported, solid) = reimport(&written);
    let clearances = seam_clearances(&exported, solid);
    assert!(
        !clearances.is_empty(),
        "{label}: the windowed band survives"
    );
    assert!(
        clearances.iter().all(|&c| c > 0.1),
        "{label}: exported seam clearances {clearances:?}"
    );
    assert_union(&exported, solid, volume, &format!("{label} exported"));

    // Reload and export again: the reloaded document keeps the clear seam.
    let rewritten = write_step(&exported, &[solid]).unwrap();
    let (reloaded, solid) = reimport(&rewritten);
    assert!(
        seam_clearances(&reloaded, solid).iter().all(|&c| c > 0.1),
        "{label}: reloaded export"
    );
    assert_union(&reloaded, solid, volume, &format!("{label} reloaded"));
}

#[test]
fn native_cylinder_union_exports_a_seam_clear_of_its_window_in_both_orders() {
    for a_first in [true, false] {
        let mut topo = Topology::new();
        let (a, b) = native_inputs(&mut topo);
        check_order(topo, a, b, a_first, &format!("native a_first={a_first}"));
    }
}

#[test]
fn reference_cylinder_union_exports_a_seam_clear_of_its_window_in_both_orders() {
    for a_first in [true, false] {
        let mut topo = Topology::new();
        let (a, b) = reference_inputs(&mut topo);
        check_order(topo, a, b, a_first, &format!("reference a_first={a_first}"));
    }
}

/// A band whose holes already leave its seam clear is written unchanged.
#[test]
fn clear_seam_export_is_unchanged() {
    let mut topo = Topology::new();
    let a = make_cylinder(&mut topo, 8.0, 16.0).unwrap();
    transform_solid(&mut topo, a, &Mat4::translation(0.0, 0.0, -8.0)).unwrap();
    // B mirrored to the -X side pokes out away from A's +X seam.
    let b = make_cylinder(&mut topo, 6.56, 13.12).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(-6.0, 1.0, -6.06)).unwrap();
    let union = boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    let clearances = seam_clearances(&topo, union);
    assert!(!clearances.is_empty() && clearances.iter().all(|&c| c > 0.1));
    let written = write_step(&topo, &[union]).unwrap();
    let (exported, solid) = reimport(&written);
    let original = solid_points(&topo, union);
    for face in solid_faces(&exported, solid).unwrap() {
        let face = exported.face(face).unwrap();
        for oriented in exported.wire(face.outer_wire()).unwrap().edges() {
            let point = exported
                .vertex(exported.edge(oriented.edge()).unwrap().start())
                .unwrap()
                .point();
            assert!(
                original.iter().any(|p| (*p - point).length() < 1e-9),
                "every exported vertex is a source vertex: {point:?}"
            );
        }
    }
}

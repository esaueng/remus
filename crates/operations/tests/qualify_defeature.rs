//! Qualification evidence for defeaturing's declared planar-feature domain.
//!
//! Axes covered (see `docs/kernel-maturity/stabilization-plan.md`, item A2):
//! feature class (through-hole, blind pocket, boss, chamfer, cylindrical
//! bore), heal strategy (cap, extend), scale (1e-3 / 1 / 1e3), and typed
//! refusals from both sides of each declared boundary. Every positive case
//! restores a closed-form volume and passes full validation.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::chamfer::chamfer;
use remus_operations::defeature::defeature;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// Outer-shell plane faces with the given normal (and plane offset, when
/// `at` is given).
fn planar_faces(topo: &Topology, solid: SolidId, target: Vec3, at: Option<f64>) -> Vec<FaceId> {
    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();
    sh.faces()
        .iter()
        .filter(|&&fid| {
            let f = topo.face(fid).unwrap();
            if let FaceSurface::Plane { normal, d } = f.surface() {
                (*normal - target).length() < 1e-6 && at.is_none_or(|v| (d - v).abs() < 1e-6)
            } else {
                false
            }
        })
        .copied()
        .collect()
}

/// Outer-shell faces strictly inside the axis-aligned box `[lo, hi]`.
fn faces_within(topo: &Topology, solid: SolidId, lo: Point3, hi: Point3) -> Vec<FaceId> {
    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();
    sh.faces()
        .iter()
        .filter(|&&fid| {
            let f = topo.face(fid).unwrap();
            let mut all_in = true;
            let mut saw_vertex = false;
            for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                for oe in topo.wire(wid).unwrap().edges() {
                    let e = topo.edge(oe.edge()).unwrap();
                    for vid in [e.start(), e.end()] {
                        saw_vertex = true;
                        let p = topo.vertex(vid).unwrap().point();
                        all_in &= p.x() >= lo.x() - 1e-9
                            && p.x() <= hi.x() + 1e-9
                            && p.y() >= lo.y() - 1e-9
                            && p.y() <= hi.y() + 1e-9
                            && p.z() >= lo.z() - 1e-9
                            && p.z() <= hi.z() + 1e-9;
                    }
                }
            }
            saw_vertex && all_in
        })
        .copied()
        .collect()
}

fn assert_valid_with_volume(topo: &Topology, solid: SolidId, expected: f64) {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "defeatured solid failed validation: {:?}",
        report.issues
    );
    let vol = solid_volume(topo, solid, 0.01 * expected.cbrt().max(1e-3)).unwrap();
    assert!(
        ((vol - expected) / expected).abs() < 1e-6,
        "expected volume {expected}, got {vol}"
    );
}

/// Cap strategy, through-hole: removing the four hole walls restores the
/// plain box exactly, at three model scales.
///
/// The 1e-3 body is produced by scaling a unit-scale holed box down, not by
/// cutting at 1e-3 — the boolean's own micron-scale gap is pinned separately
/// in `boolean_scale_gap.rs` and is not defeature's cell.
#[test]
fn through_hole_removed_across_scales() {
    for s in [1e-3_f64, 1.0, 1e3] {
        let mut topo = Topology::new();
        let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let cutter = make_box(&mut topo, 0.4, 0.4, 2.0).unwrap();
        transform_solid(&mut topo, cutter, &Mat4::translation(0.3, 0.3, -0.5)).unwrap();
        let holed = boolean(&mut topo, BooleanOp::Cut, cube, cutter).unwrap();
        if (s - 1.0).abs() > f64::EPSILON {
            transform_solid(&mut topo, holed, &Mat4::scale(s, s, s)).unwrap();
        }

        // The hole walls are the faces whose vertices all lie inside the
        // hole prism footprint.
        let walls = faces_within(
            &topo,
            holed,
            Point3::new(0.3 * s - 1e-7, 0.3 * s - 1e-7, -1e-7),
            Point3::new(0.7 * s + 1e-7, 0.7 * s + 1e-7, s + 1e-7),
        );
        assert_eq!(walls.len(), 4, "scale {s}: expected the 4 hole walls");

        let healed = defeature(&mut topo, holed, &walls).unwrap();
        assert_valid_with_volume(&topo, healed, s * s * s);
    }
}

/// Cap strategy, blind pocket: removing the pocket's four walls and floor
/// restores the plain box.
#[test]
fn blind_pocket_removed() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let pocket = make_box(&mut topo, 0.4, 0.4, 0.6).unwrap();
    transform_solid(&mut topo, pocket, &Mat4::translation(0.3, 0.3, 0.6)).unwrap();
    let pocketed = boolean(&mut topo, BooleanOp::Cut, cube, pocket).unwrap();

    let feature = faces_within(
        &topo,
        pocketed,
        Point3::new(0.3 - 1e-7, 0.3 - 1e-7, 0.6 - 1e-7),
        Point3::new(0.7 + 1e-7, 0.7 + 1e-7, 1.0 + 1e-7),
    );
    assert_eq!(feature.len(), 5, "expected 4 pocket walls + floor");

    let healed = defeature(&mut topo, pocketed, &feature).unwrap();
    assert_valid_with_volume(&topo, healed, 1.0);
}

/// Cap strategy, boss: removing a fused boss's five faces restores the base
/// block.
#[test]
fn boss_removed() {
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 2.0, 2.0, 1.0).unwrap();
    let boss = make_box(&mut topo, 0.5, 0.5, 0.4).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(0.75, 0.75, 1.0)).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, base, boss).unwrap();

    let feature = faces_within(
        &topo,
        fused,
        Point3::new(0.75 - 1e-7, 0.75 - 1e-7, 1.0 - 1e-7),
        Point3::new(1.25 + 1e-7, 1.25 + 1e-7, 1.4 + 1e-7),
    );
    assert_eq!(feature.len(), 5, "expected 4 boss walls + top");

    let healed = defeature(&mut topo, fused, &feature).unwrap();
    assert_valid_with_volume(&topo, healed, 4.0);
}

/// Cap strategy, cylindrical bore: the removed wall may be curved — only
/// kept faces must stay planar along the wound. Removing the bore wall
/// restores the plain box.
#[test]
fn cylindrical_bore_removed() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let drill = make_cylinder(&mut topo, 0.2, 2.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(0.5, 0.5, -0.5)).unwrap();
    let bored = boolean(&mut topo, BooleanOp::Cut, cube, drill).unwrap();

    let s = topo.solid(bored).unwrap();
    let bore_walls: Vec<FaceId> = topo
        .shell(s.outer_shell())
        .unwrap()
        .faces()
        .iter()
        .filter(|&&fid| {
            matches!(
                topo.face(fid).unwrap().surface(),
                FaceSurface::Cylinder { .. }
            )
        })
        .copied()
        .collect();
    assert!(!bore_walls.is_empty(), "expected a cylindrical bore wall");

    let healed = defeature(&mut topo, bored, &bore_walls).unwrap();
    assert_valid_with_volume(&topo, healed, 1.0);
}

/// Extend strategy: removing a chamfer face restores the sharp box corner
/// and its exact volume.
#[test]
fn chamfer_removed_restores_sharp_edge() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

    // The top +X edge: from (1,0,1) to (1,1,1).
    let s = topo.solid(cube).unwrap();
    let mut target: Vec<EdgeId> = Vec::new();
    for &fid in topo.shell(s.outer_shell()).unwrap().faces() {
        let f = topo.face(fid).unwrap();
        for oe in topo.wire(f.outer_wire()).unwrap().edges() {
            let e = topo.edge(oe.edge()).unwrap();
            let a = topo.vertex(e.start()).unwrap().point();
            let b = topo.vertex(e.end()).unwrap().point();
            let on_edge = |p: Point3| (p.x() - 1.0).abs() < 1e-9 && (p.z() - 1.0).abs() < 1e-9;
            if on_edge(a) && on_edge(b) && !target.contains(&oe.edge()) {
                target.push(oe.edge());
            }
        }
    }
    assert_eq!(target.len(), 1);

    let chamfered = chamfer(&mut topo, cube, &target, 0.2).unwrap();
    let expected_chamfered = 1.0 - 0.2 * 0.2 / 2.0;
    let vol = solid_volume(&topo, chamfered, 0.01).unwrap();
    assert!(
        ((vol - expected_chamfered) / expected_chamfered).abs() < 1e-9,
        "chamfer setup: expected {expected_chamfered}, got {vol}"
    );

    // The chamfer face is the one whose normal is the (1,0,1) diagonal.
    let diag = Vec3::new(1.0, 0.0, 1.0).normalize().unwrap();
    let chamfer_face = planar_faces(&topo, chamfered, diag, None);
    assert_eq!(chamfer_face.len(), 1);

    let healed = defeature(&mut topo, chamfered, &chamfer_face).unwrap();
    assert_valid_with_volume(&topo, healed, 1.0);
}

/// Empty selection and foreign faces are invalid input; removing so much
/// that fewer than four faces remain is refused too.
#[test]
fn invalid_selections_refused_typed() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    assert!(matches!(
        defeature(&mut topo, cube, &[]),
        Err(OperationsError::InvalidInput { .. })
    ));

    let mut topo = Topology::new();
    let a = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let b = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let b_wall = planar_faces(&topo, b, Vec3::new(1.0, 0.0, 0.0), None);
    assert!(matches!(
        defeature(&mut topo, a, &b_wall),
        Err(OperationsError::InvalidInput { .. })
    ));

    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let s = topo.solid(cube).unwrap();
    let faces = topo.shell(s.outer_shell()).unwrap().faces().to_vec();
    assert!(
        defeature(&mut topo, cube, &faces[0..3]).is_err(),
        "removing 3 of 6 faces must be refused"
    );
}

/// A cavity-bearing solid is refused by name.
#[test]
fn cavity_solid_refused_typed() {
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let void = make_box(&mut topo, 0.4, 0.4, 0.4).unwrap();
    transform_solid(&mut topo, void, &Mat4::translation(0.3, 0.3, 0.3)).unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, cube, void).unwrap();
    assert!(!topo.solid(hollow).unwrap().inner_shells().is_empty());

    let wall = planar_faces(&topo, hollow, Vec3::new(1.0, 0.0, 0.0), Some(1.0));
    let err = defeature(&mut topo, hollow, &wall).unwrap_err();
    assert!(
        matches!(&err, OperationsError::Unsupported { operation, .. } if *operation == "defeature"),
        "expected a typed refusal, got {err:?}"
    );
}

/// A wound that runs across a curved KEPT face is refused by name: removing
/// a cylindrical boss's top cap leaves the wound on the kept lateral wall,
/// which cannot be healed by plane extension.
#[test]
fn curved_kept_face_refused_typed() {
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 2.0, 2.0, 1.0).unwrap();
    let boss = make_cylinder(&mut topo, 0.4, 0.5).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(1.0, 1.0, 1.0)).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, base, boss).unwrap();

    let cap = planar_faces(&topo, fused, Vec3::new(0.0, 0.0, 1.0), Some(1.5));
    assert_eq!(cap.len(), 1, "expected the boss top cap at z=1.5");

    let err = defeature(&mut topo, fused, &cap).unwrap_err();
    assert!(
        matches!(&err, OperationsError::Unsupported { operation, .. } if *operation == "defeature"),
        "expected a typed refusal, got {err:?}"
    );
}

/// The same defeature in two fresh topologies yields bit-identical volume —
/// the operation is deterministic.
#[test]
fn defeature_is_deterministic() {
    let run = || {
        let mut topo = Topology::new();
        let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let cutter = make_box(&mut topo, 0.4, 0.4, 2.0).unwrap();
        transform_solid(&mut topo, cutter, &Mat4::translation(0.3, 0.3, -0.5)).unwrap();
        let holed = boolean(&mut topo, BooleanOp::Cut, cube, cutter).unwrap();
        let walls = faces_within(
            &topo,
            holed,
            Point3::new(0.3 - 1e-7, 0.3 - 1e-7, -1e-7),
            Point3::new(0.7 + 1e-7, 0.7 + 1e-7, 1.0 + 1e-7),
        );
        let healed = defeature(&mut topo, holed, &walls).unwrap();
        solid_volume(&topo, healed, 0.01).unwrap().to_bits()
    };
    assert_eq!(run(), run());
}

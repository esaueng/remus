//! `fix_shape` must not break the analytic solids it is handed.
//!
//! The default heal path — reachable from JS through the `heal` bindings — was
//! unusable on most real geometry, and destructive on the rest. Three defects
//! stacked, and each was masked by the one in front of it:
//!
//! 1. **Seam edges aborted the whole pipeline.** A face uses a seam edge twice,
//!    once in each sense, so `(edge, face)` does not name one p-curve. The
//!    registry refuses that by design (RFC 0002 fails closed rather than picking
//!    a side), but `fix_same_parameter_on_face` let the refusal escape and
//!    `fix_shape` aborts on the first error. Every cylinder, cone, revolve and
//!    most imported STEP carries a seam, so all of them came back
//!    `seam_pcurve_ambiguous`.
//! 2. **Behind that: circular caps measured zero and were deleted.** Face size
//!    was the bounding box of the wire's VERTEX positions, and a face bounded by
//!    one closed curve has `start == end`, so its box collapses to a point. With
//!    the seam abort gone, a plain cylinder healed from 3 faces to 1 — both caps
//!    removed, a third of the volume with them.
//! 3. **Behind that: correctly-oriented faces were flipped.** The shell
//!    orientation BFS compared raw wire senses without composing
//!    `Face::is_reversed`, so a reversed face read as traversing its shared edge
//!    the wrong way. A box with a cylindrical through-hole healed to volume
//!    1041.9 instead of 874.3; disabling `fix_orientation` alone restored it.
//!
//! Each case below is pinned to its measured volume and face count. The suite
//! did not catch any of this, because nothing ran `fix_shape` over ordinary
//! analytic primitives.

#![allow(clippy::unwrap_used, clippy::panic)]

use remus_heal::fix::{FixConfig, fix_shape};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

/// Heal a solid and assert it came back with the same volume and face count.
fn assert_heal_preserves(label: &str, topo: &mut Topology, solid: SolidId) {
    let before_volume = solid_volume(topo, solid, 0.001).unwrap();
    let before_faces = solid_faces(topo, solid).unwrap().len();

    let (healed, _report) = fix_shape(topo, solid, &FixConfig::default())
        .unwrap_or_else(|e| panic!("{label}: fix_shape must not refuse an ordinary solid: {e}"));

    let after_volume = solid_volume(topo, healed, 0.001).unwrap();
    let after_faces = solid_faces(topo, healed).unwrap().len();

    assert_eq!(
        after_faces, before_faces,
        "{label}: heal changed the face count {before_faces} -> {after_faces}"
    );
    assert!(
        (after_volume - before_volume).abs() <= 1e-6 * before_volume.abs().max(1.0),
        "{label}: heal changed the volume {before_volume} -> {after_volume}"
    );
}

/// A cylinder carries both defects at once: a seam edge on its wall, and two
/// circular caps each bounded by a single closed circle. Before the fix this
/// refused outright; with only the seam fix it returned 1 face and 523.6.
#[test]
fn heal_preserves_a_cylinder() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    assert_eq!(solid_faces(&topo, cylinder).unwrap().len(), 3);
    assert_heal_preserves("cylinder", &mut topo, cylinder);
}

#[test]
fn heal_preserves_a_cone() {
    let mut topo = Topology::new();
    let cone = make_cone(&mut topo, 6.0, 2.0, 12.0).unwrap();
    assert_heal_preserves("cone", &mut topo, cone);
}

#[test]
fn heal_preserves_a_sphere() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 3.0, 24).unwrap();
    assert_heal_preserves("sphere", &mut topo, sphere);
}

/// The orientation defect's own case: the hole's walls are reversed faces, and
/// flipping them inflated the volume to 1041.9 against a correct 874.3.
#[test]
fn heal_preserves_a_bored_block() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let drill = make_cylinder(&mut topo, 2.0, 20.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(5.0, 5.0, -5.0)).unwrap();
    let bored = boolean(&mut topo, BooleanOp::Cut, block, drill).unwrap();

    let expected = 1000.0 - std::f64::consts::PI * 4.0 * 10.0;
    let measured = solid_volume(&topo, bored, 0.001).unwrap();
    assert!(
        (measured - expected).abs() < 1e-6,
        "test setup: expected {expected}, got {measured}"
    );

    assert_heal_preserves("bored block", &mut topo, bored);
}

/// A cavity shell must survive too — heal walks inner shells, and the
/// orientation pass sees a reversed cavity boundary.
#[test]
fn heal_preserves_a_solid_with_a_cavity() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let void = make_sphere(&mut topo, 2.0, 24).unwrap();
    transform_solid(&mut topo, void, &Mat4::translation(5.0, 5.0, 5.0)).unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, block, void).unwrap();
    assert!(
        !topo.solid(hollow).unwrap().inner_shells().is_empty(),
        "test setup: expected a cavity shell"
    );
    assert_heal_preserves("cavity", &mut topo, hollow);
}

/// The face-size defect stated directly: a face bounded by ONE closed curve is
/// not degenerate, whatever its endpoints say. Measured through the analysis
/// that the small-face pass consults.
#[test]
fn a_face_bounded_by_one_closed_curve_is_not_degenerate() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let tolerance = remus_math::tolerance::Tolerance::new();

    for fid in solid_faces(&topo, cylinder).unwrap() {
        let analysis = remus_heal::analysis::face::analyze_face(&topo, fid, &tolerance).unwrap();
        assert!(
            analysis.bbox_diagonal > 1.0,
            "face {fid:?} measured {:.3e}: a cap bounded by one closed circle collapses to a \
             point when only its endpoints are read",
            analysis.bbox_diagonal
        );
        assert!(
            !analysis.is_degenerate,
            "face {fid:?} reported degenerate; the small-face pass removes those"
        );
    }
}

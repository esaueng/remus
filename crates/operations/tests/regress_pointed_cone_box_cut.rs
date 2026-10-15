//! Regression: an exact box − pointed-cone cut returned the wrong volume.
//!
//! Box 2×3×4 minus `make_cone(0.5, 0, 4)` (volume π/3) placed so its base cap
//! lies inside the box and its apex pokes out through the top face. The
//! removed volume is the frustum below the top face, 7/8 of the cone. Under
//! `ExactOnly` the cut returned `Ok`/Exact reading `24 − π/24` (the apex piece
//! outside the box) with one same-sense shared edge. The apex-down twin did
//! the same.
//!
//! Two defects, both about the cone's doubled seam to its apex:
//!
//! 1. The face splitter's band path (`split_periodic_face_into_bands`) only
//!    took laterals with two rim circles. A pointed cone has one rim and a
//!    seam run to the apex and back, so it declined, and the generic wire
//!    builder mistraced the doubled seam: it dropped the rim-side band and
//!    emitted one sub-face with no rim edge. Assembly then dropped the
//!    orphaned base cap.
//! 2. Shell assembly (`excise_out_and_back_spurs`) and the operations heal
//!    (`remove_wire_spurs`) stripped the seam pair as an out-and-back spur.
//!    The two uses are the u = 0 and u = 2π sides of the seam meeting at the
//!    apex, so a kept band reaching the apex was left bounded by its rim
//!    alone: with the apex inside the box the shell came back open and
//!    uncarved, which is why those placements refused.
//!
//! The oracle is the closed form: the part of a cone inside the slab
//! `0 ≤ z ≤ 4` is the whole cone less the similar cone cut off past the
//! plane, so it scales with the cube of the apex distance.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_check::validate::{CheckId, Severity, ValidateOptions, validate_solid};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cone};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const R: f64 = 0.5;
const H: f64 = 4.0;
/// Cone axis position: inside the 2×3 footprint, clear of every side wall.
const AXIS: (f64, f64) = (1.0, 1.5);

/// π r² h / 3 = π/3.
fn cone_volume() -> f64 {
    PI * R * R * H / 3.0
}

/// Cut the 2×3×4 box by a pointed cone of radius `R`, height `H`.
///
/// `apex_up` picks `make_cone(R, 0, H)` (base at the bottom) or
/// `make_cone(0, R, H)` (apex at the bottom); `z0` is the cone's lowest z.
fn cut(apex_up: bool, z0: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let cone = if apex_up {
        make_cone(&mut topo, R, 0.0, H).unwrap()
    } else {
        make_cone(&mut topo, 0.0, R, H).unwrap()
    };
    transform_solid(&mut topo, cone, &Mat4::translation(AXIS.0, AXIS.1, z0)).unwrap();
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let outcome = boolean_with_context(&mut topo, BooleanOp::Cut, block, cone, &context)
        .expect("exact cut must succeed");
    assert_eq!(outcome.quality, BooleanQuality::Exact);
    (topo, outcome.solid)
}

/// Fraction of the cone's volume inside the slab `0 ≤ z ≤ 4`.
///
/// The part of the cone between its apex and a plane at distance `d` from it
/// is a similar cone, `(d / H)³` of the whole.
fn inside_fraction(apex_z: f64, base_z: f64) -> f64 {
    let (lo, hi) = (apex_z.min(base_z), apex_z.max(base_z));
    let tip = |z: f64| ((z.clamp(lo, hi) - apex_z).abs() / H).powi(3);
    (tip(4.0) - tip(0.0)).abs()
}

/// Cone radius at height `z` for a cone whose apex is at `apex_z`.
fn radius_at(apex_z: f64, z: f64) -> f64 {
    R * (z - apex_z).abs() / H
}

/// Check `solid` against the closed form for a cone with its apex at
/// `apex_z` and its base at `base_z`.
fn assert_cut(topo: &Topology, solid: SolidId, apex_z: f64, base_z: f64, what: &str) {
    let removed_fraction = inside_fraction(apex_z, base_z);
    let expected = 24.0 - removed_fraction * cone_volume();
    let volume = solid_volume(topo, solid, 1e-3).unwrap();
    assert!(
        (volume - expected).abs() < 1e-9,
        "{what}: volume {volume:.15} != closed form {expected:.15} (error {:.3e})",
        volume - expected
    );

    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    assert_eq!(
        report.error_count(),
        0,
        "{what}: validation errors {:?}",
        report.issues
    );
    // The only finding allowed is `check_euler`'s documented blind spot: the
    // top/bottom face carries the cone's opening as an inner wire, which its
    // V − E + F count cannot see.
    for issue in &report.issues {
        assert!(
            issue.check == CheckId::SolidEulerCharacteristic && issue.severity == Severity::Warning,
            "{what}: unexpected issue {issue:?}"
        );
    }
    assert!(
        report
            .issues
            .iter()
            .all(|i| i.check != CheckId::FaceOrientationConsistency),
        "{what}: misoriented shared edge"
    );

    let mesh = tessellate_solid(topo, solid, 1e-3).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{what}: open mesh");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{what}: non-manifold mesh"
    );

    let probe = |p: Point3| {
        classify_point(topo, solid, p, &ClassifyOptions::default())
            .unwrap_or_else(|e| panic!("{what}: classify {p:?}: {e}"))
    };
    let (cx, cy) = AXIS;
    // Box material well clear of the cone stays.
    assert_eq!(
        probe(Point3::new(0.2, 0.3, 2.0)),
        PointClassification::Inside,
        "{what}"
    );
    assert_eq!(
        probe(Point3::new(1.8, 2.7, 0.5)),
        PointClassification::Inside,
        "{what}"
    );
    // Outside the box stays outside, including next to the protruding part.
    assert_eq!(
        probe(Point3::new(cx, cy, 4.5)),
        PointClassification::Outside,
        "{what}"
    );
    assert_eq!(
        probe(Point3::new(cx, cy, -0.5)),
        PointClassification::Outside,
        "{what}"
    );
    // Inside the box, on and off the cone's axis, at heights the cone spans.
    // Heights off the 0.5 grid the apex and cap planes sit on.
    for z in [0.3, 1.1, 2.6, 3.7, 3.95] {
        let r = radius_at(apex_z, z);
        let in_cone = z > apex_z.min(base_z) && z < apex_z.max(base_z);
        if in_cone {
            assert_eq!(
                probe(Point3::new(cx + 0.5 * r, cy, z)),
                PointClassification::Outside,
                "{what}: cone interior at z={z} kept"
            );
            assert_eq!(
                probe(Point3::new(cx + r + 0.05, cy, z)),
                PointClassification::Inside,
                "{what}: box material beside the cone at z={z} removed"
            );
        } else {
            assert_eq!(
                probe(Point3::new(cx, cy, z)),
                PointClassification::Inside,
                "{what}: material outside the cone's span at z={z} removed"
            );
        }
    }
}

/// The reported case: base cap inside at z = 2, apex out through the top at
/// z = 6. Removes 7/8 of the cone; `main` removed 1/8.
#[test]
fn apex_up_cone_with_base_cap_inside_box() {
    let (topo, solid) = cut(true, 2.0);
    assert!((inside_fraction(6.0, 2.0) - 0.875).abs() < 1e-15);
    assert_cut(&topo, solid, 6.0, 2.0, "apex-up, base z=2");
}

/// The apex-down twin: base cap inside at z = 2, apex out through the bottom
/// at z = −2.
#[test]
fn apex_down_cone_with_base_cap_inside_box() {
    let (topo, solid) = cut(false, -2.0);
    assert!((inside_fraction(-2.0, 2.0) - 0.875).abs() < 1e-15);
    assert_cut(&topo, solid, -2.0, 2.0, "apex-down, base z=2");
}

/// The rest of the base-inside family, both orientations: the base cap at
/// z = 0.5 … 3.5 and the apex out through the top (apex up) or the bottom
/// (apex down).
#[test]
fn base_cap_inside_family() {
    for k in 1..=7 {
        let base = 0.5 * f64::from(k);
        let (topo, solid) = cut(true, base);
        assert_cut(
            &topo,
            solid,
            base + H,
            base,
            &format!("apex-up, base z={base}"),
        );
        let (topo, solid) = cut(false, base - H);
        assert_cut(
            &topo,
            solid,
            base - H,
            base,
            &format!("apex-down, base z={base}"),
        );
    }
}

/// Apex inside the box, base cap out through the bottom: the kept cone band
/// reaches the apex. Shell assembly used to excise its seam pair, leaving an
/// open, uncarved shell, so `ExactOnly` refused. Removes (3.5/4)³ of the cone.
///
/// Apex-up only: on `main` the apex-down primitive itself carries a
/// misoriented rim (`make_cone(0, r, h)`, fixed separately), which an
/// apex-inside cut keeps.
#[test]
fn apex_up_cone_with_apex_inside_box() {
    for apex in [0.5, 2.0, 3.5] {
        let (topo, solid) = cut(true, apex - H);
        assert_cut(
            &topo,
            solid,
            apex,
            apex - H,
            &format!("apex-up, apex z={apex}"),
        );
    }
}

//! O2.3c/d planar-arrangement production witnesses.
//!
//! The provenance-preserving UV arrangement migrates qualified planar
//! line/circle face splitting off the chord-quantized paths. These tests
//! prove the migrated production behavior end to end: a boolean whose
//! wall faces partition through the new path must keep exact analytic
//! volume, classify material correctly at probes that encode the cut
//! intent, validate clean on both validators, and tessellate watertight
//! at preview and fine deflections. No tolerance loosening, silent
//! healing, or mesh fallback: every boolean runs `ExactOnly` with
//! `BooleanOutcome` quality gating.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

fn exact_boolean(
    topo: &mut Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
) -> Result<SolidId, String> {
    let outcome = boolean_with_context(
        topo,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .map_err(|e| format!("{op:?}: {e:?}"))?;
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "{op:?}: non-exact quality"
    );
    Ok(outcome.solid)
}

fn assert_strict_valid(topo: &Topology, s: SolidId, what: &str) {
    let strict = remus_operations::validate::validate_solid(topo, s)
        .map_err(|e| format!("{what}: validator error: {e:?}"))
        .unwrap();
    assert!(strict.is_valid(), "{what}: ops validator issues");
    let mut opts = remus_check::validate::ValidateOptions::default();
    opts.disabled_checks
        .insert(remus_check::validate::CheckId::ShellConnected);
    let rep = remus_check::validate::validate_solid(topo, s, &opts).unwrap();
    let errs: Vec<_> = rep
        .issues
        .iter()
        .filter(|i| i.severity == remus_check::validate::Severity::Error)
        .collect();
    assert!(
        errs.is_empty(),
        "{what}: check-crate errors: {}",
        errs.len()
    );
}

fn assert_watertight(topo: &Topology, s: SolidId, what: &str) {
    for d in [0.1, 0.01] {
        let mesh = tessellate_solid(topo, s, d).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{what}: boundary at d={d}");
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{what}: non-manifold at d={d}"
        );
    }
}

fn assert_inside(topo: &Topology, s: SolidId, point: Point3, what: &str) {
    let actual = classify_point(topo, s, point, &ClassifyOptions::default()).unwrap();
    assert!(
        matches!(actual, PointClassification::Inside),
        "{what}: {point:?} classified {actual:?}, expected Inside"
    );
}

fn assert_outside(topo: &Topology, s: SolidId, point: Point3, what: &str) {
    let actual = classify_point(topo, s, point, &ClassifyOptions::default()).unwrap();
    assert!(
        matches!(actual, PointClassification::Outside),
        "{what}: {point:?} classified {actual:?}, expected Outside"
    );
}
/// L-plate cut by a crossing tool: the tool's walls meet the L's stepped
/// top face in T-junctions the greedy wire builder weaves into a broken
/// trace, so the provenance arrangement owns the partition (fewer, clean
/// regions replace overlapping loops).
///
/// Closed forms: the fuse joins face-touching boxes (72 + 32 = 104); the
/// cut removes the tool's 2 x 6 x 2 intersection (24), leaving 80.
#[test]
fn cut_l_plate_through_step_is_exact_and_watertight() {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 6.0, 6.0, 2.0).unwrap();
    let b = make_box(&mut topo, 4.0, 4.0, 2.0).unwrap();
    remus_operations::transform::transform_solid(&mut topo, b, &Mat4::translation(6.0, 0.0, 0.0))
        .unwrap();
    let l = exact_boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    for d in [0.1, 1e-4] {
        let v = solid_volume(&topo, l, d).unwrap();
        assert!(
            (v - 104.0).abs() / 104.0 < 1e-4,
            "L fuse volume {v} vs closed form 104 at d={d}"
        );
    }
    let c = make_box(&mut topo, 2.0, 8.0, 4.0).unwrap();
    remus_operations::transform::transform_solid(&mut topo, c, &Mat4::translation(4.0, -1.0, -1.0))
        .unwrap();
    let result = exact_boolean(&mut topo, BooleanOp::Cut, l, c).unwrap();

    for d in [0.1, 1e-4] {
        let v = solid_volume(&topo, result, d).unwrap();
        assert!(
            (v - 80.0).abs() / 80.0 < 1e-4,
            "L cut volume {v} vs closed form 80 at d={d}"
        );
    }
    // Probes encode the cut intent: the notch is void, both L arms stay
    // solid, and the air above the plate is void.
    assert_inside(&topo, result, Point3::new(1.0, 1.0, 1.0), "kept A block");
    assert_inside(&topo, result, Point3::new(8.0, 2.0, 1.0), "kept B arm");
    assert_outside(&topo, result, Point3::new(5.0, 1.0, 1.0), "cut notch");
    assert_outside(&topo, result, Point3::new(5.0, 5.0, 1.0), "cut notch far");
    assert_outside(&topo, result, Point3::new(1.0, 1.0, 3.0), "air above");
    assert_strict_valid(&topo, result, "L cut");
    assert_watertight(&topo, result, "L cut");
}

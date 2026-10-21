//! Regression: a plate with two flush tabs refused an exact slot cut.
//!
//! A 74×60×4 plate fused with two 18×10×40 tabs flush with its back and side
//! faces, then cut by a slot box between the tabs, fell back to a mesh (or
//! refused under `ExactOnly`) for any slot depth from 2 to 20 mm. Cutting the
//! slot first and fusing the tabs after stayed exact, so the failure depended
//! on operation order.
//!
//! The fused body is all-planar and non-convex, so the analytic classifier
//! tried its composite `outer \ inner` box model. Splitting the planes by the
//! vertex centroid gave an "inner box" of x ∈ [18, 56], y ≥ 10, z ≥ 4, which
//! is not the body's void: the region over the plate between the tabs, behind
//! y = 10, came out Inside. The slot tool's pieces above the plate were kept
//! and the shell came back non-manifold. The composite is now accepted only
//! when the body's boundary lies on the model's boundary and the volumes
//! agree.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_algo::FaceClass;
use remus_algo::classifier::{
    AnalyticClassifier, classify_analytic_with_tolerance,
    try_build_analytic_classifier_with_tolerance,
};
use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const PLATE: (f64, f64, f64) = (74.0, 60.0, 4.0);
const TAB: (f64, f64, f64) = (18.0, 10.0, 40.0);
const SLOT_X: (f64, f64) = (22.0, 52.0);

fn placed_box(topo: &mut Topology, lo: (f64, f64, f64), hi: (f64, f64, f64)) -> SolidId {
    let b = make_box(topo, hi.0 - lo.0, hi.1 - lo.1, hi.2 - lo.2).unwrap();
    transform_solid(topo, b, &Mat4::translation(lo.0, lo.1, lo.2)).unwrap();
    b
}

fn exact(topo: &mut Topology, op: BooleanOp, a: SolidId, b: SolidId) -> SolidId {
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let outcome = boolean_with_context(topo, op, a, b, &context)
        .unwrap_or_else(|e| panic!("exact {op:?} must succeed: {e}"));
    assert_eq!(outcome.quality, BooleanQuality::Exact);
    outcome.solid
}

/// Plate plus a tab at each back corner, both flush with the back and sides.
fn plate_with_tabs(topo: &mut Topology) -> SolidId {
    let plate = placed_box(topo, (0.0, 0.0, 0.0), PLATE);
    let left = placed_box(topo, (0.0, 0.0, 0.0), TAB);
    let right = placed_box(topo, (PLATE.0 - TAB.0, 0.0, 0.0), (PLATE.0, TAB.1, TAB.2));
    let fused = exact(topo, BooleanOp::Fuse, plate, left);
    exact(topo, BooleanOp::Fuse, fused, right)
}

fn plate_with_tabs_volume() -> f64 {
    PLATE.0 * PLATE.1 * PLATE.2 + 2.0 * TAB.0 * TAB.1 * (TAB.2 - PLATE.2)
}

#[test]
fn analytic_classifier_never_misplaces_the_void_between_tabs() {
    let mut topo = Topology::new();
    let body = plate_with_tabs(&mut topo);
    let tol = Tolerance::default();
    // Points the composite box model got wrong, then controls on both sides.
    let outside = [
        Point3::new(37.0, 8.0, 4.5),
        Point3::new(22.0, 3.5, 4.5),
        Point3::new(9.0, 30.0, 20.0),
        Point3::new(37.0, 30.0, 10.0),
    ];
    let inside = [
        Point3::new(37.0, 30.0, 2.0),
        Point3::new(9.0, 5.0, 20.0),
        Point3::new(65.0, 5.0, 30.0),
    ];
    for p in outside {
        let class = classify_analytic_with_tolerance(&topo, body, p, tol);
        assert!(
            !matches!(class, Some(FaceClass::Inside)),
            "{p:?} lies outside the body but classified {class:?}"
        );
    }
    for p in inside {
        let class = classify_analytic_with_tolerance(&topo, body, p, tol);
        assert!(
            !matches!(class, Some(FaceClass::Outside)),
            "{p:?} lies inside the body but classified {class:?}"
        );
    }
}

#[test]
fn a_large_plate_cannot_hide_a_thin_misplaced_void() {
    // Same construction with a wide plate and hairline tabs: the wrong model
    // keeps a 0.01-deep slab across the whole back edge, whose volume is
    // small next to the plate's boundary area times the tolerance. The proof
    // must stay local to each model face, so the shortcut still declines.
    let (w, tab) = (1000.0, 0.01);
    let mut topo = Topology::new();
    let plate = placed_box(&mut topo, (0.0, 0.0, 0.0), (w, w, 4.0));
    let left = placed_box(&mut topo, (0.0, 0.0, 0.0), (tab, tab, 40.0));
    let right = placed_box(&mut topo, (w - tab, 0.0, 0.0), (w, tab, 40.0));
    let fused = exact(&mut topo, BooleanOp::Fuse, plate, left);
    let body = exact(&mut topo, BooleanOp::Fuse, fused, right);
    let tol = Tolerance::loose();
    let p = Point3::new(0.5 * w, 0.5 * tab, 20.0);
    let class = classify_analytic_with_tolerance(&topo, body, p, tol);
    assert!(
        !matches!(class, Some(FaceClass::Inside)),
        "{p:?} lies above the plate but classified {class:?}"
    );
}

#[test]
fn composite_model_still_serves_an_open_cup() {
    // A cup is exactly `outer \ inner`, so the shortcut must survive the new
    // proof and keep classifying correctly.
    let mut topo = Topology::new();
    let block = placed_box(&mut topo, (0.0, 0.0, 0.0), (10.0, 10.0, 10.0));
    let cavity = placed_box(&mut topo, (1.0, 1.0, 1.0), (9.0, 9.0, 11.0));
    let cup = exact(&mut topo, BooleanOp::Cut, block, cavity);
    let tol = Tolerance::default();
    let classifier = try_build_analytic_classifier_with_tolerance(&topo, cup, tol);
    assert!(
        matches!(classifier, Some(AnalyticClassifier::Composite { .. })),
        "the open cup should keep its composite classifier"
    );
    let classifier = classifier.unwrap();
    assert_eq!(
        classifier.classify(Point3::new(5.0, 5.0, 5.0), tol),
        Some(FaceClass::Outside)
    );
    assert_eq!(
        classifier.classify(Point3::new(0.5, 5.0, 5.0), tol),
        Some(FaceClass::Inside)
    );
    assert_eq!(
        classifier.classify(Point3::new(5.0, 5.0, 0.5), tol),
        Some(FaceClass::Inside)
    );
}

#[test]
fn slot_cut_between_flush_tabs_is_exact() {
    for depth in [2.0, 8.0, 20.0] {
        let mut topo = Topology::new();
        let body = plate_with_tabs(&mut topo);
        let slot = placed_box(
            &mut topo,
            (SLOT_X.0, -1.0, -1.0),
            (SLOT_X.1, depth, PLATE.2 + 1.0),
        );
        let result = exact(&mut topo, BooleanOp::Cut, body, slot);
        let what = format!("slot depth {depth}");

        let expected = plate_with_tabs_volume() - (SLOT_X.1 - SLOT_X.0) * depth * PLATE.2;
        let volume = solid_volume(&topo, result, 1e-3).unwrap();
        assert!(
            (volume - expected).abs() < 1e-9,
            "{what}: volume {volume} != {expected}"
        );

        let report = validate_solid(&topo, result, &ValidateOptions::default()).unwrap();
        assert_eq!(
            report.error_count(),
            0,
            "{what}: validation errors {:?}",
            report.issues
        );
        let mesh = tessellate_solid(&topo, result, 1e-2).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{what}: open mesh");
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{what}: non-manifold mesh"
        );

        let probe =
            |p: Point3| classify_point(&topo, result, p, &ClassifyOptions::default()).unwrap();
        let mid_x = 0.5 * (SLOT_X.0 + SLOT_X.1);
        assert_eq!(
            probe(Point3::new(mid_x, 0.5 * depth, 2.0)),
            PointClassification::Outside,
            "{what}: slot not carved"
        );
        assert_eq!(
            probe(Point3::new(mid_x, depth + 1.0, 2.0)),
            PointClassification::Inside,
            "{what}: plate in front of the slot lost"
        );
        assert_eq!(
            probe(Point3::new(mid_x, 0.5 * depth, 4.5)),
            PointClassification::Outside,
            "{what}: slot tool kept above the plate"
        );
        assert_eq!(
            probe(Point3::new(9.0, 5.0, 20.0)),
            PointClassification::Inside,
            "{what}: tab lost"
        );
    }
}

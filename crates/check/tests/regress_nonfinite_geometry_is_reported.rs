//! Validation must not pass a shape carrying NaN or infinite geometry.
//!
//! Every geometric check in `validate` decides by comparing a measured
//! deviation against a tolerance, and a comparison against NaN is always
//! false. Before `CheckId::GeometryFinite` existed, a solid with a poisoned
//! vertex therefore reported *zero* issues: the vertex-on-curve, degenerate,
//! and range checks all measured NaN and all concluded "within tolerance".
//! Downstream, that shape measures, exports, and re-imports as if sound.
//!
//! The tests pin both directions: a clean cube reports no finiteness issue,
//! and each poison vector is reported as an `Error`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::validate::{CheckId, Severity, ValidateOptions, validate_solid};
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::test_utils::make_unit_cube_manifold;

fn finiteness_issues(topo: &Topology, solid: remus_topology::solid::SolidId) -> Vec<String> {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    report
        .issues
        .iter()
        .filter(|i| i.check == CheckId::GeometryFinite)
        .map(|i| {
            assert_eq!(i.severity, Severity::Error, "finiteness issues are errors");
            i.description.clone()
        })
        .collect()
}

#[test]
fn clean_cube_reports_no_finiteness_issue() {
    let mut topo = Topology::new();
    let solid = make_unit_cube_manifold(&mut topo);
    assert!(finiteness_issues(&topo, solid).is_empty());
}

#[test]
fn nan_vertex_position_is_reported() {
    for poison in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);

        // Poison one vertex in place, exactly as a bad import or a NaN-bearing
        // transform would.
        let vid = topo.vertex_id_from_index(0).expect("cube has vertices");
        topo.vertex_mut(vid)
            .unwrap()
            .set_point(Point3::new(poison, 0.0, 0.0));

        let issues = finiteness_issues(&topo, solid);
        assert!(
            !issues.is_empty(),
            "poison {poison} went unreported; issues: {issues:?}"
        );
    }
}

/// Pins the exact failure mode the check exists to close: with
/// `GeometryFinite` disabled, every remaining check measures NaN, compares it
/// against a tolerance, and concludes the shape is fine.
#[test]
fn without_the_finiteness_check_a_nan_solid_reports_clean() {
    let mut topo = Topology::new();
    let solid = make_unit_cube_manifold(&mut topo);
    let vid = topo.vertex_id_from_index(0).expect("cube has vertices");
    topo.vertex_mut(vid)
        .unwrap()
        .set_point(Point3::new(f64::NAN, 0.0, 0.0));

    let mut options = ValidateOptions::default();
    options.disabled_checks.insert(CheckId::GeometryFinite);
    let report = validate_solid(&topo, solid, &options).unwrap();
    assert!(
        report.is_valid(),
        "pre-change behaviour changed; the NaN is now caught by another \
         check and this pin needs rewriting: {:?}",
        report.issues
    );
}

#[test]
fn nonfinite_geometry_makes_the_report_invalid() {
    let mut topo = Topology::new();
    let solid = make_unit_cube_manifold(&mut topo);
    let vid = topo.vertex_id_from_index(0).expect("cube has vertices");
    topo.vertex_mut(vid)
        .unwrap()
        .set_point(Point3::new(f64::NAN, 0.0, 0.0));

    let report = validate_solid(&topo, solid, &ValidateOptions::default()).unwrap();
    assert!(
        !report.is_valid(),
        "a NaN-bearing solid must not validate clean"
    );
}

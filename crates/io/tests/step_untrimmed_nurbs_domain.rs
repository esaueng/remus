//! Regression coverage for untrimmed polynomial NURBS whose endpoint domain
//! is uniquely witnessed without a monotone world-axis coordinate.
//!
//! The MAMBO B1 triage fixture pins the O1.1d class where the scalar-witness
//! certificate admits the polygon but the witness bisection converges to the
//! wrong lobe: bare-carrier intersection seams whose endpoints sit strictly
//! inside their carriers. Projection-based recovery imports the exact-foot
//! ones (#241/#243/#244/#245 — the scalar witness refuses them); the
//! off-carrier ones (#253/#258, 6.2e-5/1.1e-4 off) still fail closed.

#![allow(clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_io::IoError;
use remus_io::step::{StepImportDiagnostic, read_step, read_step_with_report, write_step};
use remus_math::diagnostic::ToDiagnostic;
use remus_operations::measure::{solid_bounding_box, solid_volume};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const SYNTHETIC_SHAPR_STYLE: &str = include_str!("data/shapr_untrimmed_nurbs_domain.step");
/// MAMBO Basic B1 (Apache-2.0), reduced to its shape spine: the untrimmed
/// cylinder-intersection seam carriers keep their file bytes verbatim; only
/// the administrative header entities were dropped. Pinned by the
/// open-kernel gauntlet smoke manifest (`mambo-basic-b1`).
const MAMBO_B1_UNTRIMMED_NURBS: &str = include_str!("data/mambo_b1_untrimmed_nurbs.step");

#[test]
fn uniquely_witnessed_untrimmed_nurbs_domain_imports() {
    let mut topology = Topology::new();
    let result = read_step_with_report(SYNTHETIC_SHAPR_STYLE, &mut topology)
        .expect("uniquely witnessed untrimmed NURBS domain");
    let solids = result.solids();

    assert_eq!(solids.len(), 1);
    assert_eq!(result.diagnostics().len(), 2);
    for diagnostic in result.diagnostics() {
        let StepImportDiagnostic::UntrimmedNurbsDomainRecovered {
            start_parameter,
            end_parameter,
            endpoint_residual_mm,
            recovery_tolerance_cap_mm,
            ..
        } = diagnostic
        else {
            panic!("unexpected STEP import diagnostic")
        };
        // The fixture vertices sit 5e-7 off the carrier, so projection
        // returns the closest feet, which sit within the foot-conditioning
        // radius (~3.5e-4 here) of the nominal 0.1/0.9 parameters — not on
        // them to roundoff scale. The residual and cap assertions below pin
        // the tolerance-scale contract that actually matters.
        assert!((*start_parameter - 0.1).abs() < 5.0e-4);
        assert!((*end_parameter - 0.9).abs() < 5.0e-4);
        assert!(*endpoint_residual_mm <= *recovery_tolerance_cap_mm);
        assert!((*recovery_tolerance_cap_mm - 1.0e-6).abs() < 1.0e-16);
        assert_eq!(
            diagnostic.diagnostic().code(),
            "step_untrimmed_nurbs_domain_recovered"
        );
    }

    let nurbs_edges: Vec<_> = solid_edges(&topology, solids[0])
        .expect("solid edges")
        .into_iter()
        .filter(|edge_id| {
            matches!(
                topology.edge(*edge_id).expect("edge").curve(),
                EdgeCurve::NurbsCurve(_)
            )
        })
        .collect();
    assert_eq!(nurbs_edges.len(), 2);
    for edge_id in nurbs_edges {
        let edge = topology.edge(edge_id).expect("NURBS edge");
        let (start, end) = edge.strict_domain().expect("stored edge authority");
        assert!((start - 0.1).abs() < 5.0e-4);
        assert!((end - 0.9).abs() < 5.0e-4);
        assert!(
            edge.tolerance().expect("edge-local tolerance") <= 1.0e-6,
            "edge-local tolerance must stay within the recovery cap"
        );
    }

    assert_valid(&topology, solids[0]);
    assert_round_trip_invariants(&topology, solids[0]);
}

#[test]
fn self_intersecting_untrimmed_nurbs_remains_refused() {
    let step = SYNTHETIC_SHAPR_STYLE
        .replace(
            "#18 = CARTESIAN_POINT('', (2.00000000000000000E0, -1.00000000000000000E0, 0.));",
            "#18 = CARTESIAN_POINT('', (2., 2., 0.));",
        )
        .replace(
            "#19 = CARTESIAN_POINT('', (1.00000000000000000E0, 2.00000000000000000E0, 0.));",
            "#19 = CARTESIAN_POINT('', (0., 2., 0.));",
        )
        .replace(
            "#20 = CARTESIAN_POINT('', (3.00000000000000000E0, 3.00000000000000000E0, 0.));",
            "#20 = CARTESIAN_POINT('', (2., 0., 0.));",
        )
        .replace(
            "#21 = B_SPLINE_CURVE_WITH_KNOTS('', 3, (#17, #18, #19, #20), .UNSPECIFIED., .F., .F., (4, 4), (0., 1.00000000000000000E0), .UNSPECIFIED.);",
            "#21 = B_SPLINE_CURVE_WITH_KNOTS('', 1, (#17, #18, #19, #20), .UNSPECIFIED., .F., .F., (2, 1, 1, 2), (0., 1., 2., 3.), .UNSPECIFIED.);",
        );

    assert_untrimmed_domain_refused(&step);
}

#[test]
fn folded_untrimmed_nurbs_without_a_unique_witness_remains_refused() {
    let step = SYNTHETIC_SHAPR_STYLE
        .replace(
            "#18 = CARTESIAN_POINT('', (2.00000000000000000E0, -1.00000000000000000E0, 0.));",
            "#18 = CARTESIAN_POINT('', (2., 0., 0.));",
        )
        .replace(
            "#19 = CARTESIAN_POINT('', (1.00000000000000000E0, 2.00000000000000000E0, 0.));",
            "#19 = CARTESIAN_POINT('', (1., 1., 0.));",
        )
        .replace(
            "#20 = CARTESIAN_POINT('', (3.00000000000000000E0, 3.00000000000000000E0, 0.));",
            "#20 = CARTESIAN_POINT('', (3., 1., 0.));",
        )
        .replace(
            "#21 = B_SPLINE_CURVE_WITH_KNOTS('', 3, (#17, #18, #19, #20), .UNSPECIFIED., .F., .F., (4, 4), (0., 1.00000000000000000E0), .UNSPECIFIED.);",
            "#21 = B_SPLINE_CURVE_WITH_KNOTS('', 1, (#17, #18, #19, #20), .UNSPECIFIED., .F., .F., (2, 1, 1, 2), (0., 1., 2., 3.), .UNSPECIFIED.);",
        );

    assert_untrimmed_domain_refused(&step);
}

#[test]
fn excessive_unique_domain_endpoint_error_is_refused_at_the_local_cap() {
    let step = SYNTHETIC_SHAPR_STYLE.replace(
        "#13 = CARTESIAN_POINT('', (5.16000500000000018E-1, -1.86000499999999997E-1, 0.));",
        "#13 = CARTESIAN_POINT('', (5.16500000000000000E-1, -1.86500000000000000E-1, 0.));",
    );
    let error = read_step(&step, &mut Topology::new()).expect_err("excessive healing");
    assert!(
        error
            .to_string()
            .contains("misses its carrier by 4.047427e-4 mm (local recovery cap 1.000000e-6 mm)"),
        "unexpected error: {error}"
    );
}

#[test]
fn model_uncertainty_cannot_raise_the_absolute_local_recovery_cap() {
    // The recovery ceiling is absolute (1e-6), not derived from the model's
    // declared uncertainty: a 1e-2 model heals nothing more than a 1e-6 one.
    let step = SYNTHETIC_SHAPR_STYLE
        .replace(
            "LENGTH_MEASURE(9.99999999999999955E-7)",
            "LENGTH_MEASURE(1.E-2)",
        )
        .replace(
            "#13 = CARTESIAN_POINT('', (5.16000500000000018E-1, -1.86000499999999997E-1, 0.));",
            "#13 = CARTESIAN_POINT('', (5.16500000000000000E-1, -1.86500000000000000E-1, 0.));",
        );
    let error = read_step(&step, &mut Topology::new()).expect_err("absolute healing cap");
    assert!(
        error
            .to_string()
            .contains("misses its carrier by 4.047427e-4 mm (local recovery cap 1.000000e-6 mm)"),
        "unexpected error: {error}"
    );
}

#[test]
fn non_finite_untrimmed_nurbs_input_is_refused() {
    let step = SYNTHETIC_SHAPR_STYLE.replace(
        "#18 = CARTESIAN_POINT('', (2.00000000000000000E0, -1.00000000000000000E0, 0.));",
        "#18 = CARTESIAN_POINT('', (1.E400, -1., 0.));",
    );
    let error = read_step(&step, &mut Topology::new()).expect_err("non-finite control point");
    assert!(matches!(error, IoError::ParseError { .. }));
}

#[test]
fn mambo_b1_exact_foot_seams_reach_projected_recovery() {
    // Edges #241/#243/#244/#245 ride bare untrimmed intersection carriers
    // whose endpoints sit strictly inside the carrier domain with exact
    // feet: the scalar witness refuses them (wrong-lobe bisection) while
    // the projected path recovers them. The full model still fails closed
    // on edges #253/#258 (vertices 6.2e-5/1.1e-4 off their carriers), so
    // this test pins the per-edge behaviour: the exact-foot edges must get
    // PAST the witness refusal and reach the projected recovery, and the
    // first failure must be the off-carrier #253 — never #241.
    let mut topology = Topology::new();
    let result = read_step_with_report(MAMBO_B1_UNTRIMMED_NURBS, &mut topology);
    let error = result.expect_err("off-carrier seam vertices must still refuse");
    assert!(
        error.to_string().contains("EDGE_CURVE #253"),
        "unexpected error: {error}"
    );
    assert!(
        error
            .to_string()
            .contains("misses its carrier by 6.164947e-5 mm (local recovery cap 1.000000e-6 mm)"),
        "unexpected error: {error}"
    );
}

#[test]
fn mambo_b1_off_carrier_vertex_still_refuses() {
    // Push one exact-foot seam vertex off its carrier past the absolute
    // recovery ceiling: the import must fail closed with the stablecoded
    // refusal, not heal the vertex onto the wrong lobe.
    let step = MAMBO_B1_UNTRIMMED_NURBS.replace(
        "#387=CARTESIAN_POINT('',(5.0,0.0,5.0));",
        "#387=CARTESIAN_POINT('',(5.0,0.0,5.5));",
    );
    assert!(step.contains("(5.0,0.0,5.5)"), "fixture edit must apply");
    let error = read_step(&step, &mut Topology::new()).expect_err("off-carrier vertex");
    assert!(
        error.to_string().contains("misses its carrier"),
        "unexpected error: {error}"
    );
}

fn assert_untrimmed_domain_refused(step: &str) {
    let error = read_step(step, &mut Topology::new()).expect_err("ambiguous NURBS domain");
    let message = error.to_string();
    // Both refusals are the same fail-closed gate: no unique branch, or no
    // branch within the recovery cap. The message names which one fired.
    assert!(
        message.contains("do not uniquely establish") || message.contains("misses its carrier"),
        "unexpected error: {error}"
    );
}

fn assert_valid(topology: &Topology, solid: SolidId) {
    let shell = topology
        .shell(topology.solid(solid).expect("solid").outer_shell())
        .expect("outer shell");
    validate_shell_closed(shell, topology).expect("closed synthetic shell");
    let validation = validate_solid(topology, solid).expect("validate synthetic solid");
    assert!(
        validation.is_valid(),
        "synthetic solid validation issues: {:?}",
        validation.issues
    );
}

fn assert_round_trip_invariants(topology: &Topology, solid: SolidId) {
    let source_faces = surface_orientation_census(topology, solid);
    let source_edges = solid_edges(topology, solid).expect("source edges").len();
    let source_bounds = solid_bounding_box(topology, solid).expect("source bounds");
    let source_volume = solid_volume(topology, solid, 0.01).expect("source volume");
    assert!(
        (source_volume - 1.062_909_27).abs() < 5.0e-5,
        "synthetic prism volume {source_volume:.12} strayed from its analytic value"
    );

    let exported = write_step(topology, &[solid]).expect("export recovered authority");
    assert!(exported.contains("TRIMMED_CURVE"));
    let mut round_topology = Topology::new();
    let round_result =
        read_step_with_report(&exported, &mut round_topology).expect("re-import exported STEP");
    assert!(
        round_result.diagnostics().is_empty(),
        "explicit exported authority must not require healing"
    );
    let [round_solid] = round_result.solids() else {
        panic!("round trip must contain exactly one solid")
    };
    assert_valid(&round_topology, *round_solid);
    assert_eq!(
        solid_edges(&round_topology, *round_solid)
            .expect("round edges")
            .len(),
        source_edges
    );
    assert_eq!(
        surface_orientation_census(&round_topology, *round_solid),
        source_faces
    );
    let round_bounds = solid_bounding_box(&round_topology, *round_solid).expect("round bounds");
    assert!((round_bounds.min - source_bounds.min).length() < 1.0e-10);
    assert!((round_bounds.max - source_bounds.max).length() < 1.0e-10);
    let round_volume = solid_volume(&round_topology, *round_solid, 0.01).expect("round volume");
    assert!((round_volume - source_volume).abs() < source_volume * 1.0e-10);
}

fn surface_orientation_census(
    topology: &Topology,
    solid: SolidId,
) -> BTreeMap<(&'static str, bool), usize> {
    let mut census = BTreeMap::new();
    for face_id in solid_faces(topology, solid).expect("solid faces") {
        let face = topology.face(face_id).expect("face");
        *census
            .entry((face.surface().type_tag(), face.is_reversed()))
            .or_default() += 1;
    }
    census
}

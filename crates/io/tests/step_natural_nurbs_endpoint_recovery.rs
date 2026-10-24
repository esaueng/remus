//! Issue #1285: the authored cubic box repro and nearby recovery refusals.

#![allow(clippy::expect_used, clippy::float_cmp, clippy::panic)]

use remus_io::step::{StepImportDiagnostic, read_step_with_report, write_step};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::solid_edges;

const BOX: &str = include_str!("data/scale_platform_v1_linear_spline_edge.step");

fn repro(width: f64, first: [f64; 3], last: [f64; 3]) -> String {
    let point = |id, [x, y, z]: [f64; 3]| {
        format!("#{id} = CARTESIAN_POINT('', ({x:.17e}, {y:.17e}, {z:.17e}));")
    };
    BOX.replace(
        "#45 = CARTESIAN_POINT('', (3.20000000000000000E2, 1.10000000000000008E-3, -1.19000000000000001E-3));",
        "#45 = CARTESIAN_POINT('', (3.20000000000000000E2, 0., 0.));",
    )
    .replace("LENGTH_MEASURE(1.E-07)", "LENGTH_MEASURE(1.E-05)")
    .replace(
        "#200 = CARTESIAN_POINT('', (0., 0., 0.));",
        &point(200, first),
    )
    .replace(
        "#201 = CARTESIAN_POINT('', (3.20000000000000000E2, 0., 0.));",
        &point(201, [width / 3.0, 0.0, 0.0]),
    )
    .replace(
        "#202 = B_SPLINE_CURVE_WITH_KNOTS('', 1, (#200, #201), .UNSPECIFIED., .F., .F., (2, 2), (0.023809524, 0.976190476), .UNSPECIFIED.);",
        &format!(
            "{}\n{}\n#202 = B_SPLINE_CURVE_WITH_KNOTS('', 3, (#200, #201, #203, #204), .UNSPECIFIED., .F., .F., (4, 4), (0., 1.), .UNSPECIFIED.);",
            point(203, [2.0 * width / 3.0, 0.0, 0.0]),
            point(204, last),
        ),
    )
    .replace("3.20000000000000000E2", &format!("{width:.17e}"))
}

#[test]
fn issue_1285_natural_endpoint_misses_import_with_bounded_edge_tolerance() {
    for (start_miss, end_miss) in [
        (0.0, 0.0),
        (3.227_018e-5, 0.0),
        (8.928e-5, 0.0),
        (0.0, 8.928e-5),
        (3.227_018e-5, 8.928e-5),
        (1.0e-4, 0.0),
    ] {
        let step = repro(320.0, [start_miss, 0.0, 0.0], [320.0 - end_miss, 0.0, 0.0]);
        let mut topology = Topology::new();
        let result = read_step_with_report(&step, &mut topology).expect("bounded natural ends");
        assert_eq!(result.solids().len(), 1);
        let solid = result.solids()[0];
        assert!(
            validate_solid(&topology, solid)
                .expect("validate box")
                .is_valid()
        );
        let edges = solid_edges(&topology, solid).expect("box edges");
        assert_eq!(edges.len(), 12);
        let edge = edges
            .iter()
            .map(|id| topology.edge(*id).expect("edge"))
            .find(|edge| matches!(edge.curve(), EdgeCurve::NurbsCurve(_)))
            .expect("cubic edge");
        assert_eq!(edge.strict_domain().expect("natural domain"), (0.0, 1.0));
        let EdgeCurve::NurbsCurve(curve) = edge.curve() else {
            panic!("cubic carrier")
        };
        assert!((curve.control_points()[0].x() - start_miss).abs() < 1e-12);
        assert!((curve.control_points()[3].x() - (320.0 - end_miss)).abs() < 1e-12);
        assert_eq!(
            topology.vertex(edge.start()).expect("start").point().x(),
            0.0
        );
        assert_eq!(topology.vertex(edge.end()).expect("end").point().x(), 320.0);
        if start_miss == 0.0 && end_miss == 0.0 {
            assert!(result.diagnostics().is_empty());
            continue;
        }
        let [
            StepImportDiagnostic::UntrimmedNurbsDomainRecovered {
                edge_curve_entity,
                start_parameter,
                end_parameter,
                endpoint_residual_mm,
                stored_edge_tolerance_mm,
                recovery_tolerance_cap_mm,
            },
        ] = result.diagnostics()
        else {
            panic!("one natural-domain recovery diagnostic")
        };
        assert_eq!(*edge_curve_entity, 51);
        assert_eq!((*start_parameter, *end_parameter), (0.0, 1.0));
        assert!((*endpoint_residual_mm - start_miss.max(end_miss)).abs() < 1e-12);
        assert_eq!(*stored_edge_tolerance_mm, *endpoint_residual_mm);
        assert_eq!(edge.tolerance(), Some(*stored_edge_tolerance_mm));
        assert_eq!(*recovery_tolerance_cap_mm, 1e-4);
        assert!(*stored_edge_tolerance_mm <= 1e-4);

        let exported = write_step(&topology, &[solid]).expect("export recovered natural domain");
        let mut round_topology = Topology::new();
        let round = read_step_with_report(&exported, &mut round_topology)
            .expect("reimport explicit domain and exported tolerance");
        assert!(round.diagnostics().is_empty());
        assert_eq!(round.solids().len(), 1);
        assert!(
            validate_solid(&round_topology, round.solids()[0])
                .expect("validate round trip")
                .is_valid()
        );
    }
}

#[test]
fn either_natural_endpoint_over_the_absolute_cap_still_refuses() {
    for miss in [1.000_1e-4, 2e-4] {
        assert_projected_refusal(&repro(320.0, [miss, 0.0, 0.0], [320.0, 0.0, 0.0]));
        assert_projected_refusal(&repro(320.0, [3e-5, 0.0, 0.0], [320.0 - miss, 0.0, 0.0]));
    }
}

#[test]
fn nearby_interior_feet_keep_the_projected_recovery_cap() {
    for inset in [3e-5, 1e-8] {
        assert_projected_refusal(&repro(320.0, [-inset, 3e-5, 0.0], [320.0, 0.0, 0.0]));
        assert_projected_refusal(&repro(320.0, [0.0, 0.0, 0.0], [320.0 + inset, 3e-5, 0.0]));
    }
}

#[test]
fn insufficient_reverse_pairing_separation_still_refuses() {
    // The reverse residuals are ~666 times the forward miss, below 1000.
    assert_projected_refusal(&repro(0.02, [3e-5, 0.0, 0.0], [0.02, 0.0, 0.0]));
}

#[test]
fn a_second_endpoint_foot_still_refuses() {
    let step = repro(320.0, [3e-5, 0.0, 0.0], [320.0, 0.0, 0.0])
        .replace(
            "#203 = CARTESIAN_POINT('', (2.13333333333333343e2, 0.00000000000000000e0, 0.00000000000000000e0));",
            "#203 = CARTESIAN_POINT('', (3e-5, 0., 0.));",
        )
        .replace(
            "3, (#200, #201, #203, #204), .UNSPECIFIED., .F., .F., (4, 4), (0., 1.)",
            "1, (#200, #201, #203, #204), .UNSPECIFIED., .F., .F., (2, 1, 1, 2), (0., 1., 2., 3.)",
        );
    assert!(step.contains("#203 = CARTESIAN_POINT('', (3e-5, 0., 0.));"));
    assert_projected_refusal(&step);
}

#[test]
fn rational_carriers_and_explicit_trims_do_not_gain_natural_end_recovery() {
    let step = repro(320.0, [3e-5, 0.0, 0.0], [320.0, 0.0, 0.0]);
    let rational = step.replace(
        "#202 = B_SPLINE_CURVE_WITH_KNOTS('', 3, (#200, #201, #203, #204), .UNSPECIFIED., .F., .F., (4, 4), (0., 1.), .UNSPECIFIED.);",
        "#202 = ( B_SPLINE_CURVE(3, (#200, #201, #203, #204), .UNSPECIFIED., .F., .F.) B_SPLINE_CURVE_WITH_KNOTS((4, 4), (0., 1.), .UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1., 1., 1.01, 1.)) REPRESENTATION_ITEM('') );",
    );
    let trimmed = step.replace(
        "#51 = EDGE_CURVE('', #44, #46, #202, .T.);",
        "#205 = TRIMMED_CURVE('', #202, (PARAMETER_VALUE(0.)), (PARAMETER_VALUE(1.)), .T., .PARAMETER.);\n#51 = EDGE_CURVE('', #44, #46, #205, .T.);",
    );
    for refused in [rational, trimmed] {
        let error = read_step_with_report(&refused, &mut Topology::new())
            .expect_err("only untrimmed polynomial carriers qualify");
        assert!(error.to_string().contains("EDGE_CURVE #51"), "{error}");
    }
}

#[test]
fn exact_interior_foot_still_uses_projected_recovery() {
    let mut topology = Topology::new();
    let step = repro(320.0, [-3e-5, 0.0, 0.0], [320.0, 0.0, 0.0])
        .replace("LENGTH_MEASURE(1.E-05)", "LENGTH_MEASURE(1.E-06)");
    let result = read_step_with_report(&step, &mut topology).expect("exact interior foot");
    let [
        StepImportDiagnostic::UntrimmedNurbsDomainRecovered {
            start_parameter,
            recovery_tolerance_cap_mm,
            ..
        },
    ] = result.diagnostics()
    else {
        panic!("one projected recovery diagnostic")
    };
    assert!(*start_parameter > 0.0);
    assert!((*recovery_tolerance_cap_mm - 1e-6).abs() < 1e-16);
}

fn assert_projected_refusal(step: &str) {
    let mut topology = Topology::new();
    let error = read_step_with_report(step, &mut topology).expect_err("refuse excessive recovery");
    let message = error.to_string();
    assert!(message.contains("EDGE_CURVE #51"), "{message}");
    assert!(
        message.contains("local recovery cap 1.000000e-6 mm"),
        "{message}"
    );
    assert_eq!(topology.num_edges(), 0, "failed import is atomic");
}

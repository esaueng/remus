//! Issue #1285: bounded recovery of clamped polynomial natural endpoints.

#![allow(clippy::expect_used, clippy::panic)]

use remus_io::step::{StepImportDiagnostic, read_step_with_report, write_step};
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::journal::{EntityEvent, EntityKey};
use remus_topology::validation::{validate_shell_closed, validate_vertex_ball};

const BOX: &str = include_str!("data/scale_platform_v1_linear_spline_edge.step");

fn cubic_box(start: f64, end: f64, reverse: bool) -> String {
    let controls = if reverse {
        "(#204, #203, #201, #200)"
    } else {
        "(#200, #201, #203, #204)"
    };
    let step = BOX
        .replace("LENGTH_MEASURE(1.E-07)", "LENGTH_MEASURE(1.E-05)")
        .replace(
            "#45 = CARTESIAN_POINT('', (3.20000000000000000E2, 1.10000000000000008E-3, -1.19000000000000001E-3));",
            "#45 = CARTESIAN_POINT('', (320., 0., 0.));",
        );
    let begin = step.find("#200 =").expect("control block start");
    let finish = step.find("#51 =").expect("control block end");
    format!(
        "{}#200 = CARTESIAN_POINT('', ({start:.17e}, 0., 0.));\n\
         #201 = CARTESIAN_POINT('', (106.66666666666667, 0., 0.));\n\
         #203 = CARTESIAN_POINT('', (213.33333333333333, 0., 0.));\n\
         #204 = CARTESIAN_POINT('', ({end:.17e}, 0., 0.));\n\
         #202 = B_SPLINE_CURVE_WITH_KNOTS('', 3, {controls}, .UNSPECIFIED., .F., .F., (4, 4), (0., 1.), .UNSPECIFIED.);\n{}",
        &step[..begin],
        &step[finish..]
    )
}

#[test]
fn displaced_natural_endpoints_import_with_local_diagnostics_and_unchanged_carriers() {
    for miss in [3.227_018e-5, 8.928e-5] {
        for reverse in [false, true] {
            // The first two values reproduce the reported endpoint errors.
            // Both vertices are outside the carrier's natural domain.
            let step = cubic_box(miss, 320.0 - miss, reverse);
            let mut topo = Topology::new();
            let result = read_step_with_report(&step, &mut topo).expect("natural-end recovery");
            assert_eq!(result.solids().len(), 1);
            assert_eq!(result.diagnostics().len(), 1);
            let diagnostic = &result.diagnostics()[0];
            let StepImportDiagnostic::UntrimmedNurbsDomainRecovered {
                edge_curve_entity,
                start_parameter,
                end_parameter,
                endpoint_residual_mm,
                stored_edge_tolerance_mm,
                recovery_tolerance_cap_mm,
            } = diagnostic
            else {
                panic!("unexpected diagnostic")
            };
            assert_eq!(*edge_curve_entity, 51);
            let expected = if reverse { (1.0, 0.0) } else { (0.0, 1.0) };
            assert_eq!((*start_parameter, *end_parameter), expected);
            assert!((*endpoint_residual_mm - miss).abs() < 1.0e-12);
            assert_eq!(
                stored_edge_tolerance_mm.to_bits(),
                endpoint_residual_mm.to_bits()
            );
            assert_eq!(recovery_tolerance_cap_mm.to_bits(), 1.0e-4_f64.to_bits());

            let solid = result.solids()[0];
            let edges = solid_edges(&topo, solid).expect("edges");
            assert_eq!(edges.len(), 12);
            assert_eq!(solid_faces(&topo, solid).expect("faces").len(), 6);
            let edge = edges
                .iter()
                .map(|id| topo.edge(*id).expect("edge"))
                .find(|edge| matches!(edge.curve(), EdgeCurve::NurbsCurve(_)))
                .expect("retained cubic carrier");
            let EdgeCurve::NurbsCurve(curve) = edge.curve() else {
                panic!("cubic")
            };
            // Exact bit checks pin retention of the authored geometry.
            let points = curve.control_points();
            let (first, last) = if reverse {
                (320.0 - miss, miss)
            } else {
                (miss, 320.0 - miss)
            };
            assert_eq!(points[0].x().to_bits(), first.to_bits());
            assert_eq!(points[3].x().to_bits(), last.to_bits());
            assert_eq!(edge.strict_domain().expect("natural trim"), expected);
            assert_eq!(edge.tolerance(), Some(*stored_edge_tolerance_mm));
            assert_eq!(
                topo.vertex(edge.start())
                    .expect("start")
                    .point()
                    .x()
                    .to_bits(),
                0.0_f64.to_bits()
            );
            assert_eq!(
                topo.vertex(edge.end()).expect("end").point().x().to_bits(),
                320.0_f64.to_bits()
            );

            for vertex in [edge.start(), edge.end()] {
                validate_vertex_ball(&topo, vertex).expect("recovered endpoint inside vertex ball");
                let parameter = if vertex == edge.start() {
                    expected.0
                } else {
                    expected.1
                };
                let point = topo.vertex(vertex).expect("vertex").point();
                let residual = (curve.evaluate(parameter) - point).length();
                assert_eq!(
                    topo.vertex(vertex).expect("vertex").tolerance().to_bits(),
                    residual.to_bits()
                );
                let ordinal = topo
                    .journal()
                    .ordinal_of(EntityKey::vertex(vertex.index()))
                    .expect("raised vertex has a journal ordinal");
                assert!(topo.journal().events_for(ordinal).iter().any(|(_, event)| {
                    matches!(event, EntityEvent::Modified { from } if *from == ordinal)
                }));
            }
            for (vertex_id, vertex) in topo.vertices().iter() {
                validate_vertex_ball(&topo, vertex_id).expect("all imported vertex balls");
                if vertex_id != edge.start() && vertex_id != edge.end() {
                    assert_eq!(vertex.tolerance().to_bits(), 1.0e-7_f64.to_bits());
                }
            }

            let shell = topo
                .shell(topo.solid(solid).expect("solid").outer_shell())
                .expect("shell");
            validate_shell_closed(shell, &topo).expect("closed shell");
            let report = validate_solid(&topo, solid).expect("validation");
            assert!(report.is_valid(), "{:?}", report.issues);
            let volume = solid_volume(&topo, solid, 0.01).expect("volume");
            assert!((volume - 4_608_000.0).abs() < 1.0e-6, "{volume}");
            let mesh = tessellate_solid(&topo, solid, 0.1).expect("mesh");
            assert_eq!(boundary_edge_count(&mesh), 0);
            assert_eq!(non_manifold_edge_count(&mesh), 0);

            let exported = write_step(&topo, &[solid]).expect("export recovered edge");
            let mut round = Topology::new();
            let imported = read_step_with_report(&exported, &mut round).expect("round trip");
            assert_eq!(imported.solids().len(), 1);
            assert!(imported.diagnostics().is_empty());
            // STEP carries no per-vertex tolerance, and conformant edge
            // geometry has no TRIMMED_CURVE whose declared trim used to
            // raise the vertex balls again on import. The read edge's own
            // tolerance must still cover both endpoint gaps.
            for edge_id in solid_edges(&round, imported.solids()[0]).expect("round edges") {
                let edge = round.edge(edge_id).expect("round edge");
                let EdgeCurve::NurbsCurve(curve) = edge.curve() else {
                    continue;
                };
                let (t0, t1) = edge.strict_domain().expect("round trip trim");
                for (vertex, parameter) in [(edge.start(), t0), (edge.end(), t1)] {
                    let gap = (curve.evaluate(parameter)
                        - round.vertex(vertex).expect("vertex").point())
                    .length();
                    assert!(
                        edge.tolerance().is_some_and(|tolerance| gap <= tolerance),
                        "round-trip edge tolerance {:?} does not cover its {gap} mm endpoint gap",
                        edge.tolerance()
                    );
                }
            }
            assert!(
                validate_solid(&round, imported.solids()[0])
                    .expect("round validation")
                    .is_valid()
            );
            let round_volume =
                solid_volume(&round, imported.solids()[0], 0.01).expect("round volume");
            assert!((round_volume - volume).abs() < 1.0e-6);
        }
    }
}

#[test]
fn single_displaced_natural_endpoint_imports() {
    for miss in [3.0e-7, 3.227_018e-5, 1.0e-4] {
        let mut topo = Topology::new();
        let step = cubic_box(miss, 320.0, false)
            .replace("LENGTH_MEASURE(1.E-05)", "LENGTH_MEASURE(1.E-07)");
        let report = read_step_with_report(&step, &mut topo)
            .expect("one displaced endpoint, including the inclusive cap");
        assert_eq!(report.diagnostics().len(), 1);
        let raised = topo
            .edges()
            .iter()
            .find_map(|(_, edge)| {
                matches!(edge.curve(), EdgeCurve::NurbsCurve(_)).then_some(edge.start())
            })
            .expect("spline start vertex");
        for (vertex_id, vertex) in topo.vertices().iter() {
            validate_vertex_ball(&topo, vertex_id).expect("single recovered endpoint containment");
            if vertex_id == raised {
                assert_eq!(vertex.tolerance().to_bits(), miss.to_bits());
            } else {
                assert_eq!(vertex.tolerance().to_bits(), 1.0e-7_f64.to_bits());
            }
        }
    }
}

#[test]
fn failed_import_rolls_back_recovered_vertex_balls_and_journal() {
    let mut topo = Topology::new();
    let seed = remus_operations::primitives::make_box(&mut topo, 1.0, 2.0, 3.0)
        .expect("existing document");
    let pending = topo.journal_begin("seed");
    topo.journal_record_barrier(pending, vec![]);
    let vertices_before = format!("{:?}", topo.vertices().iter().collect::<Vec<_>>());
    let journal_before = topo.journal().snapshot();
    let malformed = cubic_box(8.928e-5, 320.0, false).replace(
        "#160 = ADVANCED_FACE('', (#159), #42, .T.);",
        "#160 = ADVANCED_FACE('', (#159), #999999, .T.);",
    );
    let error = read_step_with_report(&malformed, &mut topo).expect_err("later invalid surface");
    assert!(error.to_string().contains("999999"), "{error}");
    assert_eq!(topo.num_solids(), 1);
    assert_eq!(
        format!("{:?}", topo.vertices().iter().collect::<Vec<_>>()),
        vertices_before
    );
    let journal_after = topo.journal().snapshot();
    assert_eq!(journal_after.entries, journal_before.entries);
    assert_eq!(journal_after.index, journal_before.index);
    // Abandoned operation/ordinal IDs remain retired; the increase proves
    // the failed import reached its tolerance-journaling path.
    assert!(journal_after.next_op > journal_before.next_op);
    assert!(journal_after.next_ordinal > journal_before.next_ordinal);
    assert_eq!(
        solid_volume(&topo, seed, 0.01)
            .expect("seed volume")
            .to_bits(),
        6.0_f64.to_bits()
    );
}

#[test]
fn natural_endpoint_recovery_never_exceeds_the_absolute_cap() {
    for miss in [1.000_1e-4, 2.0e-4, 1.6e-3] {
        let step = cubic_box(miss, 320.0, false);
        let error = read_step_with_report(&step, &mut Topology::new()).expect_err("over cap");
        assert!(error.to_string().contains("misses its carrier"), "{error}");
    }
}

#[test]
fn interior_feet_keep_the_projection_cap() {
    // Natural starts extend past the vertex; the closest foot is interior.
    let step = cubic_box(-8.928e-5, 320.0, false);
    let step = step
        .lines()
        .map(|line| {
            if ["#200 =", "#201 =", "#203 =", "#204 ="]
                .iter()
                .any(|id| line.starts_with(id))
            {
                line.replace(", 0., 0.));", ", 8.928E-5, 0.));")
            } else {
                line.to_owned()
            }
        })
        .collect::<Vec<_>>()
        .join("\n");
    let error = read_step_with_report(&step, &mut Topology::new()).expect_err("interior foot");
    assert!(
        error
            .to_string()
            .contains("local recovery cap 1.000000e-6 mm"),
        "{error}"
    );
}

#[test]
fn declared_parameter_trims_keep_the_declared_endpoint_cap() {
    let step = cubic_box(8.928e-5, 320.0, false)
        .replace("#51 = EDGE_CURVE('', #44, #46, #202, .T.);",
            "#205 = TRIMMED_CURVE('', #202, (PARAMETER_VALUE(0.)), (PARAMETER_VALUE(1.)), .T., .PARAMETER.);\n#51 = EDGE_CURVE('', #44, #46, #205, .T.);");
    let error =
        read_step_with_report(&step, &mut Topology::new()).expect_err("declared trim mismatch");
    assert!(
        error.to_string().contains("declared cap 1.000000e-5 mm"),
        "{error}"
    );
}

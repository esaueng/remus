//! Regression for HOOPS Exchange's degenerate straight-side spelling.
//!
//! Shapr3D 26.160 / HOOPS Exchange 2026.3 writes straight solid sides as
//! degree-1 two-point `B_SPLINE_CURVE_WITH_KNOTS` instead of `LINE`
//! (Scale Platform v1 edges #127/#135: a 320 mm side with the HOOPS knot
//! pair `(0.023809524, 0.976190476)`). That spelling is exactly the
//! control chord, so the reader normalizes it to [`EdgeCurve::Line`],
//! whose geometry the edge's vertices determine. Routing it through the
//! untrimmed-NURBS domain-recovery adapter instead projects the shared
//! vertices onto the carrier and refuses the file: the fixture spells
//! one edge of an otherwise generated 320 x 320 x 45 box as that HOOPS
//! spline and offsets its end vertex 1.620525e-3 mm off the chord —
//! mirroring Scale edge #128, whose end vertex misses its carrier by
//! 1.619256e-3 mm against a 1.0e-5 mm local recovery cap — and the
//! pre-fix reader fails the fixture with that same
//! `uniquely witnessed NURBS carrier` error.
//!
//! The companion unit tests in `crates/io/src/step/reader.rs` pin the
//! normalization boundary itself (rational and multi-span splines stay
//! NURBS); this test pins the end-to-end consequence: the solid imports,
//! validates, and survives a STEP export/reimport with every edge a Line.
//!
//! What this test deliberately does NOT cover: the Scale Platform v1
//! file as a whole still fails after this fix on its genuinely
//! inconsistent cubic edges (edge #252: start vertex 6.176e-3 mm off
//! carrier with a negative chord projection; edges #7635/#6565: shared
//! vertex #6506 3.209e-3 mm off a duplicated cubic carrier; edges
//! #302/#312: 1.1-1.6e-3 mm). Those carriers disagree with each other
//! at the shared vertices, so no exactness-preserving normalization
//! admits them; they remain fail-closed rejections.

#![allow(clippy::expect_used, clippy::panic)]

use remus_io::step::{read_step_with_report, write_step};
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::validation::validate_shell_closed;

const LINEAR_SPLINE_EDGE: &str = include_str!("data/scale_platform_v1_linear_spline_edge.step");

/// Analytic volume of the 320 x 320 x 45 fixture box.
const EXPECTED_VOLUME: f64 = 4_608_000.0;

#[test]
fn degenerate_linear_spline_edge_imports_as_a_line() {
    let mut topology = Topology::new();
    let result = read_step_with_report(LINEAR_SPLINE_EDGE, &mut topology)
        .expect("import linear-spline solid");
    assert_eq!(result.solids().len(), 1, "fixture must contain one solid");
    assert!(
        result.diagnostics().is_empty(),
        "vertex-defined lines need no NURBS domain recovery: {:?}",
        result.diagnostics()
    );
    let solid = result.solids()[0];

    let edges = solid_edges(&topology, solid).expect("enumerate solid edges");
    assert_eq!(edges.len(), 12, "box edge census changed");
    for edge_id in &edges {
        let edge = topology.edge(*edge_id).expect("edge");
        assert!(
            matches!(edge.curve(), EdgeCurve::Line),
            "every edge must read as a vertex-defined Line, got {:?}",
            edge.curve().type_tag()
        );
    }

    let shell = topology
        .shell(topology.solid(solid).expect("solid").outer_shell())
        .expect("outer shell");
    validate_shell_closed(shell, &topology).expect("closed fixture shell");
    let report = validate_solid(&topology, solid).expect("validate fixture solid");
    assert!(
        report.is_valid(),
        "fixture validation issues: {:?}",
        report.issues
    );

    let volume = solid_volume(&topology, solid, 0.01).expect("measure fixture solid");
    // The off-chord end vertex perturbs one box corner by ~1.6e-3 mm, so
    // the volume sits ~8e-6 above the analytic box, not on it; the bound
    // pins gross change while admitting that exact perturbation.
    assert!(
        (volume - EXPECTED_VOLUME).abs() <= EXPECTED_VOLUME * 1.0e-4,
        "volume {volume} differs from analytic reference {EXPECTED_VOLUME}"
    );

    let mesh = tessellate_solid(&topology, solid, 0.1).expect("tessellate fixture solid");
    assert_eq!(boundary_edge_count(&mesh), 0, "mesh must be watertight");
    assert_eq!(non_manifold_edge_count(&mesh), 0, "mesh must be manifold");

    let faces = solid_faces(&topology, solid).expect("enumerate faces");
    assert_eq!(faces.len(), 6, "box face census changed");

    let exported = write_step(&topology, &[solid]).expect("export fixture solid");
    assert!(
        !exported.contains("B_SPLINE_CURVE_WITH_KNOTS"),
        "a Line-only solid must export without spline carriers"
    );
    let mut round_topology = Topology::new();
    let round_result =
        read_step_with_report(&exported, &mut round_topology).expect("re-import exported STEP");
    assert_eq!(round_result.solids().len(), 1, "round trip keeps one solid");
    assert!(
        round_result.diagnostics().is_empty(),
        "explicit export authority must not require healing: {:?}",
        round_result.diagnostics()
    );
    let round_solid = round_result.solids()[0];
    let round_report =
        validate_solid(&round_topology, round_solid).expect("validate round-tripped solid");
    assert!(
        round_report.is_valid(),
        "round-trip validation issues: {:?}",
        round_report.issues
    );
    let round_volume =
        solid_volume(&round_topology, round_solid, 0.01).expect("measure round-tripped solid");
    assert!(
        (round_volume - volume).abs() <= volume * 1.0e-12,
        "round-trip volume {round_volume} strayed from import volume {volume}"
    );
}

#[test]
fn near_linear_cubic_spline_with_off_chord_vertex_still_fails_closed() {
    // The normalization admits only the exact two-point degree-1 Bezier.
    // A cubic through the same chord with the same 1.6e-3 mm off-chord
    // vertex — the shape of Scale's remaining cubic failures — must
    // still refuse rather than import with a silently reinterpreted
    // carrier.
    let cubic = LINEAR_SPLINE_EDGE
        .replace(
            "#202 = B_SPLINE_CURVE_WITH_KNOTS('', 1, (#200, #201), .UNSPECIFIED., .F., .F., (2, 2), (0.023809524, 0.976190476), .UNSPECIFIED.);",
            "#203 = CARTESIAN_POINT('', (1.06666666666666665E2, 0., 0.));\n#204 = CARTESIAN_POINT('', (2.13333333333333331E2, 0., 0.));\n#202 = B_SPLINE_CURVE_WITH_KNOTS('', 3, (#200, #203, #204, #201), .UNSPECIFIED., .F., .F., (4, 4), (0.023809524, 0.976190476), .UNSPECIFIED.);",
        );
    let error =
        read_step_with_report(&cubic, &mut Topology::new()).expect_err("cubic must not normalize");
    assert!(
        error
            .to_string()
            .contains("uniquely witnessed NURBS carrier"),
        "off-chord cubic vertex must still refuse at the recovery cap, got: {error}"
    );
}

//! IO-10 (kernel review 2026-10-14): the STEP B-spline attribute parser.
//!
//! - It split `B_SPLINE_CURVE_WITH_KNOTS` / `B_SPLINE_SURFACE_WITH_KNOTS`
//!   attributes on every parenthesis and comma, inside strings too, so an
//!   entity named `'Curve (1)'` read its name's `(1)` as the control-point
//!   list and failed to import. STEP strings are now opaque to it.
//! - Its knot expansion zipped the multiplicity list with the knot list, so
//!   lists of different lengths were silently truncated to the shorter one
//!   and could still form a valid knot vector. ISO 10303-42 requires them to
//!   be the same length; a mismatch is now refused.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::read_step;
use remus_topology::Topology;

const FIXTURE: &str = include_str!("data/shapr_untrimmed_nurbs_domain.step");
const CURVE: &str = "#21 = B_SPLINE_CURVE_WITH_KNOTS('', 3,";
const CURVE_KNOTS: &str = "(#17, #18, #19, #20), .UNSPECIFIED., .F., .F., (4, 4), (0., 1.00000000000000000E0), .UNSPECIFIED.);";
const SURFACE: &str = "#89 = B_SPLINE_SURFACE_WITH_KNOTS('', 3, 1,";
const SURFACE_KNOTS: &str =
    "(4, 4), (2, 2), (0., 1.00000000000000000E0), (0., 1.00000000000000000E0), .UNSPECIFIED.);";

fn with(replacements: &[(&str, &str)]) -> String {
    let mut text = FIXTURE.to_owned();
    for (from, to) in replacements {
        assert_eq!(text.matches(from).count(), 1, "`{from}` must occur once");
        text = text.replacen(from, to, 1);
    }
    text
}

fn import(text: &str) -> Result<usize, remus_io::IoError> {
    let mut topo = Topology::new();
    read_step(text, &mut topo).map(|solids| {
        assert_eq!(solids.len(), 1);
        topo.num_faces()
    })
}

#[test]
fn bspline_names_with_parentheses_and_commas_import() {
    let faces = import(FIXTURE).unwrap();
    let renamed = with(&[
        (CURVE, "#21 = B_SPLINE_CURVE_WITH_KNOTS('Curve (1)', 3,"),
        (
            "#47 = B_SPLINE_CURVE_WITH_KNOTS('', 3,",
            "#47 = B_SPLINE_CURVE_WITH_KNOTS('a, 2 (b''s) ((', 3,",
        ),
        (
            SURFACE,
            "#89 = B_SPLINE_SURFACE_WITH_KNOTS('Patch (u, v), 7', 3, 1,",
        ),
    ]);
    assert_eq!(import(&renamed).unwrap(), faces);
}

#[track_caller]
fn assert_knot_lists_refused(text: &str, what: &str) {
    let error = import(text)
        .err()
        .unwrap_or_else(|| panic!("{what} imported"));
    assert!(
        error.to_string().contains("multiplicities"),
        "{what} was refused for another reason: {error}"
    );
}

#[test]
fn curve_knot_and_multiplicity_lists_must_have_equal_length() {
    let extra_knot = with(&[(
        CURVE_KNOTS,
        "(#17, #18, #19, #20), .UNSPECIFIED., .F., .F., (4, 4), (0., 1.00000000000000000E0, 2.), .UNSPECIFIED.);",
    )]);
    assert_knot_lists_refused(&extra_knot, "a curve with 3 knots for 2 multiplicities");
    let extra_multiplicity = with(&[(
        CURVE_KNOTS,
        "(#17, #18, #19, #20), .UNSPECIFIED., .F., .F., (4, 4, 1), (0., 1.00000000000000000E0), .UNSPECIFIED.);",
    )]);
    assert_knot_lists_refused(
        &extra_multiplicity,
        "a curve with 2 knots for 3 multiplicities",
    );
}

#[test]
fn surface_knot_and_multiplicity_lists_must_have_equal_length() {
    let extra_u_knot = with(&[(
        SURFACE_KNOTS,
        "(4, 4), (2, 2), (0., 1.00000000000000000E0, 2.), (0., 1.00000000000000000E0), .UNSPECIFIED.);",
    )]);
    assert_knot_lists_refused(
        &extra_u_knot,
        "a surface with 3 u knots for 2 multiplicities",
    );
    let extra_v_multiplicity = with(&[(
        SURFACE_KNOTS,
        "(4, 4), (2, 2, 1), (0., 1.00000000000000000E0), (0., 1.00000000000000000E0), .UNSPECIFIED.);",
    )]);
    assert_knot_lists_refused(
        &extra_v_multiplicity,
        "a surface with 2 v knots for 3 multiplicities",
    );
}

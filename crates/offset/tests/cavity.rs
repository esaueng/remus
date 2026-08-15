//! Offsetting a solid that bounds its volume with cavity shells.
//!
//! Every figure asserted here is a closed form, not a recorded output. The
//! bodies are boxes, whose offset with sharp (mitred) joints is another box,
//! so both the outer boundary and every cavity have an exact volume.
//!
//! The failure this file exists to catch is the cavity's *sign*. A cavity's
//! outward normal points into the void, so an outward offset must **shrink**
//! it. Getting that backwards produces a closed, well-oriented, entirely
//! plausible solid with the wrong volume, which no topological check would
//! notice — so each volume assertion also names the figure the wrong sign
//! would have produced.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use brepkit_check::validate::{Severity, ValidateOptions, validate_solid};
use brepkit_math::mat::Mat4;
use brepkit_offset::{OffsetError, OffsetOptions, offset_solid, thick_solid};
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::measure::{mass_properties, solid_volume};
use brepkit_operations::primitives::make_box;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;
use brepkit_topology::solid::SolidId;

fn opts() -> OffsetOptions {
    OffsetOptions {
        remove_self_intersections: false,
        ..Default::default()
    }
}

/// A `6s` cube with a concentric `2s` cubical void: volume `(216 - 8) s^3`.
///
/// The void is cut with the boolean engine rather than hand-assembled, so the
/// cavity shell carries whatever orientation the rest of the kit produces.
fn hollow_cube(topo: &mut Topology, s: f64) -> SolidId {
    let outer = make_box(topo, 6.0 * s, 6.0 * s, 6.0 * s).unwrap();
    let void = make_box(topo, 2.0 * s, 2.0 * s, 2.0 * s).unwrap();
    transform_solid(topo, void, &Mat4::translation(2.0 * s, 2.0 * s, 2.0 * s)).unwrap();
    let hollow = boolean(topo, BooleanOp::Cut, outer, void).unwrap();
    assert_eq!(
        topo.solid(hollow).unwrap().inner_shells().len(),
        1,
        "fixture must be a solid with exactly one cavity"
    );
    hollow
}

/// Assert the solid is watertight: every shell closed and 2-manifold with no
/// free or over-shared edge, consistently oriented, and measuring the same
/// volume through both integrators.
fn assert_watertight(topo: &Topology, solid: SolidId, tessellation_tol: f64) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|issue| issue.severity == Severity::Error)
        .map(|issue| format!("{:?}: {}", issue.check, issue.description))
        .collect();
    assert!(
        errors.is_empty(),
        "offset result must be valid, got: {errors:?}"
    );

    let volume = solid_volume(topo, solid, tessellation_tol).unwrap();
    let mass = mass_properties(topo, solid).unwrap().mass;
    let relative = (volume - mass).abs() / volume.abs().max(f64::MIN_POSITIVE);
    assert!(
        relative < 1e-9,
        "solid_volume {volume} and mass_properties {mass} must agree, relative {relative:e}"
    );
}

// ── The cavity's sign ──────────────────────────────────────────

#[test]
fn an_outward_offset_grows_the_body_and_shrinks_its_cavity() {
    let mut topo = Topology::new();
    let hollow = hollow_cube(&mut topo, 1.0);
    let result = offset_solid(&mut topo, hollow, 0.5, opts()).unwrap();

    // Outer 6 -> 7, cavity 2 -> 1.
    let expected = 7.0_f64.powi(3) - 1.0_f64.powi(3);
    let wrong_sign = 7.0_f64.powi(3) - 3.0_f64.powi(3);
    let volume = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (volume - expected).abs() < 1e-9,
        "expected 7^3 - 1^3 = {expected}, got {volume}; a cavity offset with the \
         wrong sign would read {wrong_sign}"
    );
    assert_watertight(&topo, result, 0.01);
}

#[test]
fn an_inward_offset_shrinks_the_body_and_grows_its_cavity() {
    let mut topo = Topology::new();
    let hollow = hollow_cube(&mut topo, 1.0);
    let result = offset_solid(&mut topo, hollow, -0.5, opts()).unwrap();

    // Outer 6 -> 5, cavity 2 -> 3.
    let expected = 5.0_f64.powi(3) - 3.0_f64.powi(3);
    let wrong_sign = 5.0_f64.powi(3) - 1.0_f64.powi(3);
    let volume = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (volume - expected).abs() < 1e-9,
        "expected 5^3 - 3^3 = {expected}, got {volume}; a cavity offset with the \
         wrong sign would read {wrong_sign}"
    );
    assert_watertight(&topo, result, 0.01);
}

#[test]
fn the_wall_thins_by_twice_an_inward_offset() {
    // The wall between the two boundaries is 2 units thick on every side.
    // An inward offset of d takes d off the outer boundary and adds d to the
    // cavity, so the wall loses 2d — a statement independent of any volume.
    let mut topo = Topology::new();
    let hollow = hollow_cube(&mut topo, 1.0);
    let result = offset_solid(&mut topo, hollow, -0.9, opts()).unwrap();

    let expected = (6.0 - 1.8_f64).powi(3) - (2.0 + 1.8_f64).powi(3);
    let volume = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (volume - expected).abs() < 1e-9,
        "expected 4.2^3 - 3.8^3 = {expected}, got {volume}"
    );
}

// ── The cavity survives ────────────────────────────────────────

#[test]
fn the_result_keeps_one_shell_per_source_shell() {
    for distance in [0.5_f64, -0.5] {
        let mut topo = Topology::new();
        let hollow = hollow_cube(&mut topo, 1.0);
        let result = offset_solid(&mut topo, hollow, distance, opts()).unwrap();

        let solid = topo.solid(result).unwrap();
        assert_eq!(
            solid.inner_shells().len(),
            1,
            "offset by {distance} must keep the cavity, not absorb it"
        );
        assert_eq!(topo.shell(solid.outer_shell()).unwrap().faces().len(), 6);
        assert_eq!(
            topo.shell(solid.inner_shells()[0]).unwrap().faces().len(),
            6
        );
    }
}

#[test]
fn two_cavities_both_survive_and_both_shrink() {
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 12.0, 6.0, 6.0).unwrap();
    let void_a = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(&mut topo, void_a, &Mat4::translation(2.0, 2.0, 2.0)).unwrap();
    let with_a = boolean(&mut topo, BooleanOp::Cut, outer, void_a).unwrap();
    let void_b = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(&mut topo, void_b, &Mat4::translation(8.0, 2.0, 2.0)).unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, with_a, void_b).unwrap();
    assert_eq!(topo.solid(hollow).unwrap().inner_shells().len(), 2);

    let result = offset_solid(&mut topo, hollow, 0.5, opts()).unwrap();
    assert_eq!(topo.solid(result).unwrap().inner_shells().len(), 2);

    // Outer 13x7x7, each cavity 1x1x1.
    let expected = 13.0 * 7.0 * 7.0 - 2.0;
    let volume = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (volume - expected).abs() < 1e-9,
        "expected 13*7*7 - 2*1^3 = {expected}, got {volume}"
    );
    assert_watertight(&topo, result, 0.01);
}

// ── Scale ──────────────────────────────────────────────────────

#[test]
fn the_same_body_offsets_the_same_way_at_every_scale() {
    // The same hollow cube at 1000x, 1x and 0.001x must give the same answer
    // relative to its own size. An absolute tolerance anywhere in the
    // pipeline shows up here as a scale that disagrees with the others.
    for scale in [1e3_f64, 1.0, 1e-3] {
        let mut topo = Topology::new();
        let hollow = hollow_cube(&mut topo, scale);
        let result = offset_solid(&mut topo, hollow, 0.5 * scale, opts()).unwrap();

        let expected = (7.0_f64.powi(3) - 1.0) * scale.powi(3);
        let volume = solid_volume(&topo, result, 0.01 * scale).unwrap();
        let relative = (volume - expected).abs() / expected;
        assert!(
            relative < 1e-12,
            "at {scale}x the offset read {volume} against {expected}, relative {relative:e}"
        );
    }
}

#[test]
fn a_body_a_few_microns_across_still_offsets() {
    // Regression: the wire builder rejected two intersection lines as
    // parallel when |da x db|^2 fell below a fixed 1e-20. That quantity
    // carries the fourth power of the model's units, so every corner of
    // every face of a body this size was thrown away and the offset failed
    // with "no reconstructed wire loops". The test is a plain box because
    // measure drops cavities below about 1e-4 (see the PR notes).
    for scale in [1e-5_f64, 1e-6, 3e-7] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0 * scale, 2.0 * scale, 2.0 * scale).unwrap();
        let result = offset_solid(&mut topo, solid, 0.5 * scale, opts()).unwrap();

        let expected = 27.0 * scale.powi(3);
        let volume = solid_volume(&topo, result, 0.01 * scale).unwrap();
        let relative = (volume - expected).abs() / expected;
        assert!(
            relative < 1e-12,
            "at {scale}x the offset read {volume} against {expected}, relative {relative:e}"
        );
    }
}

// ── What is still refused, loudly and typed ────────────────────

#[test]
fn a_cavity_touching_the_outer_boundary_is_refused() {
    // Not a cavity at all: the inner box shares three faces with the outer.
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let inner = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let cavity_shell = topo.solid(inner).unwrap().outer_shell();
    topo.solid_mut(outer).unwrap().add_inner_shell(cavity_shell);

    let error = offset_solid(&mut topo, outer, 0.5, opts()).unwrap_err();
    assert!(
        matches!(&error, OffsetError::InvalidInput { reason }
            if reason.contains("cavity shells")),
        "expected a typed refusal naming cavity shells, got {error}"
    );
    assert_eq!(
        topo.solid(outer).unwrap().inner_shells(),
        &[cavity_shell],
        "a refused offset must leave the caller's solid untouched"
    );
}

#[test]
fn an_inward_offset_that_would_eat_the_wall_is_refused() {
    // The wall is 2 units thick, so an inward offset of 1 closes it exactly.
    // Anything at or past that must refuse rather than return interpenetrating
    // shells, which would still be closed and would still measure something.
    for distance in [-1.0_f64, -1.5, -3.0] {
        let mut topo = Topology::new();
        let hollow = hollow_cube(&mut topo, 1.0);
        let error = offset_solid(&mut topo, hollow, distance, opts()).unwrap_err();
        assert!(
            matches!(&error, OffsetError::InvalidInput { reason }
                if reason.contains("cavity shells")),
            "offset by {distance} must refuse, got {error}"
        );
    }
}

#[test]
fn an_outward_offset_that_would_collapse_the_cavity_is_refused() {
    // The cavity is 2 units across, so opposing walls meet at distance 1.
    // Past that point the old pipeline assembled an inverted live shell and
    // reported its negative-width volume as valid geometry.
    for distance in [1.0_f64, 1.1, 3.0] {
        let mut topo = Topology::new();
        let hollow = hollow_cube(&mut topo, 1.0);
        let cavity_shell = topo.solid(hollow).unwrap().inner_shells()[0];
        let error = offset_solid(&mut topo, hollow, distance, opts()).unwrap_err();
        assert!(
            matches!(&error, OffsetError::InvalidInput { reason }
                if reason.contains("cavity shells") && reason.contains("survive")),
            "offset by {distance} must refuse, got {error}"
        );
        assert_eq!(
            topo.solid(hollow).unwrap().inner_shells(),
            &[cavity_shell],
            "a refused offset must leave the caller's solid untouched"
        );
    }
}

#[test]
fn hollowing_a_solid_that_already_has_a_cavity_is_refused() {
    // thick_solid's wall builder only knows the outer shell, so it would
    // silently leave the cavity's openings unwalled.
    let mut topo = Topology::new();
    let hollow = hollow_cube(&mut topo, 1.0);
    let outer_shell = topo.solid(hollow).unwrap().outer_shell();
    let face = topo.shell(outer_shell).unwrap().faces()[0];

    let error = thick_solid(&mut topo, hollow, -0.2, &[face], opts()).unwrap_err();
    assert!(
        matches!(&error, OffsetError::InvalidInput { reason }
            if reason.contains("cavity shells")),
        "expected a typed refusal naming cavity shells, got {error}"
    );
}

#[test]
fn excessive_cavity_count_is_refused_before_pairwise_checks() {
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let inner = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let cavity_shell = topo.solid(inner).unwrap().outer_shell();
    for _ in 0..1_025 {
        topo.solid_mut(outer).unwrap().add_inner_shell(cavity_shell);
    }

    let error = offset_solid(&mut topo, outer, 0.5, opts()).unwrap_err();
    assert!(
        matches!(&error, OffsetError::InvalidInput { reason }
            if reason.contains("supports at most 1024 cavities") && reason.contains("has 1025")),
        "expected a typed cavity work-budget refusal, got {error}"
    );
}

#[test]
fn a_solid_without_cavities_is_unaffected() {
    // The cavity work must not change what a plain body does.
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 3.0, 5.0, 7.0).unwrap();
    let result = offset_solid(&mut topo, solid, 1.0, opts()).unwrap();

    assert!(topo.solid(result).unwrap().inner_shells().is_empty());
    let volume = solid_volume(&topo, result, 0.01).unwrap();
    assert!(
        (volume - 5.0 * 7.0 * 9.0).abs() < 1e-9,
        "expected 5*7*9 = 315, got {volume}"
    );
    assert_watertight(&topo, result, 0.01);
}

//! A blend that fails must leave the input solid exactly as it was.
//!
//! `fillet_v2` and `chamfer_v2` take `&mut Topology`, so a failure part-way
//! through has the opportunity to leave the caller's solid mutated — split
//! faces, rewired wires, a shell missing a face. A caller that handles the
//! `Err` and carries on would then be working on corrupted geometry without
//! ever being told.
//!
//! The existing blend tests assert only that the error paths return `Err`.
//! These assert the other half of the contract: after the error, the input
//! still measures, counts, and validates exactly as it did before.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use brepkit_check::validate::{ValidateOptions, validate_solid};
use brepkit_operations::blend_ops::{chamfer_v2, fillet_v2};
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::{make_box, make_cylinder};
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::explorer::{solid_edges, solid_faces};
use brepkit_topology::solid::SolidId;

const DEFLECTION: f64 = 0.01;

/// The observable shape of a solid: what a caller would notice if a failed
/// operation had quietly damaged it.
#[derive(Debug, PartialEq)]
struct Fingerprint {
    faces: usize,
    edges: usize,
    /// Volume, quantised so the comparison does not hinge on float equality.
    volume_nano: i128,
    valid: bool,
}

fn fingerprint(topo: &Topology, solid: SolidId) -> Fingerprint {
    let volume = solid_volume(topo, solid, DEFLECTION).unwrap();
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    Fingerprint {
        faces: solid_faces(topo, solid).unwrap().len(),
        edges: solid_edges(topo, solid).unwrap().len(),
        #[allow(clippy::cast_possible_truncation)]
        volume_nano: (volume * 1e9).round() as i128,
        valid: report.is_valid(),
    }
}

/// Assert that `op` fails and that the solid is untouched either side of it.
fn assert_failure_leaves_input_intact<F>(label: &str, topo: &mut Topology, solid: SolidId, op: F)
where
    F: FnOnce(&mut Topology, SolidId) -> bool,
{
    let before = fingerprint(topo, solid);
    assert!(
        before.valid,
        "{label}: the fixture itself must start valid, got {before:?}"
    );

    let errored = op(topo, solid);
    assert!(errored, "{label}: this case is only meaningful if it fails");

    let after = fingerprint(topo, solid);
    assert_eq!(
        before, after,
        "{label}: a failed blend must not mutate the input solid"
    );
}

/// A radius far larger than the part cannot produce a valid blend. Whatever
/// the engine attempts before giving up must not reach the caller's solid.
#[test]
fn oversized_fillet_radius_leaves_box_intact() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let target: Vec<EdgeId> = edges[..1].to_vec();

    assert_failure_leaves_input_intact(
        "fillet radius 50 on a 10mm box",
        &mut topo,
        solid,
        |t, s| fillet_v2(t, s, &target, 50.0).is_err(),
    );
}

/// An out-of-range chamfer currently *succeeds* (see the ignored test below),
/// so this asserts the weaker guarantee that still has to hold either way:
/// the blend writes its result into a new solid and never edits the input.
#[test]
fn oversized_chamfer_does_not_mutate_input() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let target: Vec<EdgeId> = edges[..1].to_vec();

    let before = fingerprint(&topo, solid);
    let _ = chamfer_v2(&mut topo, solid, &target, 40.0, 40.0);
    let after = fingerprint(&topo, solid);

    assert_eq!(
        before, after,
        "a blend must leave the input solid alone whether it succeeds or fails"
    );
}

/// A chamfer removes material. It cannot make a part bigger.
///
/// Ignored: this is a live defect in the blend engine, recorded here as a
/// ready-to-run repro rather than left undocumented. `chamfer_v2` with 40 mm
/// setbacks on a 10 mm box — setbacks four times the edge length, so the
/// chamfer plane misses the part entirely — returns `is_partial = false`,
/// `failed = []`, and a solid that passes `validate_solid`, yet whose volume
/// has grown from 1000 mm³ to ~2333 mm³.
///
/// The failure mode is the one the release checklist calls out: a modifier
/// returning a confident success value instead of a typed error. A caller
/// has no signal that the result is nonsense. Un-ignore once the engine
/// range-checks setbacks against the edge it is blending.
#[test]
#[ignore = "known defect: out-of-range chamfer returns a valid-looking larger solid instead of an error"]
fn out_of_range_chamfer_must_not_grow_the_solid() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let before = solid_volume(&topo, solid, DEFLECTION).unwrap();

    match chamfer_v2(&mut topo, solid, &edges[..1], 40.0, 40.0) {
        // Rejecting it outright is the correct outcome.
        Err(_) => {}
        Ok(result) => {
            let after = solid_volume(&topo, result.solid, DEFLECTION).unwrap();
            assert!(
                after < before,
                "a chamfer may only remove material: {before} mm³ -> {after} mm³"
            );
        }
    }
}

/// Rejected arguments are the cheapest failure path — they must also be the
/// cleanest, returning before anything is allocated against the solid.
#[test]
fn rejected_arguments_leave_box_intact() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let target: Vec<EdgeId> = edges[..1].to_vec();

    for (label, radius) in [
        ("zero radius", 0.0),
        ("negative radius", -2.0),
        ("NaN radius", f64::NAN),
        ("infinite radius", f64::INFINITY),
    ] {
        assert_failure_leaves_input_intact(label, &mut topo, solid, |t, s| {
            fillet_v2(t, s, &target, radius).is_err()
        });
    }

    assert_failure_leaves_input_intact("empty edge list", &mut topo, solid, |t, s| {
        fillet_v2(t, s, &[], 1.0).is_err()
    });
}

/// A chamfer on a cylinder's closed rim is rejected by design. The cylinder
/// must survive that rejection unchanged — including its analytic surfaces,
/// which a partial trim would have replaced.
#[test]
fn rejected_closed_rim_chamfer_leaves_cylinder_intact() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 5.0, 10.0).unwrap();

    // Either rim will do — both are closed circular edges.
    let rim = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            topo.edge(e)
                .is_ok_and(|edge| matches!(edge.curve(), EdgeCurve::Circle(_)))
        })
        .expect("cylinder must have a circular rim");

    let before_analytic = count_analytic_faces(&topo, solid);

    assert_failure_leaves_input_intact(
        "chamfer on a closed cylinder rim",
        &mut topo,
        solid,
        |t, s| chamfer_v2(t, s, &[rim], 0.4, 0.4).is_err(),
    );

    assert_eq!(
        count_analytic_faces(&topo, solid),
        before_analytic,
        "a rejected chamfer must not degrade analytic surfaces to NURBS"
    );
}

/// Number of faces still carrying an exact analytic surface.
fn count_analytic_faces(topo: &Topology, solid: SolidId) -> usize {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| topo.face(f).is_ok_and(|face| face.surface().is_analytic()))
        .count()
}

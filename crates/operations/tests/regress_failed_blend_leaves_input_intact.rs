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

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_operations::blend_ops::{chamfer_distance_angle, chamfer_v2, fillet_v2};
use remus_operations::chamfer::chamfer;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::solid::SolidId;

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

/// A setback larger than the part is rejected, and the box is untouched.
#[test]
fn oversized_chamfer_leaves_box_intact() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let target: Vec<EdgeId> = edges[..1].to_vec();

    assert_failure_leaves_input_intact("chamfer 40x40 on a 10mm box", &mut topo, solid, |t, s| {
        chamfer_v2(t, s, &target, 40.0, 40.0).is_err()
    });
}

/// A chamfer removes material. It cannot make a part bigger.
///
/// Regression for the defect this file originally recorded as an ignored
/// repro: `chamfer_v2` with 40 mm setbacks on a 10 mm box — four times the
/// edge length, so the bevel overran the faces it was cutting — used to
/// return `is_partial = false`, `failed = []`, and a solid that passed
/// `validate_solid`, yet whose volume had *grown* from 1000 mm³ to
/// ~2333 mm³. Closed and manifold, and completely wrong.
///
/// The engine now range-checks each setback against the wire edge it slides
/// along, so this is refused outright.
#[test]
fn out_of_range_chamfer_is_rejected_not_silently_wrong() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();

    let msg = match chamfer_v2(&mut topo, solid, &edges[..1], 40.0, 40.0) {
        Err(e) => e.to_string(),
        Ok(_) => panic!("a 40mm setback on a 10mm box must be refused"),
    };
    assert!(
        msg.contains("does not fit"),
        "the error should name the real problem, got: {msg}"
    );

    // And nothing was published: the box is exactly as it was.
    let after = solid_volume(&topo, solid, DEFLECTION).unwrap();
    assert!(
        (after - 1000.0).abs() < 1e-9,
        "input volume must be untouched, got {after}"
    );
}

/// The accepted range still works, and removes exactly the right material.
///
/// A symmetric chamfer of one edge of a cube cuts a triangular prism:
/// `½·d²·length`. Anything else means the bevel landed in the wrong place.
/// The largest legal setback is the full edge length, where the chamfer
/// consumes the adjacent face entirely — that boundary is excluded.
#[test]
fn in_range_chamfer_removes_the_exact_prism() {
    for d in [0.1_f64, 1.0, 5.0, 9.99] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();

        let result = chamfer_v2(&mut topo, solid, &edges[..1], d, d)
            .unwrap_or_else(|e| panic!("d={d} is within range but was refused: {e}"));
        let after = solid_volume(&topo, result.solid, DEFLECTION).unwrap();
        let expected = 1000.0 - 0.5 * d * d * 10.0;
        assert!(
            (after - expected).abs() < 1e-6,
            "d={d}: expected {expected} mm³, got {after} mm³"
        );
    }

    // Exactly the edge length is degenerate — the bevel would eat the whole
    // adjacent face — so it is refused rather than producing a sliver.
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    assert!(
        chamfer_v2(&mut topo, solid, &edges[..1], 10.0, 10.0).is_err(),
        "a setback equal to the edge length is degenerate and must be refused"
    );
}

/// An oversized setback is caught whichever face carries it — `d1` fitting
/// must not excuse `d2` overrunning, and the *fit* check must be what fires,
/// not some downstream symptom.
#[test]
fn oversized_setback_is_caught_on_either_face() {
    for (d1, d2) in [(1.0_f64, 40.0_f64), (40.0, 1.0)] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();

        let msg = match chamfer_v2(&mut topo, solid, &edges[..1], d1, d2) {
            Err(e) => e.to_string(),
            Ok(_) => panic!("d1={d1}, d2={d2}: an oversized setback must be refused"),
        };
        assert!(
            msg.contains("does not fit"),
            "d1={d1}, d2={d2}: the fit check should fire, got: {msg}"
        );
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

/// A chamfer that the engine cannot build must leave the cylinder unchanged —
/// including its analytic surfaces, which a partial trim would have replaced.
///
/// This used to target a closed rim, which was rejected outright by
/// `reject_closed_edges`. Closed rims are now built by the annular assembler,
/// so the invariant is exercised on the cylinder's SEAM line instead: a seam is
/// not a blendable convex edge, both engines still fail it, and the failure
/// runs deeper than an argument check — the stripe is attempted and abandoned,
/// which is exactly the path that could corrupt the arena.
#[test]
fn failed_seam_chamfer_leaves_cylinder_intact() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 5.0, 10.0).unwrap();

    let seam = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            topo.edge(e)
                .is_ok_and(|edge| matches!(edge.curve(), EdgeCurve::Line))
        })
        .expect("cylinder must have a seam line");

    let before_analytic = count_analytic_faces(&topo, solid);

    assert_failure_leaves_input_intact(
        "chamfer on a cylinder seam line",
        &mut topo,
        solid,
        |t, s| chamfer_v2(t, s, &[seam], 0.4, 0.4).is_err(),
    );

    assert_eq!(
        count_analytic_faces(&topo, solid),
        before_analytic,
        "a rejected chamfer must not degrade analytic surfaces to NURBS"
    );
}

/// The counterpart to the above: a closed rim is now BUILT, not rejected, and
/// the result must keep every analytic surface (wall, caps) plus gain an exact
/// conical band. A NURBS band here would mean the annular assembler was
/// bypassed for the walker.
#[test]
fn closed_rim_chamfer_keeps_surfaces_analytic() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 5.0, 10.0).unwrap();

    let rim = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            topo.edge(e)
                .is_ok_and(|edge| matches!(edge.curve(), EdgeCurve::Circle(_)))
        })
        .expect("cylinder must have a circular rim");

    let before_analytic = count_analytic_faces(&topo, solid);
    let result = chamfer_v2(&mut topo, solid, &[rim], 0.4, 0.4).expect("closed rim must chamfer");
    assert_eq!(
        count_analytic_faces(&topo, result.solid),
        before_analytic + 1,
        "the band must be analytic too, so every face stays exact"
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

/// Chamfers on neighbouring edges eat into each other. Once two of them
/// consume a shared edge entirely, the bevel between them inverts.
///
/// This is the same defect as `out_of_range_chamfer_is_rejected_not_silently_wrong`
/// reached from the other direction, and it survived the first fix: with
/// *every* edge chamfered there is no un-chamfered edge left to overrun, so a
/// check that only looked at untouched edges saw nothing wrong. The raw
/// flat-bevel engine returned a 10 mm box as a 425,666 mm³ solid.
///
/// The geometric limit for chamfering all edges of a cube is `d < L/2`: two
/// bevels approach each other from the ends of every 10 mm edge.
#[test]
fn overlapping_chamfers_on_every_edge_are_rejected() {
    // Comfortably inside the limit: still works, still removes material.
    for d in [1.0_f64, 4.0] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();
        let result = chamfer(&mut topo, solid, &edges, d)
            .unwrap_or_else(|e| panic!("d={d} is within the limit but was refused: {e}"));
        let volume = solid_volume(&topo, result, DEFLECTION).unwrap();
        assert!(
            volume < 1000.0 && volume > 0.0,
            "d={d}: a chamfer must remove material, got {volume} mm³"
        );
    }

    // At and beyond the limit the bevels collide and must be refused.
    for d in [5.0_f64, 6.0, 40.0] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();
        let msg = match chamfer(&mut topo, solid, &edges, d) {
            Err(e) => e.to_string(),
            Ok(r) => {
                let volume = solid_volume(&topo, r, DEFLECTION).unwrap();
                panic!("d={d} makes the bevels collide but was accepted, volume {volume} mm³");
            }
        };
        assert!(
            msg.contains("does not fit"),
            "d={d}: the fit check should fire, got: {msg}"
        );
    }
}

/// A well-sized asymmetric chamfer must close the shell and take exactly the
/// wedge its two setbacks describe.
///
/// Regression for a defect that predated the setback guard: the side faces at
/// each end of a chamfered edge split their corner using a single distance,
/// `max(d1, d2)`, for both directions. The neighbouring faces had placed their
/// chamfer points at their own setbacks, so as soon as `d1 != d2` the points
/// no longer coincided and the shell opened — 6 free edges, Euler 0 instead
/// of 2 — for *any* asymmetry, even `d2 = d1 + 1e-4`.
///
/// It went unnoticed because `chamfer_v2` refused the open shell (fail-closed,
/// so never a wrong answer to a caller) while the unit test that covered this
/// only measured volume and never checked closure.
///
/// Volume is the oracle: `1000 - (d1·d2/2)·10` is reached only when both
/// setbacks are honoured, which no single-distance scheme can do.
#[test]
fn asymmetric_chamfer_closes_and_takes_the_exact_wedge() {
    for (d1, d2) in [
        (1.0_f64, 2.0_f64),
        (2.0, 1.0),
        (0.5, 9.0),
        (9.0, 0.5),
        (3.0, 7.0),
        (1.0, 1.0001),
    ] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();

        let result = chamfer_v2(&mut topo, solid, &edges[..1], d1, d2)
            .unwrap_or_else(|e| panic!("d1={d1}, d2={d2} should chamfer cleanly: {e}"));

        let report = validate_solid(&topo, result.solid, &ValidateOptions::default()).unwrap();
        assert!(
            report.is_valid(),
            "d1={d1}, d2={d2}: shell must close, got {:#?}",
            report.issues
        );

        let volume = solid_volume(&topo, result.solid, DEFLECTION).unwrap();
        let expected = 1000.0 - 0.5 * d1 * d2 * 10.0;
        assert!(
            (volume - expected).abs() < 1e-6,
            "d1={d1}, d2={d2}: expected {expected} mm³, got {volume} mm³"
        );
    }
}

/// Asymmetric chamfers on several edges at once must also close.
#[test]
fn asymmetric_chamfer_closes_on_multiple_edges() {
    for n in [2usize, 4, 12] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();
        let targets: Vec<EdgeId> = edges[..n].to_vec();

        let result = chamfer_v2(&mut topo, solid, &targets, 1.0, 2.0)
            .unwrap_or_else(|e| panic!("{n} asymmetric edges should chamfer cleanly: {e}"));

        let report = validate_solid(&topo, result.solid, &ValidateOptions::default()).unwrap();
        assert!(
            report.is_valid(),
            "{n} edges: shell must close, got {:#?}",
            report.issues
        );

        let volume = solid_volume(&topo, result.solid, DEFLECTION).unwrap();
        assert!(
            volume < 1000.0 && volume > 0.0,
            "{n} edges: a chamfer must remove material, got {volume} mm³"
        );
    }
}

/// `chamfer_distance_angle` is asymmetric for every angle but 45°, so it was
/// caught by the same open-shell defect.
///
/// It sets `d2 = distance · tan(angle)`, which equals `distance` only at
/// π/4 — and π/4 was the sole angle the existing test covered, which is why
/// the breakage never showed. Any other angle produced an open shell.
#[test]
fn distance_angle_chamfer_closes_at_angles_other_than_45_degrees() {
    for angle_deg in [15.0_f64, 30.0, 45.0, 60.0, 75.0] {
        let angle = angle_deg.to_radians();
        let distance = 1.0_f64;

        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();

        let result = chamfer_distance_angle(&mut topo, solid, &edges[..1], distance, angle)
            .unwrap_or_else(|e| panic!("{angle_deg}° should chamfer cleanly: {e}"));

        let report = validate_solid(&topo, result.solid, &ValidateOptions::default()).unwrap();
        assert!(
            report.is_valid(),
            "{angle_deg}°: shell must close, got {:#?}",
            report.issues
        );

        // Wedge legs are `distance` and `distance·tan(angle)`.
        let volume = solid_volume(&topo, result.solid, DEFLECTION).unwrap();
        let expected = 1000.0 - 0.5 * distance * (distance * angle.tan()) * 10.0;
        assert!(
            (volume - expected).abs() < 1e-6,
            "{angle_deg}°: expected {expected} mm³, got {volume} mm³"
        );
    }
}

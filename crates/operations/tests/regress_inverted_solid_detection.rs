//! An inside-out solid must be detectable.
//!
//! `measure::solid_volume` returns the MAGNITUDE of its integral. That is a
//! reasonable contract for a volume — every caller reads one as a positive
//! quantity — but it made a real corruption invisible. remus#59's segmented
//! revolve built solids whose every face pointed inward: closed, 2-manifold,
//! consistently wound, `validate_solid`-clean, and reported at exactly the
//! right positive volume. The winding sign is what an STL facet normal is
//! derived from, so those bodies exported inside out and nothing in the kernel
//! said a word.
//!
//! Four production call sites had already written the guard they wanted, each
//! under the same comment — "A shell can pass the structural checks and still
//! be turned inside out" — and each testing `solid_volume(..) <= 0.0`:
//! `defeature`, `draft`, `split` (per half) and `chamfer`. With the magnitude
//! returned, not one of them could ever fire. They were dead code guarding the
//! exact failure they were written for.
//!
//! The resolution keeps `solid_volume` positive — changing its sign convention
//! would be a breaking change for every caller and for `kernel.volume` in the
//! app — and makes the inversion detectable instead:
//!
//! * `measure::solid_is_inverted` asks the question directly;
//! * `validate::validate_solid` reports it as an error, which is what makes
//!   those four guards live, since every one of them runs `validate_solid`
//!   first and bails on any error.
//!
//! The bodies here are inverted by explicitly reversing a shell, which is what
//! #59's revolve produced and the only way to reach it now that #59 is fixed.
//! Each case asserts BOTH halves of the contract: the volume magnitude is still
//! the closed form (so the fix did not change what `solid_volume` answers), and
//! the inversion is now reported.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{solid_is_inverted, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder, make_sphere};
use remus_operations::transform::transform_solid;
use remus_operations::validate;
use remus_topology::Topology;
use remus_topology::shell::ShellId;
use remus_topology::solid::SolidId;

/// Coarsest first, so nothing passes by being swept first. The verdict is a
/// SIGN, so it must not move with the units — and the threshold that separates
/// "inverted" from "too small to say" is a fraction of the model's own extent
/// cubed, never an absolute volume.
const SCALES: [f64; 3] = [1000.0, 1.0, 0.001];

/// Flip every face of a shell. The shell stays closed, 2-manifold and
/// consistently wound; only the direction it faces changes.
fn reverse_shell(topo: &mut Topology, shell: ShellId) {
    for fid in topo.shell(shell).unwrap().faces().to_vec() {
        let face = topo.face_mut(fid).unwrap();
        let flipped = !face.is_reversed();
        face.set_reversed(flipped);
    }
}

/// Turn the WHOLE body inside out — every shell, cavities included. Flipping
/// only the outer shell leaves a body whose cavity still faces into its own
/// void, which is not an inverted body but an inconsistent one: its magnitude
/// is `V_outer + V_void` rather than `V_outer - V_void`, so it would not be
/// measuring the same quantity.
fn invert(topo: &mut Topology, solid: SolidId) -> SolidId {
    let data = topo.solid(solid).unwrap();
    let shells: Vec<ShellId> = std::iter::once(data.outer_shell())
        .chain(data.inner_shells().iter().copied())
        .collect();
    for shell in shells {
        reverse_shell(topo, shell);
    }
    solid
}

// ── Models ────────────────────────────────────────────────────

fn a_box(topo: &mut Topology, k: f64) -> (SolidId, f64) {
    let s = make_box(topo, 20.0 * k, 12.0 * k, 8.0 * k).unwrap();
    (s, (20.0 * k) * (12.0 * k) * (8.0 * k))
}

fn a_cylinder(topo: &mut Topology, k: f64) -> (SolidId, f64) {
    let s = make_cylinder(topo, 5.0 * k, 14.0 * k).unwrap();
    (s, PI * (5.0 * k).powi(2) * (14.0 * k))
}

fn a_sphere(topo: &mut Topology, k: f64) -> (SolidId, f64) {
    let s = make_sphere(topo, 6.0 * k, 24).unwrap();
    (s, 4.0 / 3.0 * PI * (6.0 * k).powi(3))
}

/// A body with a sealed cavity: the outer shell and the cavity shell must be
/// judged by opposite standards, so a hollow body is the case that catches a
/// check that only knows one sign.
fn a_hollow_box(topo: &mut Topology, k: f64) -> (SolidId, f64) {
    let blank = make_box(topo, 20.0 * k, 20.0 * k, 20.0 * k).unwrap();
    let tool = make_box(topo, 6.0 * k, 6.0 * k, 6.0 * k).unwrap();
    transform_solid(topo, tool, &Mat4::translation(2.0 * k, 2.0 * k, 2.0 * k)).unwrap();
    let s = boolean(topo, BooleanOp::Cut, blank, tool).unwrap();
    (s, (20.0 * k).powi(3) - (6.0 * k).powi(3))
}

type Build = (&'static str, fn(&mut Topology, f64) -> (SolidId, f64));
const MODELS: [Build; 4] = [
    ("box", a_box),
    ("cylinder", a_cylinder),
    ("sphere", a_sphere),
    ("hollow box", a_hollow_box),
];

fn errors(topo: &Topology, solid: SolidId) -> Vec<String> {
    validate::validate_solid(topo, solid)
        .expect("validate")
        .issues
        .iter()
        .filter(|i| i.severity == validate::Severity::Error)
        .map(|i| i.description.clone())
        .collect()
}

/// A correctly wound body is not reported as inverted, and validation stays
/// clean. Without this control the check could be a constant `true`.
#[test]
fn a_correctly_wound_body_is_not_reported_as_inverted() {
    for k in SCALES {
        for (name, build) in MODELS {
            let mut topo = Topology::new();
            let (solid, want) = build(&mut topo, k);
            let what = format!("{name} at {k}x");

            assert!(
                !solid_is_inverted(&topo, solid).unwrap(),
                "{what}: a correctly wound body was called inverted"
            );

            // The hollow body carries a cavity, and `validate_solid`'s Euler
            // check takes no account of inner shells — a separate, pre-existing
            // gap. Assert only the orientation verdict on that one.
            let reported = errors(&topo, solid);
            let orientation: Vec<&String> = reported
                .iter()
                .filter(|d| d.contains("inside out") || d.contains("wound outward"))
                .collect();
            assert!(
                orientation.is_empty(),
                "{what}: correctly wound, yet validation said {orientation:?}"
            );

            let got = solid_volume(&topo, solid, 1e-3 * k).unwrap();
            let rel = (got - want).abs() / want;
            assert!(
                rel < 1e-3,
                "{what}: volume {got} against the closed form {want} ({rel:e})"
            );
        }
    }
}

/// Turn the outer shell inside out and the kernel must say so — while still
/// reporting the same volume magnitude it did before.
#[test]
fn an_inside_out_body_is_reported_and_still_measures_its_volume() {
    for k in SCALES {
        for (name, build) in MODELS {
            let mut topo = Topology::new();
            let (solid, want) = build(&mut topo, k);
            let upright = solid_volume(&topo, solid, 1e-3 * k).unwrap();
            let solid = invert(&mut topo, solid);
            let what = format!("inverted {name} at {k}x");

            assert!(
                solid_is_inverted(&topo, solid).unwrap(),
                "{what}: an inside-out body was not detected"
            );

            let reported = errors(&topo, solid);
            assert!(
                reported.iter().any(|d| d.contains("inside out")),
                "{what}: validate_solid reported {reported:?}, none of it about orientation"
            );

            // The magnitude contract is unchanged: this is what the app reads,
            // and it must not have moved.
            let got = solid_volume(&topo, solid, 1e-3 * k).unwrap();
            let rel = (got - want).abs() / want;
            assert!(
                rel < 1e-3,
                "{what}: volume {got} against the closed form {want} ({rel:e}) — the \
                 magnitude contract moved"
            );
            assert!(
                (got - upright).abs() <= upright * 1e-12,
                "{what}: volume {got} against the upright body's {upright}"
            );
        }
    }
}

/// The orientation check's cost knob, both ways.
///
/// `boolean/assembly.rs` runs the check at Gauss order 1 because integrating a
/// trimmed quadric at the default order 5 was 45 % of a whole boolean, for a
/// report that site only logs. That is only safe if a coarse order still gets
/// the SIGN right — which is the whole verdict, since it only has to clear
/// `diag^3 * 1e-9`. Asserted here on every model, upright and inverted, so the
/// gating cannot quietly stop detecting anything.
///
/// `Skip` is asserted too, so the off position is known to be off rather than
/// merely untested.
#[test]
fn a_coarse_order_reaches_the_same_verdict_and_skip_reaches_none() {
    use remus_operations::validate::{OrientationCheck, ValidationOptions};

    fn orientation_errors(topo: &Topology, solid: SolidId, check: OrientationCheck) -> usize {
        let opts = ValidationOptions {
            orientation: check,
            ..ValidationOptions::default()
        };
        validate::validate_solid_with_options(topo, solid, &opts)
            .expect("validate")
            .issues
            .iter()
            .filter(|i| i.severity == validate::Severity::Error)
            .filter(|i| {
                i.description.contains("inside out") || i.description.contains("wound outward")
            })
            .count()
    }

    for k in SCALES {
        for (name, build) in MODELS {
            for inverted in [false, true] {
                let mut topo = Topology::new();
                let (solid, _) = build(&mut topo, k);
                let solid = if inverted {
                    invert(&mut topo, solid)
                } else {
                    solid
                };
                let what = format!("{name} at {k}x, inverted={inverted}");

                let strict = orientation_errors(&topo, solid, OrientationCheck::Order(5));
                let coarse = orientation_errors(&topo, solid, OrientationCheck::Order(1));
                assert_eq!(
                    coarse, strict,
                    "{what}: Gauss order 1 reported {coarse} orientation error(s) where \
                     order 5 reported {strict} — the sign does not survive the coarse order"
                );
                assert!(
                    (strict > 0) == inverted,
                    "{what}: order 5 reported {strict} orientation error(s)"
                );
                assert_eq!(
                    orientation_errors(&topo, solid, OrientationCheck::Skip),
                    0,
                    "{what}: Skip still ran the check"
                );
            }
        }
    }
}

/// A cavity wound the wrong way round is the mirror statement: its void adds
/// material instead of removing it, and it is invisible to every other check
/// for the same reason.
#[test]
fn a_cavity_wound_outward_is_reported() {
    for k in SCALES {
        let mut topo = Topology::new();
        let (solid, _) = a_hollow_box(&mut topo, k);
        let inner = topo.solid(solid).unwrap().inner_shells().to_vec();
        assert_eq!(inner.len(), 1, "at {k}x: expected one cavity shell");
        reverse_shell(&mut topo, inner[0]);

        let reported = errors(&topo, solid);
        assert!(
            reported.iter().any(|d| d.contains("wound outward")),
            "at {k}x: validate_solid reported {reported:?}, none of it about the cavity's \
             orientation"
        );
    }
}

//! Regression: a work-budget change must not reroute the face splitter.
//!
//! The UV-arrangement adapters used to decide whether to decline silently
//! by testing whether the caller's `WorkBudgets` equalled
//! `WorkBudgets::new()`. Any other value, even one that has nothing to do
//! with face splitting (`newton_iterations` 20 -> 21), turned the adapters'
//! budget exhaustion into a typed `FaceSplitFailed`, so these exact cylinder
//! booleans came out as mesh fallbacks (5 -> 41, 9 -> 57, 6 -> 50 faces) or,
//! under `ExactOnly`, as errors; and a raised march budget let the adapter
//! claim cylinder faces ahead of the legacy emitters (5 -> 4, 9 -> 8 faces).
//! WASM `booleanWithQuality`, `booleanWithCancellation` and the batch
//! `marchSteps` option set exactly these fields.
//!
//! Routing is now an explicit per-family constant in the splitter, so every
//! budget below must give the default-budget result: exact, the same face
//! count, the closed-form volume.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::context::{FallbackPolicy, OperationContext, WorkBudgets};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

type Build = fn(&mut Topology) -> (SolidId, SolidId);

/// Two r = 5, h = 20 cylinders whose axes are 3 apart.
fn offset_cylinders(topo: &mut Topology) -> (SolidId, SolidId) {
    let a = make_cylinder(topo, 5.0, 20.0).unwrap();
    let b = make_cylinder(topo, 5.0, 20.0).unwrap();
    transform_solid(topo, b, &Mat4::translation(3.0, 0.0, 0.0)).unwrap();
    (a, b)
}

/// A coaxial r = 2, h = 3 magnet and r = 0.8, h = 8 screw.
fn magnet_and_screw(topo: &mut Topology) -> (SolidId, SolidId) {
    let magnet = make_cylinder(topo, 2.0, 3.0).unwrap();
    let screw = make_cylinder(topo, 0.8, 8.0).unwrap();
    (magnet, screw)
}

/// Volume of the lens shared by the two offset cylinders.
fn lens_volume() -> f64 {
    let (r, d, h) = (5.0_f64, 3.0_f64, 20.0_f64);
    let area = 2.0 * r * r * (d / (2.0 * r)).acos() - 0.5 * d * (4.0 * r * r - d * d).sqrt();
    area * h
}

struct Case {
    name: &'static str,
    build: Build,
    op: BooleanOp,
    /// Face count of the exact result at default budgets.
    faces: usize,
    volume: f64,
}

fn cases() -> [Case; 3] {
    let cylinder = PI * 25.0 * 20.0;
    [
        Case {
            name: "offset cylinders (intersect)",
            build: offset_cylinders,
            op: BooleanOp::Intersect,
            faces: 5,
            volume: lens_volume(),
        },
        Case {
            name: "offset cylinders (fuse)",
            build: offset_cylinders,
            op: BooleanOp::Fuse,
            faces: 9,
            volume: 2.0 * cylinder - lens_volume(),
        },
        Case {
            name: "magnet and screw (fuse)",
            build: magnet_and_screw,
            op: BooleanOp::Fuse,
            faces: 6,
            volume: PI * 4.0 * 3.0 + PI * 0.64 * 5.0,
        },
    ]
}

fn budgets() -> [(&'static str, WorkBudgets); 4] {
    [
        ("default", WorkBudgets::new()),
        ("newton+1", WorkBudgets::new().with_newton_iterations(21)),
        (
            "raised march/queue/segments",
            WorkBudgets::new()
                .with_march_steps(2_000_000)
                .with_queue_size(20_000)
                .with_segments(10_000),
        ),
        (
            "newton+1, raised march",
            WorkBudgets::new()
                .with_newton_iterations(21)
                .with_march_steps(2_000_000),
        ),
    ]
}

/// Run one case and describe any departure from the default-budget result.
fn check(case: &Case, label: &str, budgets: WorkBudgets, exact_only: bool) -> Option<String> {
    let mut topo = Topology::new();
    let (a, b) = (case.build)(&mut topo);
    let mut context = OperationContext::new().with_budgets(budgets);
    if exact_only {
        context = context.with_fallback(FallbackPolicy::ExactOnly);
    }
    let policy = if exact_only { "exact-only" } else { "default" };
    let tag = format!("{} [{label}, {policy} policy]", case.name);
    let outcome = match boolean_with_context(&mut topo, case.op, a, b, &context) {
        Ok(outcome) => outcome,
        Err(error) => return Some(format!("{tag}: error {error}")),
    };
    let faces = solid_faces(&topo, outcome.solid).unwrap().len();
    let volume = solid_volume(&topo, outcome.solid, 0.01).unwrap();
    let valid = validate_solid(&topo, outcome.solid).unwrap().is_valid();
    let relative = (volume - case.volume).abs() / case.volume;
    let exact = matches!(outcome.quality, BooleanQuality::Exact);
    (!exact || faces != case.faces || relative > 1e-3 || !valid).then(|| {
        format!(
            "{tag}: quality {:?}, {faces} faces (want {}), volume {volume:.4} (want {:.4}), valid {valid}",
            outcome.quality, case.faces, case.volume
        )
    })
}

#[test]
fn non_default_budgets_keep_exact_cylinder_booleans() {
    let mut failures = Vec::new();
    for case in &cases() {
        for (label, budgets) in budgets() {
            for exact_only in [false, true] {
                failures.extend(check(case, label, budgets, exact_only));
            }
        }
    }
    assert!(
        failures.is_empty(),
        "budget changes rerouted the splitter:\n{}",
        failures.join("\n")
    );
}

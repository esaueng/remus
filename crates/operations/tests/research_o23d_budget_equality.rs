//! Research probe (O2.3d budget policy): does a budget knob that has nothing
//! to do with the face splitter change what the splitter does?
//!
//! The arrangement adapters decline on budget only when the context's
//! budgets equal `WorkBudgets::new()`. Raising `newton_iterations` by one
//! makes them unequal, so the adapters stop declining.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::print_stderr)]

use remus_math::context::{FallbackPolicy, OperationContext, WorkBudgets};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

type Build = fn(
    &mut Topology,
) -> (
    remus_topology::solid::SolidId,
    remus_topology::solid::SolidId,
);

fn drilled_box(
    topo: &mut Topology,
) -> (
    remus_topology::solid::SolidId,
    remus_topology::solid::SolidId,
) {
    let block = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let drill = make_cylinder(topo, 2.0, 20.0).unwrap();
    transform_solid(topo, drill, &Mat4::translation(0.0, 0.0, -10.0)).unwrap();
    (block, drill)
}

fn slab_through_post(
    topo: &mut Topology,
) -> (
    remus_topology::solid::SolidId,
    remus_topology::solid::SolidId,
) {
    let post = make_cylinder(topo, 3.0, 10.0).unwrap();
    let slab = make_box(topo, 12.0, 12.0, 2.0).unwrap();
    transform_solid(topo, slab, &Mat4::translation(0.0, 0.0, 5.0)).unwrap();
    (post, slab)
}

fn offset_post_cut(
    topo: &mut Topology,
) -> (
    remus_topology::solid::SolidId,
    remus_topology::solid::SolidId,
) {
    let post = make_cylinder(topo, 3.0, 10.0).unwrap();
    let cutter = make_box(topo, 4.0, 12.0, 4.0).unwrap();
    transform_solid(topo, cutter, &Mat4::translation(3.0, 0.0, 5.0)).unwrap();
    (post, cutter)
}

fn offset_cylinders(
    topo: &mut Topology,
) -> (
    remus_topology::solid::SolidId,
    remus_topology::solid::SolidId,
) {
    let a = make_cylinder(topo, 5.0, 20.0).unwrap();
    let b = make_cylinder(topo, 5.0, 20.0).unwrap();
    transform_solid(topo, b, &Mat4::translation(3.0, 0.0, 0.0)).unwrap();
    (a, b)
}

fn magnet_and_screw(
    topo: &mut Topology,
) -> (
    remus_topology::solid::SolidId,
    remus_topology::solid::SolidId,
) {
    let magnet = make_cylinder(topo, 2.0, 3.0).unwrap();
    let screw = make_cylinder(topo, 0.8, 8.0).unwrap();
    (magnet, screw)
}

fn run(build: Build, op: BooleanOp, budgets: WorkBudgets, exact: bool) -> String {
    let mut topo = Topology::new();
    let (a, b) = build(&mut topo);
    let mut context = OperationContext::new().with_budgets(budgets);
    if exact {
        context = context.with_fallback(FallbackPolicy::ExactOnly);
    }
    match boolean_with_context(&mut topo, op, a, b, &context) {
        Ok(outcome) => {
            let faces = remus_topology::explorer::solid_faces(&topo, outcome.solid).unwrap();
            let (f, e, v) =
                remus_topology::explorer::solid_entity_counts(&topo, outcome.solid).unwrap();
            let volume = solid_volume(&topo, outcome.solid, 0.01).unwrap_or(f64::NAN);
            let report = remus_operations::validate::validate_solid(&topo, outcome.solid).unwrap();
            let errors: Vec<String> = report
                .issues
                .iter()
                .filter(|i| !report.is_valid() && format!("{:?}", i.severity) == "Error")
                .take(3)
                .map(|i| format!("{i:?}"))
                .collect();
            format!(
                "ok faces={} F/E/V={f}/{e}/{v} volume={volume:.6} valid={} {errors:?} quality={:?}",
                faces.len(),
                report.is_valid(),
                outcome.quality
            )
        }
        Err(error) => format!("ERR {error}"),
    }
}

#[test]
#[ignore = "O2.3d budget-policy research probe; run with --ignored --nocapture"]
fn an_unrelated_budget_knob_changes_the_splitter() {
    let _ = env_logger::builder().is_test(true).try_init();
    let cases: [(&str, Build, BooleanOp); 7] = [
        (
            "offset cylinders (intersect)",
            offset_cylinders,
            BooleanOp::Intersect,
        ),
        ("offset cylinders (fuse)", offset_cylinders, BooleanOp::Fuse),
        ("magnet and screw (fuse)", magnet_and_screw, BooleanOp::Fuse),
        ("drilled box", drilled_box, BooleanOp::Cut),
        (
            "slab through post (fuse)",
            slab_through_post,
            BooleanOp::Fuse,
        ),
        ("slab through post (cut)", slab_through_post, BooleanOp::Cut),
        ("offset box cut from post", offset_post_cut, BooleanOp::Cut),
    ];
    let budgets = [
        ("default", WorkBudgets::new()),
        ("newton+1", WorkBudgets::new().with_newton_iterations(21)),
        (
            "raised",
            WorkBudgets::new()
                .with_march_steps(2_000_000)
                .with_queue_size(20_000)
                .with_segments(10_000),
        ),
    ];
    for (name, build, op) in cases {
        for (label, b) in budgets {
            eprintln!("RESEARCH {name} [{label}]: {}", run(build, op, b, false));
            eprintln!(
                "RESEARCH {name} [{label}, exact-only]: {}",
                run(build, op, b, true)
            );
        }
    }
}

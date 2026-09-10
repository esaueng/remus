//! Replay the exact-only 46 -> 50 mm hammer-holder opening experiment.
//!
//! This is a diagnostic, not a supported modeling feature. The current kernel
//! completes both side reconstructions with exact booleans and strict validation.
//! Fixture tests additionally check dimensions, mounting bores, mesh closure and
//! STEP round trips. Application parameterization is not yet enabled.
//! No approximate fallback is enabled.
//!
//! Run with `cargo run --profile ci-test -p remus-io --example hammer_opening`.

#![allow(clippy::print_stdout)] // This diagnostic reports each operation to its CLI caller.

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
};
use remus_operations::{
    boolean::{BooleanOp, BooleanQuality, boolean_with_context},
    copy::copy_solid,
    primitives::make_box,
    transform::transform_solid,
};
use remus_topology::{Topology, solid::SolidId};

fn checked(
    topo: &mut Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
) -> Result<SolidId, Box<dyn std::error::Error>> {
    println!("Starting {op:?}");
    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let result = boolean_with_context(topo, op, a, b, &context)?;
    if result.quality != BooleanQuality::Exact {
        return Err("operation did not return exact geometry".into());
    }
    let report = validate_solid(topo, result.solid, &ValidateOptions::default())?;
    if !report.is_valid() {
        return Err(format!("invalid {op:?}: {:?}", report.issues).into());
    }
    println!("Completed {op:?}");
    Ok(result.solid)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    struct Logger;
    impl log::Log for Logger {
        fn enabled(&self, _: &log::Metadata) -> bool {
            true
        }
        fn log(&self, r: &log::Record) {
            let s = r.args().to_string();
            if s.contains("GFA") || s.contains("multi-region") {
                println!("{s}");
            }
        }
        fn flush(&self) {}
    }
    let _ = log::set_logger(&Logger);
    log::set_max_level(log::LevelFilter::Debug);
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(
        include_str!("../tests/data/shapr3d_hammer_holder.step"),
        &mut topo,
    )?;
    let source = *solids.first().ok_or("fixture contains no solid")?;
    let mut current = source;
    for (x0, dx) in [(-18.0, -2.0), (11.0, 2.0)] {
        let mask = make_box(&mut topo, 29.0, 53.0, 70.0)?;
        transform_solid(&mut topo, mask, &Mat4::translation(x0, -10.0, 0.0))?;
        let shifted = copy_solid(&mut topo, source)?;
        transform_solid(&mut topo, shifted, &Mat4::translation(dx, 0.0, 0.0))?;
        let outside = checked(&mut topo, BooleanOp::Cut, current, mask)?;
        let inside = checked(&mut topo, BooleanOp::Intersect, current, mask)?;
        let inside = checked(&mut topo, BooleanOp::Intersect, inside, shifted)?;
        current = checked(&mut topo, BooleanOp::Fuse, outside, inside)?;
    }
    println!(
        "All operations completed; dimensional and preservation acceptance is still required."
    );
    Ok(())
}

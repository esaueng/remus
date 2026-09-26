//! Single-process diagnostic workload; setup is excluded from operation counters.
use remus_math::mat::Mat4;
use remus_operations::{boolean::{boolean, BooleanOp}, primitives::make_box, transform::transform_solid};
use remus_topology::{Topology, transaction::run_transacted, transaction_probe as probe};
use std::time::Instant;
fn nested(topo: &mut Topology, a: remus_topology::SolidId, b: remus_topology::SolidId, depth: usize) -> Result<remus_topology::SolidId, remus_operations::OperationsError> {
    if depth == 0 { boolean(topo, BooleanOp::Fuse, a, b) }
    else { run_transacted(topo, |t| nested(t, a, b, depth - 1)) }
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    let size: usize = args[1].parse()?;
    let depth: usize = args[2].parse()?;
    let gfa = args.get(3).is_some_and(|v| v == "gfa");
    let mut topo = Topology::new();
    for _ in 0..size { make_box(&mut topo, 1.0, 1.0, 1.0)?; }
    let a = make_box(&mut topo, 1.0, 1.0, 1.0)?;
    let b = make_box(&mut topo, 1.0, 1.0, 1.0)?;
    let matrix = if gfa { Mat4::translation(0.5, 0.5, 0.0) * Mat4::rotation_z(std::f64::consts::FRAC_PI_4) * Mat4::translation(-0.5, -0.5, 0.0) } else { Mat4::translation(0.5, 0.0, 0.0) };
    transform_solid(&mut topo, b, &matrix)?;
    probe::reset();
    let start = Instant::now();
    let r = nested(&mut topo, a, b, depth)?;
    let ms = start.elapsed().as_secs_f64() * 1000.0;
    let metrics = probe::read();
    let volume = remus_operations::measure::solid_volume(&topo, r, 0.01)?;
    assert!((volume - if gfa { 4.0 - 2.0 * 2.0_f64.sqrt() } else { 1.5 }).abs() < 1e-8);
    println!("{{\"gfa\":{gfa},\"size\":{size},\"depth\":{depth},\"ms\":{ms},\"metrics\":{metrics:?}}}");
    Ok(())
}

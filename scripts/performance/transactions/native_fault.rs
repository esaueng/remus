use remus_math::mat::Mat4;
use remus_operations::{boolean::{boolean, BooleanOp}, primitives::make_box, transaction_fault as fault, transform::transform_solid};
use remus_topology::{Topology, attributes::EntityAttributes, transaction::run_transacted, transaction_probe::logical_state};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    for stage in [1, 2] { for gfa in [false, true] {
        let mut topo = Topology::new();
        let a = make_box(&mut topo, 1.0, 1.0, 1.0)?;
        let b = make_box(&mut topo, 1.0, 1.0, 1.0)?;
        let matrix = if gfa { Mat4::translation(0.5, 0.5, 0.0) * Mat4::rotation_z(std::f64::consts::FRAC_PI_4) * Mat4::translation(-0.5, -0.5, 0.0) } else { Mat4::translation(0.5, 0.0, 0.0) };
        transform_solid(&mut topo, b, &matrix)?;
        let checkpoint = topo.clone();
        let before = logical_state(&topo);
        let mut stale = Vec::new();
        run_transacted(&mut topo, |topo| -> Result<(), remus_operations::OperationsError> {
            topo.set_solid_attributes(a, EntityAttributes { name: Some("outer".into()), ..EntityAttributes::default() })?;
            let outer = logical_state(topo);
            for _ in 0..3 {
                fault::set(stage);
                assert!(boolean(topo, BooleanOp::Fuse, a, b).is_err());
                let id = fault::failed().ok_or(remus_operations::OperationsError::NonManifoldResult)?;
                stale.push(id);
                assert!(topo.solid(id).is_err());
                assert_eq!(logical_state(topo), outer);
            }
            Ok(())
        })?;
        topo.restore_preserving_handle_slots(&checkpoint);
        assert_eq!(logical_state(&topo), before);
        for _ in 0..4 { make_box(&mut topo, 1.0, 1.0, 1.0)?; }
        for id in stale { assert!(topo.solid(id).is_err()); }
    }}
    println!("{{\"nativeFaultCasesPassed\":4,\"repeatedCaughtFailures\":3}}");
    Ok(())
}

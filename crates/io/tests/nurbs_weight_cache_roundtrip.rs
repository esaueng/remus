//! Deserialized NURBS geometry (whose cached weight scale starts empty) must
//! validate and integrate to exactly the same numbers as freshly constructed
//! geometry. Guards the `max_weight` cache in `NurbsSurface`/`NurbsCurve`.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_io::arena_io::{deserialize_solid, serialize_solid};
use remus_operations::{measure::mass_properties, validate::validate_solid};
use remus_topology::Topology;

#[test]
fn arena_roundtrip_preserves_nurbs_validation_and_mass_bits() {
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(
        include_str!("data/shapr3d_hammer_holder.step"),
        &mut topo,
    )
    .expect("fixture parses");
    let source = solids[0];
    let bytes = serialize_solid(&topo, source).expect("serialize");
    let mut fresh = Topology::new();
    let restored = deserialize_solid(&bytes, &mut fresh).expect("deserialize");

    let a = validate_solid(&topo, source).expect("validate source");
    let b = validate_solid(&fresh, restored).expect("validate restored");
    assert!(a.is_valid());
    assert_eq!(a.error_count(), b.error_count());

    let pa = mass_properties(&topo, source).expect("mass source");
    let pb = mass_properties(&fresh, restored).expect("mass restored");
    assert_eq!(pa.mass.to_bits(), pb.mass.to_bits());
    assert_eq!(pa.center.x().to_bits(), pb.center.x().to_bits());
    assert_eq!(pa.center.y().to_bits(), pb.center.y().to_bits());
    assert_eq!(pa.center.z().to_bits(), pb.center.z().to_bits());
    for (x, y) in pa.inertia.iter().zip(pb.inertia.iter()) {
        assert_eq!(x.to_bits(), y.to_bits());
    }
}

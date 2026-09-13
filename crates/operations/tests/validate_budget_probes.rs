//! Regression coverage for the work observed by strict validation.
#![allow(clippy::unwrap_used, clippy::expect_used)]
use remus_operations::{
    primitives::make_box,
    validate::{
        OrientationCheck, ValidationOptions, validate_solid_with_budget_probes,
        validate_solid_with_options,
    },
};
use remus_topology::Topology;

#[test]
fn probes_observe_outer_and_cavity_integrals_only_when_enabled() {
    let mut topo = Topology::new();
    let stock = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let cavity = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        cavity,
        &remus_math::mat::Mat4::translation(0.5, 0.5, 0.5),
    )
    .unwrap();
    let inner = topo.solid(cavity).unwrap().outer_shell();
    for fid in topo.shell(inner).unwrap().faces().to_vec() {
        let face = topo.face_mut(fid).unwrap();
        face.set_reversed(!face.is_reversed());
    }
    let outer = topo.solid(stock).unwrap().outer_shell();
    let hollow = topo.add_solid(remus_topology::solid::Solid::new(outer, vec![inner]));
    for orientation in [
        OrientationCheck::Order(3),
        OrientationCheck::Order(5),
        OrientationCheck::Skip,
    ] {
        let options = ValidationOptions {
            orientation,
            ..Default::default()
        };
        let (observed, probes) =
            validate_solid_with_budget_probes(&topo, hollow, &options).unwrap();
        let ordinary = validate_solid_with_options(&topo, hollow, &options).unwrap();
        assert_eq!(format!("{observed:?}"), format!("{ordinary:?}"));
        match orientation {
            OrientationCheck::Skip => assert!(probes.shells.is_empty()),
            OrientationCheck::Order(order) => {
                assert_eq!(probes.shells.len(), 2);
                assert_eq!(probes.measured_faces(), 12);
                for (index, probe) in probes.shells.iter().enumerate() {
                    assert_eq!(probe.shell, index);
                    assert_eq!(probe.order, order);
                    assert_eq!(probe.faces, 6);
                    let expected = if index == 0 { 64.0 } else { -27.0 };
                    assert!((probe.signed_volume - expected).abs() < 1e-8);
                }
            }
        }
    }
}

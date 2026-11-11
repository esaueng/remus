//! A unit-knot periodic spline band must retain its material under reflection.

#![allow(clippy::unwrap_used)]

use remus_io::step::read_step;
use remus_math::vec::{Point3, Vec3};
use remus_operations::{measure, mirror, push_pull, validate};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

fn left_cap(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.x() < -0.99)
        })
        .unwrap()
}

#[test]
fn periodic_shaft_mirror_preserves_volume_after_cap_move() {
    let mut topo = Topology::new();
    let solid = read_step(include_str!("data/periodic-shaft.step"), &mut topo).unwrap()[0];
    let cap = left_cap(&topo, solid);
    let solid = push_pull::move_faces(&mut topo, solid, &[cap], -0.26).unwrap();
    let reflected = mirror::mirror(
        &mut topo,
        solid,
        Point3::new(0.0, 17.9, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
    )
    .unwrap();
    let reference = measure::solid_volume(&topo, solid, 0.024).unwrap();
    for deflection in [0.024, 0.01, 0.001] {
        let before = measure::solid_volume(&topo, solid, deflection).unwrap();
        let after = measure::solid_volume(&topo, reflected, deflection).unwrap();
        assert!(
            (after - before).abs() < before * 1e-9,
            "reflection must preserve volume: {before} vs {after} at {deflection}"
        );
        assert!(
            (before - reference).abs() < reference * 1e-9,
            "the exact face integral must not depend on mesh deflection"
        );
    }
    assert!(
        validate::validate_solid(&topo, reflected)
            .unwrap()
            .is_valid()
    );

    // The saved mirrored offset and the newly requested 220 mm total both
    // stretch only the radius-9 shaft. Their material changes have the
    // independent cylinder oracle delta V = pi r^2 delta x.
    for (source, distance) in [(reflected, 60.0), (solid, -20.0)] {
        let cap = left_cap(&topo, source);
        let before = measure::solid_volume(&topo, source, 0.024).unwrap();
        let moved = push_pull::move_faces(&mut topo, source, &[cap], distance).unwrap();
        let after = measure::solid_volume(&topo, moved, 0.024).unwrap();
        let expected = std::f64::consts::PI * 9.0_f64.powi(2) * distance;
        assert!(
            ((after - before) - expected).abs() < before * 1e-9,
            "shaft offset changed volume by {}, expected {expected}",
            after - before
        );
        assert!(validate::validate_solid(&topo, moved).unwrap().is_valid());
    }
}

//! A cylinder chart seam must not displace its shared sphere intersection.
#![allow(clippy::unwrap_used, clippy::panic)]

use remus_math::{
    context::{FallbackPolicy, OperationContext},
    mat::Mat4,
};
use remus_operations::{
    boolean::{BooleanOp, boolean_with_context},
    primitives::{make_cylinder, make_sphere},
    tessellate::{tessellate_solid, welded_mesh_quality},
    transform::transform_solid,
};
use remus_topology::{Topology, face::FaceSurface, solid::SolidId};

fn assert_union(topo: &Topology, solid: SolidId, scale: f64) {
    assert!(
        remus_operations::validate::validate_solid(topo, solid)
            .unwrap()
            .is_valid()
    );
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    assert!(
        faces
            .iter()
            .any(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Sphere(_)))
    );
    assert!(
        faces
            .iter()
            .any(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
    );
    // Independent Simpson integration of the horizontal disk overlap,
    // subtracted from the sum of the primitive volumes.
    let expected = 43_378.347_939_725_434 * scale.powi(3);
    let exact_volume = remus_operations::measure::solid_volume(topo, solid, 0.01 * scale).unwrap();
    assert!(
        (exact_volume - expected).abs() / expected < 0.001,
        "B-rep volume {exact_volume} vs {expected}"
    );
    for relative_deflection in [0.1, 0.01] {
        let mesh = tessellate_solid(topo, solid, relative_deflection * scale).unwrap();
        let quality = welded_mesh_quality(&mesh);
        assert!(
            quality.is_watertight(),
            "scale={scale} deflection={relative_deflection}: {quality:?}"
        );
        let mut edges = std::collections::BTreeMap::<_, Vec<bool>>::new();
        let origin = mesh.positions[0];
        let mut volume = 0.0;
        for tri in mesh.indices.chunks_exact(3) {
            for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                edges.entry((a.min(b), a.max(b))).or_default().push(a < b);
            }
            let a = mesh.positions[tri[0] as usize] - origin;
            let b = mesh.positions[tri[1] as usize] - origin;
            let c = mesh.positions[tri[2] as usize] - origin;
            volume += a.dot(b.cross(c)) / 6.0;
        }
        assert!(
            edges
                .values()
                .all(|uses| uses.len() == 2 && uses[0] != uses[1]),
            "mesh must have two oppositely directed uses of every edge"
        );
        assert!(
            (volume - expected).abs() / expected < 0.01,
            "mesh volume {volume} vs {expected}"
        );
    }
}

#[test]
fn sphere_on_cylinder_wall_has_closed_oriented_mesh() {
    for scale in [0.1, 1.0, 10.0] {
        let mut topo = Topology::new();
        let cylinder = make_cylinder(&mut topo, 15.0 * scale, 60.0 * scale).unwrap();
        let sphere = make_sphere(&mut topo, 7.5 * scale, 24).unwrap();
        transform_solid(
            &mut topo,
            sphere,
            &Mat4::translation(15.0 * scale, 0.0, 30.0 * scale),
        )
        .unwrap();
        let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
        let solid = boolean_with_context(&mut topo, BooleanOp::Fuse, cylinder, sphere, &context)
            .unwrap()
            .solid;
        assert_union(&topo, solid, scale);
        let placement =
            Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale) * Mat4::rotation_y(0.37);
        transform_solid(&mut topo, solid, &placement).unwrap();
        assert_union(&topo, solid, scale);
        let step = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();
        let mut imported = Topology::new();
        let solids = remus_io::step::reader::read_step(&step, &mut imported).unwrap();
        assert_eq!(solids.len(), 1);
        assert_union(&imported, solids[0], scale);
    }
}

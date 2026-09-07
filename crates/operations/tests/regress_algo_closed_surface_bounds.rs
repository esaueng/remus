//! Closed carriers must survive the boolean broad phase despite seam-only boundaries.
#![allow(clippy::unwrap_used)]

use remus_math::mat::Mat4;
use remus_operations::{
    primitives::{make_sphere, make_torus},
    transform::transform_solid,
};
use remus_topology::Topology;

#[test]
fn sphere_and_torus_bounds_contain_their_surface_bulges() {
    for scale in [0.1, 1.0, 10.0] {
        for placed in [false, true] {
            let mut topo = Topology::new();
            let sphere = make_sphere(&mut topo, 3.0 * scale, 24).unwrap();
            let torus = make_torus(&mut topo, 6.0 * scale, 2.0 * scale, 32).unwrap();
            let placement = Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale)
                * Mat4::rotation_y(0.37);
            for solid in [sphere, torus] {
                if placed {
                    transform_solid(&mut topo, solid, &placement).unwrap();
                }
                let bbox = remus_algo::classifier::compute_solid_bbox(&topo, solid).unwrap();
                for fid in remus_topology::explorer::solid_faces(&topo, solid).unwrap() {
                    let surface = topo.face(fid).unwrap().surface();
                    for u in [0.0, 0.71, 1.57, 2.4, 3.8, 5.2] {
                        for v in [-1.0, 0.0, 0.6, 1.3] {
                            let point = surface.evaluate(u, v).unwrap();
                            assert!(
                                bbox.expanded(1e-10).contains_point(point),
                                "scale={scale} placed={placed} {} misses {point:?}: {bbox:?}",
                                surface.type_tag()
                            );
                        }
                    }
                }
            }
        }
    }
}

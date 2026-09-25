//! Closed carriers must survive the boolean broad phase despite seam-only boundaries.
#![allow(clippy::unwrap_used)]

use remus_math::mat::Mat4;
use remus_operations::{
    primitives::{make_cone, make_cylinder, make_sphere, make_torus},
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

/// Full-circle rims bound the solid box wherever the placement rotates their
/// seam. Endpoints plus the parameter midpoint gave each rim one chord: the
/// B52 cylinder (r = 2) at rot = 2.5 and the B32 frustum (r = 1.5 → 2.5) at
/// rot = 0 were boxed so short that VV, VE and EF reported the overlapping
/// box 2.5×1×1 disjoint and skipped.
#[test]
fn circular_rims_bound_the_solid_box_at_every_seam_rotation() {
    for cone in [false, true] {
        for k in 0..24 {
            let rot = f64::from(k) * std::f64::consts::TAU / 24.0 + 0.1;
            for rot in [rot, if cone { 0.0 } else { 2.5 }] {
                let mut topo = Topology::new();
                let solid = if cone {
                    make_cone(&mut topo, 1.5, 2.5, 1.0).unwrap()
                } else {
                    make_cylinder(&mut topo, 2.0, 1.0).unwrap()
                };
                let placement = Mat4::translation(0.5, -1.5, -0.5) * Mat4::rotation_z(rot);
                transform_solid(&mut topo, solid, &placement).unwrap();
                let bbox = remus_algo::classifier::compute_solid_bbox(&topo, solid).unwrap();
                let (r_bottom, r_top): (f64, f64) = if cone { (1.5, 2.5) } else { (2.0, 2.0) };
                for (z, r) in [(-0.5, r_bottom), (0.5, r_top)] {
                    for i in 0..360 {
                        let a = f64::from(i).to_radians();
                        let point = remus_math::vec::Point3::new(
                            r.mul_add(a.cos(), 0.5),
                            r.mul_add(a.sin(), -1.5),
                            z,
                        );
                        assert!(
                            bbox.expanded(1e-10).contains_point(point),
                            "cone={cone} rot={rot}: rim point {point:?} escapes {bbox:?}"
                        );
                    }
                }
            }
        }
    }
}

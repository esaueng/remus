//! Parity of the fixed-order area-only integrator and the full contribution.
#![allow(clippy::unwrap_used)]

use remus_check::properties::face_integrator::{
    analytic_face_area, integrate_face, integrate_face_area,
};
use remus_math::mat::Mat4;
use remus_operations::{measure, primitives, transform};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

#[test]
fn curved_primitive_area_matches_full_integral_after_transform_and_reversal() {
    let mut topo = Topology::new();
    let solids = [
        primitives::make_cylinder(&mut topo, 2.0, 7.0).unwrap(),
        primitives::make_cone(&mut topo, 2.0, 0.5, 7.0).unwrap(),
        primitives::make_sphere(&mut topo, 2.0, 32).unwrap(),
        primitives::make_torus(&mut topo, 5.0, 1.0, 32).unwrap(),
    ];
    for (index, solid) in solids.into_iter().enumerate() {
        if index % 2 == 1 {
            transform::transform_solid(
                &mut topo,
                solid,
                &Mat4::translation(12_000.0, -8_000.0, 3_000.0),
            )
            .unwrap();
        }
        for face in solid_faces(&topo, solid).unwrap() {
            if matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Plane { .. }
            ) {
                continue;
            }
            for reversed in [false, true] {
                topo.face_mut(face).unwrap().set_reversed(reversed);
                let full = integrate_face(&topo, face, 8).unwrap().area;
                let area_only = integrate_face_area(&topo, face, 8).unwrap();
                assert_eq!(
                    area_only.to_bits(),
                    full.to_bits(),
                    "primitive {index}, face {face:?}, reversed {reversed}"
                );
                // The public area reduces these faces to exact boundary
                // integrals; on untrimmed primitives the quadrature is
                // converged, so both agree to its rounding, which grows with
                // the primitives' 1.5e4 offset from the origin.
                let public = measure::face_area(&topo, face, 0.1).unwrap();
                assert_eq!(
                    public.to_bits(),
                    analytic_face_area(&topo, face, 8).unwrap().abs().to_bits(),
                    "public face area, primitive {index}, face {face:?}"
                );
                assert!(
                    (public - full.abs()).abs() <= 1e-10 * full.abs(),
                    "public face area {public} vs quadrature {full}, primitive {index}, face {face:?}"
                );
            }
        }
    }
}

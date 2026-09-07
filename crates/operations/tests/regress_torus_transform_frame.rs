//! A similarity transform must carry the torus frame along with its boundary.
#![allow(clippy::unwrap_used)]
use remus_math::mat::Mat4;
use remus_operations::{
    primitives::make_torus,
    transform::{transform_face, transform_solid},
};
use remus_topology::Topology;

#[test]
fn torus_surface_points_follow_solid_and_face_transforms() {
    for face_only in [false, true] {
        for scale in [0.1, 1.0, 10.0] {
            let mut topo = Topology::new();
            let solid = make_torus(&mut topo, 6.0, 2.0, 32).unwrap();
            let face = remus_topology::explorer::solid_faces(&topo, solid).unwrap()[0];
            let original = topo.face(face).unwrap().surface().clone();
            let matrix = Mat4::translation(17.0, -23.0, 31.0)
                * Mat4::rotation_y(0.37)
                * Mat4::scale(scale, scale, scale);
            if face_only {
                transform_face(&mut topo, face, &matrix).unwrap();
            } else {
                transform_solid(&mut topo, solid, &matrix).unwrap();
            }
            let transformed = topo.face(face).unwrap().surface();
            for u in [0.0, 0.71, 1.57, 3.2, 5.8] {
                for v in [0.0, 0.4, 1.7, 3.4, 5.9] {
                    let expected = matrix.mul_point(original.evaluate(u, v).unwrap());
                    let actual = transformed.evaluate(u, v).unwrap();
                    assert!(
                        (expected - actual).length() < 1e-10,
                        "scale={scale} face_only={face_only}: {actual:?} vs {expected:?}"
                    );
                }
            }
        }
    }
}

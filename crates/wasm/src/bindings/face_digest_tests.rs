//! `faceContentDigests` contract: drag ticks keep the digests of the faces
//! they leave untouched.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::vec::Vec3;
use remus_topology::explorer::solid_faces;
use serde_json::Value;

use crate::handles::face_id_to_u32;
use crate::kernel::RemusKernel;

fn face_with_normal(kernel: &RemusKernel, solid: u32, normal: Vec3) -> u32 {
    let id = kernel.resolve_solid(solid).unwrap();
    let face = solid_faces(kernel.topo(), id)
        .unwrap()
        .into_iter()
        .find(|&face| {
            kernel
                .topo()
                .face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(normal) > 0.999)
        })
        .unwrap();
    face_id_to_u32(face)
}

fn moved(kernel: &mut RemusKernel, solid: u32, face: u32, distance: f64) -> u32 {
    let result: Value =
        serde_json::to_value(kernel.move_faces_detailed_impl(solid, &[face], distance)).unwrap();
    assert_eq!(result["status"], "ok", "{result}");
    u32::try_from(result["value"].as_u64().unwrap()).unwrap()
}

#[test]
fn drag_ticks_keep_the_digests_of_untouched_faces() {
    let mut kernel = RemusKernel::new();
    let source = kernel.make_box_solid(10.0, 6.0, 4.0).unwrap();
    let top = face_with_normal(&kernel, source, Vec3::new(0.0, 0.0, 1.0));
    let ticks = [2.0, 2.5, 2.0].map(|distance| moved(&mut kernel, source, top, distance));
    let digest = |kernel: &RemusKernel, solid: u32, normal: Vec3| {
        let face = face_with_normal(kernel, solid, normal);
        let digests = kernel.face_content_digests_impl(&[face]).unwrap();
        assert_eq!(digests.len(), 1);
        assert_eq!(digests[0].len(), 32);
        assert!(digests[0].bytes().all(|b| b.is_ascii_hexdigit()));
        digests[0].clone()
    };
    let (up, down) = (Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, -1.0));
    assert_ne!(digest(&kernel, ticks[0], up), digest(&kernel, ticks[1], up));
    assert_eq!(digest(&kernel, ticks[0], up), digest(&kernel, ticks[2], up));
    assert_eq!(
        digest(&kernel, ticks[0], down),
        digest(&kernel, ticks[1], down)
    );
    assert!(kernel.face_content_digests_impl(&[]).unwrap().is_empty());
    assert!(kernel.face_content_digests_impl(&[top, u32::MAX]).is_err());
}

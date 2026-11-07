//! Regression: the ruled-band volume shortcut must not read the unused
//! control column of an over-clamped degree-one end knot.
//!
//! Each face of a unit box is replaced by a degree (2, 1) NURBS sheet whose
//! V knots `[0, 0, 0.5, 1, 1, 1]` end with an empty span. The fourth control
//! column is pushed far away; the evaluator never uses it, so the solid is
//! still the unit cube. Reading that column at `v = 1` inflated every face
//! flux to ~100 while the fluxes still cancelled, so closure checks passed
//! and `solid_volume` reported ~100.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::properties::face_integrator::{
    integrate_face_fixed_about, integrate_ruled_guard_volume_about,
};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

#[test]
fn ruled_shortcut_ignores_overclamped_end_column() {
    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    let reference = Point3::new(0.5, 0.5, 0.5);
    for face in faces.iter().copied() {
        let FaceSurface::Plane { normal, .. } = *topo.face(face).unwrap().surface() else {
            panic!("box faces are planes");
        };
        let u = if normal.x().abs() > 0.5 {
            Vec3::new(0.0, 1.0, 0.0)
        } else if normal.y().abs() > 0.5 {
            Vec3::new(0.0, 0.0, 1.0)
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };
        let v = (-normal).cross(u);
        let origin = reference + normal * 0.5 - u * 0.5 - v * 0.5;
        let poles = [0.0, 0.5, 1.0]
            .into_iter()
            .map(|x| {
                [0.0, 0.5, 1.0, 100.0]
                    .into_iter()
                    .map(|y| origin + u * x + v * y)
                    .collect()
            })
            .collect();
        let surface = NurbsSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
            poles,
            vec![vec![1.0; 4]; 3],
        )
        .unwrap();
        let face_mut = topo.face_mut(face).unwrap();
        face_mut.set_surface(FaceSurface::Nurbs(surface));
        face_mut.set_reversed(true);

        let fixed = integrate_face_fixed_about(&topo, face, 5, reference).unwrap();
        if let Some(ruled) = integrate_ruled_guard_volume_about(&topo, face, 5, reference).unwrap()
        {
            assert!(
                (ruled.volume - fixed.volume).abs() < 1e-9,
                "face {}: ruled {} vs fixed {}",
                face.index(),
                ruled.volume,
                fixed.volume
            );
        }
    }
    let volume = remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap();
    assert!((volume - 1.0).abs() < 1e-9, "unit cube volume {volume}");
}

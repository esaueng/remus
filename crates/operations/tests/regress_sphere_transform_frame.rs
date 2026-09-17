//! A rigid transform must carry a sphere's frame, and a hemisphere's pole must
//! be chosen in that frame.
//!
//! `transform_solid` rebuilt every sphere on world axes, so a rotated
//! primitive's hemispheres kept `z` up while their shared equator polygon
//! tilted; and the sweep's pole choice projected the rim onto the WORLD XY
//! plane, which for a rotated frame is a degenerate line whose signed area is
//! rounding noise. Near the origin the noise happened to differ between the two
//! hemispheres; far from it both swept the same half. Either way the solid
//! meshed open and the mesh volume read low while the exact route stayed right
//! (B41: a disjoint sphere fuse at (0,-1000,3000) and its unit twin, and the
//! Fuzz Smoke box–sphere case rotated 45° about Y).
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_box, make_cone, make_sphere};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
    tessellate_solid_grouped_with_tolerance,
};
use remus_operations::transform::{transform_face, transform_solid};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn assert_watertight(topo: &Topology, solid: SolidId, what: &str) {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    let harness = ((aabb.max - aabb.min).length() * 4e-5).max(1e-7) * 4.0;
    for d in [0.1, 0.01, harness] {
        let mesh = tessellate_solid(topo, solid, d).unwrap();
        let (b, n) = (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh));
        assert!(
            b == 0 && n == 0,
            "{what}: mesh at deflection {d} has {b} boundary and {n} non-manifold edges"
        );
    }
}

fn assert_volume(topo: &Topology, solid: SolidId, expected: f64, what: &str) {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    let d = ((aabb.max - aabb.min).length() * 4e-5).max(1e-7);
    let mesh = solid_volume(topo, solid, d).unwrap();
    let exact = mass_properties(topo, solid).unwrap().mass;
    for (route, v) in [("solid_volume", mesh), ("mass_properties", exact)] {
        assert!(
            ((v - expected) / expected).abs() < 2e-3,
            "{what}: {route} {v} vs expected {expected}"
        );
    }
}

#[test]
fn sphere_surface_points_follow_solid_and_face_transforms() {
    for face_only in [false, true] {
        for scale in [0.1, 1.0, 10.0] {
            let mut topo = Topology::new();
            let solid = make_sphere(&mut topo, 2.0, 12).unwrap();
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
            assert!(matches!(transformed, FaceSurface::Sphere(_)));
            for u in [0.0, 0.71, 1.57, 3.2, 5.8] {
                for v in [-1.5, -0.4, 0.0, 0.9, 1.5] {
                    let expected = matrix.mul_point(original.evaluate(u, v).unwrap());
                    let actual = transformed.evaluate(u, v).unwrap();
                    assert!(
                        (expected - actual).length() < 1e-9 * scale.max(1.0),
                        "scale={scale} face_only={face_only}: {actual:?} vs {expected:?}"
                    );
                }
            }
        }
    }
}

/// Both hemispheres of a rotated, far-placed primitive must sweep opposite
/// poles: the meshes stay watertight and each face's mesh lies on its own
/// side of the frame's equator.
#[test]
fn rotated_far_sphere_hemispheres_sweep_opposite_poles() {
    let rot = Mat4::rotation_x(3.0 * FRAC_PI_2);
    for (label, steps) in [
        ("rotate", vec![rot]),
        (
            "rotate then translate far",
            vec![rot, Mat4::translation(10.0, 20.0, 30.0)],
        ),
        (
            "translate far then rotate about origin",
            vec![Mat4::translation(10.0, 20.0, 30.0), rot],
        ),
        (
            "single placement",
            vec![Mat4::translation(10.0, 20.0, 30.0) * rot],
        ),
        (
            "oblique far",
            vec![Mat4::translation(-2000.0, 2500.0, 2500.0) * Mat4::rotation_y(FRAC_PI_4)],
        ),
    ] {
        let mut topo = Topology::new();
        let sphere = make_sphere(&mut topo, 1.0, 8).unwrap();
        for m in &steps {
            transform_solid(&mut topo, sphere, m).unwrap();
        }
        assert_watertight(&topo, sphere, label);
        assert_volume(&topo, sphere, 4.0 / 3.0 * PI, label);

        let (mesh, offsets) = tessellate_solid_grouped_with_tolerance(
            &topo,
            sphere,
            5e-4,
            remus_math::chord::DEFAULT_ANGULAR_TOL,
        )
        .unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, sphere).unwrap();
        let mut sides = Vec::new();
        for (fi, w) in offsets.windows(2).enumerate() {
            let FaceSurface::Sphere(sp) = topo.face(faces[fi]).unwrap().surface() else {
                panic!("{label}: face {:?} is not a sphere", faces[fi]);
            };
            let mut acc = 0.0;
            for t in (w[0] as usize..w[1] as usize).step_by(3) {
                for k in 0..3 {
                    let p = mesh.positions[mesh.indices[t + k] as usize];
                    acc += (p - sp.center()).dot(sp.z_axis());
                }
            }
            assert!(
                (w[1] - w[0]) > 0,
                "{label}: face {:?} emitted no triangles",
                faces[fi]
            );
            sides.push(acc.signum());
        }
        assert_eq!(sides.len(), 2, "{label}: two hemisphere faces expected");
        assert!(
            sides[0] * sides[1] < 0.0,
            "{label}: both hemispheres swept the same pole (sides {sides:?})"
        );
    }
}

/// The Fuzz Smoke witness: a box fused with a disjoint sphere rotated 45°
/// about Y must measure the closed-form sum through both routes and mesh
/// watertight.
#[test]
fn disjoint_box_sphere_fuse_rotated_measures_closed_form() {
    let mut topo = Topology::new();
    let bx = make_box(&mut topo, 1.0, 1.5, 1.5).unwrap();
    let sphere = make_sphere(&mut topo, 1.5, 13).unwrap();
    transform_solid(
        &mut topo,
        sphere,
        &(Mat4::translation(-2.0, 2.5, 2.5) * Mat4::rotation_y(FRAC_PI_4)),
    )
    .unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, bx, sphere).unwrap();
    assert!(validate_solid(&topo, fused).unwrap().is_valid());
    assert_watertight(&topo, fused, "box ∪ rotated sphere");
    assert_volume(
        &topo,
        fused,
        2.25 + 4.0 / 3.0 * PI * 1.5_f64.powi(3),
        "box ∪ rotated sphere",
    );
}

/// The B41 witness family: a disjoint cone–sphere fuse at the unit and 1e3
/// scales, x-rotated 3π/2, must measure the closed-form sum through both
/// routes, mesh watertight, and re-measure identically after a translation.
#[test]
fn disjoint_cone_sphere_fuse_measures_closed_form_and_survives_translation() {
    for scale in [1.0, 1000.0] {
        let mut topo = Topology::new();
        let cone = make_cone(&mut topo, scale, 0.5 * scale, scale).unwrap();
        let sphere = make_sphere(&mut topo, scale, 8).unwrap();
        transform_solid(
            &mut topo,
            sphere,
            &(Mat4::translation(0.0, -2.0 * scale, 3.0 * scale)
                * Mat4::rotation_x(3.0 * FRAC_PI_2)),
        )
        .unwrap();
        let fused = boolean(&mut topo, BooleanOp::Fuse, cone, sphere).unwrap();
        let expected = PI * scale * (1.0 + 0.5 + 0.25) * scale * scale / 3.0
            + 4.0 / 3.0 * PI * scale * scale * scale;
        let what = format!("cone ∪ sphere at scale {scale}");
        assert!(validate_solid(&topo, fused).unwrap().is_valid(), "{what}");
        assert_watertight(&topo, fused, &what);
        assert_volume(&topo, fused, expected, &what);
        transform_solid(
            &mut topo,
            fused,
            &Mat4::translation(13.0 * scale, -7.0 * scale, 5.0 * scale),
        )
        .unwrap();
        assert_watertight(&topo, fused, &format!("{what}, translated"));
        assert_volume(&topo, fused, expected, &format!("{what}, translated"));
    }
}

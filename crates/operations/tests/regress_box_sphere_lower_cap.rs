//! Regression (#1790): a box cut, fused or intersected with a sphere whose
//! centre sits above the box top must keep the sphere's lower cap.
//!
//! A 60×60×20 box meets a radius-10 sphere centred at (30, 30, z). For a
//! centre above the top face (z = 24, 27, 29) the box top splits the sphere
//! into a lower cap inside the box and the rest outside. Two defects hid that
//! cap:
//!
//! 1. The box top cuts the sphere face along a latitude circle, an internal
//!    loop of that face. The splitter classified the cap bounded by the loop
//!    from the loop's centroid: the centre of the circle, which lies inside
//!    the ball on the box top, not on the sphere. Read from there the cap was
//!    on the box face: the cut was refused and the fuse silently lost the
//!    cap. The splitter now samples a point on the sphere inside the loop,
//!    as it already did on cylinder and cone walls.
//! 2. The intersect is exactly the cap closed by the disc, a two-face solid,
//!    and the boolean result gate refused every result with fewer than three
//!    faces.
//!
//! A cut or fuse with the centre below the box top (z = 12, 15) never reached
//! the first defect and is kept as a control; its intersect keeps the
//! sphere's equator seam, three faces, so the gate never refused it. z = 20 puts the equator in the box top:
//! its cut and fuse stay refused (tracked on #1790), and its intersect, the
//! lower hemisphere, now builds.
//!
//! Every oracle is a closed form: the spherical cap `πh²(3r − h)/3` and its
//! centroid `3(2r − h)²/(4(3r − h))` below the sphere centre.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const R: f64 = 10.0;
const BOX: (f64, f64, f64) = (60.0, 60.0, 20.0);

/// Height of the sphere part inside the box (below the top face, z = 20).
fn cap_height(centre_z: f64) -> f64 {
    (BOX.2 - (centre_z - R)).min(2.0 * R)
}

fn cap_volume(h: f64) -> f64 {
    PI * h * h * (3.0 * R - h) / 3.0
}

/// Distance of the cap's centroid below the sphere centre.
fn cap_centroid_depth(h: f64) -> f64 {
    3.0 * (2.0 * R - h).powi(2) / (4.0 * (3.0 * R - h))
}

fn box_sphere(op: BooleanOp, centre_z: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, BOX.0, BOX.1, BOX.2).unwrap();
    let sphere = make_sphere(&mut topo, R, 16).unwrap();
    transform_solid(
        &mut topo,
        sphere,
        &Mat4::translation(BOX.0 / 2.0, BOX.1 / 2.0, centre_z),
    )
    .unwrap();
    let body = boolean(&mut topo, op, block, sphere)
        .unwrap_or_else(|e| panic!("{op:?} at z = {centre_z}: {e}"));
    (topo, body)
}

fn assert_rel(got: f64, want: f64, what: &str) {
    assert!(
        (got - want).abs() <= 1e-9 * want.abs().max(1.0),
        "{what}: {got} vs closed form {want}"
    );
}

/// The cut and the fuse keep the lower cap on the right side of the box top.
#[test]
fn cut_and_fuse_keep_the_lower_sphere_cap() {
    let box_volume = BOX.0 * BOX.1 * BOX.2;
    let sphere_volume = 4.0 * PI * R.powi(3) / 3.0;
    for centre_z in [12.0, 15.0, 24.0, 27.0, 29.0] {
        let cap = cap_volume(cap_height(centre_z));
        for (op, want) in [
            (BooleanOp::Cut, box_volume - cap),
            (BooleanOp::Fuse, box_volume + sphere_volume - cap),
        ] {
            let what = format!("{op:?} at z = {centre_z}");
            let (topo, body) = box_sphere(op, centre_z);
            assert_rel(
                solid_volume(&topo, body, 1e-3).unwrap(),
                want,
                &format!("{what} solid_volume"),
            );
            assert_rel(
                mass_properties(&topo, body).unwrap().mass,
                want,
                &format!("{what} mass_properties"),
            );
        }
    }
}

/// The intersect is the cap closed by a disc in the box top: the original
/// sphere and the plane z = 20, with the cap's volume and centroid. A cap no
/// taller than the radius is one sphere face; a taller one also keeps the
/// sphere's equator seam and so carries both hemispheres' patches.
/// The centroid separates the lower hemisphere from the upper one at z = 20,
/// where both have the same volume.
#[test]
fn intersect_is_the_lower_cap_closed_by_a_disc() {
    for centre_z in [12.0, 15.0, 20.0, 24.0, 27.0, 29.0] {
        let what = format!("Intersect at z = {centre_z}");
        let h = cap_height(centre_z);
        let (topo, body) = box_sphere(BooleanOp::Intersect, centre_z);

        let faces = solid_faces(&topo, body).unwrap();
        let (mut spheres, mut discs) = (0, 0);
        for f in faces {
            match topo.face(f).unwrap().surface() {
                FaceSurface::Sphere(s) => {
                    assert_rel(s.radius(), R, &format!("{what} sphere radius"));
                    assert_rel(s.center().z(), centre_z, &format!("{what} sphere centre"));
                    spheres += 1;
                }
                FaceSurface::Plane { normal, d } => {
                    assert_rel(normal.z().abs(), 1.0, &format!("{what} disc normal"));
                    assert_rel(d.abs(), BOX.2, &format!("{what} disc plane"));
                    discs += 1;
                }
                other => panic!("{what}: unexpected {} face", other.type_tag()),
            }
        }
        let cap_faces = if h <= R { 1 } else { 2 };
        assert_eq!((spheres, discs), (cap_faces, 1), "{what}: cap and disc");

        let props = mass_properties(&topo, body).unwrap();
        assert_rel(props.mass, cap_volume(h), &format!("{what} volume"));
        assert_rel(
            props.center.z(),
            centre_z - cap_centroid_depth(h),
            &format!("{what} centroid z"),
        );
    }
}

/// A radius-2 sphere already cut above z = 1, then cut below z = -1. On main
/// the second cut silently returned its input (volume of the top-trimmed
/// sphere, not the slab): the ray-cast classifier approximates each sphere
/// face by its boundary polygon, so it read the trimmed sphere's interior as
/// outside and dropped the closing disc, while the centroid fallback kept the
/// lower cap. With the cap now classified on the sphere, the dropped disc
/// leaves an open shell and the exact-only boolean refuses. The cut must
/// either refuse with the typed exact-only reason or build the slab.
#[test]
fn second_cut_of_a_trimmed_sphere_refuses_or_builds_the_slab() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 2.0, 16).unwrap();
    let above = make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    transform_solid(&mut topo, above, &Mat4::translation(-10.0, -10.0, 1.0)).unwrap();
    let trimmed = boolean(&mut topo, BooleanOp::Cut, sphere, above).unwrap();
    // The first cut is exact: the sphere less its cap of height 1.
    let sphere_volume = 4.0 * PI * 8.0 / 3.0;
    let cap = PI * 1.0 * (3.0 * 2.0 - 1.0) / 3.0;
    assert_rel(
        mass_properties(&topo, trimmed).unwrap().mass,
        sphere_volume - cap,
        "top-trimmed sphere",
    );

    let below = make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    transform_solid(&mut topo, below, &Mat4::translation(-10.0, -10.0, -21.0)).unwrap();
    match boolean(&mut topo, BooleanOp::Cut, trimmed, below) {
        Ok(slab) => assert_rel(
            mass_properties(&topo, slab).unwrap().mass,
            sphere_volume - 2.0 * cap,
            "slab between z = -1 and z = 1",
        ),
        Err(error) => assert!(
            matches!(
                error,
                remus_operations::OperationsError::ExactOnlyUnattainable
            ),
            "a refusal must be the typed exact-only one: {error:?}"
        ),
    }
}

/// The sphere as the object: a radius-2 sphere at the origin cut by a box
/// covering everything above z = c, or intersected with one covering
/// everything below it, keeps the cap below z = c. Below the centre that cap
/// and the disc closing it are a two-face solid whose disc is the tool's face
/// kept reversed (so its stored edge senses match the cap's), which the face-
/// count gate must accept by effective sense.
#[test]
fn a_sphere_keeps_the_cap_below_a_box() {
    let r = 2.0_f64;
    for c in [-1.5, -1.0, -0.5, 0.5, 1.0] {
        let h = c + r;
        let want = PI * h * h * (3.0 * r - h) / 3.0;
        for (op, floor) in [(BooleanOp::Cut, c), (BooleanOp::Intersect, c - 20.0)] {
            let what = format!("{op:?} with the box from z = {floor}");
            let mut topo = Topology::new();
            let sphere = make_sphere(&mut topo, r, 16).unwrap();
            let block = make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
            transform_solid(&mut topo, block, &Mat4::translation(-10.0, -10.0, floor)).unwrap();
            let body =
                boolean(&mut topo, op, sphere, block).unwrap_or_else(|e| panic!("{what}: {e}"));
            assert_rel(mass_properties(&topo, body).unwrap().mass, want, &what);
        }
    }
}

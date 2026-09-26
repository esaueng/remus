//! A sphere face bounded by one latitude ring must be integrated on its own
//! side of that ring.
//!
//! `solid_volume` sends a sphere face whose outer wire is a constant-`v`
//! parallel to the analytic cap integrator, which picks the cap above or
//! below the ring from the ring's `u`-winding in the sphere's own frame
//! (B50, PR #612). The 2026-09-25 mutation run left that choice unpinned:
//! reversing it, or letting a full ring read as zero winding, survived the
//! whole operations suite. Two blind spots hid it:
//!
//! * A sphere centred at the origin gives both caps of a ring the same
//!   volume term, `(1/3)∫ P·n dA` with `P·n = r`, so either side "measures
//!   right". Only the centre's offset along the sphere axis separates them,
//!   so every body here is placed off the origin along that axis.
//! * The two hemispheres of a whole sphere swap together: a reversed choice
//!   integrates each hemisphere on the other's side and the sum is unchanged.
//!   A body with ONE cap face exposes a reversed choice; a whole sphere
//!   exposes a choice that puts both hemispheres on the same side.
//! * A ring that reads as zero winding falls back to projecting the ring's
//!   centroid. Off the equator that centroid sits on the axis on the cap's
//!   own side and still answers right; ON the equator it is the sphere centre
//!   and carries no side at all. The exact booleans here never produce a
//!   single-circle equatorial rim (a box face through the centre is refused
//!   as `ExactOnlyUnattainable`), so that case is a hand-assembled dome, the
//!   shape an imported part with a hemispherical boss carries.
//!
//! Oracles: closed forms (spherical cap `πh²(3r − h)/3`, box, bore), the
//! boundary-trimmed Gauss integral (`mass_properties`) as a second route,
//! and rigid-translation invariance.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder, make_sphere};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::wire::{OrientedEdge, Wire};

/// Both routes within round-off of the closed form. The analytic cap and
/// the Gauss integral are exact here (measured to 1 ulp), so any cap on the
/// wrong side of its ring, off by a whole-cap term, fails by orders of
/// magnitude.
fn assert_volume(topo: &Topology, solid: SolidId, expected: f64, what: &str) {
    let measured = solid_volume(topo, solid, 1e-3).unwrap();
    let gauss = mass_properties(topo, solid).unwrap().mass;
    for (route, v) in [("solid_volume", measured), ("mass_properties", gauss)] {
        assert!(
            (v - expected).abs() <= 1e-9 * expected,
            "{what}: {route} {v} vs closed form {expected}"
        );
    }
}

/// The number of sphere faces bounded by a single ring (no holes), the
/// premise every body here relies on.
fn ring_bounded_sphere_faces(topo: &Topology, solid: SolidId) -> usize {
    remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            matches!(face.surface(), FaceSurface::Sphere(_)) && face.inner_wires().is_empty()
        })
        .count()
}

/// Rigid placements of a finished body. Each moves the sphere centre off
/// the origin ALONG the sphere axis, the only offset that separates a cap
/// from its complement.
fn placements() -> [Mat4; 3] {
    [
        Mat4::translation(7.0, -3.0, 11.0),
        Mat4::translation(-4.0, 2.0, -9.0),
        Mat4::translation(1.0, 1.0, 25.0) * Mat4::rotation_x(0.6),
    ]
}

/// A 10×10×5 box with a spherical cap of radius 3 and height 2 on it: fused
/// on the top (a dome) or cut from the bottom (a dimple). The sphere is
/// placed with its axis up, or flipped upside down, so the cap is the upper
/// or the lower cap of its own frame and the ring is traversed each way.
fn capped_box(op: BooleanOp, flip: bool) -> (Topology, SolidId, f64) {
    let (r, h) = (3.0_f64, 2.0_f64);
    let cap = PI * h * h * (3.0 * r - h) / 3.0;
    let (centre_z, expected) = match op {
        BooleanOp::Fuse => (5.0 - (r - h), 500.0 + cap),
        BooleanOp::Cut => (-(r - h), 500.0 - cap),
        BooleanOp::Intersect => unreachable!("only a dome and a dimple are built"),
    };
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 10.0, 10.0, 5.0).unwrap();
    let sphere = make_sphere(&mut topo, r, 16).unwrap();
    let frame = if flip {
        Mat4::rotation_x(PI)
    } else {
        Mat4::identity()
    };
    transform_solid(
        &mut topo,
        sphere,
        &(Mat4::translation(5.0, 5.0, centre_z) * frame),
    )
    .unwrap();
    let body = boolean(&mut topo, op, block, sphere).unwrap();
    (topo, body, expected)
}

/// One cap face per body: its side of the ring decides the volume.
#[test]
fn a_single_sphere_cap_is_measured_on_its_own_side_of_the_ring() {
    for (op, what) in [(BooleanOp::Fuse, "dome"), (BooleanOp::Cut, "dimple")] {
        for flip in [false, true] {
            let what = format!("{what} (sphere axis flipped: {flip})");
            let (topo, body, expected) = capped_box(op, flip);
            assert_eq!(
                ring_bounded_sphere_faces(&topo, body),
                1,
                "{what}: premise: one ring-bounded cap face"
            );
            assert_volume(&topo, body, expected, &what);
            for place in placements() {
                let mut moved = topo.clone();
                transform_solid(&mut moved, body, &place).unwrap();
                assert_volume(&moved, body, expected, &format!("{what} moved"));
            }
        }
    }
}

/// A plate with a through bore (so the body takes the per-face route) and a
/// separate whole sphere: its two hemispheres meet on one equator ring and
/// must land on opposite sides of it.
#[test]
fn the_hemispheres_of_a_whole_sphere_land_on_opposite_sides_of_their_ring() {
    let r = 1.5_f64;
    let expected = 30.0 * 30.0 * 10.0 - PI * 9.0 * 10.0 + 4.0 / 3.0 * PI * r.powi(3);
    for (i, place) in [
        Mat4::translation(40.0, 5.0, 3.0),
        Mat4::translation(40.0, 5.0, 3.0) * Mat4::rotation_x(PI),
        Mat4::translation(40.0, 5.0, -7.0) * Mat4::rotation_y(0.7),
    ]
    .into_iter()
    .enumerate()
    {
        let mut topo = Topology::new();
        let plate = make_box(&mut topo, 30.0, 30.0, 10.0).unwrap();
        let bore = make_cylinder(&mut topo, 3.0, 30.0).unwrap();
        transform_solid(&mut topo, bore, &Mat4::translation(15.0, 15.0, -10.0)).unwrap();
        let bored = boolean(&mut topo, BooleanOp::Cut, plate, bore).unwrap();
        let sphere = make_sphere(&mut topo, r, 13).unwrap();
        transform_solid(&mut topo, sphere, &place).unwrap();
        let body = boolean(&mut topo, BooleanOp::Fuse, bored, sphere).unwrap();
        assert_eq!(
            ring_bounded_sphere_faces(&topo, body),
            2,
            "placement {i}: premise: two hemispheres on one ring"
        );
        assert_volume(&topo, body, expected, &format!("placement {i}"));
    }
}

/// A 10×10×5 block with a radius-3 hemispherical dome on its top, centred at
/// (5, 5, 5): the dome's only boundary is one closed circle edge on the
/// sphere's own equator, which the block's top keeps as its hole.
///
/// `rim_ccw` chooses the circle's parametric direction (counter-clockwise
/// about +z, so the dome runs it forward, or clockwise, so the dome runs it
/// reversed) and `start` the angle of its seam vertex in the sphere frame.
fn domed_block(topo: &mut Topology, rim_ccw: bool, start: f64) -> SolidId {
    let block = make_box(topo, 10.0, 10.0, 5.0).unwrap();
    let mut faces = remus_topology::explorer::solid_faces(topo, block).unwrap();
    let top = *faces
        .iter()
        .find(|&&f| {
            matches!(topo.face(f).unwrap().surface(),
                FaceSurface::Plane { normal, d } if normal.z() > 0.5 && (*d - 5.0).abs() < 1e-12)
        })
        .unwrap();
    let centre = Point3::new(5.0, 5.0, 5.0);
    let up = Vec3::new(0.0, 0.0, 1.0);
    let rim = remus_topology::builder::make_circle_edge_with_ref(
        topo,
        centre,
        if rim_ccw { up } else { -up },
        3.0,
        Vec3::new(start.cos(), start.sin(), 0.0),
        1e-7,
    )
    .unwrap();
    // The top runs the rim clockwise about +z (a hole), the dome
    // counter-clockwise about its outward normal.
    let hole = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, !rim_ccw)], true).unwrap());
    let outer = topo.face(top).unwrap().outer_wire();
    topo.set_face_boundary_wires(top, outer, vec![hole])
        .unwrap();
    let dome_rim = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, rim_ccw)], true).unwrap());
    let sphere = SphericalSurface::with_frame(centre, 3.0, up, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    faces.push(topo.add_face(Face::new(dome_rim, vec![], FaceSurface::Sphere(sphere))));
    remus_operations::sew::make_solid_from_shared_faces(topo, &faces).unwrap()
}

/// The dome is the cap ABOVE its equatorial rim for every rim direction and
/// seam angle: the rim's winding decides it, and the centroid fallback
/// cannot (on the equator it is the sphere centre).
#[test]
fn an_equatorial_dome_is_measured_above_its_rim() {
    let expected = 500.0 + 2.0 / 3.0 * PI * 27.0;
    // Seam angles: the frame's x-axis, and two where the ring's first and
    // last samples sit in the windows that a wrong closing step would push
    // below a half turn.
    for rim_ccw in [true, false] {
        for start in [0.0, -0.3 * PI, 1.1] {
            let what = format!("dome (rim counter-clockwise: {rim_ccw}, seam at {start:.3})");
            let mut topo = Topology::new();
            let body = domed_block(&mut topo, rim_ccw, start);
            assert!(
                remus_operations::validate::validate_solid(&topo, body)
                    .unwrap()
                    .is_valid(),
                "{what}: premise: the assembled dome validates"
            );
            assert_eq!(ring_bounded_sphere_faces(&topo, body), 1, "{what}");
            assert_volume(&topo, body, expected, &what);
            for place in placements() {
                let mut moved = topo.clone();
                transform_solid(&mut moved, body, &place).unwrap();
                assert_volume(&moved, body, expected, &format!("{what} moved"));
            }
        }
    }
}

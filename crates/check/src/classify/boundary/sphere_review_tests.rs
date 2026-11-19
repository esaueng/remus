//! Review reproducers for the stereographic sphere trims (#1869): a circle
//! hole's sagitta, a hole too close to the cap's far pole for one fixed
//! chart, and the boundary test on a cap whose rim the ray path decides
//! exactly. Contract: every answer is correct or an abstention, never a
//! miscount.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{FaceTrimData, count_face_ray_crossings};
use crate::CheckError;

/// A closed circle edge on the unit sphere about the origin: the circle at
/// polar angle `theta` from `axis`, counter-clockwise about `normal` (which
/// is `axis` or its negation), its seam at `reference`.
fn circle_wire(
    topo: &mut Topology,
    axis: Vec3,
    theta: f64,
    normal: Vec3,
    reference: Vec3,
) -> remus_topology::wire::WireId {
    let axis = axis.normalize().unwrap();
    let center = Point3::new(0.0, 0.0, 0.0) + axis * theta.cos();
    let circle = Circle3D::new_with_ref(center, normal, theta.sin(), reference).unwrap();
    let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut edge = Edge::with_tolerance(seam, seam, EdgeCurve::Circle(circle), Some(1e-7));
    edge.set_trim(Some((0.0, TAU)));
    let edge = topo.add_edge(edge);
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
}

fn sphere_face(
    topo: &mut Topology,
    outer: remus_topology::wire::WireId,
    holes: Vec<remus_topology::wire::WireId>,
) -> FaceId {
    topo.add_face(Face::new(
        outer,
        holes,
        FaceSurface::Sphere(SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap()),
    ))
}

fn on_sphere(polar: f64, azimuth: f64, axis_frame: [Vec3; 3]) -> Point3 {
    let [x, y, z] = axis_frame;
    Point3::new(0.0, 0.0, 0.0)
        + (x * (polar.sin() * azimuth.cos()) + y * (polar.sin() * azimuth.sin()) + z * polar.cos())
}

fn is_abstention(error: &CheckError) -> bool {
    matches!(error, CheckError::ClassificationFailed(message)
        if message == crate::classify::NURBS_RAY_GRAZE)
}

/// Crossings of the radial ray from the centre through `point`.
fn radial(topo: &Topology, face: FaceId, point: Point3) -> Result<u32, CheckError> {
    let direction = (point - Point3::new(0.0, 0.0, 0.0)).normalize().unwrap();
    count_face_ray_crossings(topo, face, Point3::new(0.0, 0.0, 0.0), direction)
}

const STANDARD: [Vec3; 3] = [
    Vec3::new(1.0, 0.0, 0.0),
    Vec3::new(0.0, 1.0, 0.0),
    Vec3::new(0.0, 0.0, 1.0),
];

/// Finding 1: the upper hemisphere less the cap `z > 0.8`, both loops one
/// closed circle edge. From the south pole the hole's image has radius 1/3;
/// its 32-sample polygon reaches only `cos(pi/32) / 3` midway between
/// samples. Hits between that chord and the circle are in the hole.
#[test]
fn circle_hole_sagitta_is_never_counted_as_material() {
    let mut topo = Topology::new();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let x = Vec3::new(1.0, 0.0, 0.0);
    let rim = circle_wire(&mut topo, z, PI / 2.0, z, x);
    let hole = circle_wire(&mut topo, z, 0.8_f64.acos(), -z, x);
    let face = sphere_face(&mut topo, rim, vec![hole]);
    // Chart radius `r` from the south pole is polar angle `2 atan(r)`.
    for (r, inside_hole) in [
        (0.332, true),
        (0.3325, true),
        (0.333, true),
        (0.3334, false),
        (0.335, false),
    ] {
        for azimuth in [PI / 32.0, -PI / 32.0, 3.0 * PI / 32.0] {
            let point = on_sphere(2.0 * f64::atan(r), azimuth, STANDARD);
            let expected = u32::from(!inside_hole);
            assert_eq!(
                radial(&topo, face, point).unwrap(),
                expected,
                "r {r} azimuth {azimuth}"
            );
        }
    }
    // On the circle's own image, the vote may abstain but never count.
    let point = on_sphere(0.8_f64.acos(), PI / 32.0, STANDARD);
    match radial(&topo, face, point) {
        Ok(count) => assert!(count <= 1),
        Err(error) => assert!(is_abstention(&error), "{error:?}"),
    }
}

/// Finding 2: the major cap of the unit sphere whose rim lies 0.005 rad
/// from the south pole, with a circular hole of angular radius 0.002 whose
/// centre is 0.008 from that pole. A chart from the pole off the cap does
/// not clear the hole. A ray from the centre to the hole centre's mirror
/// image across `z = 0` meets retained northern material: one crossing (or
/// an abstention), never the zero the axis-aligned flattening reported.
#[test]
fn hole_near_the_far_pole_keeps_the_mirror_hit() {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let x = Vec3::new(1.0, 0.0, 0.0);
    let hole_axis = Vec3::new(0.008_f64.sin(), 0.0, -0.008_f64.cos());
    for rim_normal in [z, -z] {
        let mut topo = Topology::new();
        let rim = circle_wire(&mut topo, -z, 0.005, rim_normal, x);
        let hole_normal = -hole_axis;
        let hole = circle_wire(
            &mut topo,
            hole_axis,
            0.002,
            hole_normal,
            Vec3::new(0.0, 1.0, 0.0),
        );
        let face = sphere_face(&mut topo, rim, vec![hole]);
        // Which way the rim is wound decides which cap the face is: keep the
        // winding for which the north pole is material.
        if radial(&topo, face, Point3::new(0.0, 0.0, 1.0)).unwrap() != 1 {
            continue;
        }
        let mirror = Point3::new(hole_axis.x(), hole_axis.y(), -hole_axis.z());
        match radial(&topo, face, mirror) {
            Ok(count) => assert_eq!(count, 1, "rim normal {rim_normal:?}"),
            Err(error) => assert!(is_abstention(&error), "{error:?}"),
        }
        // The hole itself is no material.
        let centre = Point3::new(0.0, 0.0, 0.0) + hole_axis;
        match radial(&topo, face, centre) {
            Ok(count) => assert_eq!(count, 0),
            Err(error) => assert!(is_abstention(&error), "{error:?}"),
        }
        return;
    }
    panic!("neither rim winding keeps the north pole");
}

/// Finding 3: the cap `z >= -0.8` of the unit sphere, its rim one closed
/// circle edge. A point of the sphere at `z = -0.8005`, midway between two
/// rim samples, is outside the cap: the boundary test must not report it on
/// the face.
#[test]
fn boundary_test_agrees_with_the_exact_cap_rim() {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let x = Vec3::new(1.0, 0.0, 0.0);
    for rim_normal in [z, -z] {
        let mut topo = Topology::new();
        let rim = circle_wire(&mut topo, -z, 0.8_f64.acos(), rim_normal, x);
        let face = sphere_face(&mut topo, rim, Vec::new());
        if radial(&topo, face, Point3::new(0.0, 0.0, 1.0)).unwrap() != 1 {
            continue;
        }
        let trim = FaceTrimData::build(&topo, face).unwrap();
        for k in 0..32 {
            let azimuth = (f64::from(k) + 0.5) * TAU / 32.0;
            let below = on_sphere((-0.8005_f64).acos(), azimuth, STANDARD);
            match crate::classify::face_trim_contains(&topo, face, &trim, None, below) {
                Ok(inside) => assert!(!inside, "azimuth {azimuth}"),
                Err(error) => assert!(is_abstention(&error), "{error:?}"),
            }
            let above = on_sphere((-0.7995_f64).acos(), azimuth, STANDARD);
            assert!(
                crate::classify::face_trim_contains(&topo, face, &trim, None, above).unwrap(),
                "azimuth {azimuth}"
            );
        }
        return;
    }
    panic!("neither rim winding keeps the north pole");
}

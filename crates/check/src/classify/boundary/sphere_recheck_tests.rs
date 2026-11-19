//! Re-check reproducers for the stereographic sphere trims (#1869, second
//! round): a spline bump between chord probes, circle samples rounded far
//! from the origin, a hole smaller than an absolute cutoff, and a rim of
//! circle arcs on two nearly equal planes. Contract: every answer is correct
//! or an abstention, never a miscount.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::{FaceTrimData, count_face_ray_crossings};
use crate::CheckError;

fn is_abstention(error: &CheckError) -> bool {
    matches!(error, CheckError::ClassificationFailed(message)
        if message == crate::classify::NURBS_RAY_GRAZE)
}

/// Correct (`expected`) or an abstention.
fn correct_or_abstain(result: Result<u32, CheckError>, expected: u32, label: &str) {
    match result {
        Ok(count) => assert_eq!(count, expected, "{label}"),
        Err(error) => assert!(is_abstention(&error), "{label}: {error:?}"),
    }
}

/// A closed circle edge `circle`, its seam at `evaluate(0)`.
fn closed_circle_wire(topo: &mut Topology, circle: Circle3D) -> WireId {
    let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut edge = Edge::with_tolerance(seam, seam, EdgeCurve::Circle(circle), Some(1e-7));
    edge.set_trim(Some((0.0, TAU)));
    let edge = topo.add_edge(edge);
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
}

fn sphere_face(
    topo: &mut Topology,
    center: Point3,
    radius: f64,
    outer: WireId,
    holes: Vec<WireId>,
) -> FaceId {
    topo.add_face(Face::new(
        outer,
        holes,
        FaceSurface::Sphere(SphericalSurface::new(center, radius).unwrap()),
    ))
}

/// The upper hemisphere of the sphere `(center, radius)`: its equator wound
/// counter-clockwise about `+z`, seamed on `+x`.
fn upper_hemisphere_rim(topo: &mut Topology, center: Point3, radius: f64) -> WireId {
    let circle = Circle3D::new_with_ref(
        center,
        Vec3::new(0.0, 0.0, 1.0),
        radius,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    closed_circle_wire(topo, circle)
}

/// Crossings of the ray from `origin` along `direction`.
fn ray(topo: &Topology, face: FaceId, origin: Point3, direction: Vec3) -> Result<u32, CheckError> {
    count_face_ray_crossings(topo, face, origin, direction.normalize().unwrap())
}

/// Inverse stereographic projection from the south pole of the unit sphere
/// onto its equatorial plane.
fn lift(x: f64, y: f64) -> Point3 {
    let s = x.mul_add(x, y * y);
    Point3::new(
        2.0 * x / (1.0 + s),
        2.0 * y / (1.0 + s),
        (1.0 - s) / (1.0 + s),
    )
}

fn polyline_edge(
    topo: &mut Topology,
    start: VertexId,
    end: VertexId,
    knots: &[f64],
    points: Vec<Point3>,
) -> remus_topology::edge::EdgeId {
    let mut full = vec![0.0, 0.0];
    full.extend_from_slice(knots);
    full.extend([1.0, 1.0]);
    let n = points.len();
    let curve = NurbsCurve::new(1, full, points, vec![1.0; n]).unwrap();
    let mut edge = Edge::with_tolerance(start, end, EdgeCurve::NurbsCurve(curve), Some(1e-7));
    edge.set_trim(Some((0.0, 1.0)));
    topo.add_edge(edge)
}

/// Finding 1: a hole whose top edge carries a narrow spline bump between the
/// interior probes of one chord. A hit inside the bump is in the hole.
#[test]
fn spline_bump_between_probes_is_never_material() {
    let mut topo = Topology::new();
    let origin = Point3::new(0.0, 0.0, 0.0);
    let rim = upper_hemisphere_rim(&mut topo, origin, 1.0);
    let corners = [(-0.15, -0.1), (0.15, -0.1), (0.15, 0.1), (-0.15, 0.1)];
    let [bl, br, tr, tl] = corners.map(|(x, y)| topo.add_vertex(Vertex::new(lift(x, y), 1e-7)));
    let x = |t: f64| 0.3f64.mul_add(t, -0.15);
    let bottom = polyline_edge(
        &mut topo,
        bl,
        br,
        &[],
        vec![lift(-0.15, -0.1), lift(0.15, -0.1)],
    );
    let right = polyline_edge(
        &mut topo,
        br,
        tr,
        &[],
        vec![lift(0.15, -0.1), lift(0.15, 0.1)],
    );
    let top = polyline_edge(
        &mut topo,
        tl,
        tr,
        &[0.5005, 0.5015, 0.5025],
        vec![
            lift(-0.15, 0.1),
            lift(x(0.5005), 0.1),
            lift(x(0.5015), 0.11),
            lift(x(0.5025), 0.1),
            lift(0.15, 0.1),
        ],
    );
    let left = polyline_edge(
        &mut topo,
        tl,
        bl,
        &[],
        vec![lift(-0.15, 0.1), lift(-0.15, -0.1)],
    );
    let hole = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(bottom, true),
                OrientedEdge::new(right, true),
                OrientedEdge::new(top, false),
                OrientedEdge::new(left, true),
            ],
            true,
        )
        .unwrap(),
    );
    let face = sphere_face(&mut topo, origin, 1.0, rim, vec![hole]);
    let towards = |p: Point3| p - origin;
    correct_or_abstain(
        ray(&topo, face, origin, towards(lift(x(0.5015), 0.105))),
        0,
        "bump",
    );
    assert_eq!(
        ray(&topo, face, origin, towards(lift(0.0, 0.0))).unwrap(),
        0,
        "hole centre"
    );
    assert_eq!(
        ray(&topo, face, origin, towards(lift(0.0, 0.2))).unwrap(),
        1,
        "material"
    );
    assert_eq!(
        ray(&topo, face, origin, towards(lift(0.4, -0.3))).unwrap(),
        1,
        "material"
    );
}

/// Finding 2: a radius-0.001 hemisphere at `(1e9, 0, 0)` with a hole above
/// `z = 0.0008`, seamed on `+x`. A hit 5e-9 below the hole's rim is
/// material; the rounding of a point a billion units out cannot resolve it,
/// so the vote may abstain but must not count it in the hole. Hits clear of
/// the rim are correct or abstain at scales 1e-3 to 1e3 and centres out to
/// 1e9, and correct near the origin.
#[test]
fn circle_samples_far_from_the_origin_keep_their_bands() {
    let azimuth = 5.85 * TAU / 256.0;
    let probe = |radius: f64, height: f64, azimuth: f64| {
        let ring = radius.mul_add(radius, -(height * height)).sqrt();
        Vec3::new(ring * azimuth.cos(), ring * azimuth.sin(), height)
    };
    let build = |center: Point3, radius: f64| {
        let mut topo = Topology::new();
        let rim = upper_hemisphere_rim(&mut topo, center, radius);
        let hole = Circle3D::new_with_ref(
            center + Vec3::new(0.0, 0.0, 0.8 * radius),
            Vec3::new(0.0, 0.0, -1.0),
            0.6 * radius,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let hole = closed_circle_wire(&mut topo, hole);
        let face = sphere_face(&mut topo, center, radius, rim, vec![hole]);
        (topo, face)
    };
    let center = Point3::new(1e9, 0.0, 0.0);
    let (topo, face) = build(center, 1e-3);
    correct_or_abstain(
        ray(&topo, face, center, probe(1e-3, 0.000_799_995, azimuth)),
        1,
        "5e-9 below the rim at 1e9",
    );
    for radius in [1e-3, 1.0, 1e3] {
        for center in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1e6, -2e6, 3e5),
            Point3::new(1e9, 0.0, 0.0),
        ] {
            let (topo, face) = build(center, radius);
            let near_origin = center.x().abs().max(center.y().abs()) / radius < 1e3;
            for k in 0..16 {
                let azimuth = (f64::from(k) + 0.37) * TAU / 16.0;
                for (height, expected) in [(0.6, 1), (0.79, 1), (0.81, 0), (0.95, 0)] {
                    let label = format!("radius {radius} centre {center:?} height {height} k {k}");
                    let result = ray(&topo, face, center, probe(radius, height * radius, azimuth));
                    if near_origin {
                        assert_eq!(result.unwrap(), expected, "{label}");
                    } else {
                        correct_or_abstain(result, expected, &label);
                    }
                }
            }
        }
    }
}

/// Finding 3: a radius-0.001 hemisphere with a hole of rim radius 4e-7
/// around its north pole. The hole must be read (or the read abstain),
/// never skipped as a loop without extent.
#[test]
fn tiny_hole_is_never_skipped() {
    let radius = 1e-3;
    let hole_radius = 4e-7;
    let origin = Point3::new(0.0, 0.0, 0.0);
    let mut topo = Topology::new();
    let rim = upper_hemisphere_rim(&mut topo, origin, radius);
    let hole = Circle3D::new_with_ref(
        Point3::new(
            0.0,
            0.0,
            radius.mul_add(radius, -(hole_radius * hole_radius)).sqrt(),
        ),
        Vec3::new(0.0, 0.0, -1.0),
        hole_radius,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let hole = closed_circle_wire(&mut topo, hole);
    let face = sphere_face(&mut topo, origin, radius, rim, vec![hole]);
    assert_eq!(
        ray(&topo, face, origin, Vec3::new(0.0, 0.0, 1.0)).unwrap(),
        0,
        "north pole"
    );
    assert_eq!(
        ray(&topo, face, origin, Vec3::new(0.3, 0.1, 0.9)).unwrap(),
        1,
        "material"
    );
}

/// Finding 4: a cap rim of two exact circle arcs on the planes `z = -0.8`
/// (for `x <= 0`) and `z = -0.8 + 2e-9 x` (for `x >= 0`). Sampled, the rim
/// passes a 2e-9 planarity test, but it is no plane section: points 3e-10
/// outside the true trim must not read inside it.
#[test]
fn rim_on_two_nearly_equal_planes_is_no_exact_cap() {
    let tilt = 2e-9;
    for reversed in [false, true] {
        let mut topo = Topology::new();
        let origin = Point3::new(0.0, 0.0, 0.0);
        let top = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.6, -0.8), 1e-12));
        let bottom = topo.add_vertex(Vertex::new(Point3::new(0.0, -0.6, -0.8), 1e-12));
        // x <= 0: the circle z = -0.8, from (0, 0.6) through -x to (0, -0.6).
        let west = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, -0.8),
            Vec3::new(0.0, 0.0, 1.0),
            0.6,
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        // x >= 0: the plane z - tilt x = -0.8, from (0, -0.6) through +x.
        let normal = Vec3::new(-tilt, 0.0, 1.0).normalize().unwrap();
        let offset = -0.8 / Vec3::new(-tilt, 0.0, 1.0).length();
        let east = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0) + normal * offset,
            normal,
            offset.mul_add(-offset, 1.0).sqrt(),
            Vec3::new(0.0, -1.0, 0.0),
        )
        .unwrap();
        let mut west_edge = Edge::with_tolerance(top, bottom, EdgeCurve::Circle(west), Some(1e-12));
        west_edge.set_trim(Some((0.0, PI)));
        let mut east_edge = Edge::with_tolerance(bottom, top, EdgeCurve::Circle(east), Some(1e-12));
        east_edge.set_trim(Some((0.0, PI)));
        let (west_edge, east_edge) = (topo.add_edge(west_edge), topo.add_edge(east_edge));
        let uses = if reversed {
            vec![
                OrientedEdge::new(east_edge, false),
                OrientedEdge::new(west_edge, false),
            ]
        } else {
            vec![
                OrientedEdge::new(west_edge, true),
                OrientedEdge::new(east_edge, true),
            ]
        };
        let rim = topo.add_wire(Wire::new(uses, true).unwrap());
        let face = sphere_face(&mut topo, origin, 1.0, rim, Vec::new());
        // Keep the winding whose face holds the north pole.
        if ray(&topo, face, origin, Vec3::new(0.0, 0.0, 1.0)).ok() != Some(1) {
            continue;
        }
        let trim = FaceTrimData::build(&topo, face).unwrap();
        // Clear of the rim, both tests decide.
        for (z, inside) in [(0.5_f64, true), (-0.5, true), (-0.9, false)] {
            let ring = z.mul_add(-z, 1.0).sqrt();
            let point = Point3::new(ring * 0.6, ring * 0.8, z);
            assert_eq!(
                crate::classify::face_trim_contains(&topo, face, &trim, None, point).unwrap(),
                inside,
                "z {z}"
            );
            assert_eq!(
                ray(&topo, face, origin, point - origin).unwrap(),
                u32::from(inside),
                "z {z}"
            );
        }
        for azimuth in [-0.9, -0.3, 0.0, 0.3, 0.9] {
            for (offset, inside) in [(-3e-10, false), (3e-10, true)] {
                // The true rim height there: z = -0.8 west of x = 0, the
                // tilted plane east of it.
                let mut z: f64 = -0.8;
                for _ in 0..4 {
                    let ring = z.mul_add(-z, 1.0).sqrt();
                    z = tilt.mul_add((ring * f64::cos(azimuth)).max(0.0), -0.8) + offset;
                }
                let ring = z.mul_add(-z, 1.0).sqrt();
                let point = Point3::new(ring * f64::cos(azimuth), ring * f64::sin(azimuth), z);
                let label = format!("azimuth {azimuth} offset {offset}");
                match crate::classify::face_trim_contains(&topo, face, &trim, None, point) {
                    Ok(found) => assert_eq!(found, inside, "{label}"),
                    Err(error) => assert!(is_abstention(&error), "{label}: {error:?}"),
                }
                correct_or_abstain(
                    ray(&topo, face, origin, point - origin),
                    u32::from(inside),
                    &label,
                );
            }
        }
        return;
    }
    panic!("neither rim winding keeps the north pole");
}

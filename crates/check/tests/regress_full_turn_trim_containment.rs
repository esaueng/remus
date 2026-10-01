//! Trim containment on the full-turn faces of the analytic primitives.
//!
//! Each face below is built exactly as `remus_operations::primitives` builds
//! it, and every expectation is closed form: a lateral wall contains every
//! surface point with `0 < z < h`, and a hemisphere bounded by the equator
//! contains exactly the points on its own side of `z = 0`.
//!
//! Two defects lived here:
//!
//! 1. `distance::point_to_face` trimmed curved faces by projecting the sampled
//!    boundary onto its best-fit plane. A cylinder wall's boundary flattens to
//!    a sliver, so `(0, ±R, h/2)` read as off the wall (distance `2.83` on
//!    `R = 2`, to the seam), and a hemisphere's equator polygon contains every
//!    projected point, so the north pole measured `0` from the SOUTH face.
//! 2. `classify::surface_point_in_face` on a pointed cone: the apex projects
//!    to an arbitrary `u` (`atan2(0, 0) = 0`) while the seam sits at
//!    `u = 3pi/2`, so the wall's UV trim came out a triangle and
//!    `(0, 1.5, 1.5)` on `cone(3, 0, 3)` tested outside its own face.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_check::classify::surface_point_in_face;
use remus_check::distance::point_to_face;
use remus_math::curves::Circle3D;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
const VERTEX_TOL: f64 = 1e-7;

/// Angles that land on the seam, on the frame's `u = 0` and between them.
fn angles() -> impl Iterator<Item = f64> {
    (0..32).map(|i| TAU * f64::from(i) / 32.0 + if i % 2 == 0 { 0.0 } else { 0.013 })
}

fn full_circle_edge(topo: &mut Topology, v: VertexId, center: Point3, radius: f64) -> EdgeId {
    let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), radius).unwrap();
    let start = circle.project(topo.vertex(v).unwrap().point());
    let mut edge = Edge::new(v, v, EdgeCurve::Circle(circle));
    edge.set_trim(Some((start, start + TAU)));
    topo.add_edge(edge)
}

/// `make_cylinder(radius, height)`'s lateral face: bottom circle, seam up, top
/// circle reversed, seam down.
fn cylinder_wall(topo: &mut Topology, radius: f64, height: f64) -> FaceId {
    let surface =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), radius)
            .unwrap();
    let v_bot = topo.add_vertex(Vertex::new(Point3::new(radius, 0.0, 0.0), VERTEX_TOL));
    let v_top = topo.add_vertex(Vertex::new(Point3::new(radius, 0.0, height), VERTEX_TOL));
    let bot = full_circle_edge(topo, v_bot, Point3::new(0.0, 0.0, 0.0), radius);
    let top = full_circle_edge(topo, v_top, Point3::new(0.0, 0.0, height), radius);
    let seam = topo.add_edge(Edge::new(v_bot, v_top, EdgeCurve::Line));
    let wire = Wire::new(
        vec![
            OrientedEdge::new(bot, true),
            OrientedEdge::new(seam, true),
            OrientedEdge::new(top, false),
            OrientedEdge::new(seam, false),
        ],
        true,
    )
    .unwrap();
    let wire = topo.add_wire(wire);
    topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(surface)))
}

/// `make_cone(radius, 0, height)`'s lateral face: base circle, seam up to the
/// apex, seam back down.
fn pointed_cone_wall(topo: &mut Topology, radius: f64, height: f64) -> FaceId {
    let apex = Point3::new(0.0, 0.0, height);
    let surface =
        ConicalSurface::new(apex, Vec3::new(0.0, 0.0, -1.0), height.atan2(radius)).unwrap();
    let v_apex = topo.add_vertex(Vertex::new(apex, VERTEX_TOL));
    let v_base = topo.add_vertex(Vertex::new(Point3::new(radius, 0.0, 0.0), VERTEX_TOL));
    let base = full_circle_edge(topo, v_base, Point3::new(0.0, 0.0, 0.0), radius);
    let seam = topo.add_edge(Edge::new(v_base, v_apex, EdgeCurve::Line));
    let wire = Wire::new(
        vec![
            OrientedEdge::new(base, true),
            OrientedEdge::new(seam, true),
            OrientedEdge::new(seam, false),
        ],
        true,
    )
    .unwrap();
    let wire = topo.add_wire(wire);
    topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(surface)))
}

/// `make_sphere(radius, segments)`'s two hemispheres, sharing an equatorial
/// polygon of line edges: `(north, south)`.
fn hemispheres(topo: &mut Topology, radius: f64, segments: u32) -> (FaceId, FaceId) {
    let verts: Vec<_> = (0..segments)
        .map(|i| {
            let theta = TAU * f64::from(i) / f64::from(segments);
            let p = Point3::new(radius * theta.cos(), radius * theta.sin(), 0.0);
            topo.add_vertex(Vertex::new(p, VERTEX_TOL))
        })
        .collect();
    let edges: Vec<_> = (0..verts.len())
        .map(|i| {
            let j = (i + 1) % verts.len();
            topo.add_edge(Edge::new(verts[i], verts[j], EdgeCurve::Line))
        })
        .collect();
    let mut hemisphere = |forward: bool| {
        let oriented: Vec<_> = if forward {
            edges.iter().map(|&e| OrientedEdge::new(e, true)).collect()
        } else {
            edges
                .iter()
                .rev()
                .map(|&e| OrientedEdge::new(e, false))
                .collect()
        };
        let wire = topo.add_wire(Wire::new(oriented, true).unwrap());
        let surface = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), radius).unwrap();
        topo.add_face(Face::new(wire, vec![], FaceSurface::Sphere(surface)))
    };
    (hemisphere(true), hemisphere(false))
}

fn assert_on_face(topo: &Topology, face: FaceId, p: Point3, scale: f64, what: &str) {
    assert!(
        surface_point_in_face(topo, face, p).unwrap(),
        "{what}: surface_point_in_face rejects {p:?} at scale {scale}"
    );
    let (dist, _) = point_to_face(topo, p, face).unwrap().unwrap();
    assert!(
        dist <= 1e-9 * scale,
        "{what}: point_to_face({p:?}) = {dist} at scale {scale}, expected 0"
    );
}

#[test]
fn cylinder_wall_contains_every_point_between_its_rims() {
    for scale in SCALES {
        let (radius, height) = (2.0 * scale, 10.0 * scale);
        let mut topo = Topology::new();
        let face = cylinder_wall(&mut topo, radius, height);
        for z in [0.05, 0.25, 0.5, 0.75, 0.95].map(|t| t * height) {
            for a in angles() {
                let p = Point3::new(radius * a.cos(), radius * a.sin(), z);
                assert_on_face(&topo, face, p, scale, "cylinder wall");
            }
        }
    }
}

#[test]
fn pointed_cone_wall_contains_every_point_between_base_and_apex() {
    for scale in SCALES {
        let (radius, height) = (3.0 * scale, 3.0 * scale);
        let mut topo = Topology::new();
        let face = pointed_cone_wall(&mut topo, radius, height);
        for t in [0.05, 0.25, 0.5, 0.75, 0.95] {
            let (z, r) = (t * height, (1.0 - t) * radius);
            for a in angles() {
                let p = Point3::new(r * a.cos(), r * a.sin(), z);
                assert_on_face(&topo, face, p, scale, "cone wall");
            }
        }
    }
}

#[test]
fn hemisphere_contains_its_own_side_of_the_equator_only() {
    for scale in SCALES {
        let radius = 3.0 * scale;
        let mut topo = Topology::new();
        let (north, south) = hemispheres(&mut topo, radius, 64);
        // Polar angle from +z; the poles and latitudes clear of the equator's
        // chord band, where the face boundary is the polygon, not the circle.
        for polar in [0.0_f64, 0.2, 0.6, 1.0, 1.3] {
            for a in angles() {
                let (s, c) = polar.sin_cos();
                let up = Point3::new(radius * s * a.cos(), radius * s * a.sin(), radius * c);
                let down = Point3::new(up.x(), up.y(), -up.z());
                for (face, other, p, what) in [
                    (north, south, up, "north hemisphere"),
                    (south, north, down, "south hemisphere"),
                ] {
                    assert_on_face(&topo, face, p, scale, what);
                    assert!(
                        !surface_point_in_face(&topo, other, p).unwrap(),
                        "{what}: {p:?} also accepted by the opposite hemisphere"
                    );
                    // Off the face, the nearest point lies on the equator.
                    let (dist, _) = point_to_face(&topo, p, other).unwrap().unwrap();
                    assert!(
                        dist >= p.z().abs() * (1.0 - 1e-12),
                        "{what}: {p:?} measured {dist} from the opposite hemisphere, \
                         closer than the equator plane"
                    );
                }
                if polar == 0.0 {
                    break; // The pole is one point, whatever the angle.
                }
            }
        }
    }
}

/// A cone sector from the apex: base arc `[a0, a1]`, line up to the apex,
/// line back down. The loop visits the apex without winding the axis, so the
/// apex becomes a segment between the two seam lines and no more.
#[test]
fn cone_sector_through_the_apex_contains_its_angular_range_only() {
    let (a0, a1) = (0.4, 2.1);
    for scale in SCALES {
        let (radius, height) = (3.0 * scale, 3.0 * scale);
        let apex = Point3::new(0.0, 0.0, height);
        let surface =
            ConicalSurface::new(apex, Vec3::new(0.0, 0.0, -1.0), height.atan2(radius)).unwrap();
        let mut topo = Topology::new();
        let base_point = |a: f64| Point3::new(radius * a.cos(), radius * a.sin(), 0.0);
        let v0 = topo.add_vertex(Vertex::new(base_point(a0), VERTEX_TOL));
        let v1 = topo.add_vertex(Vertex::new(base_point(a1), VERTEX_TOL));
        let va = topo.add_vertex(Vertex::new(apex, VERTEX_TOL));
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), radius).unwrap();
        let (t0, t1) = (
            circle.project(base_point(a0)),
            circle.project(base_point(a1)),
        );
        let mut arc = Edge::new(v0, v1, EdgeCurve::Circle(circle));
        arc.set_trim(Some((t0, if t1 > t0 { t1 } else { t1 + TAU })));
        let arc = topo.add_edge(arc);
        let up = topo.add_edge(Edge::new(v1, va, EdgeCurve::Line));
        let down = topo.add_edge(Edge::new(va, v0, EdgeCurve::Line));
        let wire = Wire::new(
            vec![
                OrientedEdge::new(arc, true),
                OrientedEdge::new(up, true),
                OrientedEdge::new(down, true),
            ],
            true,
        )
        .unwrap();
        let wire = topo.add_wire(wire);
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(surface)));

        for t in [0.05, 0.5, 0.95] {
            let (z, r) = (t * height, (1.0 - t) * radius);
            for a in angles() {
                let p = Point3::new(r * a.cos(), r * a.sin(), z);
                // Angles on the sector's own seam lines are boundary; skip them.
                if (a - a0).abs() < 1e-3 || (a - a1).abs() < 1e-3 {
                    continue;
                }
                let inside = a > a0 && a < a1;
                let got = surface_point_in_face(&topo, face, p).unwrap();
                assert_eq!(
                    got, inside,
                    "cone sector: {p:?} (angle {a}) at scale {scale}"
                );
            }
        }
    }
}

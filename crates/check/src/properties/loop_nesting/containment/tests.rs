//! Loop containment of planar faces: holes that cross or leave the outer
//! loop and inner loops that cross each other are reported; holes inside,
//! islands inside holes, and loops that touch or run tangent within
//! tolerance are not.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::{LoopContainmentDefect, face_loop_containment_defect};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e6];
const UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);
/// The validator's default tolerance, at scale.
const TOL: f64 = 1e-6;

fn p(s: f64, x: f64, y: f64) -> Point3 {
    Point3::new(x * s, y * s, 5.0 * s)
}

fn rect(t: &mut Topology, s: f64, [x0, x1, y0, y1]: [f64; 4]) -> WireId {
    let corners = [p(s, x0, y0), p(s, x1, y0), p(s, x1, y1), p(s, x0, y1)];
    make_polygon_wire(t, &corners, 1e-9 * s).unwrap()
}

/// A circle of `arcs` edges starting at angle zero.
#[allow(clippy::cast_precision_loss)]
fn circle(t: &mut Topology, s: f64, c: [f64; 2], r: f64, arcs: usize) -> WireId {
    let curve =
        Circle3D::new_with_ref(p(s, c[0], c[1]), UP, r * s, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let angle = |i: usize| TAU * i as f64 / arcs as f64;
    let vertices: Vec<_> = (0..arcs)
        .map(|i| t.add_vertex(Vertex::new(curve.evaluate(angle(i)), 1e-9 * s)))
        .collect();
    let edges = (0..arcs)
        .map(|i| {
            let mut edge = Edge::new(
                vertices[i],
                vertices[(i + 1) % arcs],
                EdgeCurve::Circle(curve.clone()),
            );
            edge.set_trim(Some((angle(i), angle(i + 1))));
            OrientedEdge::new(t.add_edge(edge), true)
        })
        .collect();
    t.add_wire(Wire::new(edges, true).unwrap())
}

fn face(t: &mut Topology, s: f64, outer: WireId, inner: Vec<WireId>) -> FaceId {
    t.add_face(Face::new(
        outer,
        inner,
        FaceSurface::Plane {
            normal: UP,
            d: 5.0 * s,
        },
    ))
}

/// Outer square `[-30, 30]²` with the given holes.
fn square_with(
    s: f64,
    holes: impl FnOnce(&mut Topology) -> Vec<WireId>,
) -> Option<LoopContainmentDefect> {
    let mut t = Topology::new();
    let outer = rect(&mut t, s, [-30.0, 30.0, -30.0, 30.0]);
    let inner = holes(&mut t);
    let f = face(&mut t, s, outer, inner);
    face_loop_containment_defect(&t, f, TOL * s)
}

#[test]
fn holes_inside_the_outer_loop_are_clean() {
    for s in SCALES {
        for arcs in [1, 3, 8] {
            let found = square_with(s, |t| {
                vec![
                    circle(t, s, [0.0, 0.0], 5.0, arcs),
                    circle(t, s, [-20.0, 20.0], 4.0, arcs),
                ]
            });
            assert_eq!(found, None, "scale {s}, {arcs} arcs");
        }
    }
}

#[test]
fn a_hole_crossing_the_outer_loop_is_reported() {
    for s in SCALES {
        for arcs in [1, 3, 8] {
            let found = square_with(s, |t| {
                vec![
                    circle(t, s, [0.0, 0.0], 5.0, arcs),
                    circle(t, s, [28.0, 0.0], 5.0, arcs),
                ]
            });
            assert_eq!(
                found,
                Some(LoopContainmentDefect::InnerOutsideOuter { inner: 1 }),
                "scale {s}, {arcs} arcs"
            );
        }
    }
}

#[test]
fn a_hole_outside_the_outer_loop_is_reported() {
    for s in SCALES {
        let found = square_with(s, |t| vec![circle(t, s, [50.0, 0.0], 5.0, 1)]);
        assert_eq!(
            found,
            Some(LoopContainmentDefect::InnerOutsideOuter { inner: 0 })
        );
    }
}

/// A hole tangent to the outer loop from inside, its seam vertex exactly on
/// the outer edge, and one poking out by a tenth of the edge tolerance: both
/// touch within tolerance and neither is reported.
#[test]
fn holes_touching_the_outer_loop_are_clean() {
    for s in SCALES {
        for arcs in [1, 4] {
            let tangent = square_with(s, |t| vec![circle(t, s, [25.0, 0.0], 5.0, arcs)]);
            assert_eq!(tangent, None, "tangent, scale {s}, {arcs} arcs");
            let grazing = square_with(s, |t| vec![circle(t, s, [25.0 + 1e-10, 0.0], 5.0, arcs)]);
            assert_eq!(grazing, None, "grazing, scale {s}, {arcs} arcs");
        }
    }
}

/// The fillet defect this check exists for: a plate's spring line 0.5 in
/// from an edge that clears a 7.75 boss by 0.137 runs 0.363 inside the
/// boss's foot circle. At the clearance itself the face is valid.
#[test]
fn a_spring_line_inside_a_boss_foot_is_reported() {
    for s in SCALES {
        let mut t = Topology::new();
        let outer = rect(&mut t, s, [-30.0, 7.887 - 0.5, -20.0, 20.0]);
        let foot = circle(&mut t, s, [0.0, 0.0], 7.75, 1);
        let f = face(&mut t, s, outer, vec![foot]);
        assert_eq!(
            face_loop_containment_defect(&t, f, TOL * s),
            Some(LoopContainmentDefect::InnerOutsideOuter { inner: 0 }),
            "scale {s}"
        );

        let mut t = Topology::new();
        let outer = rect(&mut t, s, [-30.0, 7.887, -20.0, 20.0]);
        let foot = circle(&mut t, s, [0.0, 0.0], 7.75, 1);
        let f = face(&mut t, s, outer, vec![foot]);
        assert_eq!(
            face_loop_containment_defect(&t, f, TOL * s),
            None,
            "scale {s}"
        );
    }
}

#[test]
fn crossing_inner_loops_are_reported() {
    for s in SCALES {
        let found = square_with(s, |t| {
            vec![
                circle(t, s, [0.0, 0.0], 5.0, 1),
                circle(t, s, [6.0, 0.0], 5.0, 2),
            ]
        });
        assert!(
            matches!(found, Some(LoopContainmentDefect::InnerLoopsCross { .. })),
            "scale {s}: {found:?}"
        );
    }
}

/// An island inside a hole, and two holes touching at one point, are valid
/// nestings.
#[test]
fn islands_and_touching_holes_are_clean() {
    for s in SCALES {
        let island = square_with(s, |t| {
            vec![
                circle(t, s, [0.0, 0.0], 10.0, 1),
                circle(t, s, [0.0, 0.0], 5.0, 1),
            ]
        });
        assert_eq!(island, None, "island, scale {s}");
        let touching = square_with(s, |t| {
            vec![
                circle(t, s, [-5.0, 0.0], 5.0, 1),
                circle(t, s, [5.0, 0.0], 5.0, 2),
            ]
        });
        assert_eq!(touching, None, "touching, scale {s}");
    }
}

#[test]
fn faces_without_holes_or_planes_are_not_read() {
    let mut t = Topology::new();
    let outer = rect(&mut t, 1.0, [-30.0, 30.0, -30.0, 30.0]);
    let f = face(&mut t, 1.0, outer, vec![]);
    assert_eq!(face_loop_containment_defect(&t, f, TOL), None);
}

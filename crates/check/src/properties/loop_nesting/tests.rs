//! Certified inner-loop nesting: depths on curved loops that nearly touch
//! (where fixed-count chord polygons misjudge containment), trims that hide
//! carrier spans, refusals for touching and crossing loops, the work budget
//! and the box-only cost of side-by-side holes.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::{InnerLoopNesting, NESTING_WORK_BUDGET, inner_loop_nesting, nesting_of};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e6];
const UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn p(s: f64, x: f64, y: f64) -> Point3 {
    Point3::new(x * s, y * s, 5.0 * s)
}

fn rect(t: &mut Topology, s: f64, [x0, x1, y0, y1]: [f64; 4]) -> WireId {
    let corners = [p(s, x0, y0), p(s, x1, y0), p(s, x1, y1), p(s, x0, y1)];
    make_polygon_wire(t, &corners, 1e-9 * s).unwrap()
}

/// A circle of `arcs` edges (one closed edge for `arcs == 1`).
fn circle(t: &mut Topology, s: f64, c: [f64; 2], r: f64, arcs: usize) -> WireId {
    let curve =
        Circle3D::new_with_ref(p(s, c[0], c[1]), UP, r * s, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    arcs_wire(
        t,
        s,
        arcs,
        |u| curve.evaluate(u),
        &EdgeCurve::Circle(curve.clone()),
    )
}

fn ellipse(t: &mut Topology, s: f64, c: [f64; 2], a: f64, b: f64, arcs: usize) -> WireId {
    let curve =
        Ellipse3D::new_with_ref(p(s, c[0], c[1]), UP, a * s, b * s, Vec3::new(1.0, 0.0, 0.0))
            .unwrap();
    arcs_wire(
        t,
        s,
        arcs,
        |u| curve.evaluate(u),
        &EdgeCurve::Ellipse(curve.clone()),
    )
}

#[allow(clippy::cast_precision_loss)]
fn arcs_wire(
    t: &mut Topology,
    s: f64,
    arcs: usize,
    at: impl Fn(f64) -> Point3,
    curve: &EdgeCurve,
) -> WireId {
    let angle = |i: usize| TAU * i as f64 / arcs as f64;
    let vertices: Vec<_> = (0..arcs)
        .map(|i| t.add_vertex(Vertex::new(at(angle(i)), 1e-9 * s)))
        .collect();
    let edges = (0..arcs)
        .map(|i| {
            let mut edge = Edge::new(vertices[i], vertices[(i + 1) % arcs], curve.clone());
            edge.set_trim(Some((angle(i), angle(i + 1))));
            OrientedEdge::new(t.add_edge(edge), true)
        })
        .collect();
    t.add_wire(Wire::new(edges, true).unwrap())
}

/// A closed degree-1 NURBS edge trimmed to the triangle `(0,0) (2,0) (2,2)`;
/// its unused spans run out to `(-10, 0)` and `(0, -20)`, around
/// `[-4, -2] × [-3, -1]`.
fn trimmed_nurbs_triangle(t: &mut Topology, s: f64) -> WireId {
    let control = [
        (-10.0, 0.0),
        (0.0, 0.0),
        (2.0, 0.0),
        (2.0, 2.0),
        (0.0, 0.0),
        (0.0, -20.0),
        (-10.0, 0.0),
    ]
    .map(|(x, y)| p(s, x, y))
    .to_vec();
    let knots = vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0];
    let nc = NurbsCurve::new(1, knots, control, vec![1.0; 7]).unwrap();
    let v = t.add_vertex(Vertex::new(p(s, 0.0, 0.0), 1e-9 * s));
    let mut edge = Edge::new(v, v, EdgeCurve::NurbsCurve(nc));
    edge.set_trim(Some((1.0, 4.0)));
    let edge = t.add_edge(edge);
    t.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
}

fn face(t: &mut Topology, s: f64, inner: Vec<WireId>) -> FaceId {
    let outer = rect(t, s, [-30.0, 30.0, -30.0, 30.0]);
    t.add_face(Face::new(
        outer,
        inner,
        FaceSurface::Plane {
            normal: UP,
            d: 5.0 * s,
        },
    ))
}

fn nesting(t: &Topology, f: FaceId) -> InnerLoopNesting {
    inner_loop_nesting(t, f, UP)
}

#[test]
fn concentric_loops_nearly_touching_nest_for_every_edge_split() {
    for s in SCALES {
        for (hole_arcs, island_arcs) in [(1, 1), (1, 4), (4, 1), (4, 4), (3, 7)] {
            let mut t = Topology::new();
            let hole = circle(&mut t, s, [0.0, 0.0], 10.0, hole_arcs);
            let island = circle(&mut t, s, [0.0, 0.0], 9.99, island_arcs);
            let f = face(&mut t, s, vec![hole, island]);
            assert_eq!(
                nesting(&t, f),
                InnerLoopNesting::Nested(vec![0, 1]),
                "scale {s}, arcs {hole_arcs}/{island_arcs}"
            );
        }
    }
}

#[test]
fn four_levels_in_every_order_count_their_containers() {
    let radii = [10.0, 9.99, 9.98, 9.97];
    let arcs = [1, 4, 1, 3];
    let mut order = [0usize, 1, 2, 3];
    let mut permutations = vec![order];
    // Heap's algorithm, iteratively.
    let mut c = [0usize; 4];
    let mut i = 0;
    while i < 4 {
        if c[i] < i {
            if i % 2 == 0 {
                order.swap(0, i);
            } else {
                order.swap(c[i], i);
            }
            permutations.push(order);
            c[i] += 1;
            i = 0;
        } else {
            c[i] = 0;
            i += 1;
        }
    }
    assert_eq!(permutations.len(), 24);
    for s in SCALES {
        for perm in &permutations {
            let mut t = Topology::new();
            let wires = perm
                .iter()
                .map(|&k| circle(&mut t, s, [1.0, -2.0], radii[k], arcs[k]))
                .collect();
            let f = face(&mut t, s, wires);
            let want = perm.to_vec();
            assert_eq!(
                nesting(&t, f),
                InnerLoopNesting::Nested(want),
                "scale {s}, order {perm:?}"
            );
        }
    }
}

#[test]
fn eccentric_circles_nearly_touching_nest() {
    let theta = std::f64::consts::PI / 32.0;
    for s in SCALES {
        for offset in [4.99, 4.999] {
            let mut t = Topology::new();
            let hole = circle(&mut t, s, [0.0, 0.0], 10.0, 1);
            let c = [offset * theta.cos(), offset * theta.sin()];
            let island = circle(&mut t, s, c, 5.0, 1);
            let f = face(&mut t, s, vec![island, hole]);
            assert_eq!(nesting(&t, f), InnerLoopNesting::Nested(vec![1, 0]));
        }
    }
}

#[test]
fn elliptical_island_in_elliptical_hole_nests() {
    for s in SCALES {
        let mut t = Topology::new();
        let hole = ellipse(&mut t, s, [0.0, 0.0], 10.0, 6.0, 1);
        let island = ellipse(&mut t, s, [0.0, 0.0], 9.99, 5.99, 4);
        let f = face(&mut t, s, vec![hole, island]);
        assert_eq!(nesting(&t, f), InnerLoopNesting::Nested(vec![0, 1]));
    }
}

#[test]
fn trimmed_nurbs_reads_only_its_trim() {
    for s in SCALES {
        let mut t = Topology::new();
        let triangle = trimmed_nurbs_triangle(&mut t, s);
        let beside = rect(&mut t, s, [-4.0, -2.0, -3.0, -1.0]);
        let inside = rect(&mut t, s, [1.5, 1.9, 0.2, 0.6]);
        let f = face(&mut t, s, vec![triangle, beside]);
        assert_eq!(nesting(&t, f), InnerLoopNesting::Flat, "scale {s}");
        let f = face(&mut t, s, vec![triangle, inside]);
        assert_eq!(nesting(&t, f), InnerLoopNesting::Nested(vec![0, 1]));
    }
}

#[test]
fn a_loop_in_a_box_but_outside_the_curve_is_not_nested() {
    // The square sits in the circle's box corner, outside the circle.
    for s in SCALES {
        let mut t = Topology::new();
        let round = circle(&mut t, s, [0.0, 0.0], 5.0, 1);
        let corner = rect(&mut t, s, [4.0, 4.5, 4.0, 4.5]);
        let f = face(&mut t, s, vec![round, corner]);
        assert_eq!(nesting(&t, f), InnerLoopNesting::Flat);
    }
}

#[test]
fn touching_and_crossing_loops_never_report_nesting() {
    for s in SCALES {
        // An island tangent to its hole from inside.
        let mut t = Topology::new();
        let hole = circle(&mut t, s, [0.0, 0.0], 10.0, 1);
        let island = circle(&mut t, s, [5.0, 0.0], 5.0, 4);
        let f = face(&mut t, s, vec![hole, island]);
        assert_eq!(nesting(&t, f), InnerLoopNesting::Undecided, "tangent");

        // Two circles that cross, with a genuinely nested pair beside them:
        // the crossing pair cannot be certified disjoint.
        let mut t = Topology::new();
        let a = circle(&mut t, s, [0.0, 0.0], 5.0, 1);
        let b = circle(&mut t, s, [0.0, 4.9], 5.0, 1);
        let ring = circle(&mut t, s, [0.0, 2.0], 9.0, 4);
        let f = face(&mut t, s, vec![a, b, ring]);
        assert_eq!(nesting(&t, f), InnerLoopNesting::Undecided, "crossing");

        // Crossing loops alone read as holes, the subtract-every-hole answer.
        let mut t = Topology::new();
        let a = circle(&mut t, s, [0.0, 0.0], 5.0, 1);
        let b = circle(&mut t, s, [6.0, 0.0], 5.0, 1);
        let f = face(&mut t, s, vec![a, b]);
        assert!(nesting(&t, f).nested_depths().is_none(), "two crossing");
    }
}

#[test]
fn side_by_side_holes_cost_only_box_comparisons() {
    let mut t = Topology::new();
    let mut wires = Vec::new();
    for i in 0..20_u32 {
        for j in 0..20_u32 {
            let c = [-28.5 + 3.0 * f64::from(i), -28.5 + 3.0 * f64::from(j)];
            wires.push(circle(&mut t, 1.0, c, 1.0, 1 + ((i + j) % 3) as usize));
        }
    }
    let f = face(&mut t, 1.0, wires.clone());
    assert_eq!(nesting(&t, f), InnerLoopNesting::Flat);
    let mut work = NESTING_WORK_BUDGET;
    assert_eq!(
        nesting_of(&t, &wires, UP, &mut work),
        Some(InnerLoopNesting::Flat)
    );
    assert_eq!(work, NESTING_WORK_BUDGET, "no containment work was spent");
}

#[test]
fn an_exhausted_budget_is_undecided() {
    let mut t = Topology::new();
    let hole = circle(&mut t, 1.0, [0.0, 0.0], 10.0, 1);
    let island = circle(&mut t, 1.0, [0.0, 0.0], 9.99, 4);
    let wires = [hole, island];
    let mut work = NESTING_WORK_BUDGET;
    assert_eq!(
        nesting_of(&t, &wires, UP, &mut work),
        Some(InnerLoopNesting::Nested(vec![0, 1]))
    );
    let spent = NESTING_WORK_BUDGET - work;
    assert!(
        spent > 0 && spent < NESTING_WORK_BUDGET / 16,
        "spent {spent}"
    );
    let mut short = spent - 1;
    assert_eq!(nesting_of(&t, &wires, UP, &mut short), None);
}

#[test]
fn fewer_than_two_inner_loops_are_flat() {
    let mut t = Topology::new();
    let hole = circle(&mut t, 1.0, [0.0, 0.0], 10.0, 1);
    let f = face(&mut t, 1.0, vec![hole]);
    assert_eq!(nesting(&t, f), InnerLoopNesting::Flat);
    let f = face(&mut t, 1.0, vec![]);
    assert_eq!(nesting(&t, f), InnerLoopNesting::Flat);
}

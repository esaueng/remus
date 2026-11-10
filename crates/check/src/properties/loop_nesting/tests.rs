//! Certified inner-loop nesting: depths on curved loops that nearly touch
//! (where fixed-count chord polygons misjudge containment), trims that hide
//! carrier spans, contact certificates for touching and crossing loops,
//! enclosures that refuse rather than certify past their resolution, the
//! translation invariance of the local frame, and the work counts.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::{
    InnerLoopNesting, NESTING_BASE_WORK, NestingUndecided, inner_loop_nesting,
    inner_loop_nesting_report, nesting_of, nesting_work_limit, with_nesting_work_limit,
};
use crate::properties::face_integrator::planar_face_exact_area;

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

/// A full circle as one closed rational quadratic NURBS edge.
fn nurbs_circle(t: &mut Topology, s: f64, c: [f64; 2], r: f64) -> WireId {
    let curve =
        Circle3D::new_with_ref(p(s, c[0], c[1]), UP, r * s, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let nc = remus_geometry::convert::circle_to_nurbs(&curve, 0.0, TAU).unwrap();
    let v = t.add_vertex(Vertex::new(curve.evaluate(0.0), 1e-10 * s));
    let e = t.add_edge(Edge::new(v, v, EdgeCurve::NurbsCurve(nc)));
    t.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap())
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

/// The same triangle with `unused` more carrier spans after the trim.
fn triangle_with_unused_spans(t: &mut Topology, s: f64, unused: u32) -> WireId {
    let mut control = vec![
        p(s, 0.0, 0.0),
        p(s, 2.0, 0.0),
        p(s, 2.0, 2.0),
        p(s, 0.0, 0.0),
    ];
    for i in 0..unused {
        control.push(p(s, -10.0 - f64::from(i % 2), -10.0));
    }
    control.push(p(s, 0.0, 0.0));
    #[allow(clippy::cast_precision_loss)]
    let end = (control.len() - 1) as f64;
    let mut knots = vec![0.0];
    #[allow(clippy::cast_precision_loss)]
    knots.extend((0..control.len()).map(|k| k as f64));
    knots.push(end);
    let count = control.len();
    let nc = NurbsCurve::new(1, knots, control, vec![1.0; count]).unwrap();
    let v = t.add_vertex(Vertex::new(p(s, 0.0, 0.0), 1e-10 * s));
    let mut e = Edge::new(v, v, EdgeCurve::NurbsCurve(nc));
    e.set_trim(Some((0.0, 3.0)));
    let e = t.add_edge(e);
    t.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap())
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
    inner_loop_nesting(t, f)
}

fn assert_close(got: f64, want: f64, relative: f64, what: &str) {
    assert!(
        (got - want).abs() <= relative * want.abs(),
        "{what}: {got} against {want}"
    );
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
    let theta = PI / 32.0;
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
fn touching_and_crossing_loops_are_certified_invalid() {
    for s in SCALES {
        // An island tangent to its hole from inside, at the vertices'
        // tolerance: certified contact.
        let mut t = Topology::new();
        let hole = circle(&mut t, s, [0.0, 0.0], 10.0, 1);
        let island = circle(&mut t, s, [5.0, 0.0], 5.0, 4);
        let f = face(&mut t, s, vec![hole, island]);
        assert_eq!(
            nesting(&t, f),
            InnerLoopNesting::Invalid,
            "tangent, scale {s}"
        );

        // Two circles that cross inside a ring: siblings whose crossing is
        // certified.
        let mut t = Topology::new();
        let a = circle(&mut t, s, [0.0, 0.0], 5.0, 1);
        let b = circle(&mut t, s, [0.0, 4.9], 5.0, 1);
        let ring = circle(&mut t, s, [0.0, 2.0], 9.0, 4);
        let f = face(&mut t, s, vec![a, b, ring]);
        assert_eq!(
            nesting(&t, f),
            InnerLoopNesting::Invalid,
            "crossing, scale {s}"
        );

        // Crossing loops alone hold no probe of each other: the
        // subtract-every-hole answer.
        let mut t = Topology::new();
        let a = circle(&mut t, s, [0.0, 0.0], 5.0, 1);
        let b = circle(&mut t, s, [6.0, 0.0], 5.0, 1);
        let f = face(&mut t, s, vec![a, b]);
        assert!(
            matches!(
                nesting(&t, f),
                InnerLoopNesting::Flat | InnerLoopNesting::Invalid
            ),
            "two crossing, scale {s}"
        );
    }
}

/// Side-by-side holes cost box comparisons with their sweep neighbours and
/// no containment work; everything, reading included, is charged.
#[test]
fn side_by_side_holes_cost_only_reading_and_box_comparisons() {
    let mut t = Topology::new();
    let mut wires = Vec::new();
    for i in 0..20_u32 {
        for j in 0..20_u32 {
            let c = [-28.5 + 3.0 * f64::from(i), -28.5 + 3.0 * f64::from(j)];
            wires.push(circle(&mut t, 1.0, c, 1.0, 1 + ((i + j) % 3) as usize));
        }
    }
    let f = face(&mut t, 1.0, wires);
    let report = inner_loop_nesting_report(&t, f);
    assert_eq!(report.nesting, InnerLoopNesting::Flat);
    // 400 loops of 8 to 9 pieces each; the sweep compares each loop with the
    // loops of its own column and the next.
    assert!(report.pieces >= 3200, "{report:?}");
    assert!(
        report.work <= report.pieces + 400 * 45,
        "reading plus sweep only: {report:?}"
    );
}

#[test]
fn an_exhausted_work_limit_is_undecided() {
    let mut t = Topology::new();
    let hole = circle(&mut t, 1.0, [0.0, 0.0], 10.0, 1);
    let island = ellipse(&mut t, 1.0, [0.0, 0.0], 9.99, 9.98, 4);
    let wires = [hole, island];
    let full = nesting_of(&t, &wires, UP);
    assert_eq!(full.nesting, InnerLoopNesting::Nested(vec![0, 1]));
    assert!(
        full.work > full.pieces && full.work < NESTING_BASE_WORK / 16,
        "{full:?}"
    );
    let short = with_nesting_work_limit(full.work - 1, || nesting_of(&t, &wires, UP));
    assert_eq!(
        short.nesting,
        InnerLoopNesting::Undecided(NestingUndecided::WorkLimit)
    );
    // The override is scoped.
    assert_eq!(nesting_of(&t, &wires, UP), full);
    assert_eq!(full.work_limit, nesting_work_limit(full.pieces));
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

// ── Review reproducers ─────────────────────────────────────────────────────

/// Review 1a: 200 separated circular hole/island pairs, radii 1 and 0.9999,
/// exhausted the old fixed 2²⁰-unit budget and read every island as a hole.
#[test]
fn review_glyph_pairs_certify_and_measure() {
    for s in SCALES {
        let mut t = Topology::new();
        let mut wires = Vec::new();
        for i in 0..200_u32 {
            let c = [3.0 * f64::from(i % 20), 3.0 * f64::from(i / 20)];
            wires.push(circle(&mut t, s, c, 1.0, 1));
            wires.push(circle(&mut t, s, c, 0.9999, 1));
        }
        let outer = rect(&mut t, s, [-2.0, 61.0, -2.0, 61.0]);
        let f = t.add_face(Face::new(
            outer,
            wires,
            FaceSurface::Plane {
                normal: UP,
                d: 5.0 * s,
            },
        ));
        let report = inner_loop_nesting_report(&t, f);
        let want: Vec<usize> = (0..400).map(|k| k % 2).collect();
        assert_eq!(report.nesting, InnerLoopNesting::Nested(want), "scale {s}");
        // One containment decision and one disjointness certificate per
        // pair: far below the limit.
        assert!(report.work < 200_000, "{report:?}");
        let area = planar_face_exact_area(&t, f).unwrap().unwrap();
        let expected = 200.0f64.mul_add(-PI * (1.0 - 0.9999_f64.powi(2)), 63.0 * 63.0);
        assert_close(area, expected * s * s, 1e-12, "glyph area");
        if s.to_bits() == 1.0f64.to_bits() {
            assert_close(area, 3_968.874_342_577_042, 1e-12, "review value");
        }
    }
}

/// Review 1b: concentric loops radii 10 and 9.99999999, analytic or as
/// rational NURBS, read 2971.68 (both subtracted) for 3599.999999371681.
#[test]
fn review_concentric_loops_one_part_in_a_billion_apart() {
    for s in SCALES {
        for rational in [false, true] {
            let mut t = Topology::new();
            let mut ring = |r: f64| {
                if rational {
                    nurbs_circle(&mut t, s, [0.0, 0.0], r)
                } else {
                    circle(&mut t, s, [0.0, 0.0], r, 1)
                }
            };
            let wires = vec![ring(10.0), ring(9.999_999_99)];
            let f = face(&mut t, s, wires);
            let report = inner_loop_nesting_report(&t, f);
            assert_eq!(
                report.nesting,
                InnerLoopNesting::Nested(vec![0, 1]),
                "scale {s}, rational {rational}: {report:?}"
            );
            if !rational {
                // Two circles: the annulus certificate, no refinement.
                assert!(report.work < 1_000, "{report:?}");
            }
            let area = planar_face_exact_area(&t, f).unwrap().unwrap();
            let want = PI.mul_add(-(100.0 - 9.999_999_99_f64.powi(2)), 3600.0) * s * s;
            assert_close(area, want, 1e-12, "concentric area");
        }
    }
}

/// Review 1c: unused carrier spans past a degree-one NURBS trim made the
/// triangle-with-island face undecided (3597.84 for 3598.16).
#[test]
fn review_unused_nurbs_spans_cost_nothing() {
    for s in SCALES {
        let mut reports = Vec::new();
        for unused in [1, 5_000] {
            let mut t = Topology::new();
            let triangle = triangle_with_unused_spans(&mut t, s, unused);
            let island = rect(&mut t, s, [1.5, 1.9, 0.2, 0.6]);
            let f = face(&mut t, s, vec![triangle, island]);
            let report = inner_loop_nesting_report(&t, f);
            assert_eq!(report.nesting, InnerLoopNesting::Nested(vec![0, 1]));
            let area = planar_face_exact_area(&t, f).unwrap().unwrap();
            assert_close(area, 3598.16 * s * s, 1e-12, "triangle with island");
            reports.push(report);
        }
        // The trim reads three spans whatever the carrier holds.
        assert_eq!(reports[0].pieces, reports[1].pieces);
        assert_eq!(reports[0].work, reports[1].work);
    }
}

/// Review 2: a square island half-width 9.999999 in a hole half-width 10
/// was nested near the origin and undecided translated by (1e6, 1e6).
#[test]
fn review_translation_keeps_the_decision() {
    fn specimen(offset: f64) -> (Topology, FaceId) {
        let mut t = Topology::new();
        let mut square = |r: f64| {
            make_polygon_wire(
                &mut t,
                &[
                    Point3::new(offset - r, offset - r, 5.0),
                    Point3::new(offset + r, offset - r, 5.0),
                    Point3::new(offset + r, offset + r, 5.0),
                    Point3::new(offset - r, offset + r, 5.0),
                ],
                1e-10,
            )
            .unwrap()
        };
        let outer = square(30.0);
        let hole = square(10.0);
        let island = square(9.999_999);
        let f = t.add_face(Face::new(
            outer,
            vec![hole, island],
            FaceSurface::Plane { normal: UP, d: 5.0 },
        ));
        (t, f)
    }
    for offset in [0.0, 1e3, 1e6, 1e9] {
        let (t, f) = specimen(offset);
        assert_eq!(
            nesting(&t, f),
            InnerLoopNesting::Nested(vec![0, 1]),
            "offset {offset}"
        );
        let area = planar_face_exact_area(&t, f).unwrap().unwrap();
        assert_close(area, 3_599.999_920_000_001, 1e-6, "translated square");
    }
}

/// A shallow arc of a circle of radius `r` centred at `(-r, 0)`, trimmed to
/// `±half` and closed by lines to `x = -10`, with a rectangle `island`.
fn shallow_arc_face(r: f64, half: f64, island: [f64; 4], left: bool) -> (Topology, FaceId) {
    let mut t = Topology::new();
    let c =
        Circle3D::new_with_ref(Point3::new(-r, 0.0, 5.0), UP, r, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let (a, b) = (c.evaluate(-half), c.evaluate(half));
    let x = if left { -10.0 } else { 10.0 };
    let corners = [a, b, Point3::new(x, b.y(), 5.0), Point3::new(x, a.y(), 5.0)];
    let verts: Vec<_> = corners
        .iter()
        .map(|&q| t.add_vertex(Vertex::new(q, 1e-10)))
        .collect();
    let mut arc = Edge::new(verts[0], verts[1], EdgeCurve::Circle(c));
    arc.set_trim(Some((-half, half)));
    let mut edges = vec![OrientedEdge::new(t.add_edge(arc), true)];
    for i in 1..4 {
        edges.push(OrientedEdge::new(
            t.add_edge(Edge::new(verts[i], verts[(i + 1) % 4], EdgeCurve::Line)),
            true,
        ));
    }
    let hole = t.add_wire(Wire::new(edges, true).unwrap());
    let island = rect(&mut t, 1.0, island);
    let extent = 2.0 * (b.y().abs() + 20.0);
    let outer = rect(&mut t, 1.0, [-30.0, 30.0, -extent, extent]);
    let f = t.add_face(Face::new(
        outer,
        vec![hole, island],
        FaceSurface::Plane { normal: UP, d: 5.0 },
    ));
    (t, f)
}

/// Review 3: a shallow arc of a circle of radius 1e12 lost its curvature to
/// cancellation and certified a rectangle the true arc crosses as nested.
#[test]
fn review_shallow_conic_is_never_certified_past_its_resolution() {
    let (t, f) = shallow_arc_face(1e12, 1e-9, [-2e-7, -1e-7, 600.0, 800.0], true);
    let answer = nesting(&t, f);
    assert!(
        !matches!(answer, InnerLoopNesting::Nested(_)),
        "crossing rectangle certified: {answer:?}"
    );
}

/// Families of shallow arcs, radius 1e6 to 1e15, sagitta 1e-9 to 1: a
/// rectangle clearly inside the hole, straddling the arc, or clearly
/// outside. Inside and outside are certified wherever the gap exceeds the
/// arc's evaluation error many times over; a straddling one is never
/// `Nested`; nothing is ever certified wrong.
#[test]
fn shallow_arc_families_certify_only_what_they_resolve() {
    let mut certified = 0;
    let mut refused = 0;
    for exponent in [6, 9, 12, 15] {
        let r = 10f64.powi(exponent);
        for sagitta in [1e-9, 1e-6, 1e-3, 1.0] {
            // Half-angle with r (1 − cos h) = sagitta.
            let half = 2.0 * (0.5 * sagitta / r).sqrt().asin();
            let y = r * half.sin();
            // True arc x at the rectangle's rows, |y| ≤ y/2.
            let x_at = |row: f64| -row * row / (r + (r * r - row * row).sqrt());
            let (y0, y1) = (0.25 * y, 0.5 * y);
            let near = x_at(y1); // the arc's x at the rectangle's far row
            let error = 32.0 * f64::EPSILON * 2.0 * r;
            let width = (0.05 * sagitta).max(1e-12);
            for (case, x0, x1) in [
                (
                    "inside",
                    near - 2.0 * width - 0.5 * sagitta,
                    near - width - 0.5 * sagitta,
                ),
                ("straddling", x_at(y0) - width, x_at(y0) + width),
                ("outside", 2.0 * width + 0.0, 3.0 * width),
            ] {
                let (t, f) = shallow_arc_face(r, half, [x0, x1, y0, y1], true);
                let answer = nesting(&t, f);
                let resolvable = (x1 - near).abs().min((x0 - near).abs()) > 1e3 * error
                    && (x0 - x_at(y0)).abs().min((x1 - x_at(y0)).abs()) > 1e3 * error;
                match (case, &answer) {
                    ("inside", InnerLoopNesting::Nested(d)) => {
                        assert_eq!(d, &vec![0, 1]);
                        certified += 1;
                    }
                    ("outside", InnerLoopNesting::Flat) => certified += 1,
                    ("straddling", InnerLoopNesting::Nested(_)) => {
                        panic!("straddling certified nested: r {r}, sagitta {sagitta}")
                    }
                    (_, InnerLoopNesting::Undecided(_) | InnerLoopNesting::Invalid) => {
                        assert!(
                            case == "straddling" || !resolvable,
                            "{case} refused although resolvable: r {r}, sagitta {sagitta}, {answer:?}"
                        );
                        refused += 1;
                    }
                    ("straddling", InnerLoopNesting::Flat) => refused += 1,
                    _ => panic!("{case} read {answer:?}: r {r}, sagitta {sagitta}"),
                }
            }
        }
    }
    assert!(
        certified > 0 && refused > 0,
        "{certified} certified, {refused} refused"
    );
}

/// The local frame: a seeded family of nested faces reads the same nesting
/// and the same work at every translation the input can still represent.
#[test]
fn translated_nested_faces_read_the_same_nesting() {
    let mut seed = 0x9e37_79b9_7f4a_7c15_u64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        #[allow(clippy::cast_precision_loss)]
        let unit = (seed >> 11) as f64 / (1u64 << 53) as f64;
        unit
    };
    for case in 0..24 {
        let gap = 10f64.powf(-2.0 - 5.0 * next());
        let arcs = 1 + (case % 4);
        let centre = [next() - 0.5, next() - 0.5];
        let mut reference = None;
        for shift in [0.0, 1e3, 1e6, 3e7] {
            let mut t = Topology::new();
            let c = [centre[0] + shift, centre[1] - shift];
            let hole = circle(&mut t, 1.0, c, 10.0, arcs);
            let island = ellipse(&mut t, 1.0, c, 10.0 - gap, 9.0, 1 + (case % 3));
            let inner = rect(
                &mut t,
                1.0,
                [c[0] - 1.0, c[0] + 1.0, c[1] - 1.0, c[1] + 1.0],
            );
            let outer = rect(
                &mut t,
                1.0,
                [c[0] - 30.0, c[0] + 30.0, c[1] - 30.0, c[1] + 30.0],
            );
            let f = t.add_face(Face::new(
                outer,
                vec![inner, hole, island],
                FaceSurface::Plane { normal: UP, d: 5.0 },
            ));
            let report = inner_loop_nesting_report(&t, f);
            assert_eq!(
                report.nesting,
                InnerLoopNesting::Nested(vec![2, 0, 1]),
                "case {case}, gap {gap}, shift {shift}"
            );
            let work = report.work;
            match reference {
                None => reference = Some(work),
                // Rounding of the translated input may move a split; the
                // work stays within a few percent.
                Some(w) => assert!(
                    work.abs_diff(w) * 20 <= w,
                    "case {case}: work {work} against {w}"
                ),
            }
        }
    }
}

/// `N` separated hole/island pairs cost `O(N)`: ten times the pairs, about
/// ten times the work, far below the face's limit.
#[test]
fn separated_pairs_cost_grows_linearly() {
    let pairs = |count: u32| {
        let mut t = Topology::new();
        let mut wires = Vec::new();
        for i in 0..count {
            let c = [3.0 * f64::from(i % 50), 3.0 * f64::from(i / 50)];
            wires.push(circle(&mut t, 1.0, c, 1.0, 1));
            wires.push(ellipse(&mut t, 1.0, c, 0.9999, 0.9, 2));
        }
        let outer = rect(&mut t, 1.0, [-2.0, 152.0, -2.0, 152.0]);
        let f = t.add_face(Face::new(
            outer,
            wires,
            FaceSurface::Plane { normal: UP, d: 5.0 },
        ));
        let report = inner_loop_nesting_report(&t, f);
        let want: Vec<usize> = (0..2 * count as usize).map(|k| k % 2).collect();
        assert_eq!(report.nesting, InnerLoopNesting::Nested(want));
        report
    };
    let (small, large) = (pairs(200), pairs(2000));
    assert!(
        large.work <= 12 * small.work,
        "200 pairs {small:?}, 2000 pairs {large:?}"
    );
    assert!(large.work * 8 < large.work_limit, "{large:?}");
}

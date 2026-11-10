//! Unnested profile matrix replayed verbatim on main before the nesting repair.
#![allow(clippy::unwrap_used, clippy::expect_used)]
use super::*;
use remus_math::{
    curves::{Circle3D, Ellipse3D},
    nurbs::NurbsCurve,
};
use remus_topology::{
    builder::{make_circle_edge, make_polygon_wire},
    explorer::solid_entity_counts,
};
use std::f64::consts::{PI, TAU};

fn p(x: f64, y: f64) -> Point3 {
    Point3::new(x, y, 5.)
}
fn z() -> Vec3 {
    Vec3::new(0., 0., 1.)
}
fn rect(t: &mut Topology, x: f64, cw: bool) -> WireId {
    let mut points = vec![p(x - 1., 0.), p(x + 1., 0.), p(x + 1., 2.), p(x - 1., 2.)];
    if cw {
        points.reverse();
    }
    make_polygon_wire(t, &points, 1e-9).unwrap()
}
fn circle(t: &mut Topology, x: f64, pieces: usize) -> WireId {
    let c = Circle3D::new_with_ref(p(x, 0.), z(), 1., Vec3::new(1., 0., 0.)).unwrap();
    if pieces == 1 {
        let e = make_circle_edge(t, p(x, 0.), z(), 1., 1e-9).unwrap();
        return t.add_wire(Wire::new(vec![OrientedEdge::new(e, false)], true).unwrap());
    }
    let vertices: Vec<_> = (0..pieces)
        .map(|i| {
            t.add_vertex(Vertex::new(
                c.evaluate(i as f64 * TAU / pieces as f64),
                1e-9,
            ))
        })
        .collect();
    let mut edges = Vec::new();
    for i in 0..pieces {
        let mut edge = Edge::new(
            vertices[i],
            vertices[(i + 1) % pieces],
            EdgeCurve::Circle(c.clone()),
        );
        edge.set_trim(Some((
            i as f64 * TAU / pieces as f64,
            (i + 1) as f64 * TAU / pieces as f64,
        )));
        edges.push(OrientedEdge::new(t.add_edge(edge), false));
    }
    edges.reverse();
    t.add_wire(Wire::new(edges, true).unwrap())
}
fn rounded(t: &mut Topology, h: f64, r: f64, x: f64, cw: bool) -> WireId {
    // A stadium: straight centre section [-h,h] with semicircular ends.
    let points = [p(x - h, -r), p(x + h, -r), p(x + h, r), p(x - h, r)];
    let vertices: Vec<_> = points
        .iter()
        .map(|&q| t.add_vertex(Vertex::new(q, 1e-9)))
        .collect();
    let mut edges = Vec::new();
    for i in 0..4 {
        let curve = if i % 2 == 0 {
            EdgeCurve::Line
        } else {
            EdgeCurve::Circle(
                Circle3D::new_with_ref(
                    p(x + if i == 1 { h } else { -h }, 0.),
                    z(),
                    r,
                    Vec3::new(1., 0., 0.),
                )
                .unwrap(),
            )
        };
        let mut edge = Edge::new(vertices[i], vertices[(i + 1) % 4], curve);
        if i % 2 == 1 {
            let a = if i == 1 { -PI / 2. } else { PI / 2. };
            edge.set_trim(Some((a, a + PI)));
        }
        edges.push(OrientedEdge::new(t.add_edge(edge), true));
    }
    if cw {
        edges = edges
            .into_iter()
            .rev()
            .map(|e| OrientedEdge::new(e.edge(), false))
            .collect();
    }
    t.add_wire(Wire::new(edges, true).unwrap())
}
fn rounded_rectangle(t: &mut Topology) -> WireId {
    let points = [
        p(-18., -20.),
        p(18., -20.),
        p(20., -18.),
        p(20., 18.),
        p(18., 20.),
        p(-18., 20.),
        p(-20., 18.),
        p(-20., -18.),
    ];
    let vertices: Vec<_> = points
        .iter()
        .map(|&q| t.add_vertex(Vertex::new(q, 1e-9)))
        .collect();
    let centers = [p(18., -18.), p(18., 18.), p(-18., 18.), p(-18., -18.)];
    let edges = (0..8)
        .map(|i| {
            let mut edge = if i % 2 == 0 {
                Edge::new(vertices[i], vertices[(i + 1) % 8], EdgeCurve::Line)
            } else {
                let c =
                    Circle3D::new_with_ref(centers[i / 2], z(), 2., Vec3::new(1., 0., 0.)).unwrap();
                let mut edge = Edge::new(vertices[i], vertices[(i + 1) % 8], EdgeCurve::Circle(c));
                let a = (i / 2) as f64 * PI / 2. - PI / 2.;
                edge.set_trim(Some((a, a + PI / 2.)));
                edge
            };
            if i % 2 == 0 {
                edge.set_trim(Some((0., 1.)));
            }
            OrientedEdge::new(t.add_edge(edge), true)
        })
        .collect();
    t.add_wire(Wire::new(edges, true).unwrap())
}
fn profile(t: &mut Topology, family: usize, sweep: usize) -> (FaceId, f64) {
    let outer = match family {
        1 => rounded_rectangle(t),
        2 => rounded(t, 15., 5., 0., false),
        3 => {
            let c =
                Ellipse3D::new_with_ref(p(0., 0.), z(), 20., 15., Vec3::new(1., 0., 0.)).unwrap();
            let vertices: Vec<_> = (0..4)
                .map(|i| t.add_vertex(Vertex::new(c.evaluate(i as f64 * PI / 2.), 1e-9)))
                .collect();
            let edges = (0..4)
                .map(|i| {
                    let mut edge = Edge::new(
                        vertices[i],
                        vertices[(i + 1) % 4],
                        EdgeCurve::Ellipse(c.clone()),
                    );
                    edge.set_trim(Some((i as f64 * PI / 2., (i + 1) as f64 * PI / 2.)));
                    OrientedEdge::new(t.add_edge(edge), true)
                })
                .collect();
            t.add_wire(Wire::new(edges, true).unwrap())
        }
        _ => {
            let w = make_polygon_wire(
                t,
                &[p(-20., -20.), p(20., -20.), p(20., 20.), p(-20., 20.)],
                1e-9,
            )
            .unwrap();
            if family == 0 || family == 4 {
                for oe in t.wire(w).unwrap().edges().to_vec() {
                    let e = t.edge(oe.edge()).unwrap();
                    let a = t.vertex(e.start()).unwrap().point();
                    let b = t.vertex(e.end()).unwrap().point();
                    let curve = if family == 0 {
                        NurbsCurve::new(1, vec![0., 0., 1., 1.], vec![a, b], vec![1.; 2]).unwrap()
                    } else {
                        let delta = b - a;
                        let mid = a + delta * 0.5 + Vec3::new(delta.y(), -delta.x(), 0.) * 0.05;
                        NurbsCurve::new(
                            2,
                            vec![0., 0., 0., 1., 1., 1.],
                            vec![a, mid, b],
                            vec![1.; 3],
                        )
                        .unwrap()
                    };
                    t.edge_mut(oe.edge())
                        .unwrap()
                        .set_curve(EdgeCurve::NurbsCurve(curve));
                }
            }
            w
        }
    };
    let holes = match family {
        0 => vec![rect(t, -7., true), rect(t, 7., true)],
        5 => vec![rounded(t, 1., 1., -7., true), rounded(t, 1., 1., 7., true)],
        _ => {
            let pieces = match family {
                6 => 2,
                7 => 4,
                _ => {
                    if (1..=3).contains(&sweep) {
                        4
                    } else {
                        1
                    }
                }
            };
            vec![circle(t, -7., pieces), circle(t, 7., pieces)]
        }
    };
    let outer_area = match family {
        1 => 1584. + 4. * PI,
        2 => 300. + 25. * PI,
        3 => 300. * PI,
        4 => 5120. / 3.,
        _ => 1600.,
    };
    let hole_area = match family {
        0 => 8.,
        5 => 8. + 2. * PI,
        _ => 2. * PI,
    };
    (
        t.add_face(Face::new(
            outer,
            holes,
            FaceSurface::Plane { normal: z(), d: 5. },
        )),
        outer_area - hole_area,
    )
}
// Obtained by running these same constructions in an isolated checkout of
// main d9d51f524388731ea178f73832225fa1de4d4101. Order is straight,
// taper, twist, herringbone, partial revolve. None is main's ellipse refusal.
const MAIN_COUNTS: [[Option<(usize, usize, usize)>; 5]; 8] = [
    [
        Some((14, 36, 24)),
        Some((14, 36, 24)),
        Some((14, 36, 24)),
        Some((26, 60, 36)),
        Some((14, 36, 24)),
    ],
    [
        Some((12, 30, 20)),
        Some((18, 48, 32)),
        Some((18, 48, 32)),
        Some((34, 80, 48)),
        Some((46, 132, 88)),
    ],
    [
        Some((8, 18, 12)),
        Some((14, 36, 24)),
        Some((14, 36, 24)),
        Some((26, 60, 36)),
        Some((42, 120, 80)),
    ],
    [Some((8, 18, 12)), None, None, None, Some((42, 120, 80))],
    [
        Some((8, 18, 12)),
        Some((14, 36, 24)),
        Some((14, 36, 24)),
        Some((26, 60, 36)),
        Some((42, 120, 80)),
    ],
    [
        Some((14, 36, 24)),
        Some((14, 36, 24)),
        Some((14, 36, 24)),
        Some((26, 60, 36)),
        Some((14, 36, 24)),
    ],
    [
        Some((10, 24, 16)),
        Some((10, 24, 16)),
        Some((10, 24, 16)),
        Some((18, 40, 24)),
        Some((10, 24, 16)),
    ],
    [
        Some((14, 36, 24)),
        Some((14, 36, 24)),
        Some((14, 36, 24)),
        Some((26, 60, 36)),
        Some((14, 36, 24)),
    ],
];

// Existing curved revolutions are construction compatibility checks only.
// Their measured volumes already disagree with Pappus on main. Keeping
// those observations separate avoids claiming geometric qualification.
const MAIN_REVOLVE_MASS: [f64; 8] = [
    9_551.999_999_982_721,
    9_562.709_709_073_653,
    504.469_703_258_701,
    4_181.250_595_000_443,
    9_761.538_999_444_214,
    9_541.597_698_164_831,
    9_589.597_698_164_616,
    9_572.220_009_191_311,
];

fn verify_family(family: usize, sweeps: std::ops::Range<usize>) {
    for sweep in sweeps {
        let mut t = Topology::new();
        let (face, area) = profile(&mut t, family, sweep);
        let result = match sweep {
            0 => extrude(&mut t, face, z(), 2.),
            1 => taper::tapered_extrude_impl(&mut t, face, p(0., 0.) + z() * 15., z(), 2.)
                .map(|r| r.solid),
            2 => twist::twisted_extrude_impl(&mut t, face, p(0., 0.), z(), 2., 0.01, 1e-7, 256)
                .map(|r| r.solid),
            3 => twist::herringbone_extrude_impl(&mut t, face, p(0., 0.), z(), 2., 0.01, 1e-7, 256)
                .map(|r| r.solid),
            _ => crate::revolve::revolve(&mut t, face, p(-30., 0.), Vec3::new(0., 1., 0.), 0.2),
        };
        if MAIN_COUNTS[family][sweep].is_none() {
            let error = result.unwrap_err();
            assert!(
                error
                    .to_string()
                    .contains("cannot sweep an ellipse or open conic edge")
            );
            continue;
        }
        let solid = result.unwrap();
        let counts = solid_entity_counts(&t, solid).unwrap();
        assert_eq!(Some(counts), MAIN_COUNTS[family][sweep]);
        let volume = remus_check::properties::solid_properties(
            &t,
            solid,
            &remus_check::properties::PropertiesOptions::default(),
        )
        .unwrap()
        .mass;
        let factor = match sweep {
            1 => 2. * (1. + 13. / 15. + (13_f64 / 15.).powi(2)) / 3.,
            4 => 6.,
            _ => 2.,
        };
        if sweep == 4 && family > 0 {
            assert!((volume - MAIN_REVOLVE_MASS[family]).abs() < 1e-7);
        } else {
            assert!(
                (volume - area * factor).abs() < 1e-8 * area,
                "family={family} sweep={sweep} volume={volume} expected={}",
                area * factor
            );
        }
        if sweep == 0 {
            assert!(
                (crate::measure::solid_volume(&t, solid, 0.02).unwrap() - area * 2.).abs() < 1e-7
            );
            assert!(
                crate::validate::validate_solid(&t, solid)
                    .unwrap()
                    .issues
                    .is_empty()
            );
        }
    }
}
macro_rules! family_test {
    ($name:ident, $family:expr) => {
        #[test]
        fn $name() {
            verify_family($family, 0..4);
        }
    };
}
family_test!(unnested_nurbs_rectangle_matches_main, 0);
family_test!(unnested_rounded_rectangle_matches_main, 1);
family_test!(unnested_slot_outer_matches_main, 2);
family_test!(unnested_elliptic_outer_matches_main, 3);
family_test!(unnested_spline_outer_matches_main, 4);
family_test!(unnested_slot_holes_match_main, 5);
family_test!(unnested_two_arc_holes_match_main, 6);
family_test!(unnested_four_arc_holes_match_main, 7);
#[test]
fn revolve_profile_construction_matches_main() {
    for family in 0..8 {
        verify_family(family, 4..5);
    }
}

#[test]
fn nested_split_circle_and_slot_containment() {
    for pieces in [2, 4] {
        let mut t = Topology::new();
        let outer = rounded_rectangle(&mut t);
        let hole = rounded(&mut t, 3., 3., 0., true);
        let island = circle(&mut t, 0., pieces);
        // Containment is independent of incoming wire winding.
        let face = t.add_face(Face::new(
            outer,
            vec![island, hole],
            FaceSurface::Plane { normal: z(), d: 5. },
        ));
        let regions = nesting::profile_regions(&t, face).unwrap();
        assert_eq!(regions.depths, vec![1, 0]);
        assert_eq!(regions.parents, vec![Some(1), None]);
        let solid = extrude(&mut t, face, z(), 2.).unwrap();
        let expected = 2. * (1584. + 4. * PI - (36. + 9. * PI) + PI);
        assert!((crate::measure::solid_volume(&t, solid, 0.01).unwrap() - expected).abs() < 1e-7);
    }
}

#[test]
fn overlapping_circle_boxes_are_decided_on_the_arcs() {
    let mut t = Topology::new();
    let outer = rounded_rectangle(&mut t);
    let a = circle(&mut t, 0., 4);
    let b = circle(&mut t, 1.5, 2);
    // Translate b vertically. The boxes overlap, but the circles are separate.
    let edges = t.wire(b).unwrap().edges().to_vec();
    let mut vertices = Vec::new();
    for oe in edges {
        let e = t.edge(oe.edge()).unwrap();
        vertices.push(e.start());
        let EdgeCurve::Circle(c) = e.curve() else {
            unreachable!()
        };
        let curve = Circle3D::new_with_ref(
            c.center() + Vec3::new(0., 1.5, 0.),
            c.normal(),
            c.radius(),
            c.u_axis(),
        )
        .unwrap();
        let trim = e.trim();
        t.edge_mut(oe.edge())
            .unwrap()
            .set_curve(EdgeCurve::Circle(curve));
        t.edge_mut(oe.edge()).unwrap().set_trim(trim);
    }
    for v in vertices {
        let p = t.vertex(v).unwrap().point();
        t.vertex_mut(v)
            .unwrap()
            .set_point(p + Vec3::new(0., 1.5, 0.));
    }
    let face = t.add_face(Face::new(
        outer,
        vec![a, b],
        FaceSurface::Plane { normal: z(), d: 5. },
    ));
    assert_eq!(
        nesting::profile_regions(&t, face).unwrap().depths,
        vec![0, 0]
    );
}

#[test]
fn nested_affine_nurbs_uses_the_existing_exact_line_certificate() {
    let mut topology = Topology::new();
    let outer = rounded_rectangle(&mut topology);
    let hole = rounded(&mut topology, 3., 3., 0., true);
    let island = rect(&mut topology, 0., false);
    let edge = topology.wire(island).unwrap().edges()[0].edge();
    let data = topology.edge(edge).unwrap();
    let a = topology.vertex(data.start()).unwrap().point();
    let b = topology.vertex(data.end()).unwrap().point();
    let curve = NurbsCurve::new(1, vec![0., 0., 1., 1.], vec![a, b], vec![1.; 2]).unwrap();
    topology
        .edge_mut(edge)
        .unwrap()
        .set_curve(EdgeCurve::NurbsCurve(curve));
    let face = topology.add_face(Face::new(
        outer,
        vec![hole, island],
        FaceSurface::Plane { normal: z(), d: 5. },
    ));
    assert_eq!(
        nesting::profile_regions(&topology, face).unwrap().depths,
        vec![0, 1]
    );
    let solid = extrude(&mut topology, face, z(), 2.).unwrap();
    assert!(
        (crate::measure::solid_volume(&topology, solid, 0.01).unwrap() - (3104. - 10. * PI)).abs()
            < 1e-7
    );
    assert!(
        crate::validate::validate_solid(&topology, solid)
            .unwrap()
            .issues
            .is_empty()
    );
}

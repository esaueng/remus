//! Planar faces whose inner loops nest, or nearly do, measured against closed
//! forms at three scales through the public entry points: the exact planar
//! area, `measure::face_area`, and for a pocketed cylinder `solid_volume`,
//! `solid_properties` and `face_area` together.
//!
//! The cases are the ones a fixed-count chord polygon misjudges: an island
//! that nearly fills its hole (whatever the edge split), eccentric circles
//! that nearly touch, an elliptical island in an elliptical hole, three
//! levels of nesting listed in every order, and a trimmed NURBS hole whose
//! unused carrier spans surround a neighbouring hole. Reversed and tilted
//! faces measure the same.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_check::properties::face_integrator::planar_face_exact_area;
use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e6];

/// A plane at `scale`: `at(x, y)` is `origin + scale·(x·u + y·v)`.
struct Placement {
    origin: Point3,
    u: Vec3,
    v: Vec3,
    n: Vec3,
    scale: f64,
}

impl Placement {
    fn flat(scale: f64) -> Self {
        Self {
            origin: Point3::new(0.0, 0.0, 5.0 * scale),
            u: Vec3::new(1.0, 0.0, 0.0),
            v: Vec3::new(0.0, 1.0, 0.0),
            n: Vec3::new(0.0, 0.0, 1.0),
            scale,
        }
    }

    /// Rotated about x by 0.4, then about z by 0.7, and shifted.
    fn tilted(scale: f64) -> Self {
        let rotate = |p: Vec3| {
            let (sa, ca) = 0.4_f64.sin_cos();
            let (sb, cb) = 0.7_f64.sin_cos();
            let (x, y, z) = (p.x(), ca * p.y() - sa * p.z(), sa * p.y() + ca * p.z());
            Vec3::new(cb * x - sb * y, sb * x + cb * y, z)
        };
        Self {
            origin: Point3::new(3.0 * scale, -2.0 * scale, 1.5 * scale),
            u: rotate(Vec3::new(1.0, 0.0, 0.0)),
            v: rotate(Vec3::new(0.0, 1.0, 0.0)),
            n: rotate(Vec3::new(0.0, 0.0, 1.0)),
            scale,
        }
    }

    fn at(&self, x: f64, y: f64) -> Point3 {
        self.origin + self.u * (x * self.scale) + self.v * (y * self.scale)
    }

    fn surface(&self) -> FaceSurface {
        let o = self.origin;
        FaceSurface::Plane {
            normal: self.n,
            d: self.n.dot(Vec3::new(o.x(), o.y(), o.z())),
        }
    }

    fn rect(&self, topo: &mut Topology, [x0, x1, y0, y1]: [f64; 4]) -> WireId {
        let corners = [
            self.at(x0, y0),
            self.at(x1, y0),
            self.at(x1, y1),
            self.at(x0, y1),
        ];
        make_polygon_wire(topo, &corners, 1e-9 * self.scale).unwrap()
    }

    /// A circle of `arcs` edges (one closed edge for `arcs == 1`).
    fn circle(&self, topo: &mut Topology, c: (f64, f64), r: f64, arcs: usize) -> WireId {
        let curve =
            Circle3D::new_with_ref(self.at(c.0, c.1), self.n, r * self.scale, self.u).unwrap();
        self.arcs(
            topo,
            arcs,
            &|t| curve.evaluate(t),
            &EdgeCurve::Circle(curve.clone()),
        )
    }

    fn ellipse(&self, topo: &mut Topology, c: (f64, f64), ab: (f64, f64), arcs: usize) -> WireId {
        let curve = Ellipse3D::new_with_ref(
            self.at(c.0, c.1),
            self.n,
            ab.0 * self.scale,
            ab.1 * self.scale,
            self.u,
        )
        .unwrap();
        self.arcs(
            topo,
            arcs,
            &|t| curve.evaluate(t),
            &EdgeCurve::Ellipse(curve.clone()),
        )
    }

    #[allow(clippy::cast_precision_loss)]
    fn arcs(
        &self,
        topo: &mut Topology,
        arcs: usize,
        at: &dyn Fn(f64) -> Point3,
        curve: &EdgeCurve,
    ) -> WireId {
        let angle = |i: usize| TAU * i as f64 / arcs as f64;
        let vertices: Vec<_> = (0..arcs)
            .map(|i| topo.add_vertex(Vertex::new(at(angle(i)), 1e-9 * self.scale)))
            .collect();
        let edges = (0..arcs)
            .map(|i| {
                let mut edge = Edge::new(vertices[i], vertices[(i + 1) % arcs], curve.clone());
                edge.set_trim(Some((angle(i), angle(i + 1))));
                OrientedEdge::new(topo.add_edge(edge), true)
            })
            .collect();
        topo.add_wire(Wire::new(edges, true).unwrap())
    }

    /// A closed degree-1 NURBS edge trimmed to the triangle `(0,0) (2,0)
    /// (2,2)` (area 2); its unused spans run out to `(-10, 0)` and
    /// `(0, -20)`, around `[-4, -2] × [-3, -1]`.
    fn trimmed_nurbs_triangle(&self, topo: &mut Topology) -> WireId {
        let control = [
            (-10.0, 0.0),
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 0.0),
            (0.0, -20.0),
            (-10.0, 0.0),
        ]
        .map(|(x, y)| self.at(x, y))
        .to_vec();
        let knots = vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0];
        let nc = NurbsCurve::new(1, knots, control, vec![1.0; 7]).unwrap();
        let v = topo.add_vertex(Vertex::new(self.at(0.0, 0.0), 1e-9 * self.scale));
        let mut edge = Edge::new(v, v, EdgeCurve::NurbsCurve(nc));
        edge.set_trim(Some((1.0, 4.0)));
        let edge = topo.add_edge(edge);
        topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
    }

    /// A face on `[-h, h]²` with these inner loops.
    fn face(&self, topo: &mut Topology, h: f64, inner: Vec<WireId>, reversed: bool) -> FaceId {
        let outer = self.rect(topo, [-h, h, -h, h]);
        let face = if reversed {
            Face::new_reversed(outer, inner, self.surface())
        } else {
            Face::new(outer, inner, self.surface())
        };
        topo.add_face(face)
    }
}

fn assert_close(got: f64, want: f64, label: &str) {
    assert!(
        (got - want).abs() <= 1e-9 * want.abs(),
        "{label}: got {got:.15e}, want {want:.15e} (error {:.3e})",
        got - want
    );
}

/// The exact planar area and `measure::face_area`, both against `want`
/// (given at unit scale).
fn assert_face_area(topo: &Topology, face: FaceId, scale: f64, want: f64, label: &str) {
    let want = want * scale * scale;
    let exact = planar_face_exact_area(topo, face)
        .unwrap()
        .expect("the exact planar path takes these boundaries");
    assert_close(exact.abs(), want, &format!("{label}: exact area"));
    let area = measure::face_area(topo, face, 0.01 * scale).unwrap();
    assert_close(area, want, &format!("{label}: face_area"));
}

#[test]
fn disjoint_trimmed_nurbs_hole_and_rectangle_both_subtract() {
    // 2500 − 2 − 4: the rectangle lies inside the NURBS carrier's unused
    // spans, never inside the trimmed triangle.
    for scale in SCALES {
        for pl in [Placement::flat(scale), Placement::tilted(scale)] {
            let mut topo = Topology::new();
            let triangle = pl.trimmed_nurbs_triangle(&mut topo);
            let rectangle = pl.rect(&mut topo, [-4.0, -2.0, -3.0, -1.0]);
            for inner in [vec![triangle, rectangle], vec![rectangle, triangle]] {
                let face = pl.face(&mut topo, 25.0, inner, false);
                assert_face_area(&topo, face, scale, 2494.0, "trimmed NURBS");
            }
            // An island truly inside the triangle is added back.
            let island = pl.rect(&mut topo, [1.5, 1.9, 0.2, 0.6]);
            let face = pl.face(&mut topo, 25.0, vec![island, triangle], false);
            assert_face_area(
                &topo,
                face,
                scale,
                2500.0 - 2.0 + 0.16,
                "trimmed NURBS island",
            );
        }
    }
}

#[test]
fn island_nearly_filling_its_hole_adds_back_for_every_edge_split() {
    // Radius ratio 0.999: 1600 − π(10² − 9.99²).
    let want = 1600.0 - PI * (100.0 - 9.99_f64.powi(2));
    for scale in SCALES {
        for (hole_arcs, island_arcs) in [(1, 1), (1, 4), (4, 1), (4, 4), (3, 7)] {
            let pl = Placement::flat(scale);
            let mut topo = Topology::new();
            let hole = pl.circle(&mut topo, (0.0, 0.0), 10.0, hole_arcs);
            let island = pl.circle(&mut topo, (0.0, 0.0), 9.99, island_arcs);
            let face = pl.face(&mut topo, 20.0, vec![hole, island], false);
            let label = format!("arcs {hole_arcs}/{island_arcs}, scale {scale}");
            assert_face_area(&topo, face, scale, want, &label);
        }
    }
}

#[test]
fn eccentric_circles_nearly_touching_add_back() {
    // An r = 5 island in an r = 10 hole, its centre `offset` out along
    // π/32: 1600 − 100π + 25π, with a gap of 5 − offset.
    let theta = PI / 32.0;
    for scale in SCALES {
        for offset in [4.99, 4.999] {
            let pl = Placement::flat(scale);
            let mut topo = Topology::new();
            let hole = pl.circle(&mut topo, (0.0, 0.0), 10.0, 1);
            let c = (offset * theta.cos(), offset * theta.sin());
            let island = pl.circle(&mut topo, c, 5.0, 1);
            let face = pl.face(&mut topo, 20.0, vec![hole, island], false);
            let label = format!("offset {offset}, scale {scale}");
            assert_face_area(&topo, face, scale, 1600.0 - 75.0 * PI, &label);
        }
    }
}

#[test]
fn elliptical_island_in_elliptical_hole_adds_back() {
    // 1600 − π(10·6 − 9.99·5.99).
    let want = 1600.0 - PI * (60.0 - 9.99 * 5.99);
    for scale in SCALES {
        for (hole_arcs, island_arcs) in [(1, 1), (1, 4), (2, 3)] {
            let pl = Placement::flat(scale);
            let mut topo = Topology::new();
            let hole = pl.ellipse(&mut topo, (0.0, 0.0), (10.0, 6.0), hole_arcs);
            let island = pl.ellipse(&mut topo, (0.0, 0.0), (9.99, 5.99), island_arcs);
            let face = pl.face(&mut topo, 20.0, vec![hole, island], false);
            let label = format!("arcs {hole_arcs}/{island_arcs}, scale {scale}");
            assert_face_area(&topo, face, scale, want, &label);
        }
    }
}

/// Every ordering of `0..4`.
fn permutations() -> Vec<[usize; 4]> {
    let mut out = Vec::new();
    for a in 0..4 {
        for b in 0..4 {
            for c in 0..4 {
                for d in 0..4 {
                    let p = [a, b, c, d];
                    if (0..4).all(|k| p.contains(&k)) {
                        out.push(p);
                    }
                }
            }
        }
    }
    out
}

/// Hole r = 10, island 9.99, hole 9.98, island 9.97 (three levels below the
/// first hole), listed in `order`, nearly touching.
fn three_levels(topo: &mut Topology, pl: &Placement, order: [usize; 4]) -> Vec<WireId> {
    const RADII: [f64; 4] = [10.0, 9.99, 9.98, 9.97];
    const ARCS: [usize; 4] = [1, 4, 1, 3];
    order
        .iter()
        .map(|&k| pl.circle(topo, (1.0, -2.0), RADII[k], ARCS[k]))
        .collect()
}

fn three_levels_area() -> f64 {
    1600.0 - PI * (100.0 - 9.99_f64.powi(2) + 9.98_f64.powi(2) - 9.97_f64.powi(2))
}

#[test]
fn three_levels_of_nesting_in_every_order() {
    let orders = permutations();
    assert_eq!(orders.len(), 24);
    for scale in SCALES {
        let pl = Placement::flat(scale);
        let mut topo = Topology::new();
        for order in &orders {
            let inner = three_levels(&mut topo, &pl, *order);
            let face = pl.face(&mut topo, 20.0, inner, false);
            let label = format!("order {order:?}, scale {scale}");
            assert_face_area(&topo, face, scale, three_levels_area(), &label);
        }
    }
}

#[test]
fn reversed_and_tilted_faces_measure_the_same() {
    for scale in SCALES {
        for pl in [Placement::flat(scale), Placement::tilted(scale)] {
            for reversed in [false, true] {
                let mut topo = Topology::new();
                let inner = three_levels(&mut topo, &pl, [2, 0, 3, 1]);
                let face = pl.face(&mut topo, 20.0, inner, reversed);
                let label = format!("reversed {reversed}, scale {scale}");
                assert_face_area(&topo, face, scale, three_levels_area(), &label);
            }
        }
    }
}

// ── A pocketed cylinder with a standing island ─────────────────────────────

/// A full circle on `z`, radius `r`, with its one seam vertex.
fn ring(topo: &mut Topology, r: f64, z: f64) -> (VertexId, EdgeId) {
    let c = Circle3D::new_with_ref(
        Point3::new(0.0, 0.0, z),
        Vec3::new(0.0, 0.0, 1.0),
        r,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let v = topo.add_vertex(Vertex::new(c.evaluate(0.0), 1e-9 * r));
    let mut e = Edge::new(v, v, EdgeCurve::Circle(c));
    e.set_trim(Some((0.0, TAU)));
    (v, topo.add_edge(e))
}

fn ring_wire(topo: &mut Topology, edge: EdgeId, forward: bool) -> WireId {
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, forward)], true).unwrap())
}

fn wall(
    topo: &mut Topology,
    (r, z0): (f64, f64),
    bottom: (VertexId, EdgeId),
    top: (VertexId, EdgeId),
    reversed: bool,
) -> FaceId {
    let seam = topo.add_edge(Edge::new(bottom.0, top.0, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(bottom.1, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(top.1, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    let surface = FaceSurface::Cylinder(
        CylindricalSurface::new(Point3::new(0.0, 0.0, z0), Vec3::new(0.0, 0.0, 1.0), r).unwrap(),
    );
    topo.add_face(if reversed {
        Face::new_reversed(wire, vec![], surface)
    } else {
        Face::new(wire, vec![], surface)
    })
}

/// Cylinder r = 10, h = 3 (scaled), with an r = 8 pocket from z = 1 and an
/// r = 2 island standing in it to the top: 180π s³, top face 40π s².
fn pocketed_cylinder(topo: &mut Topology, s: f64) -> (SolidId, FaceId) {
    let b0 = ring(topo, 10.0 * s, 0.0);
    let b3 = ring(topo, 10.0 * s, 3.0 * s);
    let h1 = ring(topo, 8.0 * s, s);
    let h3 = ring(topo, 8.0 * s, 3.0 * s);
    let i1 = ring(topo, 2.0 * s, s);
    let i3 = ring(topo, 2.0 * s, 3.0 * s);
    let up = Vec3::new(0.0, 0.0, 1.0);
    let outer = ring_wire(topo, b3.1, true);
    let pocket = ring_wire(topo, h3.1, false);
    let island = ring_wire(topo, i3.1, true);
    let top = topo.add_face(Face::new(
        outer,
        vec![pocket, island],
        FaceSurface::Plane {
            normal: up,
            d: 3.0 * s,
        },
    ));
    let floor_outer = ring_wire(topo, h1.1, true);
    let floor_island = ring_wire(topo, i1.1, false);
    let floor = topo.add_face(Face::new(
        floor_outer,
        vec![floor_island],
        FaceSurface::Plane { normal: up, d: s },
    ));
    let bottom_wire = ring_wire(topo, b0.1, false);
    let bottom = topo.add_face(Face::new(
        bottom_wire,
        vec![],
        FaceSurface::Plane {
            normal: -up,
            d: 0.0,
        },
    ));
    let faces = vec![
        top,
        floor,
        bottom,
        wall(topo, (10.0 * s, 0.0), b0, b3, false),
        wall(topo, (8.0 * s, s), h1, h3, true),
        wall(topo, (2.0 * s, s), i1, i3, false),
    ];
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    (topo.add_solid(Solid::new(shell, vec![])), top)
}

#[test]
fn pocketed_cylinder_volume_properties_and_area_agree() {
    for s in SCALES {
        let mut topo = Topology::new();
        let (solid, top) = pocketed_cylinder(&mut topo, s);
        let volume = 180.0 * PI * s.powi(3);
        let label = format!("scale {s}");
        assert_close(
            measure::solid_volume(&topo, solid, 0.01 * s).unwrap(),
            volume,
            &format!("{label}: solid_volume"),
        );
        let props = remus_check::properties::solid_properties(
            &topo,
            solid,
            &remus_check::properties::PropertiesOptions::default(),
        )
        .unwrap();
        assert_close(props.mass, volume, &format!("{label}: solid_properties"));
        assert_close(
            measure::face_area(&topo, top, 0.01 * s).unwrap(),
            40.0 * PI * s * s,
            &format!("{label}: top face_area"),
        );
    }
}

/// The pocketed cylinder read warm through the face-integral cache equals
/// its cold reading bit for bit, and with its nesting left undecided it is
/// measured on its mesh (to the chord error), never with the island
/// subtracted; `solid_properties`, which has no mesh route, refuses.
#[test]
fn pocketed_cylinder_reads_the_same_warm_and_falls_back_to_the_mesh() {
    use remus_check::properties::face_cache::{
        clear_thread_face_cache, enable_thread_face_cache, set_thread_face_cache_limits,
    };
    use remus_check::properties::loop_nesting::with_nesting_work_limit;
    for s in SCALES {
        let mut topo = Topology::new();
        let (solid, top) = pocketed_cylinder(&mut topo, s);
        let options = remus_check::properties::PropertiesOptions::default();
        let read = |topo: &Topology| {
            (
                measure::face_area(topo, top, 0.01 * s).unwrap(),
                measure::solid_volume(topo, solid, 0.01 * s).unwrap(),
                remus_check::properties::solid_properties(topo, solid, &options)
                    .unwrap()
                    .mass,
            )
        };
        let cold = read(&topo);
        enable_thread_face_cache();
        clear_thread_face_cache();
        let primed = read(&topo);
        let warm = read(&topo);
        set_thread_face_cache_limits(0, 0);
        for reading in [primed, warm] {
            assert_eq!(
                reading.0.to_bits(),
                cold.0.to_bits(),
                "scale {s}: face_area"
            );
            assert_eq!(
                reading.1.to_bits(),
                cold.1.to_bits(),
                "scale {s}: solid_volume"
            );
            assert_eq!(
                reading.2.to_bits(),
                cold.2.to_bits(),
                "scale {s}: solid_properties"
            );
        }

        let deflection = 1e-3 * s;
        let (area, volume, props) = with_nesting_work_limit(0, || {
            (
                measure::face_area(&topo, top, deflection).unwrap(),
                measure::solid_volume(&topo, solid, deflection).unwrap(),
                remus_check::properties::solid_properties(&topo, solid, &options),
            )
        });
        // Chords on the three rims (radii 10, 8, 2), each within the
        // deflection of its arc.
        let rims = TAU * 20.0 * s;
        let top_area = 40.0 * PI * s * s;
        assert!(
            (area - top_area).abs() <= deflection * rims,
            "scale {s}: fallback top area {area} against {top_area}"
        );
        let solid_want = 180.0 * PI * s.powi(3);
        assert!(
            (volume - solid_want).abs() <= deflection * rims * 3.0 * s * 2.0,
            "scale {s}: fallback volume {volume} against {solid_want}"
        );
        assert!(
            matches!(props, Err(remus_check::CheckError::UndecidedLoopNesting(_))),
            "scale {s}: {props:?}"
        );
    }
}

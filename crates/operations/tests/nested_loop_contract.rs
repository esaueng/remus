//! The planar inner-loop nesting contract, end to end.
//!
//! A planar face with two or more inner wires is measured with a certified
//! nesting (exact arithmetic, islands added back), as an invalid face
//! (touching loops: every inner wire subtracted, as always), or, where the
//! nesting cannot be certified, on its mesh: never with every inner wire
//! guessed to be a hole.
//!
//! The first tests are the reproducers of the second review of PR 1639, run
//! through entry points that already existed at its head c02ec634 (where
//! each of them fails); then the mesh fallback against the tessellator; then
//! a seeded randomized differential over nested layouts with their areas
//! known by construction.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr
)]

use std::f64::consts::{PI, TAU};

use remus_check::properties::face_cache::{
    clear_thread_face_cache, enable_thread_face_cache, integrate_face_volume_about_memoized,
    set_thread_face_cache_limits,
};
use remus_check::properties::face_integrator::{
    integrate_face_fixed_about, planar_face_exact_area,
};
use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_operations::{measure, tessellate};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e6];
const UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// A plane: `at(x, y)` is `origin + scale·(x·u + y·v)`.
#[derive(Clone, Copy)]
struct Placement {
    origin: Point3,
    u: Vec3,
    v: Vec3,
    n: Vec3,
    scale: f64,
}

impl Placement {
    const fn flat(scale: f64) -> Self {
        Self {
            origin: Point3::new(0.0, 0.0, 5.0 * scale),
            u: Vec3::new(1.0, 0.0, 0.0),
            v: Vec3::new(0.0, 1.0, 0.0),
            n: UP,
            scale,
        }
    }

    /// Rotated by `a` about x and `b` about z, then moved to `origin`.
    fn tilted(scale: f64, a: f64, b: f64, origin: Point3) -> Self {
        let rotate = |p: Vec3| {
            let (sa, ca) = a.sin_cos();
            let (sb, cb) = b.sin_cos();
            let (x, y, z) = (p.x(), ca * p.y() - sa * p.z(), sa * p.y() + ca * p.z());
            Vec3::new(cb * x - sb * y, sb * x + cb * y, z)
        };
        Self {
            origin,
            u: rotate(Vec3::new(1.0, 0.0, 0.0)),
            v: rotate(Vec3::new(0.0, 1.0, 0.0)),
            n: rotate(UP),
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

    fn tolerance(&self) -> f64 {
        1e-10 * self.scale
    }

    fn polygon(&self, topo: &mut Topology, points: &[(f64, f64)]) -> WireId {
        let points: Vec<_> = points.iter().map(|&(x, y)| self.at(x, y)).collect();
        make_polygon_wire(topo, &points, self.tolerance()).unwrap()
    }

    fn rect(&self, topo: &mut Topology, [x0, x1, y0, y1]: [f64; 4]) -> WireId {
        self.polygon(topo, &[(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
    }

    fn square(&self, topo: &mut Topology, c: (f64, f64), h: f64) -> WireId {
        self.rect(topo, [c.0 - h, c.0 + h, c.1 - h, c.1 + h])
    }

    /// A square as one closed degree-1 NURBS edge trimmed to it, with
    /// `unused` carrier spans past the trim running far outside the face.
    fn nurbs_square(&self, topo: &mut Topology, c: (f64, f64), h: f64, unused: u32) -> WireId {
        let mut control = vec![
            self.at(c.0 - h, c.1 - h),
            self.at(c.0 + h, c.1 - h),
            self.at(c.0 + h, c.1 + h),
            self.at(c.0 - h, c.1 + h),
            self.at(c.0 - h, c.1 - h),
        ];
        for i in 0..unused {
            control.push(self.at(-500.0 - f64::from(i % 2), 400.0));
        }
        control.push(self.at(c.0 - h, c.1 - h));
        let count = control.len();
        #[allow(clippy::cast_precision_loss)]
        let end = (count - 1) as f64;
        let mut knots = vec![0.0];
        #[allow(clippy::cast_precision_loss)]
        knots.extend((0..count).map(|k| k as f64));
        knots.push(end);
        let nc = NurbsCurve::new(1, knots, control, vec![1.0; count]).unwrap();
        let v = topo.add_vertex(Vertex::new(self.at(c.0 - h, c.1 - h), self.tolerance()));
        let mut edge = Edge::new(v, v, EdgeCurve::NurbsCurve(nc));
        edge.set_trim(Some((0.0, 4.0)));
        let e = topo.add_edge(edge);
        topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap())
    }

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

    fn rational_circle(&self, topo: &mut Topology, c: (f64, f64), r: f64) -> WireId {
        let curve =
            Circle3D::new_with_ref(self.at(c.0, c.1), self.n, r * self.scale, self.u).unwrap();
        let nc = remus_geometry::convert::circle_to_nurbs(&curve, 0.0, TAU).unwrap();
        let v = topo.add_vertex(Vertex::new(curve.evaluate(0.0), self.tolerance()));
        let e = topo.add_edge(Edge::new(v, v, EdgeCurve::NurbsCurve(nc)));
        topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap())
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
            .map(|i| topo.add_vertex(Vertex::new(at(angle(i)), self.tolerance())))
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

fn assert_close(got: f64, want: f64, relative: f64, label: &str) {
    assert!(
        (got - want).abs() <= relative * want.abs(),
        "{label}: got {got:.15e}, want {want:.15e}"
    );
}

/// The exact planar area and `measure::face_area`, both against `want`.
fn assert_measures(topo: &Topology, face: FaceId, want: f64, label: &str) {
    let exact = planar_face_exact_area(topo, face).unwrap();
    let exact = exact.unwrap_or_else(|| panic!("{label}: the exact path declined"));
    assert_close(exact.abs(), want, 1e-12, &format!("{label}: exact area"));
    let area = measure::face_area(topo, face, 1e-3).unwrap();
    assert_close(area, want, 1e-12, &format!("{label}: face_area"));
}

// ── Review reproducers (each fails at c02ec634) ────────────────────────────

/// Review 1a: 200 separated hole/island pairs exhausted a fixed global budget: the
/// head read 2712.488595987033 for 3_968.874_342_577_042.
#[test]
fn review_glyph_pairs_measure_their_islands() {
    for s in SCALES {
        let pl = Placement::flat(s);
        let mut topo = Topology::new();
        let mut inner = Vec::new();
        for i in 0..200_u32 {
            let c = (
                3.0 * f64::from(i % 20) - 29.5,
                3.0 * f64::from(i / 20) - 29.5,
            );
            inner.push(pl.circle(&mut topo, c, 1.0, 1));
            inner.push(pl.circle(&mut topo, c, 0.9999, 1));
        }
        let face = pl.face(&mut topo, 31.5, inner, false);
        let want = 200.0f64.mul_add(-PI * (1.0 - 0.9999_f64.powi(2)), 63.0 * 63.0);
        assert_measures(&topo, face, want * s * s, &format!("glyphs, scale {s}"));
        if s.to_bits() == 1.0f64.to_bits() {
            assert_close(want, 3_968.874_342_577_042, 1e-15, "review value");
        }
    }
}

/// Review 1b: Concentric loops radii 10 and 9.99999999, analytic or rational NURBS:
/// the head read 2971.681469910355 for 3599.999999371681.
#[test]
fn review_concentric_loops_a_billionth_apart() {
    for s in SCALES {
        for rational in [false, true] {
            let pl = Placement::flat(s);
            let mut topo = Topology::new();
            let mut ring = |r: f64| {
                if rational {
                    pl.rational_circle(&mut topo, (0.0, 0.0), r)
                } else {
                    pl.circle(&mut topo, (0.0, 0.0), r, 1)
                }
            };
            let inner = vec![ring(10.0), ring(9.999_999_99)];
            let face = pl.face(&mut topo, 30.0, inner, false);
            let want = PI.mul_add(-(100.0 - 9.999_999_99_f64.powi(2)), 3600.0);
            assert_measures(
                &topo,
                face,
                want * s * s,
                &format!("concentric, rational {rational}, scale {s}"),
            );
        }
    }
}

/// Review 1c: 5,000 unused carrier spans past a degree-one NURBS trim made the
/// triangle-with-island face undecided: 3597.84 for 3598.16.
#[test]
fn review_unused_carrier_spans_change_nothing() {
    for s in SCALES {
        let pl = Placement::flat(s);
        let mut topo = Topology::new();
        let mut control = vec![
            pl.at(0.0, 0.0),
            pl.at(2.0, 0.0),
            pl.at(2.0, 2.0),
            pl.at(0.0, 0.0),
        ];
        for i in 0..5000 {
            control.push(pl.at(-10.0 - f64::from(i % 2), -10.0));
        }
        control.push(pl.at(0.0, 0.0));
        let count = control.len();
        #[allow(clippy::cast_precision_loss)]
        let end = (count - 1) as f64;
        let mut knots = vec![0.0];
        #[allow(clippy::cast_precision_loss)]
        knots.extend((0..count).map(|k| k as f64));
        knots.push(end);
        let nc = NurbsCurve::new(1, knots, control, vec![1.0; count]).unwrap();
        let v = topo.add_vertex(Vertex::new(pl.at(0.0, 0.0), pl.tolerance()));
        let mut edge = Edge::new(v, v, EdgeCurve::NurbsCurve(nc));
        edge.set_trim(Some((0.0, 3.0)));
        let e = topo.add_edge(edge);
        let triangle = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
        let island = pl.rect(&mut topo, [1.5, 1.9, 0.2, 0.6]);
        let face = pl.face(&mut topo, 30.0, vec![triangle, island], false);
        assert_measures(
            &topo,
            face,
            3598.16 * s * s,
            &format!("unused spans, scale {s}"),
        );
    }
}

/// The review's square island (half-width 9.999999) in a hole (half-width
/// 10) in a square of half-width 30, centred at `(offset, offset, 5)`.
fn review_squares(offset: f64) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let mut square = |r: f64| {
        make_polygon_wire(
            &mut topo,
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
    let face = topo.add_face(Face::new(
        outer,
        vec![hole, island],
        FaceSurface::Plane { normal: UP, d: 5.0 },
    ));
    (topo, face)
}

/// Review 2: The decision depended on absolute position: the translated face read
/// 2800.000080000602, and priming the face cache with it made the origin
/// face's volume term read 4666.666800001004 for 5999.999866666668.
#[test]
fn review_translated_face_and_face_cache() {
    let (near, near_face) = review_squares(0.0);
    let (far, far_face) = review_squares(1e6);
    for (topo, face, offset) in [(&near, near_face, 0.0), (&far, far_face, 1e6)] {
        let exact = planar_face_exact_area(topo, face).unwrap().unwrap();
        assert_close(exact, 3_599.999_920_000_001, 1e-12, "translated exact area");
        // `face_area` takes the sampled Newell sum for straight edges, which
        // sums products of world coordinates (as on `main`): its rounding
        // grows as `ε · offset²`, far below the island's 800.
        let area = measure::face_area(topo, face, 1e-3).unwrap();
        let world = 64.0 * f64::EPSILON * (offset + 30.0f64).powi(2);
        assert!(
            (area - 3_599.999_920_000_001).abs() <= world + 1e-9,
            "translated face_area {area} at {offset}"
        );
    }
    let reference = Point3::new(0.0, 0.0, 0.0);
    let cold_near = integrate_face_fixed_about(&near, near_face, 5, reference).unwrap();
    let cold_far = integrate_face_fixed_about(&far, far_face, 5, reference).unwrap();
    for prime_far in [true, false] {
        enable_thread_face_cache();
        clear_thread_face_cache();
        let (first, second, cold) = if prime_far {
            ((&far, far_face), (&near, near_face), &cold_near)
        } else {
            ((&near, near_face), (&far, far_face), &cold_far)
        };
        integrate_face_volume_about_memoized(first.0, first.1, 5, reference).unwrap();
        let warm = integrate_face_volume_about_memoized(second.0, second.1, 5, reference).unwrap();
        set_thread_face_cache_limits(0, 0);
        // A translated reading is the source's re-expressed about the new
        // reference: equal to integrating afresh up to the input's rounding.
        assert_close(warm.area, cold.area, 1e-9, "warm area");
        assert_close(warm.volume, cold.volume, 1e-9, "warm volume");
    }
}

/// Review 3: A shallow arc of a circle of radius 1e12 lost its curvature: a
/// rectangle the true arc crosses was certified inside the hole and added
/// back. The arc's evaluation error (about 1e-2 here) is wider than the gap
/// it would have to certify, so the exact path declines.
#[test]
fn review_shallow_arc_of_a_huge_circle_declines() {
    let mut topo = Topology::new();
    let c = Circle3D::new_with_ref(
        Point3::new(-1e12, 0.0, 5.0),
        UP,
        1e12,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let (a, b) = (c.evaluate(-1e-9), c.evaluate(1e-9));
    let corners = [
        a,
        b,
        Point3::new(-10.0, b.y(), 5.0),
        Point3::new(-10.0, a.y(), 5.0),
    ];
    let verts: Vec<_> = corners
        .iter()
        .map(|&q| topo.add_vertex(Vertex::new(q, 1e-10)))
        .collect();
    let mut arc = Edge::new(verts[0], verts[1], EdgeCurve::Circle(c));
    arc.set_trim(Some((-1e-9, 1e-9)));
    let mut edges = vec![OrientedEdge::new(topo.add_edge(arc), true)];
    for i in 1..4 {
        edges.push(OrientedEdge::new(
            topo.add_edge(Edge::new(verts[i], verts[(i + 1) % 4], EdgeCurve::Line)),
            true,
        ));
    }
    let hole = topo.add_wire(Wire::new(edges, true).unwrap());
    let pl = Placement::flat(1.0);
    let island = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(-2e-7, 600.0, 5.0),
            Point3::new(-1e-7, 600.0, 5.0),
            Point3::new(-1e-7, 800.0, 5.0),
            Point3::new(-2e-7, 800.0, 5.0),
        ],
        1e-10,
    )
    .unwrap();
    let outer = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(-30.0, -2000.0, 5.0),
            Point3::new(30.0, -2000.0, 5.0),
            Point3::new(30.0, 2000.0, 5.0),
            Point3::new(-30.0, 2000.0, 5.0),
        ],
        1e-10,
    )
    .unwrap();
    let face = topo.add_face(Face::new(outer, vec![hole, island], pl.surface()));
    assert_eq!(planar_face_exact_area(&topo, face).unwrap(), None);
    // Measured on its mesh instead.
    assert!(measure::face_area(&topo, face, 1e-3).unwrap().is_finite());
}

// ── The mesh fallback ──────────────────────────────────────────────────────

fn mesh_area(topo: &Topology, face: FaceId, deflection: f64) -> f64 {
    let mesh = tessellate::tessellate(topo, face, deflection).unwrap();
    mesh.indices
        .chunks(3)
        .map(|t| {
            let p = |k: usize| mesh.positions[t[k] as usize];
            (p(1) - p(0)).cross(p(2) - p(0)).length() * 0.5
        })
        .sum()
}

/// Where the nesting cannot be certified (here: no work allowed), the face
/// is measured on its mesh, whose parity the tessellator decides: the
/// islands still count as material, within the mesh's chord error of the
/// closed form, and in agreement with the tessellator's own mesh.
#[test]
fn an_uncertified_nesting_is_measured_on_the_mesh() {
    use remus_check::properties::loop_nesting::with_nesting_work_limit;
    for s in SCALES {
        let pl = Placement::flat(s);
        let mut topo = Topology::new();
        let stack = [(10.0, 1), (8.0, 4), (5.0, 3), (2.0, 2)]
            .map(|(r, arcs)| pl.circle(&mut topo, (0.0, 0.0), r, arcs))
            .to_vec();
        let lines = vec![
            pl.square(&mut topo, (0.0, 0.0), 12.0),
            pl.square(&mut topo, (0.0, 0.0), 6.0),
        ];
        let circles = pl.face(&mut topo, 30.0, stack, false);
        let squares = pl.face(&mut topo, 30.0, lines, true);
        let deflection = 1e-4 * s;
        for (label, face, want, arc_length) in [
            (
                "circles",
                circles,
                PI.mul_add(-(100.0 - 64.0 + 25.0 - 4.0), 3600.0),
                TAU * 25.0,
            ),
            ("squares", squares, 3600.0 - 576.0 + 144.0, 0.0),
        ] {
            let want = want * s * s;
            let certified = measure::face_area(&topo, face, deflection).unwrap();
            assert_close(
                certified,
                want,
                1e-12,
                &format!("{label}, certified, scale {s}"),
            );
            let (fallback, exact) = with_nesting_work_limit(0, || {
                (
                    measure::face_area(&topo, face, deflection).unwrap(),
                    planar_face_exact_area(&topo, face).unwrap(),
                )
            });
            assert_eq!(exact, None, "{label}: the exact path declines");
            let chord = deflection * (arc_length * s) + 1e-9 * want;
            assert!(
                (fallback - want).abs() <= chord,
                "{label}, fallback, scale {s}: {fallback} against {want}"
            );
            let mesh = mesh_area(&topo, face, deflection);
            assert!(
                (fallback - mesh).abs() <= chord,
                "{label}, fallback against the tessellator, scale {s}: {fallback} against {mesh}"
            );
        }
    }
}

// ── Seeded randomized differential ─────────────────────────────────────────

struct Rng(u64);

impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        #[allow(clippy::cast_precision_loss)]
        let unit = (self.0 >> 11) as f64 / (1u64 << 53) as f64;
        unit
    }

    fn below(&mut self, n: usize) -> usize {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let k = (self.unit() * n as f64) as usize;
        k.min(n - 1)
    }

    /// Log-uniform in `[lo, hi]`.
    fn log(&mut self, lo: f64, hi: f64) -> f64 {
        (self.unit() * (hi / lo).ln()).exp() * lo
    }
}

/// The loop shapes a stack is made of; every loop of a stack is the same
/// shape scaled about the stack's centre, so the stack nests by construction.
#[derive(Clone, Copy, Debug)]
enum Shape {
    Lines,
    TrimmedNurbs,
    Circle,
    RationalCircle,
    Ellipse,
}

impl Shape {
    const ALL: [Self; 5] = [
        Self::Lines,
        Self::TrimmedNurbs,
        Self::Circle,
        Self::RationalCircle,
        Self::Ellipse,
    ];

    /// The closest relative gap a stack of this shape is built with: below
    /// it, the rational and elliptical pieces need more refinement than a
    /// debug test run should spend (a billionth costs a few million units).
    const fn min_gap(self) -> f64 {
        match self {
            Self::Lines | Self::TrimmedNurbs | Self::Circle => 1e-9,
            Self::Ellipse => 1e-7,
            Self::RationalCircle => 1e-6,
        }
    }

    fn area(self, r: f64) -> f64 {
        match self {
            Self::Lines | Self::TrimmedNurbs => 4.0 * r * r,
            Self::Circle | Self::RationalCircle => PI * r * r,
            Self::Ellipse => PI * r * (0.6 * r),
        }
    }

    fn build(
        self,
        pl: &Placement,
        topo: &mut Topology,
        c: (f64, f64),
        r: f64,
        rng: &mut Rng,
    ) -> WireId {
        let arcs = 1 + rng.below(4);
        match self {
            Self::Lines => pl.square(topo, c, r),
            Self::TrimmedNurbs => {
                #[allow(clippy::cast_possible_truncation)]
                let unused = rng.below(40) as u32;
                pl.nurbs_square(topo, c, r, unused)
            }
            Self::Circle => pl.circle(topo, c, r, arcs),
            Self::RationalCircle => pl.rational_circle(topo, c, r),
            Self::Ellipse => pl.ellipse(topo, c, (r, 0.6 * r), arcs),
        }
    }
}

/// Random valid nested layouts, measured against their areas by
/// construction: up to three stacks of up to four levels (each level a
/// scaled copy of the last, as close as one part in a billion), lines,
/// arcs, full circles, ellipses, trimmed and rational NURBS, translated up
/// to 1e9, tilted and reversed. Every case must read the closed form,
/// whichever path measured it.
#[test]
fn seeded_nested_layouts_measure_their_closed_forms() {
    use remus_check::properties::loop_nesting::{InnerLoopNesting, inner_loop_nesting_report};
    let mut rng = Rng(0x0f1e_2d3c_4b5a_6978);
    let centres = [(-14.0, -10.0), (14.0, -10.0), (0.0, 14.0)];
    let (mut fast, mut slow, mut sampled, mut unresolved, mut fallback) = (0, 0, 0, 0, 0);
    let mut most_work = 0;
    for case in 0..120 {
        let scale = SCALES[rng.below(3)];
        let shift = [0.0, 1e3, 1e6, 1e9][rng.below(4)] * scale;
        let origin = Point3::new(shift, -0.7 * shift, 0.3f64.mul_add(shift, 5.0 * scale));
        let pl = if rng.unit() < 0.5 {
            Placement {
                origin,
                ..Placement::flat(scale)
            }
        } else {
            Placement::tilted(scale, 2.0 * rng.unit(), 6.0 * rng.unit(), origin)
        };
        let mut topo = Topology::new();
        let mut inner = Vec::new();
        let mut want = 3600.0;
        let mut shapes = Vec::new();
        for &c in centres.iter().take(1 + rng.below(3)) {
            let shape = Shape::ALL[rng.below(Shape::ALL.len())];
            shapes.push(shape);
            let levels = 1 + rng.below(4);
            // The input's own rounding at this translation, relative to the
            // stack's size, bounds how close two levels can be built.
            let rounding = 1e4 * f64::EPSILON * (shift / scale + 30.0) / 8.0;
            let mut r = 8.0;
            let mut loops = Vec::new();
            for level in 0..levels {
                loops.push(shape.build(&pl, &mut topo, c, r, &mut rng));
                let sign: f64 = if level % 2 == 0 { -1.0 } else { 1.0 };
                want = sign.mul_add(shape.area(r), want);
                let gap = rng.log(shape.min_gap().max(rounding), 0.3);
                r *= 1.0 - gap;
            }
            // Inner loops in a shuffled order.
            while !loops.is_empty() {
                let k = rng.below(loops.len());
                inner.push(loops.swap_remove(k));
            }
        }
        let reversed = rng.unit() < 0.3;
        let face = pl.face(&mut topo, 30.0, inner, reversed);
        let want = want * scale * scale;
        let label = format!("case {case}: scale {scale}, shift {shift}");
        // The stored coordinates are the intended ones rounded at the
        // translation: a boundary (total length under 1000 at unit scale)
        // moved by `2 ε |coordinate|`.
        let input = 1e3 * scale * 2.0 * f64::EPSILON * (shift + 60.0 * scale);
        let exact_tolerance = 1e-9f64.mul_add(want, input);

        let report = inner_loop_nesting_report(&topo, face);
        most_work = most_work.max(report.work);
        let exact = planar_face_exact_area(&topo, face).unwrap();
        // How `face_area` measures it, and to what accuracy.
        let deflection = 1e-4 * scale;
        let tolerance = match (&report.nesting, exact) {
            (InnerLoopNesting::Flat | InnerLoopNesting::Nested(_), Some(exact)) => {
                if report.work <= 1 << 16 {
                    fast += 1;
                } else {
                    slow += 1;
                }
                assert!(
                    (exact.abs() - want).abs() <= exact_tolerance,
                    "{label}: exact area {exact} against {want}"
                );
                exact_tolerance
            }
            // The exact path declines some boundaries for reasons of its own
            // (a rational NURBS its adaptive rule cannot certify there): the
            // 256-sample polygon then measures the face, with the certified
            // nesting and its chord error.
            (InnerLoopNesting::Flat | InnerLoopNesting::Nested(_), None) => {
                sampled += 1;
                1e-3f64.mul_add(want, input)
            }
            (InnerLoopNesting::Undecided(_), exact) => {
                fallback += 1;
                assert_eq!(exact, None, "{label}: undecided yet exact");
                deflection.mul_add(400.0 * scale, input)
            }
            (InnerLoopNesting::Invalid, _) => panic!("{label}: a valid layout read invalid"),
        };
        // Whichever path measured it, `face_area` reads the closed form. Its
        // straight-edged and sampled paths take a Newell sum of world
        // coordinates (as on `main`), whose rounding grows as `ε · offset²`
        // and, far enough from the origin, swamps a small face; there only
        // the exact area above is compared.
        let world = 2560.0 * f64::EPSILON * (shift + 60.0 * scale).powi(2);
        if world > 1e-4 * want {
            unresolved += 1;
            continue;
        }
        let area = measure::face_area(&topo, face, deflection).unwrap();
        assert!(
            (area - want).abs() <= tolerance + world,
            "{label}: face_area {area} against {want} ({shapes:?}, {:?})",
            report.nesting
        );
    }
    // The run's paths, for the record.
    eprintln!(
        "seeded nested layouts: {fast} certified within 2^16 units, {slow} certified past it \
         (exact area); {sampled} certified with a boundary the exact path declines (sampled \
         area); {fallback} measured on the mesh; {unresolved} too far from the origin for \
         face_area's world-coordinate sums to be compared; most work {most_work}"
    );
    assert_eq!(fast + slow + sampled + fallback, 120);
}

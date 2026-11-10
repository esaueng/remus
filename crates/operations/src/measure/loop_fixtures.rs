//! Planar faces with several inner loops for the measurement paths'
//! nesting oracles (tests): loops that do not nest (lines, circles, split
//! arcs, ellipses, trimmed NURBS, a shallow arc of a far circle, some in
//! another's box), and a nested stack of nearly touching circles; flat,
//! tilted and translated far from the origin.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, dead_code)]

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

pub const SCALES: [f64; 3] = [1e-3, 1.0, 1e6];

/// A plane at `scale`: `at(x, y)` is `origin + scale·(x·u + y·v)`.
pub struct Placement {
    origin: Point3,
    u: Vec3,
    v: Vec3,
    pub n: Vec3,
    pub scale: f64,
}

impl Placement {
    pub fn flat(scale: f64) -> Self {
        Self {
            origin: Point3::new(0.0, 0.0, 5.0 * scale),
            u: Vec3::new(1.0, 0.0, 0.0),
            v: Vec3::new(0.0, 1.0, 0.0),
            n: Vec3::new(0.0, 0.0, 1.0),
            scale,
        }
    }

    /// [`Self::flat`] moved by `shift` along a skew direction: the face is
    /// unchanged, its coordinates are not.
    pub fn translated(scale: f64, shift: f64) -> Self {
        Self {
            origin: Point3::new(shift, -0.5 * shift, 0.25f64.mul_add(shift, 5.0 * scale)),
            ..Self::flat(scale)
        }
    }

    /// Every placement the oracles run on: flat, tilted, and translated by a
    /// million and a billion times the scale.
    pub fn all(scale: f64) -> [Self; 4] {
        [
            Self::flat(scale),
            Self::tilted(scale),
            Self::translated(scale, 1e6 * scale),
            Self::translated(scale, 1e9 * scale),
        ]
    }

    /// Whether the placement lies a billion times its scale from the origin.
    pub fn far(&self) -> bool {
        let o = self.origin;
        o.x().abs().max(o.y().abs()).max(o.z().abs()) >= 1e8 * self.scale
    }

    /// Rotated about x by 0.4, then about z by 0.7, and shifted.
    pub fn tilted(scale: f64) -> Self {
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

    pub fn at(&self, x: f64, y: f64) -> Point3 {
        self.origin + self.u * (x * self.scale) + self.v * (y * self.scale)
    }

    fn surface(&self) -> FaceSurface {
        let o = self.origin;
        FaceSurface::Plane {
            normal: self.n,
            d: self.n.dot(Vec3::new(o.x(), o.y(), o.z())),
        }
    }

    pub fn polygon(&self, topo: &mut Topology, points: &[(f64, f64)]) -> WireId {
        let points: Vec<_> = points.iter().map(|&(x, y)| self.at(x, y)).collect();
        make_polygon_wire(topo, &points, 1e-9 * self.scale).unwrap()
    }

    pub fn rect(&self, topo: &mut Topology, [x0, x1, y0, y1]: [f64; 4]) -> WireId {
        self.polygon(topo, &[(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
    }

    /// A circle of `arcs` edges (one closed edge for `arcs == 1`).
    pub fn circle(&self, topo: &mut Topology, c: (f64, f64), r: f64, arcs: usize) -> WireId {
        let curve =
            Circle3D::new_with_ref(self.at(c.0, c.1), self.n, r * self.scale, self.u).unwrap();
        self.arcs(
            topo,
            arcs,
            &|t| curve.evaluate(t),
            &EdgeCurve::Circle(curve.clone()),
        )
    }

    pub fn ellipse(
        &self,
        topo: &mut Topology,
        c: (f64, f64),
        ab: (f64, f64),
        arcs: usize,
    ) -> WireId {
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
    /// (2,2)`; its unused spans run out around `[-4, -2] × [-3, -1]`.
    pub fn trimmed_nurbs_triangle(&self, topo: &mut Topology) -> WireId {
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

    /// A face on `[-30, 30]²` with these inner loops.
    pub fn face(&self, topo: &mut Topology, inner: Vec<WireId>) -> FaceId {
        let outer = self.rect(topo, [-30.0, 30.0, -30.0, 30.0]);
        topo.add_face(Face::new(outer, inner, self.surface()))
    }
}

/// Faces whose inner loops do not nest, each with at least two of them.
pub fn flat_faces(topo: &mut Topology, pl: &Placement) -> Vec<(&'static str, FaceId)> {
    let mut faces = Vec::new();
    // Lines: an L-shaped slot whose box holds a square it does not surround.
    let slot = pl.polygon(
        topo,
        &[
            (0.0, 0.0),
            (20.0, 0.0),
            (20.0, 2.0),
            (2.0, 2.0),
            (2.0, 20.0),
            (0.0, 20.0),
        ],
    );
    let square = pl.rect(topo, [10.0, 12.0, 10.0, 12.0]);
    let far = pl.rect(topo, [-20.0, -10.0, -20.0, -15.0]);
    faces.push(("lines", pl.face(topo, vec![slot, square, far])));
    // Circles and split arcs side by side, one in another's box corner.
    let one = pl.circle(topo, (-10.0, -10.0), 5.0, 1);
    let four = pl.circle(topo, (10.0, 10.0), 3.0, 4);
    let three = pl.circle(topo, (10.0, -10.0), 4.0, 3);
    let corner = pl.circle(topo, (-6.0, -6.0), 0.5, 2);
    faces.push(("circles", pl.face(topo, vec![one, four, three, corner])));
    // Ellipses: a thin one with a circle in its box, outside it.
    let thin = pl.ellipse(topo, (0.0, 0.0), (20.0, 1.0), 1);
    let beside = pl.circle(topo, (18.0, 0.7), 0.1, 1);
    let split = pl.ellipse(topo, (-10.0, 15.0), (6.0, 3.0), 2);
    faces.push(("ellipses", pl.face(topo, vec![thin, beside, split])));
    // A trimmed NURBS whose unused spans surround the rectangle beside it.
    let triangle = pl.trimmed_nurbs_triangle(topo);
    let rectangle = pl.rect(topo, [-4.0, -2.0, -3.0, -1.0]);
    faces.push(("trimmed NURBS", pl.face(topo, vec![triangle, rectangle])));
    // A far carrier: a shallow arc of a circle of radius 1e4 centred far
    // outside the face, closing a hole to its right, with a square in the
    // hole's box but left of the arc.
    let big = 1e4;
    let arc = Circle3D::new_with_ref(pl.at(5.0 - big, 0.0), pl.n, big * pl.scale, pl.u).unwrap();
    let h = (3.0 / big).asin();
    let ends = [
        arc.evaluate(-h),
        arc.evaluate(h),
        pl.at(10.0, 3.0),
        pl.at(10.0, -3.0),
    ];
    let vertices: Vec<_> = ends
        .iter()
        .map(|&q| topo.add_vertex(Vertex::new(q, 1e-9 * pl.scale)))
        .collect();
    let mut rim = Edge::new(vertices[0], vertices[1], EdgeCurve::Circle(arc));
    rim.set_trim(Some((-h, h)));
    let mut edges = vec![OrientedEdge::new(topo.add_edge(rim), true)];
    for i in 1..4 {
        let line = Edge::new(vertices[i], vertices[(i + 1) % 4], EdgeCurve::Line);
        edges.push(OrientedEdge::new(topo.add_edge(line), true));
    }
    let shallow = topo.add_wire(Wire::new(edges, true).unwrap());
    let left_of_arc = pl.rect(topo, [4.9996, 4.9997, -0.5, 0.5]);
    faces.push((
        "far circle carrier",
        pl.face(topo, vec![shallow, left_of_arc]),
    ));
    faces
}

/// Hole r = 10, island 9.99, hole 9.98, island 9.97, nearly touching:
/// `1600 − π(10² − 9.99² + 9.98² − 9.97²)` at unit scale.
pub fn nested_face(topo: &mut Topology, pl: &Placement) -> (FaceId, f64) {
    let inner = [(9.98, 1), (10.0, 1), (9.97, 3), (9.99, 4)]
        .map(|(r, arcs)| pl.circle(topo, (1.0, -2.0), r, arcs))
        .to_vec();
    let area = 3600.0
        - std::f64::consts::PI * (100.0 - 9.99_f64.powi(2) + 9.98_f64.powi(2) - 9.97_f64.powi(2));
    (pl.face(topo, inner), area * pl.scale * pl.scale)
}

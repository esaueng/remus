//! Differential oracle: on a planar face whose inner loops do not nest, both
//! planar paths (the exact Green's-theorem rule and the chord-polygon fan)
//! give every field bit for bit what they gave before nesting was read. The
//! pre-nesting functions are kept below verbatim (only renamed), and the
//! faces carry at least two inner loops each, so the nesting reader runs:
//! line, circle, split-arc, ellipse and trimmed NURBS holes, including holes
//! whose boxes overlap (an L-shaped slot around a square, a thin ellipse
//! with a circle in its box corner) and a trimmed NURBS whose unused
//! carrier spans surround a neighbouring hole, and a shallow arc of a circle
//! whose centre lies far outside the face. Three scales, flat, tilted, and
//! translated by a million and a billion times the face's scale.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use std::f64::consts::TAU;

use super::*;
use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::Edge;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

/// The fan fallback as it was before inner-wire nesting, verbatim.
fn main_planar_face_fan(
    topo: &Topology,
    face_id: FaceId,
    normal: Vec3,
    reference: Point3,
) -> Result<FaceContribution, CheckError> {
    let polygon = crate::util::face_polygon(topo, face_id)?;
    let mut contrib = integrate_planar_polygon(&polygon, normal, reference);

    let face = topo.face(face_id)?;
    let inner: Vec<_> = face.inner_wires().to_vec();
    for wid in inner {
        let hole = crate::util::wire_polygon(topo, wid)?;
        let h = integrate_planar_polygon(&hole, normal, reference);
        contrib.area -= h.area;
        contrib.volume -= h.volume;
        contrib.volume_moment_x -= h.volume_moment_x;
        contrib.volume_moment_y -= h.volume_moment_y;
        contrib.volume_moment_z -= h.volume_moment_z;
        contrib.volume_second_x -= h.volume_second_x;
        contrib.volume_second_y -= h.volume_second_y;
        contrib.volume_second_z -= h.volume_second_z;
        contrib.volume_product_xy -= h.volume_product_xy;
        contrib.volume_product_xz -= h.volume_product_xz;
        contrib.volume_product_yz -= h.volume_product_yz;
        contrib.centroid_x -= h.centroid_x;
        contrib.centroid_y -= h.centroid_y;
        contrib.centroid_z -= h.centroid_z;
    }

    Ok(contrib)
}

/// `integrate_planar_face_exact` as it was before inner-wire nesting,
/// verbatim.
fn main_integrate_planar_face_exact(
    topo: &Topology,
    face_id: FaceId,
    normal: Vec3,
    reference: Point3,
) -> Result<Option<FaceContribution>, CheckError> {
    let face = topo.face(face_id)?;
    let outer_wire = face.outer_wire();
    let inner: Vec<_> = face.inner_wires().to_vec();

    // In-plane frame anchored at the first boundary vertex. The frame's
    // normal is derived from the boundary geometry itself (Newell), NOT the
    // face's stored plane normal: a malformed face whose stored normal is
    // inconsistent with its boundary would otherwise project to a collapsed
    // (zero-area) region, where the chord-polygon fan path still measures
    // the true geometric area. The flux terms below keep using the passed
    // `normal`, exactly like the fan path.
    let wire = topo.wire(outer_wire)?;
    let Some(first_edge) = wire.edges().first() else {
        return Ok(None);
    };
    let origin = topo.vertex(topo.edge(first_edge.edge())?.start())?.point();
    let Some(boundary_normal) = wire_newell_normal(topo, outer_wire)? else {
        return Ok(None);
    };
    let Ok(frame) = remus_math::frame::Frame3::from_normal(origin, boundary_normal) else {
        return Ok(None);
    };
    let (e1, e2) = (frame.x, frame.y);

    let Some(mut moments) = planar_wire_monomial_moments(topo, outer_wire, origin, e1, e2)? else {
        return Ok(None);
    };
    for wid in inner {
        let Some(hole) = planar_wire_monomial_moments(topo, wid, origin, e1, e2)? else {
            return Ok(None);
        };
        for k in 0..10 {
            moments[k] -= hole[k];
        }
    }

    // Linear forms of the coordinates about `reference` in the in-plane
    // basis: x − reference.x = (origin − reference).x + e1.x·s + e2.x·t, etc.
    // The monomial moments above are already local to the face.
    let anchor = origin - reference;
    let lx = [anchor.x(), e1.x(), e2.x()];
    let ly = [anchor.y(), e1.y(), e2.y()];
    let lz = [anchor.z(), e1.z(), e2.z()];
    let lin = |l: [f64; 3]| -> Poly2 {
        let mut p = [0.0; 10];
        p[0] = l[0];
        p[1] = l[1];
        p[2] = l[2];
        p
    };
    let (px, py, pz) = (lin(lx), lin(ly), lin(lz));
    let x2 = poly2_mul_linear(&px, lx);
    let y2 = poly2_mul_linear(&py, ly);
    let z2 = poly2_mul_linear(&pz, lz);
    let ig = |p: &Poly2| poly2_integrate(p, &moments);

    let area = moments[0];
    let (ix, iy, iz) = (ig(&px), ig(&py), ig(&pz));
    Ok(Some(FaceContribution {
        area,
        volume: (normal.x() * ix + normal.y() * iy + normal.z() * iz) / 3.0,
        volume_moment_x: 0.5 * normal.x() * ig(&x2),
        volume_moment_y: 0.5 * normal.y() * ig(&y2),
        volume_moment_z: 0.5 * normal.z() * ig(&z2),
        volume_second_x: normal.x() * ig(&poly2_mul_linear(&x2, lx)) / 3.0,
        volume_second_y: normal.y() * ig(&poly2_mul_linear(&y2, ly)) / 3.0,
        volume_second_z: normal.z() * ig(&poly2_mul_linear(&z2, lz)) / 3.0,
        volume_product_xy: 0.5 * normal.x() * ig(&poly2_mul_linear(&x2, ly)),
        volume_product_xz: 0.5 * normal.x() * ig(&poly2_mul_linear(&x2, lz)),
        volume_product_yz: 0.5 * normal.y() * ig(&poly2_mul_linear(&y2, lz)),
        centroid_x: ix,
        centroid_y: iy,
        centroid_z: iz,
    }))
}

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

    /// [`Self::flat`] moved by `shift` along a skew direction: the face is
    /// unchanged, its coordinates are not.
    fn translated(scale: f64, shift: f64) -> Self {
        Self {
            origin: Point3::new(shift, -0.5 * shift, 0.25f64.mul_add(shift, 5.0 * scale)),
            ..Self::flat(scale)
        }
    }

    /// Every placement the oracles run on.
    fn all(scale: f64) -> [Self; 4] {
        [
            Self::flat(scale),
            Self::tilted(scale),
            Self::translated(scale, 1e6 * scale),
            Self::translated(scale, 1e9 * scale),
        ]
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

    fn polygon(&self, topo: &mut Topology, points: &[(f64, f64)]) -> WireId {
        let points: Vec<_> = points.iter().map(|&(x, y)| self.at(x, y)).collect();
        make_polygon_wire(topo, &points, 1e-9 * self.scale).unwrap()
    }

    fn rect(&self, topo: &mut Topology, [x0, x1, y0, y1]: [f64; 4]) -> WireId {
        self.polygon(topo, &[(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
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

    /// One closed NURBS edge over `trim`.
    fn nurbs(&self, topo: &mut Topology, nc: NurbsCurve, trim: Option<(f64, f64)>) -> WireId {
        let start = trim.map_or_else(|| nc.domain().0, |t| t.0);
        let v = topo.add_vertex(Vertex::new(nc.evaluate(start), 1e-9 * self.scale));
        let mut edge = Edge::new(v, v, EdgeCurve::NurbsCurve(nc));
        edge.set_trim(trim);
        let edge = topo.add_edge(edge);
        topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
    }

    fn face(&self, topo: &mut Topology, inner: Vec<WireId>) -> FaceId {
        let outer = self.rect(topo, [-30.0, 30.0, -30.0, 30.0]);
        topo.add_face(Face::new(outer, inner, self.surface()))
    }
}

/// Faces whose inner loops do not nest, each with at least two of them.
fn flat_faces(topo: &mut Topology, pl: &Placement) -> Vec<(&'static str, FaceId)> {
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

    // Circles and arcs side by side, one in another's box corner.
    let one = pl.circle(topo, (-10.0, -10.0), 5.0, 1);
    let four = pl.circle(topo, (10.0, 10.0), 3.0, 4);
    let three = pl.circle(topo, (10.0, -10.0), 4.0, 3);
    let corner = pl.circle(topo, (-6.0, -6.0), 0.5, 2);
    faces.push((
        "circles and arcs",
        pl.face(topo, vec![one, four, three, corner]),
    ));

    // Ellipses: a thin one with a circle in its box, outside it.
    let thin = pl.ellipse(topo, (0.0, 0.0), (20.0, 1.0), 1);
    let beside = pl.circle(topo, (18.0, 0.7), 0.1, 1);
    let split = pl.ellipse(topo, (-10.0, 15.0), (6.0, 3.0), 2);
    faces.push(("ellipses", pl.face(topo, vec![thin, beside, split])));

    // Trimmed NURBS: the trim keeps a triangle; the unused spans run around
    // the rectangle beside it.
    let control = [
        (-10.0, 0.0),
        (0.0, 0.0),
        (2.0, 0.0),
        (2.0, 2.0),
        (0.0, 0.0),
        (0.0, -20.0),
        (-10.0, 0.0),
    ]
    .map(|(x, y)| pl.at(x, y))
    .to_vec();
    let knots = vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0];
    let nc = NurbsCurve::new(1, knots, control, vec![1.0; 7]).unwrap();
    let triangle = pl.nurbs(topo, nc, Some((1.0, 4.0)));
    let rectangle = pl.rect(topo, [-4.0, -2.0, -3.0, -1.0]);
    faces.push((
        "trimmed NURBS beside a rectangle",
        pl.face(topo, vec![triangle, rectangle]),
    ));

    // A rational NURBS circle with a square in its box corner.
    let round = Circle3D::new_with_ref(pl.at(0.0, 0.0), pl.n, 5.0 * pl.scale, pl.u).unwrap();
    let nc = remus_geometry::convert::circle_to_nurbs(&round, 0.0, TAU).unwrap();
    let rational = pl.nurbs(topo, nc, None);
    let in_corner = pl.rect(topo, [4.0, 4.5, 4.0, 4.5]);
    faces.push((
        "rational NURBS circle",
        pl.face(topo, vec![rational, in_corner]),
    ));

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

fn bits(c: &FaceContribution) -> [u64; 14] {
    [
        c.area,
        c.volume,
        c.volume_moment_x,
        c.volume_moment_y,
        c.volume_moment_z,
        c.volume_second_x,
        c.volume_second_y,
        c.volume_second_z,
        c.volume_product_xy,
        c.volume_product_xz,
        c.volume_product_yz,
        c.centroid_x,
        c.centroid_y,
        c.centroid_z,
    ]
    .map(f64::to_bits)
}

#[test]
fn unnested_inner_loops_integrate_bit_for_bit_as_before() {
    for scale in [1e-3, 1.0, 1e6] {
        for pl in Placement::all(scale) {
            let mut topo = Topology::new();
            let reference = pl.at(0.5, -1.0) + pl.n * (2.0 * scale);
            for (label, face) in flat_faces(&mut topo, &pl) {
                let normal = pl.n.normalize().unwrap();
                let main_exact =
                    main_integrate_planar_face_exact(&topo, face, normal, reference).unwrap();
                let exact = integrate_planar_face_exact(&topo, face, normal, reference).unwrap();
                assert_eq!(
                    main_exact.as_ref().map(bits),
                    exact.as_ref().map(bits),
                    "exact path, {label}, scale {scale}"
                );
                let main_fan = main_planar_face_fan(&topo, face, normal, reference).unwrap();
                let fan = integrate_planar_face_fan(&topo, face, normal, reference).unwrap();
                assert_eq!(
                    bits(&main_fan),
                    bits(&fan),
                    "fan path, {label}, scale {scale}"
                );
            }
        }
    }
}

#[test]
fn the_oracle_faces_are_certified_flat() {
    use crate::properties::loop_nesting::{InnerLoopNesting, inner_loop_nesting};
    for scale in [1e-3, 1.0, 1e6] {
        for pl in Placement::all(scale) {
            let mut topo = Topology::new();
            for (label, face) in flat_faces(&mut topo, &pl) {
                assert_eq!(
                    inner_loop_nesting(&topo, face),
                    InnerLoopNesting::Flat,
                    "{label}, scale {scale}"
                );
            }
        }
    }
}

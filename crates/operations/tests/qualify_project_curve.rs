//! P-Class 7.4 acceptance oracles for directional curve projection.
//!
//! Contract: `docs/design/p74-curve-projection.md`. Cell names (P1, C3, K4,
//! ...) refer to that note. Every expected value is the note's closed form;
//! the implementation (M5) may not change them.
//!
//! The `oracle_self_check_*` tests build each cell's
//! closed-form answer as edges by hand and pass it through the same checker
//! the API tests use, so the closed forms, the placements and the independent
//! ray-cast oracle are independently consistent. All 48 API acceptance
//! tests run alongside these two self-checks.

#![allow(
    clippy::cast_precision_loss,
    clippy::expect_used,
    clippy::panic,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::unwrap_used
)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::curves2d::Curve2D;
use remus_math::frame::Frame3;
use remus_math::mat::Mat4;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::project_curve::{
    ProjectCurveError, ProjectCurveOptions, ProjectedCurves, ProjectionQuality,
    project_curve_onto_face, project_curves_onto_plane, project_curves_onto_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::make_nurbs_edge;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::solid::SolidId;
use remus_topology::vertex::{Vertex, VertexId};

/// Angle between `q − p` and the direction, for every sampled image point.
const DIRECTION_RESIDUAL: f64 = 1e-9;
/// On-face residual of exact images, relative to the cell extent.
const ON_FACE_REL: f64 = 1e-7;
/// Carrier parameters and endpoints, relative to the cell extent.
const MATCH_REL: f64 = 1e-9;
/// Source-parameter agreement (segment parameters in [0, 1], arc angles).
const PARAM_ABS: f64 = 1e-9;
const VERTEX_TOL: f64 = 1e-7;
const SAMPLES: usize = 32;

// ---------------------------------------------------------------------------
// Placement: three scales plus one rigid motion per cell.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
struct Placement {
    scale: f64,
    rigid: Option<Mat4>,
}

impl Placement {
    fn p(&self, x: f64, y: f64, z: f64) -> Point3 {
        let local = Point3::new(x * self.scale, y * self.scale, z * self.scale);
        self.rigid.map_or(local, |m| m.mul_point(local))
    }

    fn v(&self, x: f64, y: f64, z: f64) -> Vec3 {
        let v = Vec3::new(x, y, z);
        self.rigid.map_or(v, |m| {
            m.mul_point(Point3::new(x, y, z)) - m.mul_point(Point3::new(0.0, 0.0, 0.0))
        })
    }

    fn unit(&self, x: f64, y: f64, z: f64) -> Vec3 {
        self.v(x, y, z).normalize().expect("unit direction")
    }

    fn len(&self, l: f64) -> f64 {
        l * self.scale
    }

    fn place(&self, topo: &mut Topology, solid: SolidId) {
        if let Some(m) = self.rigid {
            transform_solid(topo, solid, &m).expect("rigid placement");
        }
    }
}

fn rigid_motion() -> Mat4 {
    Mat4::translation(12.5, -7.25, 3.125)
        * Mat4::rotation_z(0.7)
        * Mat4::rotation_x(-0.4)
        * Mat4::rotation_y(1.1)
}

fn placements() -> Vec<Placement> {
    vec![
        Placement {
            scale: 1e-3,
            rigid: None,
        },
        Placement {
            scale: 1.0,
            rigid: None,
        },
        Placement {
            scale: 1e3,
            rigid: None,
        },
        Placement {
            scale: 1.0,
            rigid: Some(rigid_motion()),
        },
    ]
}

const UNIT: Placement = Placement {
    scale: 1.0,
    rigid: None,
};

// ---------------------------------------------------------------------------
// Sources.
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
enum Source {
    Segment {
        a: Point3,
        b: Point3,
    },
    Arc {
        circle: Circle3D,
        range: (f64, f64),
    },
    Ellipse {
        ellipse: Ellipse3D,
        range: (f64, f64),
    },
}

impl Source {
    fn point(&self, s: f64) -> Point3 {
        match self {
            Self::Segment { a, b } => *a + (*b - *a) * s,
            Self::Arc { circle, .. } => circle.evaluate(s),
            Self::Ellipse { ellipse, .. } => ellipse.evaluate(s),
        }
    }

    fn domain(&self) -> (f64, f64) {
        match self {
            Self::Segment { .. } => (0.0, 1.0),
            Self::Arc { range, .. } | Self::Ellipse { range, .. } => *range,
        }
    }
}

fn arc(center: Point3, normal: Vec3, radius: f64, reference: Vec3, range: (f64, f64)) -> Source {
    Source::Arc {
        circle: Circle3D::new_with_ref(center, normal, radius, reference).expect("source circle"),
        range,
    }
}

fn add_edge_between(
    topo: &mut Topology,
    curve: EdgeCurve,
    (start, end): (Point3, Point3),
    trim: Option<(f64, f64)>,
    closed: bool,
    shared_start: Option<VertexId>,
) -> EdgeId {
    let v0 = shared_start.unwrap_or_else(|| topo.add_vertex(Vertex::new(start, VERTEX_TOL)));
    let v1 = if closed {
        v0
    } else {
        topo.add_vertex(Vertex::new(end, VERTEX_TOL))
    };
    let mut edge = Edge::new(v0, v1, curve);
    edge.set_trim(trim);
    topo.add_edge(edge)
}

fn add_periodic_edge(
    topo: &mut Topology,
    curve: EdgeCurve,
    start: Point3,
    end: Point3,
    range: (f64, f64),
    closed: bool,
) -> EdgeId {
    add_edge_between(topo, curve, (start, end), Some(range), closed, None)
}

fn is_full_turn(range: (f64, f64)) -> bool {
    (range.1 - range.0 - TAU).abs() < 1e-12
}

fn add_source(topo: &mut Topology, source: &Source) -> EdgeId {
    match source {
        Source::Segment { a, b } => {
            let v0 = topo.add_vertex(Vertex::new(*a, VERTEX_TOL));
            let v1 = topo.add_vertex(Vertex::new(*b, VERTEX_TOL));
            topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line))
        }
        Source::Arc { circle, range } => add_periodic_edge(
            topo,
            EdgeCurve::Circle(circle.clone()),
            circle.evaluate(range.0),
            circle.evaluate(range.1),
            *range,
            is_full_turn(*range),
        ),
        Source::Ellipse { ellipse, range } => add_periodic_edge(
            topo,
            EdgeCurve::Ellipse(ellipse.clone()),
            ellipse.evaluate(range.0),
            ellipse.evaluate(range.1),
            *range,
            is_full_turn(*range),
        ),
    }
}

/// Recovers the source parameter of an image point by pushing it back along
/// the direction: `(s, λ, angle)` with `q ≈ source(s) + λ·d̂` and `angle` the
/// angle between `q − source(s)` and `d̂`.
fn preimage(source: &Source, q: Point3, d: Vec3) -> (f64, f64, f64) {
    match source {
        Source::Segment { a, b } => {
            let u = *b - *a;
            let v = -d;
            let w = *a - q;
            let (aa, bb, cc) = (u.dot(u), u.dot(v), v.dot(v));
            let (dd, ee) = (u.dot(w), v.dot(w));
            let den = aa.mul_add(cc, -(bb * bb));
            let s = bb.mul_add(ee, -(cc * dd)) / den;
            let lambda = aa.mul_add(ee, -(bb * dd)) / den;
            let p = *a + u * s;
            let gap = (p - (q - d * lambda)).length();
            let reach = (q - p).length();
            let angle = if reach > 0.0 { gap / reach } else { 0.0 };
            (s, lambda, angle)
        }
        Source::Arc { circle, .. } => {
            let n = circle.normal();
            let lambda = (q - circle.center()).dot(n) / d.dot(n);
            let q0 = q - d * lambda;
            let residual = ((q0 - circle.center()).length() - circle.radius()).abs();
            let angle = if lambda.abs() > 0.0 {
                residual / lambda.abs()
            } else {
                0.0
            };
            (circle.project(q0), lambda, angle)
        }
        Source::Ellipse { ellipse, .. } => {
            let n = ellipse.normal();
            let lambda = (q - ellipse.center()).dot(n) / d.dot(n);
            let q0 = q - d * lambda;
            let s = ellipse.project(q0);
            let residual = (q0 - ellipse.evaluate(s)).length();
            let angle = if lambda.abs() > 0.0 {
                residual / lambda.abs()
            } else {
                0.0
            };
            (s, lambda, angle)
        }
    }
}

/// Lifts a periodic source parameter into `[start, start + 2π)`, folding a
/// value within `PARAM_ABS` of the period back onto `start`.
fn unwrap_from(s: f64, start: f64) -> f64 {
    let lifted = start + (s - start).rem_euclid(TAU);
    if lifted - start > TAU - PARAM_ABS {
        lifted - TAU
    } else {
        lifted
    }
}

// ---------------------------------------------------------------------------
// Independent oracle: ray casting against each face's implicit surface.
// ---------------------------------------------------------------------------

fn quadratic_roots(a: f64, b: f64, c: f64) -> Vec<f64> {
    let scale = a.abs().max(b.abs()).max(c.abs());
    if a.abs() <= 1e-14 * scale {
        return if b.abs() > 0.0 { vec![-c / b] } else { vec![] };
    }
    let disc = b.mul_add(b, -4.0 * a * c);
    if disc < -1e-12 * b * b {
        return vec![];
    }
    let root = disc.max(0.0).sqrt();
    let q = -0.5 * (b + b.signum() * root);
    let mut roots = if q == 0.0 {
        vec![0.0]
    } else {
        vec![q / a, c / q]
    };
    roots.sort_by(f64::total_cmp);
    roots
}

fn ray_roots(surface: &FaceSurface, p: Point3, d: Vec3) -> Vec<f64> {
    match surface {
        FaceSurface::Plane { normal, d: offset } => {
            let n = normal.normalize().expect("plane normal");
            let scale = normal.length();
            let denom = n.dot(d);
            if denom.abs() < 1e-15 {
                return vec![];
            }
            let along = n.dot(p - Point3::new(0.0, 0.0, 0.0));
            vec![(offset / scale - along) / denom]
        }
        FaceSurface::Cylinder(cyl) => {
            let a = cyl.axis();
            let w = p - cyl.origin();
            let wp = w - a * w.dot(a);
            let dp = d - a * d.dot(a);
            quadratic_roots(
                dp.dot(dp),
                2.0 * wp.dot(dp),
                wp.dot(wp) - cyl.radius() * cyl.radius(),
            )
        }
        FaceSurface::Sphere(sph) => {
            let w = p - sph.center();
            quadratic_roots(
                d.dot(d),
                2.0 * w.dot(d),
                w.dot(w) - sph.radius() * sph.radius(),
            )
        }
        FaceSurface::Cone(cone) => {
            let a = cone.axis();
            let k = cone.half_angle().sin().powi(2);
            let w = p - cone.apex();
            let (wa, da) = (w.dot(a), d.dot(a));
            quadratic_roots(
                da.mul_add(da, -k * d.dot(d)),
                2.0 * wa.mul_add(da, -k * w.dot(d)),
                wa.mul_add(wa, -k * w.dot(w)),
            )
            .into_iter()
            .filter(|&t| (w + d * t).dot(a) >= -1e-12 * w.length().max(1.0))
            .collect()
        }
        FaceSurface::Torus(_) | FaceSurface::Nurbs(_) => {
            panic!("oracle covers plane, cylinder, sphere and cone targets only")
        }
    }
}

/// Trimmed region of a fixture face, in fixture-local (unplaced, unscaled)
/// coordinates.
///
/// The oracle does not use the kernel's trim tests, so it stays independent
/// of the code under test; every fixture face's region is simple enough to
/// state in closed form.
#[derive(Clone, Copy, Debug)]
enum Region {
    /// A face of the box `[0, dx] × [0, dy] × [0, dz]`.
    BoxFace { dx: f64, dy: f64, dz: f64 },
    /// Top face of the 10 × 8 × 4 box bored by a radius-1 hole at (5, 4).
    BoredTop,
    /// Lateral face of a primitive with its base at z = 0 and top at z = h.
    Lateral { h: f64 },
    /// The z ≥ 0 hemisphere.
    North,
}

/// Region-aware on-face oracle for one fixture.
#[derive(Clone, Debug)]
struct Oracle {
    inverse: Option<Mat4>,
    scale: f64,
    regions: Vec<(FaceId, Region)>,
}

const REGION_TOL: f64 = 1e-9;

impl Oracle {
    fn new(pl: &Placement) -> Self {
        Self {
            inverse: pl.rigid.map(|m| m.inverse().expect("rigid inverse")),
            scale: pl.scale,
            regions: Vec::new(),
        }
    }

    fn with(mut self, face: FaceId, region: Region) -> Self {
        self.regions.push((face, region));
        self
    }

    fn local(&self, q: Point3) -> Point3 {
        let p = self.inverse.map_or(q, |m| m.mul_point(q));
        Point3::new(p.x() / self.scale, p.y() / self.scale, p.z() / self.scale)
    }

    fn contains(&self, face: FaceId, q: Point3) -> bool {
        let region = self
            .regions
            .iter()
            .find(|(f, _)| *f == face)
            .map(|(_, r)| *r)
            .expect("face has a fixture region");
        let p = self.local(q);
        let within = |v: f64, hi: f64| (-REGION_TOL..=hi + REGION_TOL).contains(&v);
        match region {
            Region::BoxFace { dx, dy, dz } => {
                within(p.x(), dx) && within(p.y(), dy) && within(p.z(), dz)
            }
            Region::BoredTop => {
                within(p.x(), 10.0)
                    && within(p.y(), 8.0)
                    && (p.x() - 5.0).hypot(p.y() - 4.0) >= 1.0 - REGION_TOL
            }
            Region::Lateral { h } => within(p.z(), h),
            Region::North => p.z() >= -REGION_TOL,
        }
    }

    /// Distance from `q` to the trimmed face: the closed-form distance to the
    /// support surface when its foot lies in the region, otherwise infinite.
    fn on_face_distance(&self, topo: &Topology, q: Point3, face: FaceId) -> f64 {
        let foot = support_projection(topo, face, q);
        if self.contains(face, foot.point) {
            foot.distance
        } else {
            f64::INFINITY
        }
    }
}

fn support_projection(
    topo: &Topology,
    face: FaceId,
    q: Point3,
) -> remus_geometry::extrema::SurfaceProjection {
    use remus_geometry::extrema::{
        point_to_cone, point_to_cylinder, point_to_plane, point_to_sphere,
    };
    match topo.face(face).expect("face").surface() {
        FaceSurface::Plane { normal, d } => {
            let n = normal.normalize().expect("plane normal");
            let origin = Point3::new(0.0, 0.0, 0.0) + n * (d / normal.length());
            point_to_plane(q, origin, n)
        }
        FaceSurface::Cylinder(c) => point_to_cylinder(q, c),
        FaceSurface::Cone(c) => point_to_cone(q, c),
        FaceSurface::Sphere(s) => point_to_sphere(q, s),
        FaceSurface::Torus(_) | FaceSurface::Nurbs(_) => {
            panic!("oracle covers plane, cylinder, sphere and cone targets only")
        }
    }
}

/// First point along `p + λ·d̂` (λ ≥ 0) that lies on any of `faces`.
fn first_hit(
    oracle: &Oracle,
    topo: &Topology,
    faces: &[FaceId],
    p: Point3,
    d: Vec3,
    extent: f64,
) -> Option<(Point3, FaceId)> {
    let mut best: Option<(f64, Point3, FaceId)> = None;
    for &face in faces {
        let surface = topo.face(face).expect("face").surface().clone();
        for lambda in ray_roots(&surface, p, d) {
            if lambda < -1e-12 * extent {
                continue;
            }
            let x = p + d * lambda;
            if oracle.on_face_distance(topo, x, face) <= ON_FACE_REL * extent
                && best.as_ref().is_none_or(|(l, _, _)| lambda < *l)
            {
                best = Some((lambda, x, face));
            }
        }
    }
    best.map(|(_, x, face)| (x, face))
}

/// The face whose support surface passes through `probe`; hemispheres that
/// share a sphere are told apart by the kernel trim test, which is correct
/// for `make_sphere`.
fn face_containing(topo: &Topology, solid: SolidId, probe: Point3, extent: f64) -> FaceId {
    let mut hits: Vec<_> = solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|&face| support_projection(topo, face, probe).distance <= 1e-9 * extent)
        .collect();
    if hits.len() > 1 {
        hits.retain(|&face| {
            remus_check::classify::surface_point_in_face(topo, face, probe).expect("trim test")
        });
    }
    assert_eq!(
        hits.len(),
        1,
        "probe {probe:?} must lie on exactly one face"
    );
    hits[0]
}

/// Angle about the axis measured from the face's seam half-plane, for
/// cylinder and cone faces that carry a seam edge.
fn seam_frame(topo: &Topology, face: FaceId) -> Option<(Point3, Vec3, Vec3)> {
    let (origin, axis) = match topo.face(face).expect("face").surface() {
        FaceSurface::Cylinder(c) => (c.origin(), c.axis()),
        FaceSurface::Cone(c) => (c.apex(), c.axis()),
        _ => return None,
    };
    let uses = topo.face_oriented_edges(face).expect("face edges");
    let seam = uses
        .iter()
        .map(remus_topology::OrientedEdge::edge)
        .find(|&e| uses.iter().filter(|oe| oe.edge() == e).count() == 2)?;
    let edge = topo.edge(seam).expect("seam edge");
    let a = topo.vertex(edge.start()).expect("seam start").point();
    let b = topo.vertex(edge.end()).expect("seam end").point();
    let far = if (a - origin).length() >= (b - origin).length() {
        a
    } else {
        b
    };
    let w = far - origin;
    let radial = (w - axis * w.dot(axis)).normalize().expect("seam radial");
    Some((origin, axis, radial))
}

fn seam_angle(frame: (Point3, Vec3, Vec3), q: Point3) -> f64 {
    let (origin, axis, radial) = frame;
    let w = q - origin;
    let r = w - axis * w.dot(axis);
    radial
        .cross(r)
        .dot(axis)
        .atan2(radial.dot(r))
        .rem_euclid(TAU)
}

// ---------------------------------------------------------------------------
// Expected answers.
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
enum Carrier {
    Line,
    Circle {
        center: Point3,
        axis: Vec3,
        radius: f64,
    },
    Ellipse {
        center: Point3,
        normal: Vec3,
        major: Vec3,
        a: f64,
        b: f64,
    },
    Hyperbola {
        center: Point3,
        normal: Vec3,
        real: Vec3,
        a: f64,
        b: f64,
    },
    Parabola {
        vertex: Point3,
        normal: Vec3,
        axis: Vec3,
        focal: f64,
    },
}

#[derive(Clone, Debug)]
struct Piece {
    face: FaceId,
    carrier: Carrier,
    start: Point3,
    end: Point3,
    range: (f64, f64),
    closed: bool,
    joins_previous: bool,
}

fn piece(face: FaceId, carrier: Carrier, start: Point3, end: Point3, range: (f64, f64)) -> Piece {
    Piece {
        face,
        carrier,
        start,
        end,
        range,
        closed: false,
        joins_previous: false,
    }
}

fn closed_piece(face: FaceId, carrier: Carrier, vertex: Point3, range: (f64, f64)) -> Piece {
    Piece {
        closed: true,
        ..piece(face, carrier, vertex, vertex, range)
    }
}

struct Cell {
    name: &'static str,
    topo: Topology,
    oracle: Oracle,
    face: FaceId,
    source: Source,
    source_edge: EdgeId,
    direction: Vec3,
    pieces: Vec<Piece>,
    clipped: bool,
    extent: f64,
}

#[derive(Clone, Copy, Debug)]
struct Got {
    edge: EdgeId,
    face: FaceId,
    range: (f64, f64),
}

fn got_from(result: &ProjectedCurves) -> Vec<Got> {
    result
        .edges
        .iter()
        .map(|e| Got {
            edge: e.edge,
            face: e.face,
            range: e.source_range,
        })
        .collect()
}

fn near(a: Point3, b: Point3, tol: f64) -> bool {
    (a - b).length() <= tol
}

fn parallel(a: Vec3, b: Vec3) -> bool {
    let (a, b) = (a.normalize().expect("a"), b.normalize().expect("b"));
    a.cross(b).length() <= 1e-9
}

fn same_direction(a: Vec3, b: Vec3) -> bool {
    let (a, b) = (a.normalize().expect("a"), b.normalize().expect("b"));
    a.dot(b) >= 1.0 - 1e-9
}

fn assert_carrier(label: &str, expected: &Carrier, curve: &EdgeCurve, tol: f64) {
    let close = |a: f64, b: f64| (a - b).abs() <= tol;
    match (expected, curve) {
        (Carrier::Line, EdgeCurve::Line) => {}
        (
            Carrier::Circle {
                center,
                axis,
                radius,
            },
            EdgeCurve::Circle(c),
        ) => assert!(
            near(c.center(), *center, tol)
                && parallel(c.normal(), *axis)
                && close(c.radius(), *radius),
            "{label}: circle {c:?} != center {center:?} axis {axis:?} radius {radius}"
        ),
        (
            Carrier::Ellipse {
                center,
                normal,
                major,
                a,
                b,
            },
            EdgeCurve::Ellipse(e),
        ) => assert!(
            near(e.center(), *center, tol)
                && parallel(e.normal(), *normal)
                && parallel(e.u_axis(), *major)
                && close(e.semi_major(), *a)
                && close(e.semi_minor(), *b),
            "{label}: ellipse {e:?} != center {center:?} normal {normal:?} major {major:?} a {a} b {b}"
        ),
        (
            Carrier::Hyperbola {
                center,
                normal,
                real,
                a,
                b,
            },
            EdgeCurve::Hyperbola(h),
        ) => assert!(
            near(h.center(), *center, tol)
                && parallel(h.normal(), *normal)
                && same_direction(h.u_axis(), *real)
                && close(h.semi_major(), *a)
                && close(h.semi_minor(), *b),
            "{label}: hyperbola {h:?} != center {center:?} real {real:?} a {a} b {b}"
        ),
        (
            Carrier::Parabola {
                vertex,
                normal,
                axis,
                focal,
            },
            EdgeCurve::Parabola(p),
        ) => assert!(
            near(p.vertex(), *vertex, tol)
                && parallel(p.normal(), *normal)
                && same_direction(p.axis_dir(), *axis)
                && close(p.focal_length(), *focal),
            "{label}: parabola {p:?} != vertex {vertex:?} axis {axis:?} focal {focal}"
        ),
        (expected, actual) => panic!(
            "{label}: expected {expected:?}, got a {} edge",
            actual.type_tag()
        ),
    }
}

fn range_matches(got: (f64, f64), expected: (f64, f64), periodic: bool) -> bool {
    if periodic {
        let start = unwrap_from(got.0, expected.0) - expected.0;
        start.abs() <= PARAM_ABS && ((got.1 - got.0) - (expected.1 - expected.0)).abs() <= PARAM_ABS
    } else {
        (got.0 - expected.0).abs() <= PARAM_ABS && (got.1 - expected.1).abs() <= PARAM_ABS
    }
}

/// The acceptance checker shared by the self-check and the API tests.
///
/// For every expected piece: face, source range, vertices, carrier type and
/// closed-form parameters, explicit domain, then per sample the on-face
/// residual, the direction residual, a monotone pre-image inside the piece's
/// source range, first-hit visibility against `visible`, and no crossing of
/// a face seam in the interior.
#[allow(clippy::too_many_arguments)]
fn check_pieces(
    label: &str,
    oracle: &Oracle,
    topo: &Topology,
    source: &Source,
    direction: Vec3,
    visible: &[FaceId],
    pieces: &[Piece],
    got: &[Got],
    extent: f64,
) {
    let d = direction.normalize().expect("direction");
    let tol = MATCH_REL * extent;
    let periodic_source = !matches!(source, Source::Segment { .. });
    assert_eq!(got.len(), pieces.len(), "{label}: piece count");
    for (i, (want, have)) in pieces.iter().zip(got).enumerate() {
        let label = format!("{label}[{i}]");
        assert_eq!(have.face, want.face, "{label}: face");
        assert!(
            range_matches(have.range, want.range, periodic_source),
            "{label}: source range {:?} != {:?}",
            have.range,
            want.range
        );
        let edge = topo.edge(have.edge).expect("result edge");
        let start = topo.vertex(edge.start()).expect("start").point();
        let end = topo.vertex(edge.end()).expect("end").point();
        assert!(
            near(start, want.start, tol),
            "{label}: start {start:?} != {:?}",
            want.start
        );
        assert!(
            near(end, want.end, tol),
            "{label}: end {end:?} != {:?}",
            want.end
        );
        assert_eq!(edge.start() == edge.end(), want.closed, "{label}: closed");
        if i > 0 {
            let previous_end = topo.edge(got[i - 1].edge).expect("previous").end();
            assert_eq!(
                previous_end == edge.start(),
                want.joins_previous,
                "{label}: vertex sharing with the previous piece"
            );
        }
        assert_carrier(&label, &want.carrier, edge.curve(), tol);
        let (t0, t1) = edge.strict_domain().expect("explicit domain");
        let curve = edge.curve();
        assert!(
            near(curve.evaluate_with_endpoints(t0, start, end), start, tol),
            "{label}: domain start"
        );
        assert!(
            near(curve.evaluate_with_endpoints(t1, start, end), end, tol),
            "{label}: domain end"
        );

        let seam = seam_frame(topo, have.face);
        let mut last_s = f64::NEG_INFINITY;
        let mut last_phi: Option<f64> = None;
        for k in 0..=SAMPLES {
            let t = t0 + (t1 - t0) * (k as f64 / SAMPLES as f64);
            let q = curve.evaluate_with_endpoints(t, start, end);
            let on_face = oracle.on_face_distance(topo, q, have.face);
            assert!(
                on_face <= ON_FACE_REL * extent,
                "{label}: on-face residual {on_face:e} at {k}"
            );
            let (s, lambda, angle) = preimage(source, q, d);
            let s = match (periodic_source, k) {
                (false, _) => s,
                (true, 0) => unwrap_from(s, have.range.0),
                (true, _) => last_s + (s - last_s + PI).rem_euclid(TAU) - PI,
            };
            assert!(
                angle <= DIRECTION_RESIDUAL,
                "{label}: direction residual {angle:e} at {k}"
            );
            assert!(
                lambda >= -tol,
                "{label}: image behind the source (λ = {lambda:e})"
            );
            assert!(
                s >= have.range.0 - PARAM_ABS && s <= have.range.1 + PARAM_ABS,
                "{label}: pre-image {s} outside {:?}",
                have.range
            );
            assert!(
                s > last_s - PARAM_ABS,
                "{label}: image runs against source order at {k}"
            );
            last_s = s;
            if k == 0 || k == SAMPLES {
                continue;
            }
            let (hit, hit_face) = first_hit(oracle, topo, visible, source.point(s), d, extent)
                .unwrap_or_else(|| panic!("{label}: oracle finds no hit at sample {k}"));
            assert!(
                near(hit, q, 1e-6 * extent) && hit_face == have.face,
                "{label}: sample {k} is not the first hit ({q:?} vs oracle {hit:?})"
            );
            if let Some(frame) = seam {
                let phi = seam_angle(frame, q);
                if let Some(previous) = last_phi {
                    assert!(
                        (phi - previous).abs() < PI,
                        "{label}: crosses the face seam at {k}"
                    );
                }
                last_phi = Some(phi);
            }
        }
    }
}

fn check_cell(cell: &Cell, got: &[Got]) {
    check_pieces(
        cell.name,
        &cell.oracle,
        &cell.topo,
        &cell.source,
        cell.direction,
        &[cell.face],
        &cell.pieces,
        got,
        cell.extent,
    );
}

// ---------------------------------------------------------------------------
// Hand-built expected edges for the self-check.
// ---------------------------------------------------------------------------

fn build_expected_edge(
    topo: &mut Topology,
    want: &Piece,
    probe: Point3,
    shared_start: Option<VertexId>,
) -> EdgeId {
    let ends = (want.start, want.end);
    let (start, end) = ends;
    let mut add = |curve: EdgeCurve, trim: Option<(f64, f64)>| {
        add_edge_between(topo, curve, ends, trim, want.closed, shared_start)
    };
    match &want.carrier {
        Carrier::Line => add(EdgeCurve::Line, None),
        Carrier::Circle {
            center,
            axis,
            radius,
        } => {
            for flip in [1.0, -1.0] {
                let c = Circle3D::new_with_ref(*center, *axis * flip, *radius, start - *center)
                    .expect("expected circle");
                let t1 = if want.closed {
                    TAU
                } else {
                    c.project(end).rem_euclid(TAU)
                };
                if c.project(probe).rem_euclid(TAU) < t1 {
                    return add(EdgeCurve::Circle(c), Some((0.0, t1)));
                }
            }
            panic!("probe not on either arc")
        }
        Carrier::Ellipse {
            center,
            normal,
            major,
            a,
            b,
        } => {
            let major = major.normalize().expect("major");
            for flip in [1.0, -1.0] {
                let n = normal.normalize().expect("normal") * flip;
                let e = Ellipse3D::with_axes(*center, n, *a, *b, major, n.cross(major))
                    .expect("expected ellipse");
                let t0 = e.project(start).rem_euclid(TAU);
                let t1 = if want.closed {
                    t0 + TAU
                } else {
                    unwrap_from(e.project(end), t0)
                };
                if unwrap_from(e.project(probe), t0) < t1 {
                    return add(EdgeCurve::Ellipse(e), Some((t0, t1)));
                }
            }
            panic!("probe not on either ellipse arc")
        }
        Carrier::Hyperbola {
            center,
            normal,
            real,
            a,
            b,
        } => {
            for flip in [1.0, -1.0] {
                let h = Hyperbola3D::with_axes(*center, *normal * flip, *real, *a, *b)
                    .expect("expected hyperbola");
                let (t0, t1) = (h.project(start), h.project(end));
                if t0 < t1 {
                    return add(EdgeCurve::Hyperbola(h), Some((t0, t1)));
                }
            }
            panic!("degenerate hyperbola span")
        }
        Carrier::Parabola {
            vertex,
            normal,
            axis,
            focal,
        } => {
            let axis = axis.normalize().expect("axis");
            for flip in [1.0, -1.0] {
                let u = normal.normalize().expect("normal").cross(axis) * flip;
                let p = Parabola3D::with_axes(*vertex, axis, u, *focal).expect("expected parabola");
                let (t0, t1) = (p.project(start), p.project(end));
                if t0 < t1 {
                    return add(EdgeCurve::Parabola(p), Some((t0, t1)));
                }
            }
            panic!("degenerate parabola span")
        }
    }
}

/// Builds the expected pieces as edges, sharing vertices where the note
/// requires it, using the independent oracle to pick each arc's side.
fn build_expected(
    oracle: &Oracle,
    topo: &mut Topology,
    source: &Source,
    visible: &[FaceId],
    direction: Vec3,
    pieces: &[Piece],
    extent: f64,
) -> Vec<Got> {
    let d = direction.normalize().expect("direction");
    let mut got: Vec<Got> = Vec::new();
    for want in pieces {
        let s = want.range.0 + 0.25 * (want.range.1 - want.range.0);
        let (probe, _) =
            first_hit(oracle, topo, visible, source.point(s), d, extent).expect("oracle probe");
        let shared = want.joins_previous.then(|| {
            topo.edge(got.last().expect("previous piece").edge)
                .expect("edge")
                .end()
        });
        let edge = build_expected_edge(topo, want, probe, shared);
        got.push(Got {
            edge,
            face: want.face,
            range: want.range,
        });
    }
    got
}

fn self_check(cell: &mut Cell) {
    let (lo, hi) = cell.source.domain();
    let covered: f64 = cell.pieces.iter().map(|p| p.range.1 - p.range.0).sum();
    assert_eq!(
        covered < hi - lo - PARAM_ABS,
        cell.clipped,
        "{}: `clipped` must mean the pieces do not cover the source domain",
        cell.name
    );
    let got = build_expected(
        &cell.oracle,
        &mut cell.topo,
        &cell.source,
        &[cell.face],
        cell.direction,
        &cell.pieces,
        cell.extent,
    );
    check_cell(cell, &got);
}

// ---------------------------------------------------------------------------
// Targets.
// ---------------------------------------------------------------------------

struct Target {
    topo: Topology,
    face: FaceId,
    oracle: Oracle,
}

const BOX: Region = Region::BoxFace {
    dx: 10.0,
    dy: 8.0,
    dz: 4.0,
};

/// Box 10 × 8 × 4 (P cells); the top face (z = 4).
fn box_target(pl: &Placement) -> Target {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, pl.len(10.0), pl.len(8.0), pl.len(4.0)).expect("box");
    pl.place(&mut topo, solid);
    let face = face_containing(&topo, solid, pl.p(5.0, 4.0, 4.0), pl.len(10.0));
    let oracle = Oracle::new(pl).with(face, BOX);
    Target { topo, face, oracle }
}

/// Box 10 × 8 × 4 bored through by a radius-1 hole at (5, 4); top face.
fn bored_plate_target(pl: &Placement) -> Target {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, pl.len(10.0), pl.len(8.0), pl.len(4.0)).expect("block");
    let drill = make_cylinder(&mut topo, pl.len(1.0), pl.len(8.0)).expect("drill");
    transform_solid(
        &mut topo,
        drill,
        &Mat4::translation(pl.len(5.0), pl.len(4.0), pl.len(-2.0)),
    )
    .expect("place drill");
    let solid = boolean(&mut topo, BooleanOp::Cut, block, drill).expect("bore");
    pl.place(&mut topo, solid);
    let face = face_containing(&topo, solid, pl.p(2.0, 2.0, 4.0), pl.len(10.0));
    let oracle = Oracle::new(pl).with(face, Region::BoredTop);
    Target { topo, face, oracle }
}

/// Cylinder r = 2, h = 10, axis +z, seam edge at +x (C cells); lateral face.
/// `turn` rotates the cylinder about its axis before placement.
fn cylinder_target(pl: &Placement, turn: f64) -> Target {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, pl.len(2.0), pl.len(10.0)).expect("cylinder");
    if turn != 0.0 {
        transform_solid(&mut topo, solid, &Mat4::rotation_z(turn)).expect("turn");
    }
    pl.place(&mut topo, solid);
    let face = face_containing(&topo, solid, pl.p(0.0, 2.0, 5.0), pl.len(10.0));
    let oracle = Oracle::new(pl).with(face, Region::Lateral { h: 10.0 });
    Target { topo, face, oracle }
}

/// Sphere R = 3 at the origin (S cells); north hemisphere face.
fn sphere_target(pl: &Placement) -> Target {
    let mut topo = Topology::new();
    let solid = make_sphere(&mut topo, pl.len(3.0), 64).expect("sphere");
    pl.place(&mut topo, solid);
    let face = face_containing(&topo, solid, pl.p(0.0, 0.0, 3.0), pl.len(10.0));
    let oracle = Oracle::new(pl).with(face, Region::North);
    Target { topo, face, oracle }
}

/// Pointed cone, base radius 3 at z = 0, apex (0, 0, 3), half-angle 45°,
/// axis −z, seam edge at +x (K cells); lateral face.
fn cone_target(pl: &Placement) -> Target {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, pl.len(3.0), 0.0, pl.len(3.0)).expect("cone");
    pl.place(&mut topo, solid);
    let face = face_containing(&topo, solid, pl.p(0.0, 1.5, 1.5), pl.len(10.0));
    let oracle = Oracle::new(pl).with(face, Region::Lateral { h: 3.0 });
    Target { topo, face, oracle }
}

fn cell(
    name: &'static str,
    target: Target,
    source: Source,
    direction: Vec3,
    pieces: impl FnOnce(FaceId) -> Vec<Piece>,
    clipped: bool,
    extent: f64,
) -> Cell {
    let Target {
        mut topo,
        face,
        oracle,
        ..
    } = target;
    let source_edge = add_source(&mut topo, &source);
    Cell {
        name,
        topo,
        oracle,
        face,
        source,
        source_edge,
        direction,
        pieces: pieces(face),
        clipped,
        extent,
    }
}

/// Semi-axes and major direction of the ellipse with conjugate
/// semi-diameters `a` and `b` (the affine-image rule of the design note).
fn gram_ellipse(a: Vec3, b: Vec3) -> (Vec3, f64, f64) {
    let (g11, g12, g22) = (a.dot(a), a.dot(b), b.dot(b));
    let tr = g11 + g22;
    let det = g11.mul_add(g22, -(g12 * g12));
    let root = tr.mul_add(tr, -4.0 * det).max(0.0).sqrt();
    let (l1, l2) = (0.5 * (tr + root), 0.5 * (tr - root));
    let (w1, w2) = if g12.abs() > 1e-15 * tr {
        (g12, l1 - g11)
    } else if g11 >= g22 {
        (1.0, 0.0)
    } else {
        (0.0, 1.0)
    };
    let major = (a * w1 + b * w2).normalize().expect("major axis");
    (major, l1.sqrt(), l2.sqrt())
}

// ---------------------------------------------------------------------------
// Cells (design note §4). Coordinates are unit-scale; the placement scales
// and moves both the target and the expected answer.
// ---------------------------------------------------------------------------

fn cell_p1(pl: &Placement) -> Cell {
    cell(
        "P1 segment → plane",
        box_target(pl),
        Source::Segment {
            a: pl.p(1.0, 2.0, 7.0),
            b: pl.p(8.0, 5.0, 9.0),
        },
        pl.v(0.3, -0.2, -1.0),
        |face| {
            vec![piece(
                face,
                Carrier::Line,
                pl.p(1.9, 1.4, 4.0),
                pl.p(9.5, 4.0, 4.0),
                (0.0, 1.0),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_p2(pl: &Placement) -> Cell {
    cell(
        "P2 arc → parallel plane",
        box_target(pl),
        arc(
            pl.p(4.0, 4.0, 7.0),
            pl.v(0.0, 0.0, 1.0),
            pl.len(2.0),
            pl.v(1.0, 0.0, 0.0),
            (0.0, PI),
        ),
        pl.v(0.3, -0.2, -1.0),
        |face| {
            vec![piece(
                face,
                Carrier::Circle {
                    center: pl.p(4.9, 3.4, 4.0),
                    axis: pl.v(0.0, 0.0, 1.0),
                    radius: pl.len(2.0),
                },
                pl.p(6.9, 3.4, 4.0),
                pl.p(2.9, 3.4, 4.0),
                (0.0, PI),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn tilted_normal(pl: &Placement) -> Vec3 {
    let alpha = PI / 3.0;
    pl.v(0.0, alpha.sin(), alpha.cos())
}

fn cell_p3(pl: &Placement) -> Cell {
    cell(
        "P3 tilted arc → plane along its normal",
        box_target(pl),
        arc(
            pl.p(5.0, 4.0, 8.0),
            tilted_normal(pl),
            pl.len(2.0),
            pl.v(1.0, 0.0, 0.0),
            (0.0, TAU),
        ),
        pl.v(0.0, 0.0, -1.0),
        |face| {
            vec![closed_piece(
                face,
                Carrier::Ellipse {
                    center: pl.p(5.0, 4.0, 4.0),
                    normal: pl.v(0.0, 0.0, 1.0),
                    major: pl.v(1.0, 0.0, 0.0),
                    a: pl.len(2.0),
                    b: pl.len(1.0),
                },
                pl.p(7.0, 4.0, 4.0),
                (0.0, TAU),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_p4(pl: &Placement) -> Cell {
    let d = pl.v(0.3, -0.2, -1.0);
    let n = pl.v(0.0, 0.0, 1.0);
    let u = pl.v(1.0, 0.0, 0.0);
    let v = tilted_normal(pl).cross(u);
    let linear = |w: Vec3| w - d * (n.dot(w) / n.dot(d));
    let (major, a, b) = gram_ellipse(linear(u) * pl.len(2.0), linear(v) * pl.len(2.0));
    assert!((a - pl.len(2.110_742_241_242_658_4)).abs() <= 1e-12 * pl.len(1.0));
    assert!((b - pl.len(1.275_769_381_221_179_4)).abs() <= 1e-12 * pl.len(1.0));
    cell(
        "P4 tilted arc → plane, oblique direction",
        box_target(pl),
        arc(
            pl.p(5.0, 4.0, 8.0),
            tilted_normal(pl),
            pl.len(2.0),
            u,
            (0.0, TAU),
        ),
        d,
        |face| {
            vec![closed_piece(
                face,
                Carrier::Ellipse {
                    center: pl.p(6.2, 3.2, 4.0),
                    normal: n,
                    major,
                    a,
                    b,
                },
                pl.p(8.2, 3.2, 4.0),
                (0.0, TAU),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_p5(pl: &Placement) -> Cell {
    let ellipse = Ellipse3D::new_with_ref(
        pl.p(5.0, 4.0, 7.0),
        pl.v(0.0, 0.0, 1.0),
        pl.len(3.0),
        pl.len(1.5),
        pl.v(1.0, 1.0, 0.0),
    )
    .expect("source ellipse");
    let h = 1.5 * 2f64.sqrt();
    cell(
        "P5 ellipse → parallel plane",
        box_target(pl),
        Source::Ellipse {
            ellipse,
            range: (0.0, TAU),
        },
        pl.v(0.3, -0.2, -1.0),
        |face| {
            vec![closed_piece(
                face,
                Carrier::Ellipse {
                    center: pl.p(5.9, 3.4, 4.0),
                    normal: pl.v(0.0, 0.0, 1.0),
                    major: pl.v(1.0, 1.0, 0.0),
                    a: pl.len(3.0),
                    b: pl.len(1.5),
                },
                pl.p(5.9 + h, 3.4 + h, 4.0),
                (0.0, TAU),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_hole_clip(pl: &Placement) -> Cell {
    cell(
        "CLIP segment across a holed plate",
        bored_plate_target(pl),
        Source::Segment {
            a: pl.p(-2.0, 4.0, 6.0),
            b: pl.p(12.0, 4.0, 6.0),
        },
        pl.v(0.0, 0.0, -1.0),
        |face| {
            vec![
                piece(
                    face,
                    Carrier::Line,
                    pl.p(0.0, 4.0, 4.0),
                    pl.p(4.0, 4.0, 4.0),
                    (1.0 / 7.0, 3.0 / 7.0),
                ),
                piece(
                    face,
                    Carrier::Line,
                    pl.p(6.0, 4.0, 4.0),
                    pl.p(10.0, 4.0, 4.0),
                    (4.0 / 7.0, 6.0 / 7.0),
                ),
            ]
        },
        true,
        pl.len(10.0),
    )
}

fn cell_c1(pl: &Placement) -> Cell {
    let r3 = 3f64.sqrt();
    cell(
        "C1 segment ∥ axis → ruling",
        cylinder_target(pl, 0.0),
        Source::Segment {
            a: pl.p(1.0, 5.0, 2.0),
            b: pl.p(1.0, 5.0, 8.0),
        },
        pl.v(0.0, -1.0, 0.0),
        |face| {
            vec![piece(
                face,
                Carrier::Line,
                pl.p(1.0, r3, 2.0),
                pl.p(1.0, r3, 8.0),
                (0.0, 1.0),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_c2(pl: &Placement) -> Cell {
    cell(
        "C2 segment ⟂ axis, d ⟂ axis → circle",
        cylinder_target(pl, 0.0),
        Source::Segment {
            a: pl.p(-1.0, 5.0, 4.0),
            b: pl.p(1.5, 5.0, 4.0),
        },
        pl.v(0.0, -1.0, 0.0),
        |face| {
            vec![piece(
                face,
                Carrier::Circle {
                    center: pl.p(0.0, 0.0, 4.0),
                    axis: pl.v(0.0, 0.0, 1.0),
                    radius: pl.len(2.0),
                },
                pl.p(-1.0, 3f64.sqrt(), 4.0),
                pl.p(1.5, 1.75f64.sqrt(), 4.0),
                (0.0, 1.0),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_c3(pl: &Placement) -> Cell {
    let r3 = 3f64.sqrt();
    cell(
        "C3 segment ⟂ axis, oblique d → ellipse",
        cylinder_target(pl, 0.0),
        Source::Segment {
            a: pl.p(-1.0, 5.0, 6.0),
            b: pl.p(1.0, 5.0, 6.0),
        },
        pl.v(0.0, -1.0, -1.0),
        |face| {
            vec![piece(
                face,
                Carrier::Ellipse {
                    center: pl.p(0.0, 0.0, 1.0),
                    normal: pl.v(0.0, 1.0, -1.0),
                    major: pl.v(0.0, 1.0, 1.0),
                    a: pl.len(2.0 * 2f64.sqrt()),
                    b: pl.len(2.0),
                },
                pl.p(-1.0, r3, 1.0 + r3),
                pl.p(1.0, r3, 1.0 + r3),
                (0.0, 1.0),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_c4_seam(pl: &Placement) -> Cell {
    let circle = Carrier::Circle {
        center: pl.p(0.0, 0.0, 3.0),
        axis: pl.v(0.0, 0.0, 1.0),
        radius: pl.len(2.0),
    };
    cell(
        "C4 circle across the cylinder seam",
        cylinder_target(pl, 0.0),
        Source::Segment {
            a: pl.p(5.0, -1.0, 3.0),
            b: pl.p(5.0, 1.5, 3.0),
        },
        pl.v(-1.0, 0.0, 0.0),
        |face| {
            vec![
                piece(
                    face,
                    circle.clone(),
                    pl.p(3f64.sqrt(), -1.0, 3.0),
                    pl.p(2.0, 0.0, 3.0),
                    (0.0, 0.4),
                ),
                Piece {
                    joins_previous: true,
                    ..piece(
                        face,
                        circle,
                        pl.p(2.0, 0.0, 3.0),
                        pl.p(1.75f64.sqrt(), 1.5, 3.0),
                        (0.4, 1.0),
                    )
                },
            ]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_c5_silhouette(pl: &Placement) -> Cell {
    cell(
        "C5 segment wider than the cylinder → silhouette clip",
        cylinder_target(pl, -FRAC_PI_2),
        Source::Segment {
            a: pl.p(-3.0, 5.0, 4.0),
            b: pl.p(3.0, 5.0, 4.0),
        },
        pl.v(0.0, -1.0, 0.0),
        |face| {
            vec![piece(
                face,
                Carrier::Circle {
                    center: pl.p(0.0, 0.0, 4.0),
                    axis: pl.v(0.0, 0.0, 1.0),
                    radius: pl.len(2.0),
                },
                pl.p(-2.0, 0.0, 4.0),
                pl.p(2.0, 0.0, 4.0),
                (1.0 / 6.0, 5.0 / 6.0),
            )]
        },
        true,
        pl.len(10.0),
    )
}

fn cell_s1(pl: &Placement) -> Cell {
    cell(
        "S1 segment → sphere circle",
        sphere_target(pl),
        Source::Segment {
            a: pl.p(-1.0, 1.0, 6.0),
            b: pl.p(2.0, 1.0, 6.0),
        },
        pl.v(0.0, 0.0, -1.0),
        |face| {
            vec![piece(
                face,
                Carrier::Circle {
                    center: pl.p(0.0, 1.0, 0.0),
                    axis: pl.v(0.0, 1.0, 0.0),
                    radius: pl.len(8f64.sqrt()),
                },
                pl.p(-1.0, 1.0, 7f64.sqrt()),
                pl.p(2.0, 1.0, 2.0),
                (0.0, 1.0),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_s2_pole(pl: &Placement) -> Cell {
    cell(
        "S2 great circle over the pole, one edge",
        sphere_target(pl),
        Source::Segment {
            a: pl.p(0.0, -1.0, 6.0),
            b: pl.p(0.0, 1.5, 6.0),
        },
        pl.v(0.0, 0.0, -1.0),
        |face| {
            vec![piece(
                face,
                Carrier::Circle {
                    center: pl.p(0.0, 0.0, 0.0),
                    axis: pl.v(1.0, 0.0, 0.0),
                    radius: pl.len(3.0),
                },
                pl.p(0.0, -1.0, 8f64.sqrt()),
                pl.p(0.0, 1.5, 6.75f64.sqrt()),
                (0.0, 1.0),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_s3_coaxial(pl: &Placement) -> Cell {
    let h = 2.5f64.sqrt();
    cell(
        "S3 coaxial arc → sphere circle",
        sphere_target(pl),
        arc(
            pl.p(0.0, 0.0, 6.0),
            pl.v(0.0, 0.0, 1.0),
            pl.len(5f64.sqrt()),
            pl.v(1.0, 0.0, 0.0),
            (FRAC_PI_4, 3.0 * FRAC_PI_4),
        ),
        pl.v(0.0, 0.0, -1.0),
        |face| {
            vec![piece(
                face,
                Carrier::Circle {
                    center: pl.p(0.0, 0.0, 2.0),
                    axis: pl.v(0.0, 0.0, 1.0),
                    radius: pl.len(5f64.sqrt()),
                },
                pl.p(h, h, 2.0),
                pl.p(-h, h, 2.0),
                (FRAC_PI_4, 3.0 * FRAC_PI_4),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_k1(pl: &Placement) -> Cell {
    let r3 = 3f64.sqrt();
    cell(
        "K1 segment → cone circle",
        cone_target(pl),
        Source::Segment {
            a: pl.p(-1.0, 5.0, 1.0),
            b: pl.p(1.0, 5.0, 1.0),
        },
        pl.v(0.0, -1.0, 0.0),
        |face| {
            vec![piece(
                face,
                Carrier::Circle {
                    center: pl.p(0.0, 0.0, 1.0),
                    axis: pl.v(0.0, 0.0, 1.0),
                    radius: pl.len(2.0),
                },
                pl.p(-1.0, r3, 1.0),
                pl.p(1.0, r3, 1.0),
                (0.0, 1.0),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_k2(pl: &Placement) -> Cell {
    let r3 = 3f64.sqrt();
    let (y0, z0) = (-0.25 - 1.5 * r3, 4.5 - 0.25 * r3);
    let (cy, cz) = (0.5, 3.0 - 0.5 * r3);
    let (e1y, e1z) = (0.5 * r3, -0.5);
    let image = |x: f64| {
        let back = (1.0 - 2.0 * x * x).sqrt();
        pl.p(x, cy - e1y * back, cz - e1z * back)
    };
    cell(
        "K2 segment → cone ellipse",
        cone_target(pl),
        Source::Segment {
            a: pl.p(0.5, y0, z0),
            b: pl.p(-0.6, y0, z0),
        },
        pl.v(0.0, e1y, e1z),
        |face| {
            vec![piece(
                face,
                Carrier::Ellipse {
                    center: pl.p(0.0, cy, cz),
                    normal: pl.v(0.0, 0.5, 0.5 * r3),
                    major: pl.v(0.0, e1y, e1z),
                    a: pl.len(1.0),
                    b: pl.len(0.5f64.sqrt()),
                },
                image(0.5),
                image(-0.6),
                (0.0, 1.0),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_k3(pl: &Placement) -> Cell {
    cell(
        "K3 segment ∥ cone axis plane → hyperbola",
        cone_target(pl),
        Source::Segment {
            a: pl.p(-1.0, 1.0, 5.0),
            b: pl.p(1.5, 1.0, 5.0),
        },
        pl.v(0.0, 0.0, -1.0),
        |face| {
            vec![piece(
                face,
                Carrier::Hyperbola {
                    center: pl.p(0.0, 1.0, 3.0),
                    normal: pl.v(0.0, 1.0, 0.0),
                    real: pl.v(0.0, 0.0, -1.0),
                    a: pl.len(1.0),
                    b: pl.len(1.0),
                },
                pl.p(-1.0, 1.0, 3.0 - 2f64.sqrt()),
                pl.p(1.5, 1.0, 3.0 - 3.25f64.sqrt()),
                (0.0, 1.0),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_k4(pl: &Placement) -> Cell {
    cell(
        "K4 sweep plane ∥ generator → parabola",
        cone_target(pl),
        Source::Segment {
            a: pl.p(-1.2, -2.0, 4.5),
            b: pl.p(0.3, -2.0, 4.5),
        },
        pl.v(0.0, 1.0, -1.0),
        |face| {
            vec![piece(
                face,
                Carrier::Parabola {
                    vertex: pl.p(0.0, -0.25, 2.75),
                    normal: pl.v(0.0, 1.0, 1.0),
                    axis: pl.v(0.0, 1.0, -1.0),
                    focal: pl.len(2f64.sqrt() / 8.0),
                },
                pl.p(-1.2, 1.19, 1.31),
                pl.p(0.3, -0.16, 2.66),
                (0.0, 1.0),
            )]
        },
        false,
        pl.len(10.0),
    )
}

fn cell_k5_coaxial(pl: &Placement) -> Cell {
    cell(
        "K5 coaxial full circle → cone, vertex at the seam",
        cone_target(pl),
        arc(
            pl.p(0.0, 0.0, 6.0),
            pl.v(0.0, 0.0, 1.0),
            pl.len(1.5),
            pl.v(0.0, 1.0, 0.0),
            (0.0, TAU),
        ),
        pl.v(0.0, 0.0, -1.0),
        |face| {
            vec![closed_piece(
                face,
                Carrier::Circle {
                    center: pl.p(0.0, 0.0, 1.5),
                    axis: pl.v(0.0, 0.0, 1.0),
                    radius: pl.len(1.5),
                },
                pl.p(1.5, 0.0, 1.5),
                (1.5 * PI, 3.5 * PI),
            )]
        },
        false,
        pl.len(10.0),
    )
}

type CellFn = fn(&Placement) -> Cell;

const EXACT_CELLS: &[CellFn] = &[
    cell_p1,
    cell_p2,
    cell_p3,
    cell_p4,
    cell_p5,
    cell_hole_clip,
    cell_c1,
    cell_c2,
    cell_c3,
    cell_c4_seam,
    cell_c5_silhouette,
    cell_s1,
    cell_s2_pole,
    cell_s3_coaxial,
    cell_k1,
    cell_k2,
    cell_k3,
    cell_k4,
    cell_k5_coaxial,
];

// ---------------------------------------------------------------------------
// Self-check: runs now, validates the closed forms and the oracle.
// ---------------------------------------------------------------------------

#[test]
fn oracle_self_check_every_exact_cell_at_every_placement() {
    for build in EXACT_CELLS {
        for pl in placements() {
            let mut cell = build(&pl);
            self_check(&mut cell);
        }
    }
}

#[test]
fn oracle_self_check_solid_level_box_pieces() {
    for pl in placements() {
        let SolidFixture {
            mut topo,
            solid,
            oracle,
            pieces,
            sources,
            direction: d,
            extent,
        } = solid_box_fixture(&pl);
        let faces = solid_faces(&topo, solid).expect("faces");
        let got = build_expected(&oracle, &mut topo, &sources[0], &faces, d, &pieces, extent);
        check_pieces(
            "solid box",
            &oracle,
            &topo,
            &sources[0],
            d,
            &faces,
            &pieces,
            &got,
            extent,
        );
        let miss = first_hit(
            &oracle,
            &topo,
            &faces,
            sources[1].point(0.5),
            d.normalize().expect("d"),
            extent,
        );
        assert!(miss.is_none(), "second source must miss the box");
    }
}

// ---------------------------------------------------------------------------
// Exact cells through the API.
// ---------------------------------------------------------------------------

fn run_cell(build: CellFn) {
    for pl in placements() {
        let mut cell = build(&pl);
        let before = Census::of(&cell.topo);
        let result = project_curve_onto_face(
            &mut cell.topo,
            cell.source_edge,
            cell.direction,
            cell.face,
            &ProjectCurveOptions::default(),
        )
        .unwrap_or_else(|e| panic!("{} at {pl:?}: {e}", cell.name));
        assert_eq!(result.face, cell.face, "{}: face", cell.name);
        assert_eq!(
            result.quality,
            ProjectionQuality::Exact,
            "{}: quality",
            cell.name
        );
        assert_eq!(result.clipped, cell.clipped, "{}: clipped", cell.name);
        assert!(
            result.edges.iter().all(|e| e.plane_curve.is_none()),
            "{}: no frame, no 2D",
            cell.name
        );
        check_cell(&cell, &got_from(&result));
        before.assert_only_free_edges_added(&cell.topo, &result);
    }
}

#[test]
fn p1_segment_onto_plane_is_a_line() {
    run_cell(cell_p1);
}

#[test]
fn p2_arc_onto_parallel_plane_is_a_congruent_circle() {
    run_cell(cell_p2);
}

#[test]
fn p3_tilted_arc_onto_plane_is_an_ellipse_r_and_r_cos_alpha() {
    run_cell(cell_p3);
}

#[test]
fn p4_tilted_arc_oblique_direction_follows_the_affine_rule() {
    run_cell(cell_p4);
}

#[test]
fn p5_ellipse_onto_parallel_plane_is_translated() {
    run_cell(cell_p5);
}

#[test]
fn clip_holed_plate_gives_two_sub_edges_in_source_order() {
    run_cell(cell_hole_clip);
}

#[test]
fn c1_segment_parallel_to_axis_is_a_ruling() {
    run_cell(cell_c1);
}

#[test]
fn c2_segment_perpendicular_to_axis_is_a_circular_arc() {
    run_cell(cell_c2);
}

#[test]
fn c3_oblique_direction_is_an_elliptic_arc_r_and_r_over_cos_theta() {
    run_cell(cell_c3);
}

#[test]
fn c4_seam_crossing_splits_into_two_edges_sharing_the_seam_vertex() {
    run_cell(cell_c4_seam);
}

#[test]
fn c5_silhouette_clips_to_the_front_half() {
    run_cell(cell_c5_silhouette);
}

#[test]
fn s1_segment_onto_sphere_is_a_circle_sqrt_r2_minus_h2() {
    run_cell(cell_s1);
}

#[test]
fn s2_image_over_the_pole_of_a_seamless_face_is_one_edge() {
    run_cell(cell_s2_pole);
}

#[test]
fn s3_coaxial_arc_onto_sphere_is_a_circle() {
    run_cell(cell_s3_coaxial);
}

#[test]
fn k1_cone_section_perpendicular_to_axis_is_a_circle() {
    run_cell(cell_k1);
}

#[test]
fn k2_cone_section_steeper_than_generator_is_an_ellipse() {
    run_cell(cell_k2);
}

#[test]
fn k3_cone_section_parallel_to_axis_is_a_hyperbola_real_branch() {
    run_cell(cell_k3);
}

#[test]
fn k4_cone_section_parallel_to_a_generator_is_a_parabola() {
    run_cell(cell_k4);
}

#[test]
fn k5_closed_coaxial_circle_on_cone_puts_its_vertex_on_the_seam() {
    run_cell(cell_k5_coaxial);
}

// ---------------------------------------------------------------------------
// Ownership, journal and rollback.
// ---------------------------------------------------------------------------

#[derive(Debug, PartialEq, Eq)]
struct Census {
    vertices: usize,
    edges: usize,
    wires: usize,
    faces: usize,
    shells: usize,
    solids: usize,
    pcurves: usize,
    journal: usize,
}

impl Census {
    fn of(topo: &Topology) -> Self {
        Self {
            vertices: topo.num_vertices(),
            edges: topo.num_edges(),
            wires: topo.num_wires(),
            faces: topo.num_faces(),
            shells: topo.num_shells(),
            solids: topo.num_solids(),
            pcurves: topo.num_pcurves(),
            journal: topo.journal().len(),
        }
    }

    /// Success adds only free edges and their vertices: no wires, faces,
    /// pcurves or journal entries, and every new edge is unused by any face.
    fn assert_only_free_edges_added(&self, topo: &Topology, result: &ProjectedCurves) {
        let after = Self::of(topo);
        let mut vertices: Vec<_> = result
            .edges
            .iter()
            .flat_map(|e| {
                let edge = topo.edge(e.edge).expect("edge");
                [edge.start(), edge.end()]
            })
            .collect();
        vertices.sort_unstable_by_key(|v| v.index());
        vertices.dedup();
        assert_eq!(after.edges, self.edges + result.edges.len(), "new edges");
        assert_eq!(
            after.vertices,
            self.vertices + vertices.len(),
            "new vertices"
        );
        assert_eq!(
            (
                after.wires,
                after.faces,
                after.shells,
                after.solids,
                after.pcurves,
                after.journal
            ),
            (
                self.wires,
                self.faces,
                self.shells,
                self.solids,
                self.pcurves,
                self.journal
            ),
            "projection must create only free edges and record no journal entry"
        );
        for e in &result.edges {
            assert!(
                topo.coedges_of_edge(e.edge).is_empty(),
                "projected edge must be free"
            );
        }
    }
}

fn assert_refused<T: std::fmt::Debug>(
    result: Result<T, ProjectCurveError>,
    topo: &Topology,
    before: &Census,
    code: &str,
) -> ProjectCurveError {
    let error = result.expect_err("projection must refuse");
    assert_eq!(error.code(), code, "refusal: {error}");
    assert_eq!(
        &Census::of(topo),
        before,
        "a refusal must leave the topology unchanged"
    );
    error
}

#[test]
fn success_leaves_source_and_target_untouched() {
    let mut cell = cell_p1(&UNIT);
    let source = topo_edge_points(&cell.topo, cell.source_edge);
    let target = cell.topo.face(cell.face).expect("face").clone();
    let result = project_curve_onto_face(
        &mut cell.topo,
        cell.source_edge,
        cell.direction,
        cell.face,
        &ProjectCurveOptions::default(),
    )
    .expect("P1");
    assert_eq!(topo_edge_points(&cell.topo, cell.source_edge), source);
    let after = cell.topo.face(cell.face).expect("face");
    assert_eq!(after.outer_wire(), target.outer_wire());
    assert_eq!(after.inner_wires(), target.inner_wires());
    assert_eq!(result.edges.len(), 1);
}

fn topo_edge_points(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let edge = topo.edge(edge).expect("edge");
    (
        topo.vertex(edge.start()).expect("start").point(),
        topo.vertex(edge.end()).expect("end").point(),
    )
}

// ---------------------------------------------------------------------------
// Solid-level projection.
// ---------------------------------------------------------------------------

struct SolidFixture {
    topo: Topology,
    solid: SolidId,
    oracle: Oracle,
    pieces: Vec<Piece>,
    sources: [Source; 2],
    direction: Vec3,
    extent: f64,
}

/// Box 10 × 8 × 4. Source 0 crosses the top/right edge along d = (−1, 0, −1):
/// top face for s ∈ [0, 3/7], right face x = 10 for s ∈ [3/7, 1], joined at
/// (10, 4, 4). Source 1 misses the box.
fn solid_box_fixture(pl: &Placement) -> SolidFixture {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, pl.len(10.0), pl.len(8.0), pl.len(4.0)).expect("box");
    pl.place(&mut topo, solid);
    let extent = pl.len(10.0);
    let top = face_containing(&topo, solid, pl.p(5.0, 4.0, 4.0), extent);
    let right = face_containing(&topo, solid, pl.p(10.0, 4.0, 2.0), extent);
    let sources = [
        Source::Segment {
            a: pl.p(11.0, 4.0, 8.0),
            b: pl.p(18.0, 4.0, 8.0),
        },
        Source::Segment {
            a: pl.p(20.0, 20.0, 20.0),
            b: pl.p(21.0, 20.0, 20.0),
        },
    ];
    let pieces = vec![
        piece(
            top,
            Carrier::Line,
            pl.p(7.0, 4.0, 4.0),
            pl.p(10.0, 4.0, 4.0),
            (0.0, 3.0 / 7.0),
        ),
        Piece {
            joins_previous: true,
            ..piece(
                right,
                Carrier::Line,
                pl.p(10.0, 4.0, 4.0),
                pl.p(10.0, 4.0, 0.0),
                (3.0 / 7.0, 1.0),
            )
        },
    ];
    let oracle = solid_faces(&topo, solid)
        .expect("faces")
        .into_iter()
        .fold(Oracle::new(pl), |oracle, face| oracle.with(face, BOX));
    SolidFixture {
        topo,
        solid,
        oracle,
        pieces,
        sources,
        direction: pl.v(-1.0, 0.0, -1.0),
        extent,
    }
}

#[test]
fn solid_level_first_hit_across_faces_in_source_order() {
    for pl in placements() {
        let SolidFixture {
            mut topo,
            solid,
            oracle,
            pieces,
            sources,
            direction: d,
            extent,
        } = solid_box_fixture(&pl);
        let edges: Vec<_> = sources.iter().map(|s| add_source(&mut topo, s)).collect();
        let before = Census::of(&topo);
        let result =
            project_curves_onto_solid(&mut topo, &edges, d, solid, &ProjectCurveOptions::default())
                .expect("solid projection");
        assert_eq!(result.quality, ProjectionQuality::Exact);
        assert_eq!(result.sources.len(), 2);
        assert_eq!(result.sources[0].source, edges[0]);
        assert_eq!(result.sources[1].source, edges[1]);
        assert!(!result.sources[0].clipped);
        assert!(result.sources[1].edges.is_empty() && result.sources[1].clipped);
        let got: Vec<_> = result.sources[0]
            .edges
            .iter()
            .map(|e| Got {
                edge: e.edge,
                face: e.face,
                range: e.source_range,
            })
            .collect();
        let faces = solid_faces(&topo, solid).expect("faces");
        check_pieces(
            "solid box",
            &oracle,
            &topo,
            &sources[0],
            d,
            &faces,
            &pieces,
            &got,
            extent,
        );
        let after = Census::of(&topo);
        assert_eq!(
            (after.edges, after.vertices),
            (before.edges + 2, before.vertices + 3)
        );
        assert_eq!(after.journal, before.journal);
    }
}

#[test]
fn solid_level_is_atomic_and_names_the_refused_source() {
    let SolidFixture {
        mut topo,
        solid,
        sources,
        direction: d,
        ..
    } = solid_box_fixture(&UNIT);
    let good = add_source(&mut topo, &sources[0]);
    let degenerate = zero_length_segment(&mut topo, UNIT.p(12.0, 4.0, 8.0));
    let before = Census::of(&topo);
    let error = assert_refused(
        project_curves_onto_solid(
            &mut topo,
            &[good, degenerate],
            d,
            solid,
            &ProjectCurveOptions::default(),
        ),
        &topo,
        &before,
        "source-refused",
    );
    match error {
        ProjectCurveError::SourceRefused { index, error } => {
            assert_eq!(index, 1);
            assert_eq!(error.code(), "degenerate-source");
        }
        other => panic!("expected SourceRefused, got {other:?}"),
    }
}

#[test]
fn solid_level_all_sources_missing_is_empty() {
    let SolidFixture {
        mut topo,
        solid,
        sources,
        direction: d,
        ..
    } = solid_box_fixture(&UNIT);
    let miss = add_source(&mut topo, &sources[1]);
    let before = Census::of(&topo);
    assert_refused(
        project_curves_onto_solid(
            &mut topo,
            &[miss],
            d,
            solid,
            &ProjectCurveOptions::default(),
        ),
        &topo,
        &before,
        "empty-projection",
    );
}

// ---------------------------------------------------------------------------
// Plane-frame 2D output.
// ---------------------------------------------------------------------------

fn top_frame(pl: &Placement) -> Frame3 {
    Frame3 {
        origin: pl.p(1.0, 1.0, 4.0),
        x: pl.v(1.0, 0.0, 0.0),
        y: pl.v(0.0, 1.0, 0.0),
        z: pl.v(0.0, 0.0, 1.0),
    }
}

fn lift(frame: &Frame3, pcurve: &PCurve, t: f64) -> Point3 {
    let uv = pcurve.evaluate(t);
    frame.origin + frame.x * uv.x() + frame.y * uv.y()
}

/// Distance from `q` to an analytic result edge, restricted to its domain.
fn distance_to_edge(topo: &Topology, edge: EdgeId, q: Point3) -> f64 {
    let e = topo.edge(edge).expect("edge");
    let start = topo.vertex(e.start()).expect("start").point();
    let end = topo.vertex(e.end()).expect("end").point();
    let (t0, t1) = e.strict_domain().expect("domain");
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    let t = match e.curve() {
        EdgeCurve::Line => {
            let d = end - start;
            ((q - start).dot(d) / d.dot(d)).clamp(0.0, 1.0)
        }
        EdgeCurve::Circle(c) => unwrap_from(c.project(q), lo),
        EdgeCurve::Ellipse(el) => unwrap_from(el.project(q), lo),
        other => panic!(
            "2D checks cover line, circle and ellipse edges, got {}",
            other.type_tag()
        ),
    };
    let t = t.clamp(lo, hi);
    (e.curve().evaluate_with_endpoints(t, start, end) - q).length()
}

fn check_plane_curves(cell: &Cell, result: &ProjectedCurves, frame: &Frame3) {
    let tol = MATCH_REL * cell.extent;
    for projected in &result.edges {
        let pcurve = projected
            .plane_curve
            .as_ref()
            .expect("plane curve on a planar target");
        let (start, end) = topo_edge_points(&cell.topo, projected.edge);
        assert!(
            near(lift(frame, pcurve, pcurve.t_start()), start, tol),
            "2D start"
        );
        assert!(
            near(lift(frame, pcurve, pcurve.t_end()), end, tol),
            "2D end"
        );
        for k in 0..=SAMPLES {
            let t = pcurve.t_start()
                + (pcurve.t_end() - pcurve.t_start()) * (k as f64 / SAMPLES as f64);
            let q = lift(frame, pcurve, t);
            assert!(
                distance_to_edge(&cell.topo, projected.edge, q) <= tol,
                "2D sample {k} off the edge"
            );
        }
    }
}

#[test]
fn plane_frame_gives_2d_curves_on_planar_targets() {
    for build in [cell_p1 as CellFn, cell_p3, cell_hole_clip] {
        for pl in placements() {
            let mut cell = build(&pl);
            let frame = top_frame(&pl);
            let options = ProjectCurveOptions {
                plane_frame: Some(frame),
                ..ProjectCurveOptions::default()
            };
            let result = project_curve_onto_face(
                &mut cell.topo,
                cell.source_edge,
                cell.direction,
                cell.face,
                &options,
            )
            .expect("projection with frame");
            check_cell(&cell, &got_from(&result));
            check_plane_curves(&cell, &result, &frame);
        }
    }
}

#[test]
fn plane_frame_2d_types_match_the_3d_carrier() {
    let mut cell = cell_p3(&UNIT);
    let frame = top_frame(&UNIT);
    let options = ProjectCurveOptions {
        plane_frame: Some(frame),
        ..ProjectCurveOptions::default()
    };
    let result = project_curve_onto_face(
        &mut cell.topo,
        cell.source_edge,
        cell.direction,
        cell.face,
        &options,
    )
    .expect("P3 with frame");
    let pcurve = result.edges[0].plane_curve.as_ref().expect("2D");
    match pcurve.curve() {
        Curve2D::Ellipse(e) => {
            assert!((e.center().x() - 4.0).abs() <= 1e-9 && (e.center().y() - 3.0).abs() <= 1e-9);
            assert!((e.semi_major() - 2.0).abs() <= 1e-9 && (e.semi_minor() - 1.0).abs() <= 1e-9);
        }
        other => panic!("expected a 2D ellipse, got {other:?}"),
    }
}

#[test]
fn plane_frame_mismatch_is_refused() {
    let tilted = Frame3 {
        origin: UNIT.p(0.0, 0.0, 4.0),
        x: UNIT.v(1.0, 0.0, 0.0),
        y: UNIT.unit(0.0, 1.0, 1e-3),
        z: UNIT.unit(0.0, -1e-3, 1.0),
    };
    let off_plane = Frame3 {
        origin: UNIT.p(0.0, 0.0, 4.5),
        ..top_frame(&UNIT)
    };
    for frame in [tilted, off_plane] {
        let mut cell = cell_p1(&UNIT);
        let before = Census::of(&cell.topo);
        let options = ProjectCurveOptions {
            plane_frame: Some(frame),
            ..ProjectCurveOptions::default()
        };
        assert_refused(
            project_curve_onto_face(
                &mut cell.topo,
                cell.source_edge,
                cell.direction,
                cell.face,
                &options,
            ),
            &cell.topo,
            &before,
            "plane-frame-mismatch",
        );
    }
    let mut cell = cell_c1(&UNIT);
    let before = Census::of(&cell.topo);
    let options = ProjectCurveOptions {
        plane_frame: Some(top_frame(&UNIT)),
        ..ProjectCurveOptions::default()
    };
    assert_refused(
        project_curve_onto_face(
            &mut cell.topo,
            cell.source_edge,
            cell.direction,
            cell.face,
            &options,
        ),
        &cell.topo,
        &before,
        "plane-frame-mismatch",
    );
}

// ---------------------------------------------------------------------------
// Read-only sketch-plane projection (OpenZCAD S-6).
// ---------------------------------------------------------------------------

fn sketch_frame(pl: &Placement) -> Frame3 {
    let alpha = PI / 3.0;
    let z = pl.v(0.0, -alpha.sin(), alpha.cos());
    let x = pl.v(1.0, 0.0, 0.0);
    Frame3 {
        origin: pl.p(0.0, 0.0, 0.0),
        x,
        y: z.cross(x),
        z,
    }
}

fn find_edge(
    topo: &Topology,
    solid: SolidId,
    pick: impl Fn(&EdgeCurve, Point3, Point3) -> bool,
) -> EdgeId {
    let mut hits = Vec::new();
    for face in solid_faces(topo, solid).expect("faces") {
        for oe in topo.face_oriented_edges(face).expect("edges") {
            let (start, end) = topo_edge_points(topo, oe.edge());
            if pick(topo.edge(oe.edge()).expect("edge").curve(), start, end)
                && !hits.contains(&oe.edge())
            {
                hits.push(oe.edge());
            }
        }
    }
    assert_eq!(hits.len(), 1, "edge selector must be unique");
    hits[0]
}

#[test]
fn sketch_plane_rim_projects_to_an_ellipse_and_box_edge_to_a_line() {
    for pl in placements() {
        let mut topo = Topology::new();
        let cylinder = make_cylinder(&mut topo, pl.len(2.0), pl.len(10.0)).expect("cylinder");
        pl.place(&mut topo, cylinder);
        let rim_center = pl.p(0.0, 0.0, 10.0);
        let rim = find_edge(
            &topo,
            cylinder,
            |curve, _, _| matches!(curve, EdgeCurve::Circle(c) if near(c.center(), rim_center, 1e-9 * pl.len(10.0))),
        );
        let frame = sketch_frame(&pl);
        let before = Census::of(&topo);
        let curves = project_curves_onto_plane(&topo, &[rim], -frame.z, &frame).expect("rim");
        assert_eq!(Census::of(&topo), before);
        assert_eq!(curves.len(), 1);
        let tol = MATCH_REL * pl.len(10.0);
        match curves[0].curve() {
            Curve2D::Ellipse(e) => {
                assert!(e.center().x().abs() <= tol);
                assert!((e.center().y() - pl.len(10.0 * (PI / 3.0).sin())).abs() <= tol);
                assert!((e.semi_major() - pl.len(2.0)).abs() <= tol);
                assert!((e.semi_minor() - pl.len(1.0)).abs() <= tol);
            }
            other => panic!("rim must project to a 2D ellipse, got {other:?}"),
        }
        assert!(((curves[0].t_end() - curves[0].t_start()).abs() - TAU).abs() <= PARAM_ABS);

        let mut topo = Topology::new();
        let block = make_box(&mut topo, pl.len(10.0), pl.len(8.0), pl.len(4.0)).expect("box");
        pl.place(&mut topo, block);
        let (a, b) = (pl.p(0.0, 0.0, 4.0), pl.p(10.0, 0.0, 4.0));
        let line = find_edge(&topo, block, |curve, s, e| {
            matches!(curve, EdgeCurve::Line)
                && ((near(s, a, tol) && near(e, b, tol)) || (near(s, b, tol) && near(e, a, tol)))
        });
        let frame = Frame3 {
            origin: pl.p(1.0, 1.0, 0.0),
            ..top_frame(&pl)
        };
        let curves = project_curves_onto_plane(&topo, &[line], -frame.z, &frame).expect("box edge");
        let (start, end) = topo_edge_points(&topo, line);
        let expect_2d = |p: Point3| {
            let w = p - frame.origin;
            (w.dot(frame.x), w.dot(frame.y))
        };
        let (s2, e2) = (
            curves[0].evaluate(curves[0].t_start()),
            curves[0].evaluate(curves[0].t_end()),
        );
        assert!(matches!(curves[0].curve(), Curve2D::Line(_)));
        let (sx, sy) = expect_2d(start);
        let (ex, ey) = expect_2d(end);
        assert!((s2.x() - sx).abs() <= tol && (s2.y() - sy).abs() <= tol);
        assert!((e2.x() - ex).abs() <= tol && (e2.y() - ey).abs() <= tol);
        assert!((sy - pl.len(-1.0)).abs() <= tol && (ey - pl.len(-1.0)).abs() <= tol);
    }
}

fn plane_image(frame: &Frame3, d: Vec3, p: Point3) -> (f64, f64) {
    let lambda = (frame.origin - p).dot(frame.z) / d.dot(frame.z);
    let w = (p + d * lambda) - frame.origin;
    (w.dot(frame.x), w.dot(frame.y))
}

#[test]
fn sketch_plane_nurbs_image_keeps_the_source_parameterization() {
    let mut topo = Topology::new();
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let curve = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(7.0, 4.0, 3.0),
            Point3::new(7.0, 6.0, 3.5),
            Point3::new(5.0, 6.0, 4.0),
        ],
        vec![1.0, w, 1.0],
    )
    .expect("quarter arc");
    let (start, end) = (curve.evaluate(0.0), curve.evaluate(1.0));
    let edge = make_nurbs_edge(&mut topo, start, end, curve.clone(), VERTEX_TOL);
    let frame = top_frame(&UNIT);
    let d = Vec3::new(0.3, -0.2, -1.0);
    let curves = project_curves_onto_plane(&topo, &[edge], d, &frame).expect("nurbs source");
    assert!(matches!(curves[0].curve(), Curve2D::Nurbs(_)));
    for k in 0..=SAMPLES {
        let t = k as f64 / SAMPLES as f64;
        let (x, y) = plane_image(&frame, d, curve.evaluate(t));
        let got = curves[0].evaluate(t);
        assert!(
            (got.x() - x).abs() <= 1e-9 && (got.y() - y).abs() <= 1e-9,
            "sample {k}"
        );
    }
}

#[test]
fn sketch_plane_hyperbola_image_is_an_exact_rational_quadratic() {
    let mut topo = Topology::new();
    let h = Hyperbola3D::with_axes(
        Point3::new(0.0, 0.0, 5.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        1.0,
        0.5,
    )
    .expect("hyperbola");
    let edge = add_periodic_edge(
        &mut topo,
        EdgeCurve::Hyperbola(h.clone()),
        h.evaluate(-1.0),
        h.evaluate(1.2),
        (-1.0, 1.2),
        false,
    );
    let frame = top_frame(&UNIT);
    let d = Vec3::new(0.0, 0.6, -0.8);
    let curves = project_curves_onto_plane(&topo, &[edge], d, &frame).expect("hyperbola source");
    let pcurve = &curves[0];
    match pcurve.curve() {
        Curve2D::Nurbs(n) => assert_eq!(n.degree(), 2),
        other => panic!("expected a rational quadratic, got {other:?}"),
    }
    for k in 0..=SAMPLES {
        let t =
            pcurve.t_start() + (pcurve.t_end() - pcurve.t_start()) * (k as f64 / SAMPLES as f64);
        let q = lift(&frame, pcurve, t);
        let n = h.normal();
        let lambda = (q - h.center()).dot(n) / d.dot(n);
        let q0 = q - d * lambda;
        let residual = (q0 - h.evaluate(h.project(q0))).length();
        assert!(
            residual <= 1e-9,
            "sample {k} off the hyperbola by {residual:e}"
        );
    }
    let (sx, sy) = plane_image(&frame, d, h.evaluate(-1.0));
    let first = pcurve.evaluate(pcurve.t_start());
    assert!((first.x() - sx).abs() <= 1e-9 && (first.y() - sy).abs() <= 1e-9);
}

#[test]
fn sketch_plane_refusals() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 2.0, 10.0).expect("cylinder");
    let rim = find_edge(
        &topo,
        cylinder,
        |curve, _, _| matches!(curve, EdgeCurve::Circle(c) if (c.center().z() - 10.0).abs() < 1e-9),
    );
    let segment = add_source(&mut topo, &seg((0.0, 0.0, 12.0), (0.0, 1.0, 12.0)));
    let frame = top_frame(&UNIT);
    let edge_on = Frame3 {
        origin: Point3::new(0.0, 0.0, 0.0),
        x: Vec3::new(0.0, 1.0, 0.0),
        y: Vec3::new(0.0, 0.0, 1.0),
        z: Vec3::new(1.0, 0.0, 0.0),
    };
    // Call-level refusals come back bare.
    for (d, code) in [
        (Vec3::new(0.0, 0.0, 0.0), "invalid-direction"),
        (Vec3::new(f64::NAN, 0.0, -1.0), "invalid-direction"),
        (Vec3::new(1.0, 0.0, 0.0), "grazing-direction"),
    ] {
        let before = Census::of(&topo);
        assert_refused(
            project_curves_onto_plane(&topo, &[segment], d, &frame),
            &topo,
            &before,
            code,
        );
    }
    // Per-source refusals are wrapped with the index of the refused source,
    // even for a single source.
    for (sources, index) in [(&[rim][..], 0), (&[segment, rim][..], 1)] {
        let before = Census::of(&topo);
        match assert_refused(
            project_curves_onto_plane(&topo, sources, Vec3::new(-1.0, 0.0, 0.0), &edge_on),
            &topo,
            &before,
            "source-refused",
        ) {
            ProjectCurveError::SourceRefused { index: got, error } => {
                assert_eq!(got, index);
                assert_eq!(error.code(), "degenerate-image");
            }
            other => panic!("{other:?}"),
        }
    }
}

// ---------------------------------------------------------------------------
// The approximate cell (arc onto a non-coaxial curved quadric).
// ---------------------------------------------------------------------------

/// Unit circle in the plane y = 5 centred (0, 5, 5), projected along −y onto
/// the r = 2 cylinder: x = cos t, y = √(4 − cos² t), z = 5 + sin t.
fn approximate_cell(pl: &Placement, center_z: f64) -> Cell {
    cell(
        "A1 arc → cylinder (approximate)",
        cylinder_target(pl, 0.0),
        arc(
            pl.p(0.0, 5.0, center_z),
            pl.v(0.0, 1.0, 0.0),
            pl.len(1.0),
            pl.v(1.0, 0.0, 0.0),
            (0.0, TAU),
        ),
        pl.v(0.0, -1.0, 0.0),
        |_| Vec::new(),
        false,
        pl.len(10.0),
    )
}

fn cylinder_distance(topo: &Topology, face: FaceId, q: Point3) -> f64 {
    match topo.face(face).expect("face").surface() {
        FaceSurface::Cylinder(c) => {
            let w = q - c.origin();
            ((w - c.axis() * w.dot(c.axis())).length() - c.radius()).abs()
        }
        _ => panic!("cylinder target"),
    }
}

#[test]
fn approximate_arc_on_cylinder_discloses_a_dominating_deviation() {
    for pl in placements() {
        let mut cell = approximate_cell(&pl, 5.0);
        let tolerance = pl.len(1e-6);
        let options = ProjectCurveOptions {
            allow_approximate: true,
            approximation_tolerance: Some(tolerance),
            ..ProjectCurveOptions::default()
        };
        let before = Census::of(&cell.topo);
        let result = project_curve_onto_face(
            &mut cell.topo,
            cell.source_edge,
            cell.direction,
            cell.face,
            &options,
        )
        .expect("approximate projection");
        let ProjectionQuality::Approximate { max_deviation } = result.quality else {
            panic!("quality must be Approximate");
        };
        assert!(
            max_deviation > 0.0 && max_deviation <= tolerance,
            "max_deviation {max_deviation:e}"
        );
        assert!(!result.clipped);
        assert_eq!(result.edges.len(), 1);
        let projected = &result.edges[0];
        let edge = cell.topo.edge(projected.edge).expect("edge");
        assert!(
            edge.start() == edge.end(),
            "closed source gives a closed edge"
        );
        assert!(matches!(edge.curve(), EdgeCurve::NurbsCurve(_)));
        let (start, end) = topo_edge_points(&cell.topo, projected.edge);
        let (t0, t1) = edge.strict_domain().expect("domain");
        let d = cell.direction.normalize().expect("d");
        let mut measured: f64 = 0.0;
        for k in 0..=4096 {
            let t = t0 + (t1 - t0) * (f64::from(k) / 4096.0);
            let q = edge.curve().evaluate_with_endpoints(t, start, end);
            let Source::Arc { circle, .. } = &cell.source else {
                unreachable!()
            };
            let lambda = (q - circle.center()).dot(circle.normal()) / d.dot(circle.normal());
            let q0 = q - d * lambda;
            let source_gap = ((q0 - circle.center()).length() - circle.radius()).abs();
            let deviation = cylinder_distance(&cell.topo, cell.face, q).max(source_gap);
            measured = measured.max(deviation);
            assert!(
                cell.oracle.on_face_distance(&cell.topo, q, cell.face) <= max_deviation,
                "on-face at {k}"
            );
        }
        assert!(
            measured <= max_deviation,
            "oracle deviation {measured:e} > disclosed {max_deviation:e}"
        );
        before.assert_only_free_edges_added(&cell.topo, &result);
    }
}

fn approximate_refusal(
    options: &ProjectCurveOptions,
    center_z: f64,
    code: &str,
) -> ProjectCurveError {
    let mut cell = approximate_cell(&UNIT, center_z);
    let before = Census::of(&cell.topo);
    assert_refused(
        project_curve_onto_face(
            &mut cell.topo,
            cell.source_edge,
            cell.direction,
            cell.face,
            options,
        ),
        &cell.topo,
        &before,
        code,
    )
}

#[test]
fn approximate_cell_is_refused_by_default() {
    approximate_refusal(
        &ProjectCurveOptions::default(),
        5.0,
        "approximation-required",
    );
}

#[test]
fn approximate_cell_budget_exhaustion_is_typed() {
    let options = ProjectCurveOptions {
        allow_approximate: true,
        max_control_points: 4,
        ..ProjectCurveOptions::default()
    };
    match approximate_refusal(&options, 5.0, "tolerance-unattainable") {
        ProjectCurveError::ToleranceUnattainable {
            requested,
            achieved,
        } => {
            assert!(
                (requested - 2.0e-6).abs() <= 1e-12,
                "default is 1e-6 · scale (scale = 2)"
            );
            assert!(achieved > requested);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn approximate_image_crossing_the_face_boundary_is_refused() {
    let options = ProjectCurveOptions {
        allow_approximate: true,
        ..ProjectCurveOptions::default()
    };
    approximate_refusal(&options, 9.5, "approximate-clip-unsupported");
}

#[test]
fn invalid_options_are_refused() {
    let bad = [
        ProjectCurveOptions {
            allow_approximate: true,
            approximation_tolerance: Some(0.0),
            ..ProjectCurveOptions::default()
        },
        ProjectCurveOptions {
            allow_approximate: true,
            approximation_tolerance: Some(f64::NAN),
            ..ProjectCurveOptions::default()
        },
        ProjectCurveOptions {
            allow_approximate: true,
            approximation_tolerance: Some(1e-12),
            ..ProjectCurveOptions::default()
        },
        ProjectCurveOptions {
            allow_approximate: true,
            max_control_points: 3,
            ..ProjectCurveOptions::default()
        },
    ];
    for options in &bad {
        approximate_refusal(options, 5.0, "invalid-options");
    }
}

// ---------------------------------------------------------------------------
// Refusals (design note §7). Each leaves the topology unchanged.
// ---------------------------------------------------------------------------

fn zero_length_segment(topo: &mut Topology, at: Point3) -> EdgeId {
    let v0 = topo.add_vertex(Vertex::new(at, VERTEX_TOL));
    let v1 = topo.add_vertex(Vertex::new(at, VERTEX_TOL));
    topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line))
}

fn refuse_on(target: Target, source: &Source, direction: Vec3, code: &str) -> ProjectCurveError {
    let Target { mut topo, face, .. } = target;
    let edge = add_source(&mut topo, source);
    let before = Census::of(&topo);
    assert_refused(
        project_curve_onto_face(
            &mut topo,
            edge,
            direction,
            face,
            &ProjectCurveOptions::default(),
        ),
        &topo,
        &before,
        code,
    )
}

fn seg(a: (f64, f64, f64), b: (f64, f64, f64)) -> Source {
    Source::Segment {
        a: Point3::new(a.0, a.1, a.2),
        b: Point3::new(b.0, b.1, b.2),
    }
}

#[test]
fn invalid_direction_is_refused() {
    for d in [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(f64::NAN, 0.0, -1.0),
        Vec3::new(0.0, f64::INFINITY, -1.0),
    ] {
        refuse_on(
            box_target(&UNIT),
            &seg((1.0, 2.0, 7.0), (8.0, 5.0, 9.0)),
            d,
            "invalid-direction",
        );
    }
}

#[test]
fn zero_length_source_is_refused() {
    let Target { mut topo, face, .. } = box_target(&UNIT);
    let edge = zero_length_segment(&mut topo, Point3::new(5.0, 4.0, 7.0));
    let before = Census::of(&topo);
    assert_refused(
        project_curve_onto_face(
            &mut topo,
            edge,
            Vec3::new(0.0, 0.0, -1.0),
            face,
            &ProjectCurveOptions::default(),
        ),
        &topo,
        &before,
        "degenerate-source",
    );
}

#[test]
fn unsupported_source_curves_are_refused() {
    let Target { mut topo, face, .. } = box_target(&UNIT);
    let curve = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(2.0, 2.0, 7.0),
            Point3::new(5.0, 6.0, 7.0),
            Point3::new(8.0, 2.0, 7.0),
        ],
        vec![1.0, 1.0, 1.0],
    )
    .expect("nurbs");
    let edge = make_nurbs_edge(
        &mut topo,
        curve.evaluate(0.0),
        curve.evaluate(1.0),
        curve,
        VERTEX_TOL,
    );
    let before = Census::of(&topo);
    match assert_refused(
        project_curve_onto_face(
            &mut topo,
            edge,
            Vec3::new(0.0, 0.0, -1.0),
            face,
            &ProjectCurveOptions::default(),
        ),
        &topo,
        &before,
        "unsupported-source-curve",
    ) {
        ProjectCurveError::UnsupportedSourceCurve { curve, surface } => {
            assert_eq!((curve, surface), ("nurbs_curve", "plane"));
        }
        other => panic!("{other:?}"),
    }

    let ellipse = Source::Ellipse {
        ellipse: Ellipse3D::new(
            Point3::new(0.0, 5.0, 5.0),
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
            0.5,
        )
        .expect("ellipse"),
        range: (0.0, TAU),
    };
    match refuse_on(
        cylinder_target(&UNIT, 0.0),
        &ellipse,
        Vec3::new(0.0, -1.0, 0.0),
        "unsupported-source-curve",
    ) {
        ProjectCurveError::UnsupportedSourceCurve { curve, surface } => {
            assert_eq!((curve, surface), ("ellipse", "cylinder"));
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn torus_and_nurbs_targets_are_refused() {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 5.0, 1.0, 16).expect("torus");
    let face = solid_faces(&topo, torus).expect("faces")[0];
    let edge = add_source(&mut topo, &seg((-1.0, 5.0, 4.0), (1.0, 5.0, 4.0)));
    let before = Census::of(&topo);
    match assert_refused(
        project_curve_onto_face(
            &mut topo,
            edge,
            Vec3::new(0.0, 0.0, -1.0),
            face,
            &ProjectCurveOptions::default(),
        ),
        &topo,
        &before,
        "unsupported-target-surface",
    ) {
        ProjectCurveError::UnsupportedTargetSurface { surface } => assert_eq!(surface, "torus"),
        other => panic!("{other:?}"),
    }

    let mut topo = Topology::new();
    let surface = remus_math::nurbs::surface::NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 10.0, 0.0)],
            vec![Point3::new(10.0, 0.0, 0.0), Point3::new(10.0, 10.0, 1.0)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .expect("bilinear patch");
    let face = remus_topology::builder::make_nurbs_face(&mut topo, surface, VERTEX_TOL)
        .expect("nurbs face");
    let edge = add_source(&mut topo, &seg((2.0, 5.0, 6.0), (8.0, 5.0, 6.0)));
    let before = Census::of(&topo);
    match assert_refused(
        project_curve_onto_face(
            &mut topo,
            edge,
            Vec3::new(0.0, 0.0, -1.0),
            face,
            &ProjectCurveOptions::default(),
        ),
        &topo,
        &before,
        "unsupported-target-surface",
    ) {
        ProjectCurveError::UnsupportedTargetSurface { surface } => assert_eq!(surface, "nurbs"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn segment_parallel_to_direction_is_refused() {
    refuse_on(
        box_target(&UNIT),
        &seg((5.0, 4.0, 6.0), (5.0, 4.0, 9.0)),
        Vec3::new(0.0, 0.0, -1.0),
        "source-parallel-to-direction",
    );
}

#[test]
fn edge_on_arc_is_a_degenerate_image() {
    refuse_on(
        box_target(&UNIT),
        &arc(
            Point3::new(5.0, 4.0, 7.0),
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
            Vec3::new(0.0, 1.0, 0.0),
            (0.0, PI),
        ),
        Vec3::new(0.0, 0.0, -1.0),
        "degenerate-image",
    );
}

#[test]
fn grazing_directions_are_refused() {
    refuse_on(
        box_target(&UNIT),
        &seg((1.0, 2.0, 7.0), (8.0, 5.0, 7.0)),
        Vec3::new(1.0, 1.0, 0.0),
        "grazing-direction",
    );
    refuse_on(
        cylinder_target(&UNIT, 0.0),
        &seg((1.0, 1.0, 12.0), (1.5, 1.0, 12.0)),
        Vec3::new(0.0, 0.0, -1.0),
        "grazing-direction",
    );
}

#[test]
fn tangent_sweep_planes_are_refused() {
    refuse_on(
        cylinder_target(&UNIT, 0.0),
        &seg((2.0, 5.0, 2.0), (2.0, 5.0, 8.0)),
        Vec3::new(0.0, -1.0, 0.0),
        "tangent-section",
    );
    refuse_on(
        sphere_target(&UNIT),
        &seg((-1.0, 3.0, 6.0), (1.0, 3.0, 6.0)),
        Vec3::new(0.0, 0.0, -1.0),
        "tangent-section",
    );
}

#[test]
fn sweep_plane_through_the_cone_apex_is_refused() {
    refuse_on(
        cone_target(&UNIT),
        &seg((-1.0, 0.0, 5.0), (1.0, 0.0, 5.0)),
        Vec3::new(0.0, 0.0, -1.0),
        "section-through-apex",
    );
}

#[test]
fn near_parabolic_cone_section_is_refused() {
    let tilt = FRAC_PI_4 + 1e-8;
    match refuse_on(
        cone_target(&UNIT),
        &seg((-1.2, -2.0, 4.5), (0.3, -2.0, 4.5)),
        Vec3::new(0.0, tilt.sin(), -tilt.cos()),
        "near-parabolic-section",
    ) {
        ProjectCurveError::NearParabolicSection { a_coefficient } => {
            assert!(a_coefficient.abs() > 1e-12 && a_coefficient.abs() < 1e-6);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn image_along_a_face_boundary_is_refused() {
    refuse_on(
        box_target(&UNIT),
        &seg((2.0, 0.0, 7.0), (8.0, 0.0, 7.0)),
        Vec3::new(0.0, 0.0, -1.0),
        "image-along-boundary",
    );
}

#[test]
fn empty_projections_are_refused() {
    refuse_on(
        box_target(&UNIT),
        &seg((1.0, 2.0, -3.0), (8.0, 5.0, -3.0)),
        Vec3::new(0.0, 0.0, -1.0),
        "empty-projection",
    );
    refuse_on(
        cylinder_target(&UNIT, 0.0),
        &seg((3.0, 5.0, 2.0), (3.0, 5.0, 8.0)),
        Vec3::new(0.0, -1.0, 0.0),
        "empty-projection",
    );
}

#[test]
fn stale_handles_surface_as_topology_errors() {
    let Target { face, .. } = box_target(&UNIT);
    let mut topo = Topology::new();
    let edge = add_source(&mut topo, &seg((1.0, 2.0, 7.0), (8.0, 5.0, 9.0)));
    let before = Census::of(&topo);
    let error = assert_refused(
        project_curve_onto_face(
            &mut topo,
            edge,
            Vec3::new(0.0, 0.0, -1.0),
            face,
            &ProjectCurveOptions::default(),
        ),
        &topo,
        &before,
        "operations",
    );
    assert!(
        matches!(
            error,
            ProjectCurveError::Operations(OperationsError::Topology(_))
        ),
        "{error:?}"
    );
}

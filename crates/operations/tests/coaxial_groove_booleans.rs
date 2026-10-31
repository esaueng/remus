//! Exact booleans and measurements behind cutting gear teeth into an existing
//! cylinder, reproduced with plain primitives. Every volume and area is
//! checked against a closed form; every boolean runs exact-only, so a GFA
//! refusal fails the test rather than falling back to a mesh.
//!
//! 1. A turned groove: a coaxial tube whose caps lie strictly inside the
//!    cylinder's height and whose outer wall stands outside it.
//! 2. A ring plate whose bore is a non-circular outline (an internal gear's
//!    tooth profile), cut by a coaxial tube in its web.
//! 3. Coincident coaxial walls whose seams sit at different angles.
//! 4. Planar faces bounded by polynomial NURBS edges: their area, and the
//!    volume of a boolean result whose ruled NURBS walls start with zero
//!    speed (an involute flank leaving its base circle).
//! 5. Tooth caps coplanar with a cylinder's caps: a prism standing on a disc,
//!    and star-shaped tooth rings fused into grooves, their tips turned down
//!    below the collars or flush with them.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, missing_docs)]

use std::f64::consts::{PI, TAU};

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::measure::{face_area, solid_volume};
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::builder::make_circle_edge_with_ref;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// Unit vector at `degrees` about +z in the xy plane.
fn direction(degrees: f64) -> Vec3 {
    let (sin, cos) = degrees.to_radians().sin_cos();
    Vec3::new(cos, sin, 0.0)
}

fn polar(radius: f64, angle: f64, z: f64) -> Point3 {
    Point3::new(radius * angle.cos(), radius * angle.sin(), z)
}

/// A full circle edge about the z axis at height `z`, its seam vertex along
/// `seam`.
fn circle(topo: &mut Topology, radius: f64, z: f64, seam: Vec3) -> EdgeId {
    make_circle_edge_with_ref(topo, Point3::new(0.0, 0.0, z), Z, radius, seam, 1e-7).unwrap()
}

/// Extrude the planar region at height `z` bounded by `outer` (forward,
/// counter-clockwise) less `holes` (each traversed clockwise) by `height`.
fn prism(
    topo: &mut Topology,
    outer: Vec<OrientedEdge>,
    holes: Vec<Vec<OrientedEdge>>,
    z: f64,
    height: f64,
) -> SolidId {
    let outer = topo.add_wire(Wire::new(outer, true).unwrap());
    let holes = holes
        .into_iter()
        .map(|hole| topo.add_wire(Wire::new(hole, true).unwrap()))
        .collect();
    let face = topo.add_face(Face::new(
        outer,
        holes,
        FaceSurface::Plane { normal: Z, d: z },
    ));
    extrude(topo, face, Z, height).unwrap()
}

/// The coaxial tube `r_in..r_out` over `z0..z0 + height`, both circles'
/// seams along `seam`: an annulus extruded once.
fn tube(
    topo: &mut Topology,
    (r_in, r_out): (f64, f64),
    (z0, height): (f64, f64),
    seam: Vec3,
) -> SolidId {
    let outer = circle(topo, r_out, z0, seam);
    let inner = circle(topo, r_in, z0, seam);
    prism(
        topo,
        vec![OrientedEdge::new(outer, true)],
        vec![vec![OrientedEdge::new(inner, false)]],
        z0,
        height,
    )
}

/// `make_cylinder(12, 10)`: the boss every groove case cuts.
fn boss(topo: &mut Topology) -> SolidId {
    make_cylinder(topo, 12.0, 10.0).unwrap()
}

/// Volume of the annulus `r_in..r_out` over `height`.
fn annulus(r_in: f64, r_out: f64, height: f64) -> f64 {
    PI * r_out.mul_add(r_out, -r_in * r_in) * height
}

fn assert_rel(actual: f64, expected: f64, rel: f64, what: &str) {
    assert!(
        ((actual - expected) / expected).abs() <= rel,
        "{what}: {actual} vs closed form {expected} (rel {:e})",
        (actual - expected) / expected
    );
}

/// Volume at two deflections finer than the clamp and one coarse request, all
/// against the closed form.
fn assert_volume(topo: &Topology, solid: SolidId, expected: f64, rel: f64, what: &str) {
    for deflection in [1e-1, 1e-3, 1e-4] {
        let volume = solid_volume(topo, solid, deflection).unwrap();
        assert_rel(
            volume,
            expected,
            rel,
            &format!("{what} volume at deflection {deflection:e}"),
        );
    }
}

fn assert_valid(topo: &Topology, solid: SolidId, what: &str) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{what}: {:?}", report.issues);
}

fn assert_class(topo: &Topology, solid: SolidId, point: Point3, want: PointClassification) {
    let got = classify_point(topo, solid, point, &ClassifyOptions::default()).unwrap();
    assert_eq!(got, want, "classification at {point:?}");
}

fn exact_boolean(
    topo: &mut Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
    what: &str,
) -> SolidId {
    boolean(topo, op, a, b).unwrap_or_else(|error| panic!("{what}: {error}"))
}

/// Census of the result's surface kinds, so a mesh fallback (all planes)
/// could not pass as exact.
fn assert_analytic(topo: &Topology, solid: SolidId, max_faces: usize, what: &str) {
    let faces = solid_faces(topo, solid).unwrap();
    assert!(faces.len() <= max_faces, "{what}: {} faces", faces.len());
    assert!(
        faces
            .iter()
            .any(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_))),
        "{what}: no cylinder survives"
    );
}

// ---------------------------------------------------------------------------
// 1. Turned groove
// ---------------------------------------------------------------------------

/// A tube r 9.55..12.5 over z 2..8 cuts a groove into the r 12 boss. Its caps
/// lie strictly inside the boss's height and its outer wall outside the boss,
/// so the boss's lateral face is split by the two cap circles alone.
#[test]
fn groove_with_caps_inside_the_cylinder_cuts_and_intersects_exactly() {
    let groove = annulus(9.55, 12.0, 6.0);
    let boss_volume = PI * 144.0 * 10.0;
    for (op, expected) in [
        (BooleanOp::Cut, boss_volume - groove),
        (BooleanOp::Intersect, groove),
    ] {
        let mut topo = Topology::new();
        let cylinder = boss(&mut topo);
        let tool = tube(&mut topo, (9.55, 12.5), (2.0, 6.0), direction(0.0));
        let what = format!("groove {op:?}");
        let result = exact_boolean(&mut topo, op, cylinder, tool, &what);
        assert_valid(&topo, result, &what);
        assert_analytic(&topo, result, 8, &what);
        assert_volume(&topo, result, expected, 1e-12, &what);
        let in_groove = PointClassification::from_cut(op == BooleanOp::Cut);
        for angle in [0.3, 2.0, 4.5] {
            assert_class(&topo, result, polar(11.0, angle, 5.0), in_groove);
            assert_class(&topo, result, polar(11.0, angle, 9.0), in_groove.flip());
            assert_class(&topo, result, polar(5.0, angle, 5.0), in_groove.flip());
        }
    }
}

trait Expectation {
    fn from_cut(cut: bool) -> Self;
    fn flip(self) -> Self;
}

impl Expectation for PointClassification {
    /// Inside the groove: removed by a cut, kept by an intersect.
    fn from_cut(cut: bool) -> Self {
        if cut { Self::Outside } else { Self::Inside }
    }

    fn flip(self) -> Self {
        match self {
            Self::Inside => Self::Outside,
            Self::Outside => Self::Inside,
            Self::OnBoundary => Self::OnBoundary,
        }
    }
}

// ---------------------------------------------------------------------------
// 2. Ring plate with a non-circular bore
// ---------------------------------------------------------------------------

/// Plate r 30, z 0..5, whose bore is a closed outline through `points` (an
/// internal gear's tooth profile stand-in), each span a line or, with
/// `bulge`, a quadratic Bezier whose control point sits at that radius.
/// Returns the solid and the bore's exact area.
fn ring_plate(topo: &mut Topology, points: &[Point3], bulge: Option<f64>) -> (SolidId, f64) {
    let n = points.len();
    let vertices: Vec<_> = points
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
        .collect();
    let mut bore_area = 0.0;
    let mut uses = Vec::new();
    for k in 0..n {
        let (a, b) = (points[k], points[(k + 1) % n]);
        let cross = |p: Point3, q: Point3| p.x() * q.y() - p.y() * q.x();
        bore_area += 0.5 * cross(a, b);
        let curve = match bulge {
            None => EdgeCurve::Line,
            Some(radius) => {
                // Equal-radius ends: their sum points along the bisector.
                let control = polar(radius, (a.y() + b.y()).atan2(a.x() + b.x()), 0.0);
                // A quadratic Bezier adds 2/3 of its control triangle.
                bore_area +=
                    2.0 / 3.0 * 0.5 * (cross(a, control) + cross(control, b) - cross(a, b));
                EdgeCurve::NurbsCurve(
                    NurbsCurve::new(
                        2,
                        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                        vec![a, control, b],
                        vec![1.0; 3],
                    )
                    .unwrap(),
                )
            }
        };
        let mut edge = Edge::new(vertices[k], vertices[(k + 1) % n], curve);
        if bulge.is_some() {
            edge.set_trim(Some((0.0, 1.0)));
        }
        uses.push(OrientedEdge::new(topo.add_edge(edge), false));
    }
    // A hole runs clockwise: the counter-clockwise edges, reversed.
    uses.reverse();
    let rim = circle(topo, 30.0, 0.0, direction(0.0));
    let solid = prism(
        topo,
        vec![OrientedEdge::new(rim, true)],
        vec![uses],
        0.0,
        5.0,
    );
    (solid, bore_area)
}

/// A 40-tooth outline: an 80-gon star alternating r 19.5 and r 21.
fn star_points() -> Vec<Point3> {
    (0..80)
        .map(|k| {
            polar(
                if k % 2 == 0 { 19.5 } else { 21.0 },
                TAU * f64::from(k) / 80.0,
                0.0,
            )
        })
        .collect()
}

/// 40 scallops through r 19.5, bulging to a control point at r 22.
fn scallop_points() -> Vec<Point3> {
    (0..40)
        .map(|k| polar(19.5, TAU * f64::from(k) / 40.0, 0.0))
        .collect()
}

/// The tube r 25..27 spans the plate's full height (caps coplanar) or
/// overshoots it: both cap faces carry two nested section circles around the
/// bore outline.
#[test]
fn tube_in_the_web_of_a_ring_with_a_non_circular_bore() {
    for (points, bulge, what) in [
        (star_points(), None, "polygon bore"),
        (scallop_points(), Some(22.0), "NURBS bore"),
    ] {
        for (z0, height) in [(0.0, 5.0), (-1.0, 7.0)] {
            let mut topo = Topology::new();
            let (ring, bore_area) = ring_plate(&mut topo, &points, bulge);
            let tool = tube(&mut topo, (25.0, 27.0), (z0, height), direction(0.0));
            let what = format!("{what}, tube z {z0}..{}", z0 + height);
            let result = exact_boolean(&mut topo, BooleanOp::Cut, ring, tool, &what);
            assert_valid(&topo, result, &what);
            let expected = (PI * 900.0 - bore_area) * 5.0 - annulus(25.0, 27.0, 5.0);
            assert_volume(&topo, result, expected, 1e-12, &what);
            for angle in [0.1, 3.0] {
                assert_class(
                    &topo,
                    result,
                    polar(26.0, angle, 2.5),
                    PointClassification::Outside,
                );
                assert_class(
                    &topo,
                    result,
                    polar(23.0, angle, 2.5),
                    PointClassification::Inside,
                );
                assert_class(
                    &topo,
                    result,
                    polar(28.5, angle, 2.5),
                    PointClassification::Inside,
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 3. Coincident coaxial walls with mismatched seams
// ---------------------------------------------------------------------------

/// The boss puts its seam at +x. A tube whose wall coincides with the boss's
/// cuts, fuses and intersects exactly wherever its own seam sits.
#[test]
fn coincident_coaxial_walls_do_not_depend_on_seam_alignment() {
    let boss_volume = PI * 144.0 * 10.0;
    let cases = [
        // (op, r_in, r_out, z0, height, closed form, probe radius, z, inside)
        (
            BooleanOp::Cut,
            9.55,
            12.0,
            2.0,
            6.0,
            boss_volume - annulus(9.55, 12.0, 6.0),
            11.0,
            5.0,
            false,
        ),
        (
            BooleanOp::Cut,
            9.55,
            12.0,
            0.0,
            8.0,
            boss_volume - annulus(9.55, 12.0, 8.0),
            11.0,
            4.0,
            false,
        ),
        (
            BooleanOp::Fuse,
            12.0,
            14.0,
            2.0,
            6.0,
            boss_volume + annulus(12.0, 14.0, 6.0),
            13.0,
            5.0,
            true,
        ),
        (
            BooleanOp::Fuse,
            11.0,
            14.0,
            2.0,
            6.0,
            boss_volume + annulus(12.0, 14.0, 6.0),
            13.0,
            5.0,
            true,
        ),
        (
            BooleanOp::Fuse,
            12.0,
            14.0,
            0.0,
            10.0,
            PI * 196.0 * 10.0,
            13.0,
            9.0,
            true,
        ),
        (
            BooleanOp::Intersect,
            9.55,
            12.0,
            2.0,
            6.0,
            annulus(9.55, 12.0, 6.0),
            11.0,
            5.0,
            true,
        ),
    ];
    for seam in [0.0, 37.0, 90.0, 180.0, 271.0] {
        for &(op, r_in, r_out, z0, height, expected, probe_r, probe_z, inside) in &cases {
            let mut topo = Topology::new();
            let cylinder = boss(&mut topo);
            let tool = tube(&mut topo, (r_in, r_out), (z0, height), direction(seam));
            let what = format!(
                "{op:?} tube {r_in}..{r_out} z {z0}..{} seam {seam}°",
                z0 + height
            );
            let result = exact_boolean(&mut topo, op, cylinder, tool, &what);
            assert_valid(&topo, result, &what);
            assert_analytic(&topo, result, 8, &what);
            assert_volume(&topo, result, expected, 1e-12, &what);
            let want = if inside {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            };
            for angle in [0.25, 1.9, 4.0] {
                assert_class(&topo, result, polar(probe_r, angle, probe_z), want);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 4. Planar faces with polynomial NURBS edges
// ---------------------------------------------------------------------------

/// Plate z 0..`height` whose outline is `teeth` cubic Bezier scallops through
/// r 10.5, its inner control points at r 12. With `cusp` the first inner
/// control point repeats the start point, so the edge (and the extruded wall
/// over it) leaves its start with zero speed, as an involute leaves its base
/// circle. An optional circular bore. Returns the solid and its exact cap
/// area.
fn scalloped_plate(
    topo: &mut Topology,
    teeth: u32,
    cusp: bool,
    bore: Option<f64>,
    height: f64,
) -> (SolidId, f64) {
    let step = TAU / f64::from(teeth);
    let vertices: Vec<_> = (0..teeth)
        .map(|k| topo.add_vertex(Vertex::new(polar(10.5, step * f64::from(k), 0.0), 1e-7)))
        .collect();
    let cross = |p: Point3, q: Point3| p.x() * q.y() - p.y() * q.x();
    let mut twice_area = 0.0;
    let mut uses = Vec::new();
    for k in 0..teeth {
        let a0 = step * f64::from(k);
        let p0 = polar(10.5, a0, 0.0);
        let p3 = polar(10.5, a0 + step, 0.0);
        let p1 = if cusp {
            p0
        } else {
            polar(12.0, a0 + step / 3.0, 0.0)
        };
        let p2 = polar(12.0, a0 + 2.0 * step / 3.0, 0.0);
        // ∮ x dy − y dx over a cubic Bezier.
        twice_area += (6.0 * cross(p0, p1)
            + 3.0 * cross(p0, p2)
            + cross(p0, p3)
            + 3.0 * cross(p1, p2)
            + 3.0 * cross(p1, p3)
            + 6.0 * cross(p2, p3))
            / 10.0;
        let curve = NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![p0, p1, p2, p3],
            vec![1.0; 4],
        )
        .unwrap();
        let next = vertices[usize::try_from((k + 1) % teeth).unwrap()];
        let mut edge = Edge::new(
            vertices[usize::try_from(k).unwrap()],
            next,
            EdgeCurve::NurbsCurve(curve),
        );
        edge.set_trim(Some((0.0, 1.0)));
        uses.push(OrientedEdge::new(topo.add_edge(edge), true));
    }
    let mut cap = 0.5 * twice_area;
    let holes = bore.map_or_else(Vec::new, |radius| {
        cap -= PI * radius * radius;
        let edge = circle(topo, radius, 0.0, direction(0.0));
        vec![vec![OrientedEdge::new(edge, false)]]
    });
    (prism(topo, uses, holes, 0.0, height), cap)
}

/// Cubic (non-conic) NURBS edges are polynomial, so the planar moment
/// integrator measures them exactly; the face area must not fall back to a
/// sampled polygon, whose chord error on the bore over-read the cap.
#[test]
fn planar_face_with_polynomial_nurbs_edges_has_exact_area() {
    let mut topo = Topology::new();
    let (plate, cap) = scalloped_plate(&mut topo, 22, false, Some(5.0), 5.0);
    let caps: Vec<_> = solid_faces(&topo, plate)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Plane { .. }
            )
        })
        .collect();
    assert_eq!(caps.len(), 2);
    for face in caps {
        for deflection in [1e-1, 1e-4] {
            assert_rel(
                face_area(&topo, face, deflection).unwrap(),
                cap,
                1e-12,
                &format!("cap area at deflection {deflection:e}"),
            );
        }
    }
    assert_volume(&topo, plate, cap * 5.0, 1e-12, "bored plate");
}

/// A shaft r 5 through the scalloped plate, as a gear hub on its shaft. The
/// walls start with zero speed, which stalled the point projection the
/// volume's boundary check runs (see the projection's own regression tests):
/// the exact face integrals were declined and the reading depended on which
/// mesh fallback the body reached. A gear on its shaft read its closed clamp
/// mesh, 1.6e-4 low at coarse requests.
#[test]
fn zero_speed_nurbs_walls_keep_the_exact_volume_at_every_deflection() {
    let mut topo = Topology::new();
    let (plate, cap) = scalloped_plate(&mut topo, 22, true, None, 10.0);
    let shaft = make_cylinder(&mut topo, 5.0, 30.0).unwrap();
    transform_solid(&mut topo, shaft, &Mat4::translation(0.0, 0.0, -10.0)).unwrap();
    let result = exact_boolean(&mut topo, BooleanOp::Fuse, plate, shaft, "plate on shaft");
    assert_valid(&topo, result, "plate on shaft");
    assert_volume(
        &topo,
        result,
        cap * 10.0 + PI * 25.0 * 20.0,
        1e-12,
        "plate on shaft",
    );
}

// ---------------------------------------------------------------------------
// 5. Coplanar tooth caps
// ---------------------------------------------------------------------------

/// The closed polygon through `points` (counter-clockwise, all at the same
/// height) extruded by `height`, with a hole of radius `bore` if given.
/// Returns the solid and the extruded area.
fn polygon_prism(
    topo: &mut Topology,
    points: &[Point3],
    bore: Option<f64>,
    height: f64,
) -> (SolidId, f64) {
    let n = points.len();
    let z = points[0].z();
    let vertices: Vec<_> = points
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
        .collect();
    let mut area = 0.0;
    let mut outline = Vec::new();
    for k in 0..n {
        let (a, b) = (points[k], points[(k + 1) % n]);
        area += 0.5 * a.x().mul_add(b.y(), -(a.y() * b.x()));
        let edge = Edge::new(vertices[k], vertices[(k + 1) % n], EdgeCurve::Line);
        outline.push(OrientedEdge::new(topo.add_edge(edge), true));
    }
    let holes = bore.map_or_else(Vec::new, |r| {
        area -= PI * r * r;
        vec![vec![OrientedEdge::new(
            circle(topo, r, z, direction(0.0)),
            false,
        )]]
    });
    (prism(topo, outline, holes, z, height), area)
}

/// `n` teeth alternating root r 10 and `tip` at height `z`, the first root
/// 0.01 rad off +x.
fn star(n: u32, tip: f64, z: f64) -> Vec<Point3> {
    (0..2 * n)
        .map(|k| {
            let radius = if k % 2 == 0 { 10.0 } else { tip };
            polar(radius, TAU * f64::from(k) / f64::from(2 * n) + 0.01, z)
        })
        .collect()
}

/// A square prism standing on the bottom cap of a cylinder: the square's top
/// edges lie inside the disc, crossing nothing, and must still split it.
#[test]
fn a_prism_standing_on_a_cylinder_cap_fuses_exactly() {
    let mut topo = Topology::new();
    let square: Vec<Point3> = (0..4)
        .map(|k| polar(11.0, TAU * f64::from(k) / 4.0 + 0.01, 0.0))
        .collect();
    let (block, area) = polygon_prism(&mut topo, &square, None, 6.0);
    let cylinder = make_cylinder(&mut topo, 12.0, 4.0).unwrap();
    transform_solid(&mut topo, cylinder, &Mat4::translation(0.0, 0.0, 6.0)).unwrap();
    let what = "square on disc";
    let result = exact_boolean(&mut topo, BooleanOp::Fuse, block, cylinder, what);
    assert_valid(&topo, result, what);
    assert_analytic(&topo, result, 8, what);
    assert_volume(&topo, result, area * 6.0 + PI * 144.0 * 4.0, 1e-12, what);
}

/// A tooth ring (a star outline around a bore r 9) fused into the groove r
/// 9..12 it exactly fills along the axis, over the middle of the boss (both
/// caps against collars) or from its bottom (one). Tips at r 11 are turned
/// down below the collars, so the outline lies inside their annular caps;
/// tips at r 12 are flush, touching the collars' rims. Seven wide teeth cross
/// the rim cylinder at a shallow angle at each tip; three leave the rim
/// circle outside the ring's caps except where the tips touch it.
#[test]
fn star_rings_fuse_into_grooves_exactly() {
    for teeth in [3_u32, 7, 20] {
        for tip in [11.0, 12.0] {
            for (z0, height) in [(2.0, 6.0), (0.0, 6.0)] {
                let what = format!("{teeth} teeth, tip {tip}, z {z0}..{}", z0 + height);
                let mut topo = Topology::new();
                let cylinder = boss(&mut topo);
                let groove = tube(&mut topo, (9.0, 12.0), (z0, height), direction(0.0));
                let grooved = exact_boolean(&mut topo, BooleanOp::Cut, cylinder, groove, &what);
                let (ring, ring_area) =
                    polygon_prism(&mut topo, &star(teeth, tip, z0), Some(9.0), height);
                let result = exact_boolean(&mut topo, BooleanOp::Fuse, grooved, ring, &what);
                assert_valid(&topo, result, &what);
                let expected = PI * 144.0 * 10.0 - annulus(9.0, 12.0, height) + ring_area * height;
                assert_volume(&topo, result, expected, 1e-12, &what);
                let mid = z0 + 0.5 * height;
                let collar = if z0 > 0.0 { 1.0 } else { 9.0 };
                for k in 0..teeth {
                    let root = TAU * f64::from(2 * k) / f64::from(2 * teeth) + 0.01;
                    let tip_angle = root + PI / f64::from(teeth);
                    assert_class(
                        &topo,
                        result,
                        polar(10.5, root, mid),
                        PointClassification::Outside,
                    );
                    assert_class(
                        &topo,
                        result,
                        polar(10.2, tip_angle, mid),
                        PointClassification::Inside,
                    );
                    assert_class(
                        &topo,
                        result,
                        polar(11.5, root, collar),
                        PointClassification::Inside,
                    );
                }
            }
        }
    }
}

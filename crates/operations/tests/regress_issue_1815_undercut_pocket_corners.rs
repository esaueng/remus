//! Regression (#1815): rounding the four leaning upright corners of an
//! undercut pocket, cut by a box drafted -3 degrees so the pocket narrows
//! toward its top, was refused with a postcondition error.
//!
//! The pocket is 40 x 40 at its floor (z = 12), 8 deep, and narrower at the
//! top (z = 20), so its walls overhang it. Each corner edge leans into the
//! pocket as it rises, and a round's contact lines, which run parallel to it
//! on the two walls, sit below the edge's own end at the top: they reach the
//! top only past the vertex's section. The top corner is mixed: the concave
//! corner edge meets two convex rim edges, and the vertex is a corner of the
//! top face's hole. The run-out past the vertex was qualified only at
//! strictly convex or strictly concave corners on a face's outer wire
//! (#1780), so both top contacts stayed on the vertex section, below the
//! top, and the rebuild was refused.
//!
//! At that corner the air is the walls' wedge joined with the air above the
//! top, and the ball rolls in the wedge, so the band is the rolling-ball
//! cylinder cut by the top plane: the contacts run up to the top and land on
//! the rims. Checked here: the floor stays at z = 12 and the top at z = 20,
//! each blend is a cylinder ending on two exact ellipses, the shell is
//! closed, material is where it belongs, and the volume matches a closed
//! form.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_check::classify::PointClassification::{Inside, Outside};
use remus_check::classify::{ClassifyOptions, classify_point};
use remus_check::validate::{Severity, ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{self, BlendEngine};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::draft::draft;
use remus_operations::measure;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// The issue's draft and radius, a shallower and a steeper draft, and a
/// smaller radius.
const DRAFTS: [f64; 3] = [-3.0, -0.5, -10.0];
const RADII: [f64; 2] = [5.0, 1.0];
const FLOOR: f64 = 12.0;
const TOP: f64 = 20.0;
/// Two volume deflections, both finer than the clamp (bbox diagonal · 5e-5).
const COARSE: f64 = 1e-3;
const FINE: f64 = 2e-4;

fn face_edges(topo: &Topology, f: FaceId) -> Vec<EdgeId> {
    let face = topo.face(f).unwrap();
    std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .flat_map(|w| topo.wire(w).unwrap().edges().to_vec())
        .map(|oe| oe.edge())
        .collect()
}

fn solid_edges(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let mut seen = Vec::new();
    for fid in solid_faces(topo, s).unwrap() {
        for e in face_edges(topo, fid) {
            if !seen.contains(&e) {
                seen.push(e);
            }
        }
    }
    seen
}

fn ends(topo: &Topology, e: EdgeId) -> (Point3, Point3) {
    let edge = topo.edge(e).unwrap();
    (
        topo.vertex(edge.start()).unwrap().point(),
        topo.vertex(edge.end()).unwrap().point(),
    )
}

fn plane_faces(topo: &Topology, s: SolidId, outward: Vec3) -> Vec<FaceId> {
    solid_faces(topo, s)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            topo.face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| (n - outward).length() < 1e-6)
        })
        .collect()
}

/// The issue's body: an 80 x 60 x 20 block cut by a 40 x 40 x 20 box whose
/// four sides are drafted `degrees` (pull +Z, neutral at its base), moved
/// to (20, 10, 12).
fn drafted_pocket(topo: &mut Topology, degrees: f64) -> SolidId {
    let body = make_box(topo, 80.0, 60.0, 20.0).unwrap();
    let tool = make_box(topo, 40.0, 40.0, 20.0).unwrap();
    let sides: Vec<FaceId> = [X, -X, Y, -Y]
        .into_iter()
        .flat_map(|n| plane_faces(topo, tool, n))
        .collect();
    assert_eq!(sides.len(), 4);
    let tool = draft(
        topo,
        tool,
        &sides,
        Z,
        Point3::new(0.0, 0.0, 0.0),
        degrees.to_radians(),
    )
    .unwrap();
    transform_solid(topo, tool, &Mat4::translation(20.0, 10.0, FLOOR)).unwrap();
    boolean(topo, BooleanOp::Cut, body, tool).unwrap()
}

/// The pocket's four leaning upright corners.
fn pocket_corners(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let inside = |p: Point3| p.x() > 1.0 && p.x() < 79.0 && p.y() > 1.0 && p.y() < 59.0;
    let corners: Vec<EdgeId> = solid_edges(topo, s)
        .into_iter()
        .filter(|&e| {
            let (a, b) = ends(topo, e);
            (a.z() - b.z()).abs() > 2.5 && inside(a) && inside(b)
        })
        .collect();
    assert_eq!(corners.len(), 4, "the pocket has four upright corners");
    corners
}

fn census(topo: &Topology, s: SolidId) -> BTreeMap<&'static str, usize> {
    let mut counts = BTreeMap::new();
    for f in solid_faces(topo, s).unwrap() {
        let kind = match topo.face(f).unwrap().surface() {
            FaceSurface::Plane { .. } => "plane",
            FaceSurface::Cylinder(_) => "cylinder",
            FaceSurface::Cone(_) => "cone",
            FaceSurface::Sphere(_) => "sphere",
            FaceSurface::Torus(_) => "torus",
            FaceSurface::Nurbs(_) => "nurbs",
        };
        *counts.entry(kind).or_default() += 1;
    }
    counts
}

fn validation_errors(topo: &Topology, s: SolidId) -> usize {
    validate_solid(topo, s, &ValidateOptions::default())
        .unwrap()
        .issues
        .iter()
        .filter(|issue| issue.severity == Severity::Error)
        .count()
}

/// Closed shell, every edge used once each way, no new validation error.
fn assert_sound(topo: &Topology, input: SolidId, result: SolidId) {
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo).expect("the shell must be closed");
    let mut uses: BTreeMap<usize, Vec<bool>> = BTreeMap::new();
    for f in solid_faces(topo, result).unwrap() {
        let face = topo.face(f).unwrap();
        for w in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(w).unwrap().edges() {
                uses.entry(oe.edge().index())
                    .or_default()
                    .push(oe.is_forward() != face.is_reversed());
            }
        }
    }
    for (edge, uses) in &uses {
        assert!(
            uses.len() == 2 && uses[0] != uses[1],
            "edge {edge} must be used once in each direction, got {uses:?}"
        );
    }
    assert!(
        validation_errors(topo, result) <= validation_errors(topo, input),
        "the blend must add no validation error"
    );
}

/// Signed-tetrahedron volume of the result's mesh, independent of the
/// B-Rep volume paths.
fn mesh_volume(topo: &Topology, s: SolidId) -> f64 {
    let mesh = tessellate_solid_with_tolerance(topo, s, 5e-4, 0.05).unwrap();
    mesh.indices
        .chunks(3)
        .map(|t| {
            let p = |i: u32| {
                let p = mesh.positions[i as usize];
                Vec3::new(p.x(), p.y(), p.z())
            };
            p(t[0]).dot(p(t[1]).cross(p(t[2]))) / 6.0
        })
        .sum()
}

/// Material one radius-`r` round adds along a concave edge whose two faces'
/// normals are `theta` apart: the kite between the edge and the two
/// contacts, less the ball's sector.
fn stripe_area(r: f64, theta: f64) -> f64 {
    r * r * ((theta / 2.0).tan() - theta / 2.0)
}

/// The face lying in the plane z = `height` with outward normal +Z.
fn level_face(topo: &Topology, s: SolidId, height: f64) -> FaceId {
    let faces: Vec<FaceId> = plane_faces(topo, s, Z)
        .into_iter()
        .filter(|&f| {
            face_edges(topo, f).into_iter().all(|e| {
                let (a, b) = ends(topo, e);
                (a.z() - height).abs() < 1e-9 && (b.z() - height).abs() < 1e-9
            })
        })
        .collect();
    assert_eq!(faces.len(), 1, "one face at z = {height}");
    faces[0]
}

fn assert_plane_at(topo: &Topology, f: FaceId, height: f64, what: &str) {
    let FaceSurface::Plane { normal, d } = topo.face(f).unwrap().surface() else {
        panic!("{what}: the face at z = {height} must stay a plane")
    };
    assert!(
        normal.x().abs() < 1e-12 && normal.y().abs() < 1e-12,
        "{what}: the face at z = {height} stays level, normal {normal:?}"
    );
    assert!(
        (d / normal.z() - height).abs() < 1e-9,
        "{what}: the plane stays at z = {height}, got {}",
        d / normal.z()
    );
}

fn assert_rounds_build(degrees: f64, radius: f64) {
    let what = format!("draft {degrees}, r{radius}");
    let mut topo = Topology::new();
    let body = drafted_pocket(&mut topo, degrees);
    let corners = pocket_corners(&topo, body);
    let result = blend_ops::fillet_cascade(&mut topo, body, &corners, radius)
        .unwrap_or_else(|e| panic!("{what}: the corner rounds must build, got {e}"));
    assert_eq!(result.engine, BlendEngine::RollingBall, "{what}");
    assert!(!result.is_partial, "{what}: every corner is rounded");
    let rounded = result.solid;

    assert_sound(&topo, body, rounded);
    assert_eq!(
        census(&topo, rounded),
        [("cylinder", 4), ("plane", 11)].into_iter().collect(),
        "{what}: four exact rounds; every plane kept"
    );

    // Every vertex stays on the base, the floor or the top, and the floor
    // and the top (whose hole the rounds' top ends reshape) stay their
    // planes.
    for e in solid_edges(&topo, rounded) {
        let (a, b) = ends(&topo, e);
        for p in [a, b] {
            assert!(
                [0.0, FLOOR, TOP].iter().any(|z| (p.z() - z).abs() < 1e-9),
                "{what}: vertex {p:?} is off the base, floor and top"
            );
        }
    }
    let floor = level_face(&topo, rounded, FLOOR);
    assert_plane_at(&topo, floor, FLOOR, &what);
    let top = level_face(&topo, rounded, TOP);
    assert_plane_at(&topo, top, TOP, &what);
    assert_eq!(
        topo.face(top).unwrap().inner_wires().len(),
        1,
        "{what}: the top keeps the pocket's mouth as its one hole"
    );

    // Each round ends on the floor and on the top in exact ellipses: the
    // cylinder's sections by those planes, not its section across the edge.
    // The top ellipses are the hole's four rounded corners.
    let mut top_ellipses = 0;
    for f in solid_faces(&topo, rounded).unwrap() {
        if !matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)) {
            continue;
        }
        let mut heights = Vec::new();
        for e in face_edges(&topo, f) {
            if let EdgeCurve::Ellipse(_) = topo.edge(e).unwrap().curve() {
                let (a, b) = ends(&topo, e);
                assert!(
                    (a.z() - b.z()).abs() < 1e-9,
                    "{what}: an end ellipse is level"
                );
                heights.push(a.z());
                if face_edges(&topo, top).contains(&e) {
                    top_ellipses += 1;
                }
            }
        }
        assert_eq!(heights.len(), 2, "{what}: a round has two end ellipses");
        let floor_end = heights
            .iter()
            .filter(|z| (**z - FLOOR).abs() < 1e-9)
            .count();
        let top_end = heights.iter().filter(|z| (**z - TOP).abs() < 1e-9).count();
        assert_eq!((floor_end, top_end), (1, 1), "{what}: {heights:?}");
    }
    assert_eq!(top_ellipses, 4, "{what}: the top face shares the top ends");

    // Material where it belongs: the pocket open right down to its floor and
    // under the overhang, the floor solid below it, the overhanging wall
    // solid just below the top, and the near corner filled along its
    // bisector to within the round, `r(√2 − 1)` deep at a square corner,
    // and open beyond it, both at mid-height and just below the top; above
    // the top, nothing.
    let options = ClassifyOptions::default();
    let class = |p: Point3| classify_point(&topo, rounded, p, &options).unwrap();
    let corner_at = |z: f64| {
        let lean = (z - FLOOR) * degrees.to_radians().tan();
        Point3::new(20.0 - lean, 10.0 - lean, z)
    };
    let bisector = Vec3::new(1.0, 1.0, 0.0) * std::f64::consts::FRAC_1_SQRT_2;
    let overhang = 7.9 * (-degrees).to_radians().tan();
    let mut probes = vec![
        (Point3::new(40.0, 30.0, FLOOR + 0.1), Outside),
        (
            Point3::new(20.0 + 0.5 * overhang, 30.0, FLOOR + 0.1),
            Outside,
        ),
        (Point3::new(20.0 + 0.5 * overhang, 30.0, TOP - 0.1), Inside),
        (Point3::new(40.0, 30.0, FLOOR - 0.1), Inside),
        (Point3::new(40.0, 30.0, TOP - 0.1), Outside),
    ];
    for z in [16.0, TOP - 0.1] {
        probes.push((corner_at(z) + bisector * (0.2 * radius), Inside));
        probes.push((corner_at(z) + bisector * (0.6 * radius), Outside));
    }
    probes.push((
        corner_at(TOP) + bisector * (0.2 * radius) + Z * 0.1,
        Outside,
    ));
    for (p, expected) in probes {
        assert_eq!(class(p), expected, "{what}: {p:?}");
    }

    // Volume: each round adds its section's area swept along the corner and
    // cut by the floor and the top, two parallel planes 8 apart, so it adds
    // `area · 8 / rise`, `rise` the corner's climb per unit length. The
    // concave rounds' chords sit in the air, so the measure reads high and
    // closes in from above as the deflection refines.
    let (sin, cos) = degrees.to_radians().sin_cos();
    let wall_x = Vec3::new(cos, 0.0, sin);
    let wall_y = Vec3::new(0.0, cos, sin);
    let theta = wall_x.dot(wall_y).acos();
    let rise = wall_x.cross(wall_y).normalize().unwrap().z().abs();
    let expected = 4.0 * stripe_area(radius, theta) * (TOP - FLOOR) / rise;
    let change = |deflection: f64| {
        measure::solid_volume(&topo, rounded, deflection).unwrap()
            - measure::solid_volume(&topo, body, deflection).unwrap()
    };
    let (coarse, fine) = (change(COARSE) - expected, change(FINE) - expected);
    assert!(
        (0.0..1e-2).contains(&coarse) && (0.0..2.5e-3).contains(&fine) && fine < coarse,
        "{what}: volume change off by {coarse} at {COARSE} and {fine} at {FINE} from {expected}"
    );
    let meshed = mesh_volume(&topo, rounded) - mesh_volume(&topo, body);
    assert!(
        (meshed - expected).abs() < 1e-3 * expected,
        "{what}: mesh volume change {meshed}, expected {expected}"
    );
}

#[test]
fn undercut_pocket_corner_rounds_run_out_on_the_top() {
    for degrees in DRAFTS {
        for radius in RADII {
            assert_rounds_build(degrees, radius);
        }
    }
}

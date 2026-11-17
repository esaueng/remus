//! Drafting faces that carry blends (#1744): a 40 x 40 x 20 box with its
//! vertical edges rounded at R5, or its top loop at R2, drafted by 2 degrees
//! about the plane of its base.
//!
//! Every accepted result is checked against measurements that do not come
//! from the draft itself: the tilted planes' closed-form normals and neutral
//! lines, the rebuilt bands' exact cylinders (radius, axis along the tilted
//! planes' section, tangent to both), a closed-form volume, ray-cast point
//! classification either side of the moved walls and inside the rounded-off
//! corners, strict validation and a watertight mesh.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeSet;
use std::f64::consts::PI;

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::{Taper, draft, draft_sharp, draft_with_evolution};
use crate::OperationsError;
use crate::blend_ops::fillet_cascade;
use crate::classify::{PointClassification, classify_point};
use crate::primitives::{make_box, make_cylinder};

const SIDE: f64 = 40.0;
const HEIGHT: f64 = 20.0;
const VERTICAL_RADIUS: f64 = 5.0;
const LID_RADIUS: f64 = 2.0;

fn angle() -> f64 {
    2.0_f64.to_radians()
}

fn pull() -> Vec3 {
    Vec3::new(0.0, 0.0, 1.0)
}

fn ends(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let data = topo.edge(edge).unwrap();
    (
        topo.vertex(data.start()).unwrap().point(),
        topo.vertex(data.end()).unwrap().point(),
    )
}

/// The box's four vertical edges, as the issue's repro selects them.
fn vertical_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let (a, b) = ends(topo, edge);
            (a.z() - b.z()).abs() > 10.0
        })
        .collect()
}

/// The box's four top edges, as the issue's repro selects them.
fn top_loop(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let (a, b) = ends(topo, edge);
            (a.z() - HEIGHT).abs() < 1e-6 && (b.z() - HEIGHT).abs() < 1e-6
        })
        .collect()
}

/// The planar face with outward normal `normal`.
fn plane_face(topo: &Topology, solid: SolidId, normal: Vec3) -> FaceId {
    let found: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            let data = topo.face(face).unwrap();
            matches!(data.surface(), FaceSurface::Plane { .. })
                && (data.effective_plane_normal().unwrap() - normal).length() < 1e-9
        })
        .collect();
    assert_eq!(found.len(), 1, "one planar face with normal {normal:?}");
    found[0]
}

const SIDE_NORMALS: [(f64, f64); 4] = [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)];

/// The four walls, in [`SIDE_NORMALS`] order.
fn walls(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    SIDE_NORMALS
        .iter()
        .map(|&(x, y)| plane_face(topo, solid, Vec3::new(x, y, 0.0)))
        .collect()
}

/// A point of the wall with horizontal normal `(x, y)` on the neutral plane.
fn neutral_point_of_wall(x: f64, y: f64) -> Point3 {
    Point3::new(
        SIDE / 2.0 + x * SIDE / 2.0,
        SIDE / 2.0 + y * SIDE / 2.0,
        0.0,
    )
}

/// The closed-form outward normal of a wall with horizontal normal `(x, y)`
/// after a draft by [`angle`] about +Z: tilted so the wall leans outward
/// with height.
fn drafted_normal(x: f64, y: f64) -> Vec3 {
    let (sin, cos) = angle().sin_cos();
    Vec3::new(x * cos, y * cos, -sin)
}

fn rounded_box(topo: &mut Topology) -> SolidId {
    let body = make_box(topo, SIDE, SIDE, HEIGHT).unwrap();
    let edges = vertical_edges(topo, body);
    assert_eq!(edges.len(), 4);
    fillet_cascade(topo, body, &edges, VERTICAL_RADIUS)
        .unwrap()
        .solid
}

fn lidded_box(topo: &mut Topology) -> SolidId {
    let body = make_box(topo, SIDE, SIDE, HEIGHT).unwrap();
    let edges = top_loop(topo, body);
    assert_eq!(edges.len(), 4);
    fillet_cascade(topo, body, &edges, LID_RADIUS)
        .unwrap()
        .solid
}

fn tags(topo: &Topology, solid: SolidId) -> Vec<(&'static str, usize)> {
    let mut count = std::collections::BTreeMap::<&'static str, usize>::new();
    for face in solid_faces(topo, solid).unwrap() {
        *count
            .entry(topo.face(face).unwrap().surface().type_tag())
            .or_default() += 1;
    }
    count.into_iter().collect()
}

fn assert_valid_and_watertight(topo: &Topology, solid: SolidId) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "invalid result: {:?}", report.issues);
    for deflection in [0.01, 0.2] {
        let mesh = crate::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
        let boundary = crate::tessellate::boundary_edge_count(&mesh);
        let non_manifold = crate::tessellate::non_manifold_edge_count(&mesh);
        assert!(
            boundary == 0 && non_manifold == 0,
            "mesh at deflection {deflection}: {boundary} boundary, {non_manifold} non-manifold edges"
        );
    }
}

fn classified(topo: &Topology, solid: SolidId, point: Point3) -> PointClassification {
    classify_point(topo, solid, point, 0.01, 1e-7).unwrap()
}

/// The area a ball of `radius` rolling in a corner of interior angle `theta`
/// leaves between the corner and its arc, in the section square to the edge:
/// two right triangles (legs `r` and `r cot(theta / 2)`) less the sector the
/// arc subtends (`pi - theta`).
fn rounded_corner_area(radius: f64, theta: f64) -> f64 {
    radius * radius * (1.0 / (theta / 2.0).tan() - (PI - theta) / 2.0)
}

/// Assert `face` is the wall with horizontal normal `(x, y)`, either tilted
/// by the draft about its neutral line or left as it was.
fn assert_wall(topo: &Topology, face: FaceId, (x, y): (f64, f64), tilted: bool) {
    let data = topo.face(face).unwrap();
    assert!(
        matches!(data.surface(), FaceSurface::Plane { .. }),
        "wall ({x}, {y}) is no longer a plane"
    );
    let normal = data.effective_plane_normal().unwrap();
    let expected = if tilted {
        drafted_normal(x, y)
    } else {
        Vec3::new(x, y, 0.0)
    };
    assert!(
        (normal - expected).length() < 1e-9,
        "wall ({x}, {y}): normal {normal:?}, expected {expected:?}"
    );
    // The angle to the pull direction: square to it before, 2 degrees off it
    // after.
    let off_square = (-normal.dot(pull())).asin();
    let wanted = if tilted { angle() } else { 0.0 };
    assert!(
        (off_square - wanted).abs() < 1e-9,
        "wall ({x}, {y}) is {} degrees off square to the pull, expected {}",
        off_square.to_degrees(),
        wanted.to_degrees()
    );
    // The neutral line stays where it was: every boundary vertex is on the
    // plane through the wall's foot with the expected normal.
    let foot = neutral_point_of_wall(x, y);
    for oriented in topo.wire(data.outer_wire()).unwrap().edges() {
        let point = topo
            .vertex(oriented.oriented_start(topo.edge(oriented.edge()).unwrap()))
            .unwrap()
            .point();
        let off = expected.dot(point - foot);
        assert!(
            off.abs() < 1e-7,
            "wall ({x}, {y}) vertex {point:?} is {off} off its plane"
        );
    }
}

/// Assert the solid has exactly one cylindrical band of `radius` rolling
/// between the planes (`n1 . p = d1`, `n2 . p = d2`, outward normals):
/// its axis along their section and `radius` inside both.
fn assert_band(topo: &Topology, solid: SolidId, radius: f64, planes: [(Vec3, Point3); 2]) {
    let direction = planes[0].0.cross(planes[1].0).normalize().unwrap();
    let mut matching = 0;
    for face in solid_faces(topo, solid).unwrap() {
        let FaceSurface::Cylinder(cylinder) = topo.face(face).unwrap().surface() else {
            continue;
        };
        if cylinder.axis().cross(direction).length() > 1e-9 {
            continue;
        }
        let inside = planes
            .iter()
            .all(|&(normal, on)| (normal.dot(cylinder.origin() - on) + radius).abs() < 1e-7);
        if !inside {
            continue;
        }
        assert!(
            (cylinder.radius() - radius).abs() < 1e-9,
            "band radius {} is not the recorded {radius}",
            cylinder.radius()
        );
        matching += 1;
    }
    assert_eq!(
        matching, 1,
        "expected one R{radius} cylinder along {direction:?} tangent to both walls"
    );
}

/// The sharp construction on its own — all `draft` was before #1744 —
/// refuses the rounded box with the issue's message, and now names the
/// bands at the refused corner so the draft can go through their group.
#[test]
fn sharp_construction_refuses_the_rounded_box_and_names_its_bands() {
    let mut topo = Topology::new();
    let rounded = rounded_box(&mut topo);
    let selected = walls(&topo, rounded);
    let before = solid_entity_counts(&topo, rounded).unwrap();

    let Err(refusal) = draft_sharp(
        &mut topo,
        rounded,
        &selected,
        Taper {
            pull: pull(),
            neutral_d: 0.0,
            tan_angle: angle().tan(),
        },
    ) else {
        panic!("the sharp construction cannot re-trim a band");
    };

    let OperationsError::Unsupported {
        operation: "draft",
        reason,
    } = &refusal.error
    else {
        panic!("expected a typed draft refusal, got {:?}", refusal.error);
    };
    assert!(
        reason.contains("of a drafted face also lies on a curved face; re-trimming a curved neighbour against the drafted plane is not implemented"),
        "unexpected refusal: {reason}"
    );
    assert!(!refusal.curved.is_empty());
    for face in &refusal.curved {
        assert!(matches!(
            topo.face(*face).unwrap().surface(),
            FaceSurface::Cylinder(_)
        ));
    }
    assert_eq!(solid_entity_counts(&topo, rounded).unwrap(), before);
}

/// The issue's first case: all four walls of the box with its vertical
/// edges at R5 drafted by 2 degrees. Refused before the fix with "corner N
/// of a drafted face also lies on a curved face".
#[test]
fn draft_all_walls_of_a_box_with_rounded_vertical_edges() {
    let mut topo = Topology::new();
    let rounded = rounded_box(&mut topo);
    let source_counts = solid_entity_counts(&topo, rounded).unwrap();
    let source_faces = solid_faces(&topo, rounded).unwrap();
    let selected = walls(&topo, rounded);

    let (drafted, evolution) = draft_with_evolution(
        &mut topo,
        rounded,
        &selected,
        pull(),
        Point3::new(0.0, 0.0, 0.0),
        angle(),
    )
    .unwrap();

    // Same topology as the input: six planes and four exact cylinders.
    assert_eq!(
        tags(&topo, drafted),
        vec![("cylinder", 4), ("plane", 6)],
        "the blends must come back as exact cylinders"
    );
    assert_eq!(solid_entity_counts(&topo, drafted).unwrap(), source_counts);
    assert_valid_and_watertight(&topo, drafted);

    // Every source face has exactly one image, each its own, of its own
    // surface kind: the walls tilted, the caps re-trimmed, the bands rebuilt.
    let mut images = BTreeSet::new();
    for &face in &source_faces {
        let [image] = evolution
            .modified
            .get(&face.index())
            .unwrap_or_else(|| panic!("face {} has no image", face.index()))
            .as_slice()
        else {
            panic!("face {} has several images", face.index());
        };
        assert!(images.insert(*image), "two faces share image {image}");
        let image = topo.face_id_from_index(*image).unwrap();
        assert_eq!(
            topo.face(image).unwrap().surface().type_tag(),
            topo.face(face).unwrap().surface().type_tag()
        );
    }
    assert!(evolution.deleted.is_empty() && evolution.unresolved.is_empty());

    // The walls tilt about their neutral lines.
    for (&wall, &normal) in selected.iter().zip(&SIDE_NORMALS) {
        let image = topo
            .face_id_from_index(evolution.modified[&wall.index()][0])
            .unwrap();
        assert_wall(&topo, image, normal, true);
    }
    // The caps stay on their planes.
    plane_face(&topo, drafted, Vec3::new(0.0, 0.0, 1.0));
    plane_face(&topo, drafted, Vec3::new(0.0, 0.0, -1.0));

    // Each band is the R5 cylinder rolling between its two tilted walls.
    for (x, y) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
        assert_band(
            &topo,
            drafted,
            VERTICAL_RADIUS,
            [
                (drafted_normal(x, 0.0), neutral_point_of_wall(x, 0.0)),
                (drafted_normal(0.0, y), neutral_point_of_wall(0.0, y)),
            ],
        );
    }

    // Closed-form volume: the frustum the tilted walls bound, less the four
    // rounded-off corner prisms. Each prism has the corner section square to
    // its (slanted) edge and runs between the parallel caps, so its volume
    // is that section times the edge length between the caps.
    let tan = angle().tan();
    let frustum = ((SIDE + 2.0 * tan * HEIGHT).powi(3) - SIDE.powi(3)) / (6.0 * tan);
    let (n1, n2) = (drafted_normal(1.0, 0.0), drafted_normal(0.0, 1.0));
    let theta = PI - n1.dot(n2).acos();
    let edge = n1.cross(n2).normalize().unwrap();
    let corner = rounded_corner_area(VERTICAL_RADIUS, theta) * HEIGHT / edge.z().abs();
    let expected = frustum - 4.0 * corner;
    let volume = crate::measure::solid_volume(&topo, drafted, 0.0005).unwrap();
    assert!(
        (volume - expected).abs() < 1e-4 * expected,
        "volume {volume}, closed form {expected}"
    );
    // The input, by the same formula at zero draft, is smaller: the draft
    // added material.
    let source =
        SIDE * SIDE * HEIGHT - 4.0 * rounded_corner_area(VERTICAL_RADIUS, PI / 2.0) * HEIGHT;
    assert!(expected > source + 500.0);

    // Ray-cast probes near the top, where the walls moved out by
    // 20 tan(2 deg) = 0.698.
    let reach = SIDE + HEIGHT * tan;
    let near_top = HEIGHT - 0.1;
    for (point, wanted) in [
        // Beyond the old +X wall, inside the tilted one.
        (
            Point3::new(SIDE + 0.5, 20.0, near_top),
            PointClassification::Inside,
        ),
        (
            Point3::new(reach + 0.1, 20.0, near_top),
            PointClassification::Outside,
        ),
        // Beyond the old -Y wall, inside the tilted one.
        (
            Point3::new(20.0, -0.5, near_top),
            PointClassification::Inside,
        ),
        (
            Point3::new(20.0, SIDE - reach - 0.1, near_top),
            PointClassification::Outside,
        ),
        // At the foot the walls did not move.
        (
            Point3::new(SIDE + 0.05, 20.0, 0.1),
            PointClassification::Outside,
        ),
        (
            Point3::new(SIDE - 0.05, 20.0, 0.1),
            PointClassification::Inside,
        ),
        // The rounded-off corner: the ball leaves r (sqrt 2 - 1) = 2.07
        // along the diagonal.
        (
            Point3::new(reach - 0.7, reach - 0.7, near_top),
            PointClassification::Outside,
        ),
        (
            Point3::new(reach - 2.2, reach - 2.2, near_top),
            PointClassification::Inside,
        ),
        (
            Point3::new(SIDE - 0.7, 0.7, 0.1),
            PointClassification::Outside,
        ),
        (
            Point3::new(SIDE - 2.2, 2.2, 0.1),
            PointClassification::Inside,
        ),
    ] {
        assert_eq!(classified(&topo, drafted, point), wanted, "probe {point:?}");
    }
}

/// The issue's second case: one wall of the same body. The bands on the
/// drafted wall are rebuilt against it; the two bands across the body end on
/// the top face the draft re-trims, so they are removed and re-applied too,
/// and come back as the cylinders they were.
#[test]
fn draft_one_wall_of_a_box_with_rounded_vertical_edges() {
    let mut topo = Topology::new();
    let rounded = rounded_box(&mut topo);
    let source_counts = solid_entity_counts(&topo, rounded).unwrap();
    let wall = plane_face(&topo, rounded, Vec3::new(1.0, 0.0, 0.0));

    let drafted = draft(
        &mut topo,
        rounded,
        &[wall],
        pull(),
        Point3::new(0.0, 0.0, 0.0),
        angle(),
    )
    .unwrap();

    assert_eq!(tags(&topo, drafted), vec![("cylinder", 4), ("plane", 6)]);
    assert_eq!(solid_entity_counts(&topo, drafted).unwrap(), source_counts);
    assert_valid_and_watertight(&topo, drafted);

    for &(x, y) in &SIDE_NORMALS {
        let tilted = x > 0.5;
        let expected = if tilted {
            drafted_normal(x, y)
        } else {
            Vec3::new(x, y, 0.0)
        };
        assert_wall(&topo, plane_face(&topo, drafted, expected), (x, y), tilted);
    }
    let wall_plane = |x: f64, y: f64| {
        let normal = if x > 0.5 {
            drafted_normal(x, y)
        } else {
            Vec3::new(x, y, 0.0)
        };
        (normal, neutral_point_of_wall(x, y))
    };
    for (x, y) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
        assert_band(
            &topo,
            drafted,
            VERTICAL_RADIUS,
            [wall_plane(x, 0.0), wall_plane(0.0, y)],
        );
    }

    // The box plus the wedge the tilted wall adds, less two upright corner
    // prisms and two along the slanted edges (still square corners: the
    // tilted wall's normal stays square to the +/-Y walls').
    let tan = angle().tan();
    let wedge = SIDE * HEIGHT * (HEIGHT * tan) / 2.0;
    let section = rounded_corner_area(VERTICAL_RADIUS, PI / 2.0);
    let expected = SIDE * SIDE * HEIGHT + wedge
        - 2.0 * section * HEIGHT
        - 2.0 * section * HEIGHT / angle().cos();
    let volume = crate::measure::solid_volume(&topo, drafted, 0.0005).unwrap();
    assert!(
        (volume - expected).abs() < 1e-4 * expected,
        "volume {volume}, closed form {expected}"
    );

    let reach = SIDE + HEIGHT * tan;
    let near_top = HEIGHT - 0.1;
    for (point, wanted) in [
        (
            Point3::new(SIDE + 0.5, 20.0, near_top),
            PointClassification::Inside,
        ),
        (
            Point3::new(reach + 0.1, 20.0, near_top),
            PointClassification::Outside,
        ),
        // The other walls did not move.
        (
            Point3::new(-0.05, 20.0, near_top),
            PointClassification::Outside,
        ),
        (
            Point3::new(20.0, SIDE + 0.05, near_top),
            PointClassification::Outside,
        ),
        (
            Point3::new(20.0, SIDE - 0.05, near_top),
            PointClassification::Inside,
        ),
        // Rounded corners: at the drafted wall and across the body.
        (
            Point3::new(reach - 0.7, SIDE - 0.7, near_top),
            PointClassification::Outside,
        ),
        (
            Point3::new(reach - 2.2, SIDE - 2.2, near_top),
            PointClassification::Inside,
        ),
        (
            Point3::new(0.7, 0.7, near_top),
            PointClassification::Outside,
        ),
        (Point3::new(2.2, 2.2, near_top), PointClassification::Inside),
    ] {
        assert_eq!(classified(&topo, drafted, point), wanted, "probe {point:?}");
    }
}

/// A convex polyhedron's volume, the polyhedron given as half-spaces
/// `n . p <= d`: its vertices are the feasible three-plane intersections,
/// and each face's vertices, ordered round the face, are fanned to an
/// interior point.
fn polyhedron_volume(planes: &[(Vec3, f64)]) -> f64 {
    let mut vertices: Vec<Vec3> = Vec::new();
    for (i, &(a, da)) in planes.iter().enumerate() {
        for (j, &(b, db)) in planes.iter().enumerate().skip(i + 1) {
            for &(c, dc) in planes.iter().skip(j + 1) {
                let det = a.dot(b.cross(c));
                if det.abs() < 1e-12 {
                    continue;
                }
                let p = (b.cross(c) * da + c.cross(a) * db + a.cross(b) * dc) * (1.0 / det);
                if planes.iter().all(|&(n, d)| n.dot(p) <= d + 1e-9)
                    && vertices.iter().all(|&q| (q - p).length() > 1e-9)
                {
                    vertices.push(p);
                }
            }
        }
    }
    let mean = |points: &[Vec3]| {
        points
            .iter()
            .fold(Vec3::new(0.0, 0.0, 0.0), |sum, &p| sum + p)
            * (1.0 / points.len() as f64)
    };
    let inside = mean(&vertices);
    let mut volume = 0.0;
    for &(normal, d) in planes {
        let mut face: Vec<Vec3> = vertices
            .iter()
            .copied()
            .filter(|&p| (normal.dot(p) - d).abs() < 1e-9)
            .collect();
        if face.len() < 3 {
            continue;
        }
        let centre = mean(&face);
        let u = (face[0] - centre).normalize().unwrap();
        let w = normal.cross(u);
        let turn = |p: Vec3| (p - centre).dot(w).atan2((p - centre).dot(u));
        face.sort_by(|&p, &q| turn(p).total_cmp(&turn(q)));
        for (k, &a) in face.iter().enumerate() {
            let b = face[(k + 1) % face.len()];
            volume += (a - inside).cross(b - inside).dot(centre - inside).abs() / 6.0;
        }
    }
    volume
}

/// How far the centre of a lid corner's ball sits inside each wall, along
/// X and Y, with the walls drafted by `angle`: the ball is [`LID_RADIUS`]
/// below the top and [`LID_RADIUS`] inside both walls.
fn lid_ball_inset(angle: f64) -> f64 {
    let (sin, cos) = angle.sin_cos();
    (LID_RADIUS - (HEIGHT - LID_RADIUS) * sin) / cos
}

/// The closed-form volume of the box with its top loop rounded at
/// [`LID_RADIUS`] and its walls drafted by `angle` (zero: not drafted).
///
/// The rolling-ball engine closes each corner, where two stripes meet and
/// the edge between the walls stays sharp, with the ball's patch over a
/// planar ledge: the stripes end on the ball's great circles square to
/// their edges, and the ledge lies in the plane through the ball's centre
/// square to the sharp edge. Those three planes, the top and the two walls
/// bound the corner's cell; the body keeps the ball's sector the three
/// planes cut at the centre and loses the rest of the cell. So the volume is
/// the frustum the walls bound, less each band's section over its length
/// between its end stations, less each corner's cell less that sector. The
/// sector's solid angle is the sum of the three planes' dihedral angles less
/// `pi`.
fn lid_volume(angle: f64) -> f64 {
    let radius = LID_RADIUS;
    let (sin, cos) = angle.sin_cos();
    let tan = angle.tan();
    let frustum = if tan.abs() < 1e-12 {
        SIDE * SIDE * HEIGHT
    } else {
        ((SIDE + 2.0 * tan * HEIGHT).powi(3) - SIDE.powi(3)) / (6.0 * tan)
    };
    // Each top edge: the R2 corner between the top and a wall at the
    // interior angle 90 degrees less the draft.
    let section = rounded_corner_area(radius, PI / 2.0 - angle);
    let inset = lid_ball_inset(angle);
    let band = SIDE - 2.0 * inset;
    // The corner at the origin: walls `n . p <= 0` through the neutral
    // lines, the ball's centre, and the sharp edge between the walls,
    // pointing up toward the top.
    let centre = Vec3::new(inset, inset, HEIGHT - radius);
    let up = Vec3::new(-sin, -sin, cos).normalize().unwrap();
    let cell = polyhedron_volume(&[
        (Vec3::new(0.0, -cos, -sin), 0.0),
        (Vec3::new(-cos, 0.0, -sin), 0.0),
        (Vec3::new(0.0, 0.0, 1.0), HEIGHT),
        (Vec3::new(1.0, 0.0, 0.0), inset),
        (Vec3::new(0.0, 1.0, 0.0), inset),
        (-up, -up.dot(centre)),
    ]);
    // Inward normals at the centre: -X and -Y (the station planes, square
    // to each other) and `up` (the ledge plane). The dihedral angle between
    // two of them is pi less the angle between their normals.
    let ledge_dihedral = PI - Vec3::new(-1.0, 0.0, 0.0).dot(up).acos();
    let solid_angle = PI / 2.0 + 2.0 * ledge_dihedral - PI;
    let sector = solid_angle * radius.powi(3) / 3.0;
    frustum - 4.0 * section * band - 4.0 * (cell - sector)
}

/// Assert the solid carries one [`LID_RADIUS`] ball patch at each top
/// corner, centred where the ball touches the top and both walls drafted by
/// `angle`.
fn assert_lid_corners(topo: &Topology, solid: SolidId, angle: f64) {
    let inset = lid_ball_inset(angle);
    let height = HEIGHT - LID_RADIUS;
    for (x, y) in [
        (inset, inset),
        (SIDE - inset, inset),
        (inset, SIDE - inset),
        (SIDE - inset, SIDE - inset),
    ] {
        let found = solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .filter(|&face| {
                let FaceSurface::Sphere(sphere) = topo.face(face).unwrap().surface() else {
                    return false;
                };
                (sphere.center() - Point3::new(x, y, height)).length() < 1e-9
                    && (sphere.radius() - LID_RADIUS).abs() < 1e-9
            })
            .count();
        assert_eq!(found, 1, "one R{LID_RADIUS} ball at ({x}, {y}, {height})");
    }
}

/// Assert `solid` is the lidded box with all four walls drafted and its top
/// loop rounded at R2 again.
fn assert_drafted_lid(topo: &Topology, solid: SolidId) {
    assert_valid_and_watertight(topo, solid);
    assert_eq!(
        tags(topo, solid),
        vec![("cylinder", 4), ("plane", 10), ("sphere", 4)]
    );
    for &(x, y) in &SIDE_NORMALS {
        assert_wall(
            topo,
            plane_face(topo, solid, drafted_normal(x, y)),
            (x, y),
            true,
        );
    }
    plane_face(topo, solid, Vec3::new(0.0, 0.0, 1.0));
    // Each top edge carries the R2 cylinder rolling between its tilted wall
    // and the top, and each corner the R2 ball touching all three.
    let top = (Vec3::new(0.0, 0.0, 1.0), Point3::new(0.0, 0.0, HEIGHT));
    for &(x, y) in &SIDE_NORMALS {
        assert_band(
            topo,
            solid,
            LID_RADIUS,
            [(drafted_normal(x, y), neutral_point_of_wall(x, y)), top],
        );
    }
    assert_lid_corners(topo, solid, angle());
    let expected = lid_volume(angle());
    for deflection in [0.0005, 0.0002] {
        let volume = crate::measure::solid_volume(topo, solid, deflection).unwrap();
        assert!(
            (volume - expected).abs() < 1e-4 * expected,
            "volume {volume} at deflection {deflection}, closed form {expected}"
        );
    }
    let reach = SIDE + HEIGHT * angle().tan();
    let inset = lid_ball_inset(angle());
    for (point, wanted) in [
        (
            Point3::new(SIDE + 0.3, 20.0, HEIGHT - 3.0),
            PointClassification::Inside,
        ),
        // Inside the sharp top edge, outside the round.
        (
            Point3::new(reach - 0.2, 20.0, HEIGHT - 0.2),
            PointClassification::Outside,
        ),
        (
            Point3::new(reach - 1.5, 20.0, HEIGHT - 1.5),
            PointClassification::Inside,
        ),
        // At a corner: above the ledge and outside the ball, then below the
        // ledge, inside the walls.
        (
            Point3::new(0.0, 0.0, HEIGHT - LID_RADIUS + 0.5),
            PointClassification::Outside,
        ),
        (
            Point3::new(0.0, 0.0, HEIGHT - LID_RADIUS - 0.5),
            PointClassification::Inside,
        ),
        // Inside the ball, under its contact with the top.
        (
            Point3::new(inset, inset, HEIGHT - 0.1),
            PointClassification::Inside,
        ),
    ] {
        assert_eq!(classified(topo, solid, point), wanted, "probe {point:?}");
    }
}

/// The issue's third case: the box with its top loop at R2, all four walls
/// drafted. Each lid corner is the ball's patch over a planar ledge; the
/// ledge goes with its corner when the group is removed to the sharp box,
/// the sharp box drafts, and the loop re-fillets on the drafted box's 88
/// and 92 degree corners.
#[test]
fn draft_all_walls_of_a_box_with_a_rounded_top_loop() {
    let mut topo = Topology::new();
    let lid = lidded_box(&mut topo);
    // The input is the closed form's undrafted body.
    let volume = crate::measure::solid_volume(&topo, lid, 0.0005).unwrap();
    let expected = lid_volume(0.0);
    assert!(
        (volume - expected).abs() < 1e-4 * expected,
        "input volume {volume}, closed form {expected}"
    );
    assert_lid_corners(&topo, lid, 0.0);
    let before = solid_entity_counts(&topo, lid).unwrap();
    let selected = walls(&topo, lid);

    let drafted = draft(
        &mut topo,
        lid,
        &selected,
        pull(),
        Point3::new(0.0, 0.0, 0.0),
        angle(),
    )
    .unwrap();

    assert_ne!(drafted, lid);
    assert_eq!(solid_entity_counts(&topo, drafted).unwrap(), before);
    assert_drafted_lid(&topo, drafted);
    // The input is left as it was.
    assert_eq!(solid_entity_counts(&topo, lid).unwrap(), before);
    assert_valid_and_watertight(&topo, lid);
}

/// The issue's point: a drafted part with rounded edges can be built in
/// either order. Rounding the top loop of the drafted box builds the same
/// body as drafting the rounded one.
#[test]
fn rounding_a_drafted_box_builds_the_drafted_lid() {
    let mut topo = Topology::new();
    let body = make_box(&mut topo, SIDE, SIDE, HEIGHT).unwrap();
    let selected = walls(&topo, body);
    let drafted = draft(
        &mut topo,
        body,
        &selected,
        pull(),
        Point3::new(0.0, 0.0, 0.0),
        angle(),
    )
    .unwrap();
    let edges = top_loop(&topo, drafted);
    assert_eq!(edges.len(), 4);
    let rounded = fillet_cascade(&mut topo, drafted, &edges, LID_RADIUS)
        .unwrap()
        .solid;
    assert_drafted_lid(&topo, rounded);
}

/// A curved face in the way that is not a blend is not the blend route's to
/// answer: the draft keeps its own refusal, on the caller's own indices.
#[test]
fn draft_keeps_its_refusal_for_a_curved_neighbour_that_is_no_blend() {
    let mut topo = Topology::new();
    let post = make_cylinder(&mut topo, 6.0, 10.0).unwrap();
    let cap = plane_face(&topo, post, Vec3::new(0.0, 0.0, 1.0));
    let before = solid_entity_counts(&topo, post).unwrap();

    let error = draft(
        &mut topo,
        post,
        &[cap],
        Vec3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
        angle(),
    )
    .unwrap_err();

    let OperationsError::Unsupported {
        operation: "draft",
        reason,
    } = &error
    else {
        panic!("expected a typed draft refusal, got {error:?}");
    };
    assert!(
        reason.contains(&format!("face {}", cap.index())) && reason.contains("curved edge"),
        "unexpected refusal: {reason}"
    );
    assert_eq!(solid_entity_counts(&topo, post).unwrap(), before);
}

//! Measurement of planar faces whose inner wires nest: a hole holding an
//! island holding a hole alternates void and material by nesting depth, as
//! the classifier and tessellator already read it. Subtracting every inner
//! wire read such a face 2 x 16 short, and a solid's volume and centre of
//! mass with it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_check::validate::{CheckId, Severity, ValidateOptions, ValidationIssue, validate_solid};
use remus_math::det_hash::DetHashMap;
use remus_math::vec::{Point3, Vec3};
use remus_operations::{measure, tessellate};
use remus_topology::Topology;
use remus_topology::builder::{make_polygon_wire, wire_plane};
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

/// `([x0, x1], [y0, y1])`.
type Rect = ([f64; 2], [f64; 2]);

/// The block's footprint.
const BLOCK: Rect = ([0.0, 14.0], [0.0, 8.0]);
/// A pocket, from z = 1 to the top.
const POCKET: Rect = ([1.0, 9.0], [1.0, 7.0]);
/// An island left standing in the pocket.
const ISLAND: Rect = ([2.0, 6.0], [2.0, 6.0]);
/// A through hole in the island.
const ISLAND_HOLE: Rect = ([3.0, 4.0], [3.0, 5.0]);
/// A through hole beside the pocket, nested in nothing.
const SIDE_HOLE: Rect = ([10.0, 13.0], [2.0, 5.0]);
const TOP: f64 = 3.0;
const FLOOR: f64 = 1.0;

fn rect_area(([x0, x1], [y0, y1]): Rect) -> f64 {
    (x1 - x0) * (y1 - y0)
}

fn rect_perimeter(([x0, x1], [y0, y1]): Rect) -> f64 {
    2.0 * ((x1 - x0) + (y1 - y0))
}

/// Corners counter-clockwise seen from +z.
fn corners(([x0, x1], [y0, y1]): Rect, z: f64) -> Vec<Point3> {
    vec![
        Point3::new(x0, y0, z),
        Point3::new(x1, y0, z),
        Point3::new(x1, y1, z),
        Point3::new(x0, y1, z),
    ]
}

fn reversed(mut points: Vec<Point3>) -> Vec<Point3> {
    points.reverse();
    points
}

/// The top face's loops: outer, then the pocket opening (depth 0), the island
/// (depth 1), the island's hole (depth 2) and the side hole (depth 0).
const TOP_INNER: [Rect; 4] = [POCKET, ISLAND, ISLAND_HOLE, SIDE_HOLE];
const TOP_SIGNS: [f64; 4] = [-1.0, 1.0, -1.0, -1.0];

/// The top face's inner loops at height `z`, wound by the material they
/// bound: clockwise about +z around a void, counter-clockwise around the
/// island.
fn top_inner_loops(z: f64) -> Vec<Vec<Point3>> {
    TOP_INNER
        .into_iter()
        .zip(TOP_SIGNS)
        .map(|(r, sign)| {
            if sign > 0.0 {
                corners(r, z)
            } else {
                reversed(corners(r, z))
            }
        })
        .collect()
}

/// Area and first moments `(∫x dA, ∫y dA)` of the top face, by signed
/// rectangles.
fn top_closed_form() -> (f64, f64, f64) {
    let moments = |r: Rect| {
        let ([x0, x1], [y0, y1]) = r;
        let area = rect_area(r);
        (area, area * 0.5 * (x0 + x1), area * 0.5 * (y0 + y1))
    };
    let (mut area, mut sx, mut sy) = moments(BLOCK);
    for (r, sign) in TOP_INNER.into_iter().zip(TOP_SIGNS) {
        let (a, x, y) = moments(r);
        area += sign * a;
        sx += sign * x;
        sy += sign * y;
    }
    (area, sx, sy)
}

/// Builds faces that share their vertices and edges, so the result closes
/// into a solid without sewing.
#[derive(Default)]
struct Shared {
    vertices: DetHashMap<[u64; 3], VertexId>,
    edges: DetHashMap<(VertexId, VertexId), EdgeId>,
}

impl Shared {
    fn vertex(&mut self, topo: &mut Topology, p: Point3) -> VertexId {
        let key = [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()];
        *self
            .vertices
            .entry(key)
            .or_insert_with(|| topo.add_vertex(Vertex::new(p, 1e-9)))
    }

    /// A closed wire through `points` in order.
    fn wire(&mut self, topo: &mut Topology, points: &[Point3]) -> WireId {
        let ids: Vec<_> = points.iter().map(|&p| self.vertex(topo, p)).collect();
        let oriented = (0..ids.len())
            .map(|k| {
                let (a, b) = (ids[k], ids[(k + 1) % ids.len()]);
                if let Some(&edge) = self.edges.get(&(b, a)) {
                    OrientedEdge::new(edge, false)
                } else {
                    let edge = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
                    self.edges.insert((a, b), edge);
                    OrientedEdge::new(edge, true)
                }
            })
            .collect();
        topo.add_wire(Wire::new(oriented, true).unwrap())
    }

    /// A planar face with outward `normal`, its outer loop counter-clockwise
    /// about it and its inner loops clockwise.
    fn face(
        &mut self,
        topo: &mut Topology,
        normal: Vec3,
        outer: &[Point3],
        inner: &[Vec<Point3>],
    ) -> FaceId {
        let d = normal.dot(outer[0] - Point3::new(0.0, 0.0, 0.0));
        let outer = self.wire(topo, outer);
        let inner = inner.iter().map(|loop_| self.wire(topo, loop_)).collect();
        topo.add_face(Face::new(outer, inner, FaceSurface::Plane { normal, d }))
    }

    /// The walls of `rect` between `z0` and `z1`, facing out of the rectangle
    /// when `material_inside`, else into it.
    fn walls(
        &mut self,
        topo: &mut Topology,
        rect: Rect,
        (z0, z1): (f64, f64),
        material_inside: bool,
    ) -> Vec<FaceId> {
        let bottom = corners(rect, z0);
        (0..4)
            .map(|k| {
                let (p, q) = (bottom[k], bottom[(k + 1) % 4]);
                let lift = Vec3::new(0.0, 0.0, z1 - z0);
                let out = (q - p).cross(lift).normalize().unwrap();
                let quad = vec![p, q, q + lift, p + lift];
                if material_inside {
                    self.face(topo, out, &quad, &[])
                } else {
                    self.face(topo, -out, &reversed(quad), &[])
                }
            })
            .collect()
    }
}

/// The pocketed block: a block with a pocket from z = 1, an island standing
/// in the pocket up to the top with a through hole, and a through hole beside
/// the pocket. Its top face carries the nested inner wires.
fn pocketed_block(topo: &mut Topology) -> (SolidId, FaceId) {
    let up = Vec3::new(0.0, 0.0, 1.0);
    let mut shared = Shared::default();
    let top = shared.face(topo, up, &corners(BLOCK, TOP), &top_inner_loops(TOP));
    let mut faces = vec![top];
    faces.push(shared.face(
        topo,
        -up,
        &reversed(corners(BLOCK, 0.0)),
        &[corners(ISLAND_HOLE, 0.0), corners(SIDE_HOLE, 0.0)],
    ));
    faces.push(shared.face(
        topo,
        up,
        &corners(POCKET, FLOOR),
        &[reversed(corners(ISLAND, FLOOR))],
    ));
    faces.extend(shared.walls(topo, BLOCK, (0.0, TOP), true));
    faces.extend(shared.walls(topo, POCKET, (FLOOR, TOP), false));
    faces.extend(shared.walls(topo, ISLAND, (FLOOR, TOP), true));
    faces.extend(shared.walls(topo, ISLAND_HOLE, (0.0, TOP), false));
    faces.extend(shared.walls(topo, SIDE_HOLE, (0.0, TOP), false));
    // Not `sew::make_solid_from_shared_faces`: its validation requires every
    // inner wire to wind against the outer one, and the island's loop must
    // wind with it for the island's walls to share its edges.
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|issue| issue.severity == Severity::Error)
        .collect();
    let island_winding = |issue: &&ValidationIssue| {
        issue.check == CheckId::FaceOrientationConsistency
            && issue.description.contains("same winding as its outer wire")
    };
    assert!(
        errors.iter().all(island_winding) && errors.len() <= 1,
        "{errors:#?}"
    );
    (solid, top)
}

/// Volume and first moments `(V, ∫x dV, ∫y dV, ∫z dV)` of the block by signed
/// boxes: the block, less the pocket, plus the island, less both through
/// holes. Inside the island's hole the pocket's −1 and the island's +1
/// cancel, so the hole removes its column once.
fn block_closed_form() -> [f64; 4] {
    let boxes = [
        (BLOCK, (0.0, TOP), 1.0),
        (POCKET, (FLOOR, TOP), -1.0),
        (ISLAND, (FLOOR, TOP), 1.0),
        (ISLAND_HOLE, (0.0, TOP), -1.0),
        (SIDE_HOLE, (0.0, TOP), -1.0),
    ];
    let mut sums = [0.0; 4];
    for (r, (z0, z1), sign) in boxes {
        let ([x0, x1], [y0, y1]) = r;
        let v = sign * rect_area(r) * (z1 - z0);
        sums[0] += v;
        sums[1] += v * 0.5 * (x0 + x1);
        sums[2] += v * 0.5 * (y0 + y1);
        sums[3] += v * 0.5 * (z0 + z1);
    }
    sums
}

fn mesh_area(topo: &Topology, face: FaceId) -> f64 {
    let mesh = tessellate::tessellate(topo, face, 0.01).unwrap();
    mesh.indices
        .chunks(3)
        .map(|t| {
            let p = |k: usize| mesh.positions[t[k] as usize];
            (p(1) - p(0)).cross(p(2) - p(0)).length() * 0.5
        })
        .sum()
}

#[test]
fn standalone_nested_face_area_alternates_material() {
    let mut topo = Topology::new();
    let outer = make_polygon_wire(&mut topo, &corners(BLOCK, 0.0), 1e-9).unwrap();
    let inner = top_inner_loops(0.0)
        .iter()
        .map(|points| make_polygon_wire(&mut topo, points, 1e-9).unwrap())
        .collect();
    let (normal, d) = wire_plane(&topo, outer).unwrap();
    let face = topo.add_face(Face::new(outer, inner, FaceSurface::Plane { normal, d }));

    let (area, _, _) = top_closed_form();
    // 112 − 48 + 16 − 2 − 9; subtracting every inner wire read 47.
    assert_eq!(area.to_bits(), 69.0_f64.to_bits());
    let measured = measure::face_area(&topo, face, 0.01).unwrap();
    assert!(
        (measured - area).abs() <= 1e-12 * area,
        "face area {measured}"
    );
    let mesh = mesh_area(&topo, face);
    assert!((mesh - area).abs() <= 1e-12 * area, "mesh area {mesh}");
}

#[test]
fn pocketed_block_with_an_island_measures_in_closed_form() {
    let mut topo = Topology::new();
    let (solid, top) = pocketed_block(&mut topo);

    // The material, independently of any measurement: the island and the
    // pocket's floor are solid, the pocket and both holes are empty.
    for (point, want) in [
        (Point3::new(5.0, 5.0, 2.0), PointClassification::Inside),
        (Point3::new(0.5, 4.0, 2.0), PointClassification::Inside),
        (Point3::new(1.5, 1.5, 0.5), PointClassification::Inside),
        (Point3::new(1.5, 1.5, 2.0), PointClassification::Outside),
        (Point3::new(3.5, 4.0, 0.5), PointClassification::Outside),
        (Point3::new(3.5, 4.0, 2.0), PointClassification::Outside),
        (Point3::new(11.5, 3.5, 1.5), PointClassification::Outside),
    ] {
        assert_eq!(
            classify_point(&topo, solid, point, &ClassifyOptions::default()).unwrap(),
            want,
            "{point:?}"
        );
    }

    let (top_area, _, _) = top_closed_form();
    let measured = measure::face_area(&topo, top, 0.01).unwrap();
    assert!(
        (measured - top_area).abs() <= 1e-12 * top_area,
        "top face area {measured} != {top_area}"
    );

    let [volume, mx, my, mz] = block_closed_form();
    // 336 − 96 + 32 − 6 − 27.
    assert_eq!(volume.to_bits(), 239.0_f64.to_bits());
    let measured = measure::solid_volume(&topo, solid, 0.01).unwrap();
    assert!(
        (measured - volume).abs() <= 1e-10 * volume,
        "volume {measured} != {volume}"
    );
    let com = measure::solid_center_of_mass(&topo, solid, 0.01).unwrap();
    let want = Point3::new(mx / volume, my / volume, mz / volume);
    assert!(
        (com - want).length() <= 1e-10,
        "centre of mass {com:?} != {want:?}"
    );

    // Top 69, bottom 101, pocket floor 32, and the walls.
    let bottom = rect_area(BLOCK) - rect_area(ISLAND_HOLE) - rect_area(SIDE_HOLE);
    let floor = rect_area(POCKET) - rect_area(ISLAND);
    let walls = TOP
        * (rect_perimeter(BLOCK) + rect_perimeter(ISLAND_HOLE) + rect_perimeter(SIDE_HOLE))
        + (TOP - FLOOR) * (rect_perimeter(POCKET) + rect_perimeter(ISLAND));
    let want_surface = top_area + bottom + floor + walls;
    let surface = measure::solid_surface_area(&topo, solid, 0.01).unwrap();
    assert!(
        (surface - want_surface).abs() <= 1e-10 * want_surface,
        "surface area {surface} != {want_surface}"
    );
}

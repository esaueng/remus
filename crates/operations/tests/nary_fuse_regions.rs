//! Qualification of the N-ary planar General Fuse (P-Class 4.8, first slice):
//! `fuse_regions` returns regions, total lineage and a disposition per
//! operand, and refuses typed and transactionally outside its domain.
//!
//! Every witness is built from primitives and judged by oracles that do not
//! read the arrangement: an exact union volume by coordinate compression
//! (the inclusion–exclusion sum, pinned by hand for the three-box witness),
//! a material probe in every cell of that grid, edge-use and vertex-link
//! manifoldness, a position-welded mesh, strict validation, and a geometric
//! reading of every lineage claim.

// `float_cmp`: the witness coordinates are dyadic, so the union oracle's sums
// are exact and are compared exactly.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::disallowed_methods,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::too_many_lines,
    clippy::unwrap_used
)]

use std::collections::{BTreeMap, BTreeSet, HashMap};

use remus_algo::error::AlgoError;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{
    EdgeEvent, NaryFuseRefusal, NaryFuseResult, OperandDisposition, VertexEvent, fuse_body_regions,
    fuse_regions,
};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::{BodyId, SolidId, Topology};

/// An axis-aligned box in the witness frame: minimum corner and size.
type Box3 = ([f64; 3], [f64; 3]);

/// A uniform scale followed by a rigid motion, applied to a whole witness.
#[derive(Clone, Copy)]
struct Placement {
    name: &'static str,
    scale: f64,
    motion: Mat4,
}

impl Placement {
    fn world(&self, p: [f64; 3]) -> Point3 {
        self.motion.mul_point(Point3::new(
            p[0] * self.scale,
            p[1] * self.scale,
            p[2] * self.scale,
        ))
    }
}

/// The frame the witnesses are written in, and the three scales of the
/// matrix each under a rigid motion with no axis left aligned.
fn placements() -> Vec<Placement> {
    let turned = |scale: f64| {
        Mat4::translation(3.7 * scale, -1.3 * scale, 2.1 * scale)
            * Mat4::rotation_z(0.3)
            * Mat4::rotation_x(-0.7)
            * Mat4::rotation_y(1.1)
    };
    vec![
        Placement {
            name: "aligned",
            scale: 1.0,
            motion: Mat4::identity(),
        },
        Placement {
            name: "1e-3 turned",
            scale: 1e-3,
            motion: turned(1e-3),
        },
        Placement {
            name: "1 turned",
            scale: 1.0,
            motion: turned(1.0),
        },
        Placement {
            name: "1e3 turned",
            scale: 1e3,
            motion: turned(1e3),
        },
    ]
}

fn place(topo: &mut Topology, (min, size): Box3, placement: &Placement) -> SolidId {
    let s = placement.scale;
    let solid = make_box(topo, size[0] * s, size[1] * s, size[2] * s).unwrap();
    let matrix = placement.motion * Mat4::translation(min[0] * s, min[1] * s, min[2] * s);
    transform_solid(topo, solid, &matrix).unwrap();
    solid
}

fn shifted((min, size): Box3, by: [f64; 3]) -> Box3 {
    ([min[0] + by[0], min[1] + by[1], min[2] + by[2]], size)
}

const A: Box3 = ([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);

fn b() -> Box3 {
    shifted(A, [1.0, 0.5, 0.25])
}

fn c() -> Box3 {
    shifted(A, [0.5, 1.0, 0.75])
}

fn unit_cube(min: [f64; 3]) -> Box3 {
    (min, [1.0, 1.0, 1.0])
}

// ── The independent union oracle ───────────────────────────────────────────

fn box_holds((min, size): Box3, p: [f64; 3]) -> bool {
    (0..3).all(|axis| p[axis] > min[axis] && p[axis] < min[axis] + size[axis])
}

/// Every cell of the grid the boxes' faces cut space into, plus a margin
/// cell on each side: `(centre, volume, index of the first box holding it)`.
///
/// Within one cell the answer to "which boxes hold this point" is constant,
/// so the cells holding a box sum to the exact union volume — the
/// inclusion–exclusion sum, evaluated without the alternating series — and
/// their centres probe every distinct region of the arrangement.
fn cells(boxes: &[Box3]) -> Vec<([f64; 3], f64, Option<usize>)> {
    let cuts: Vec<Vec<f64>> = (0..3)
        .map(|axis| {
            let mut values: Vec<f64> = boxes
                .iter()
                .flat_map(|(min, size)| [min[axis], min[axis] + size[axis]])
                .collect();
            values.sort_by(f64::total_cmp);
            values.dedup();
            let (low, high) = (values[0], values[values.len() - 1]);
            values.insert(0, low - 0.5);
            values.push(high + 0.5);
            values
        })
        .collect();
    let mut out = Vec::new();
    for x in cuts[0].windows(2) {
        for y in cuts[1].windows(2) {
            for z in cuts[2].windows(2) {
                let centre = [
                    f64::midpoint(x[0], x[1]),
                    f64::midpoint(y[0], y[1]),
                    f64::midpoint(z[0], z[1]),
                ];
                let volume = (x[1] - x[0]) * (y[1] - y[0]) * (z[1] - z[0]);
                let holder = boxes.iter().position(|&bx| box_holds(bx, centre));
                out.push((centre, volume, holder));
            }
        }
    }
    out
}

fn union_volume(boxes: &[Box3]) -> f64 {
    cells(boxes)
        .into_iter()
        .filter(|(_, _, holder)| holder.is_some())
        .map(|(_, volume, _)| volume)
        .sum()
}

#[test]
fn the_union_oracle_reproduces_the_hand_inclusion_exclusion_sum() {
    // |A| + |B| + |C| − |A∩B| − |A∩C| − |B∩C| + |A∩B∩C|
    //   = 24 − (2.625 + 1.875 + 3.375) + 1.25.
    assert_eq!(union_volume(&[A, b(), c()]), 17.375);
    assert_eq!(union_volume(&[A, b()]), 16.0 - 2.625);
    assert_eq!(union_volume(&[A, c()]), 16.0 - 1.875);
    assert_eq!(union_volume(&[b(), c()]), 16.0 - 3.375);
    assert_eq!(
        union_volume(&[unit_cube([0.0; 3]), unit_cube([0.5, 0.0, 0.0])]),
        1.5
    );
}

// ── Reading a result ───────────────────────────────────────────────────────

/// A face's outer boundary as an ordered polygon, and its plane.
fn face_polygon(topo: &Topology, face: FaceId) -> (Vec<Point3>, Vec3, f64) {
    let stored = topo.face(face).unwrap();
    let FaceSurface::Plane { normal, d } = *stored.surface() else {
        panic!("face {face:?} is not planar");
    };
    let points = topo
        .wire(stored.outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(|oriented| {
            let edge = topo.edge(oriented.edge()).unwrap();
            let start = if oriented.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            topo.vertex(start).unwrap().point()
        })
        .collect();
    (points, normal, d)
}

/// Whether `p` lies on the convex polygon `polygon` in the plane
/// `normal · x = d`, within `eps`.
fn on_convex_face(p: Point3, polygon: &[Point3], normal: Vec3, d: f64, eps: f64) -> bool {
    let origin = Vec3::new(p.x(), p.y(), p.z());
    if (normal.dot(origin) - d).abs() > eps {
        return false;
    }
    let mut sign = 0.0_f64;
    for (index, &from) in polygon.iter().enumerate() {
        let to = polygon[(index + 1) % polygon.len()];
        let edge = to - from;
        let side = normal.dot(edge.cross(p - from)) / edge.length();
        if side.abs() <= eps {
            continue;
        }
        if sign == 0.0 {
            sign = side.signum();
        } else if sign != side.signum() {
            return false;
        }
    }
    true
}

fn distance_to_segment(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}

fn edge_ends(index: usize, edges: &BTreeMap<usize, (Point3, Point3)>) -> (Point3, Point3) {
    *edges
        .get(&index)
        .unwrap_or_else(|| panic!("edge {index} is not an edge of the result or an operand"))
}

fn edge_table(topo: &Topology, solids: &[SolidId]) -> BTreeMap<usize, (Point3, Point3)> {
    let mut table = BTreeMap::new();
    for &solid in solids {
        for edge in solid_edges(topo, solid).unwrap() {
            let stored = topo.edge(edge).unwrap();
            table.insert(
                edge.index(),
                (
                    topo.vertex(stored.start()).unwrap().point(),
                    topo.vertex(stored.end()).unwrap().point(),
                ),
            );
        }
    }
    table
}

/// Mesh edges with an incidence other than two, after welding vertices by
/// position at `weld`.
fn open_mesh_edges(topo: &Topology, solid: SolidId, scale: f64) -> usize {
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01 * scale, 0.1).unwrap();
    assert!(!mesh.indices.is_empty(), "empty mesh");
    let weld = 1e-6 * scale;
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let remap: Vec<u32> = mesh
        .positions
        .iter()
        .map(|p| {
            let key = (
                (p.x() / weld).round() as i64,
                (p.y() / weld).round() as i64,
                (p.z() / weld).round() as i64,
            );
            let next = canon.len() as u32;
            *canon.entry(key).or_insert(next)
        })
        .collect();
    let mut uses: HashMap<(u32, u32), u32> = HashMap::new();
    for triangle in mesh.indices.chunks_exact(3) {
        let v = [
            remap[triangle[0] as usize],
            remap[triangle[1] as usize],
            remap[triangle[2] as usize],
        ];
        for (from, to) in [(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            *uses.entry((from.min(to), from.max(to))).or_insert(0) += 1;
        }
    }
    uses.values().filter(|&&count| count != 2).count()
}

/// What a witness must come out as.
struct Expect {
    /// The boxes of each region, in region order.
    regions: Vec<Vec<usize>>,
    /// One disposition per operand; `None` leaves that operand's disposition
    /// to the lineage oracle alone (it depends on which of two coincident
    /// faces the arrangement keeps, or on the placement).
    operands: Vec<Option<OperandDisposition>>,
}

/// Run one witness under one placement and hold the result to every oracle.
fn check(label: &str, boxes: &[Box3], placement: &Placement, expect: &Expect) -> NaryFuseResult {
    let label = format!("{label} [{}]", placement.name);
    let scale = placement.scale;
    let eps = 1e-9 * scale;
    let mut topo = Topology::new();
    let solids: Vec<SolidId> = boxes
        .iter()
        .map(|&bx| place(&mut topo, bx, placement))
        .collect();
    let operand_faces: Vec<BTreeSet<usize>> = solids
        .iter()
        .map(|&solid| {
            solid_faces(&topo, solid)
                .unwrap()
                .into_iter()
                .map(FaceId::index)
                .collect()
        })
        .collect();

    let result = fuse_regions(&mut topo, &solids).unwrap_or_else(|error| {
        panic!("{label}: fuse_regions refused: {error}");
    });

    // Shape of the answer.
    assert_eq!(
        result.regions.len(),
        expect.regions.len(),
        "{label}: region count"
    );
    assert_eq!(result.operands.len(), boxes.len(), "{label}: dispositions");
    let members = topo.compound(result.compound).unwrap().solids().to_vec();
    assert_eq!(
        members,
        result.regions.iter().map(|r| r.solid).collect::<Vec<_>>(),
        "{label}: compound order"
    );
    for (index, wanted) in expect.operands.iter().enumerate() {
        if let Some(wanted) = wanted {
            assert_eq!(&result.operands[index], wanted, "{label}: operand {index}");
        }
    }

    // Operands are untouched: still the boxes they were.
    for (index, &solid) in solids.iter().enumerate() {
        let (_, size) = boxes[index];
        let volume = size[0] * size[1] * size[2] * scale.powi(3);
        let measured = solid_volume(&topo, solid, 0.01 * scale).unwrap();
        assert!(
            (measured - volume).abs() <= 1e-9 * volume,
            "{label}: operand {index} changed volume {measured} vs {volume}"
        );
        assert_eq!(solid_faces(&topo, solid).unwrap().len(), 6);
    }

    let mut claimed_faces = BTreeSet::new();
    for (position, region) in result.regions.iter().enumerate() {
        let region_label = format!("{label}: region {position}");
        let region_boxes: Vec<Box3> = expect.regions[position]
            .iter()
            .map(|&index| boxes[index])
            .collect();

        // Volume: the exact union of this region's boxes.
        let expected = union_volume(&region_boxes) * scale.powi(3);
        let measured = solid_volume(&topo, region.solid, 0.01 * scale).unwrap();
        assert!(
            (measured - expected).abs() <= 1e-9 * expected,
            "{region_label}: volume {measured} vs {expected}"
        );

        // Structure: strict validation, edge use, vertex links by position.
        assert!(
            validate_solid(&topo, region.solid).unwrap().is_valid(),
            "{region_label}: validation"
        );
        let faces = solid_faces(&topo, region.solid).unwrap();
        let mut edge_uses: BTreeMap<usize, usize> = BTreeMap::new();
        let mut position_uses: BTreeMap<[(i64, i64, i64); 2], usize> = BTreeMap::new();
        let quantize = |p: Point3| {
            let grid = 1e-6 * scale;
            (
                (p.x() / grid).round() as i64,
                (p.y() / grid).round() as i64,
                (p.z() / grid).round() as i64,
            )
        };
        for &face in &faces {
            let stored = topo.face(face).unwrap();
            for &wire in std::iter::once(&stored.outer_wire()).chain(stored.inner_wires()) {
                for oriented in topo.wire(wire).unwrap().edges() {
                    *edge_uses.entry(oriented.edge().index()).or_insert(0) += 1;
                    let edge = topo.edge(oriented.edge()).unwrap();
                    let mut key = [
                        quantize(topo.vertex(edge.start()).unwrap().point()),
                        quantize(topo.vertex(edge.end()).unwrap().point()),
                    ];
                    key.sort_unstable();
                    assert_ne!(key[0], key[1], "{region_label}: degenerate edge");
                    *position_uses.entry(key).or_insert(0) += 1;
                }
            }
        }
        assert!(
            edge_uses.values().all(|&uses| uses == 2),
            "{region_label}: an edge is not used exactly twice"
        );
        assert!(
            position_uses.values().all(|&uses| uses == 2),
            "{region_label}: a segment is not used exactly twice by position"
        );

        // Mesh: closed after welding by position.
        assert_eq!(
            open_mesh_edges(&topo, region.solid, scale),
            0,
            "{region_label}: mesh is not watertight"
        );

        // Lineage, faces: one claim per result face, by a face of the named
        // operand, and the result face lies on that face.
        assert_eq!(region.face_operands.len(), region.evolution.faces.len());
        let result_faces: BTreeSet<usize> = faces.iter().map(|face| face.index()).collect();
        let claims: BTreeSet<usize> = region.evolution.faces.iter().map(|&(f, _)| f).collect();
        assert_eq!(claims, result_faces, "{region_label}: face claims");
        assert_eq!(claims.len(), region.evolution.faces.len());
        let mut contributing = BTreeSet::new();
        for (&(face, source), &operand) in region.evolution.faces.iter().zip(&region.face_operands)
        {
            assert!(
                claimed_faces.insert(face),
                "{region_label}: face {face} claimed twice"
            );
            let source = source.unwrap_or_else(|| panic!("{region_label}: face {face} unsourced"));
            assert!(
                operand_faces[operand].contains(&source),
                "{region_label}: face {face} names face {source}, not of operand {operand}"
            );
            assert!(
                expect.regions[position].contains(&operand),
                "{region_label}: operand {operand} is not a box of this region"
            );
            contributing.insert(operand);
            let result_id = faces.iter().copied().find(|f| f.index() == face).unwrap();
            let source_id = solid_faces(&topo, solids[operand])
                .unwrap()
                .into_iter()
                .find(|f| f.index() == source)
                .unwrap();
            let (source_polygon, source_normal, source_d) = face_polygon(&topo, source_id);
            let (result_polygon, result_normal, _) = face_polygon(&topo, result_id);
            assert!(
                (result_normal - source_normal).length() <= 1e-9,
                "{region_label}: face {face} does not face the way its source does"
            );
            for point in result_polygon {
                assert!(
                    on_convex_face(point, &source_polygon, source_normal, source_d, eps),
                    "{region_label}: face {face} leaves its source face {source}"
                );
            }
        }
        assert_eq!(
            region.operands,
            contributing.iter().copied().collect::<Vec<_>>(),
            "{region_label}: contributing operands"
        );

        // Lineage, edges and vertices: every event read back as geometry.
        let mut all = solids.clone();
        all.push(region.solid);
        let edges = edge_table(&topo, &all);
        let result_edges: BTreeSet<usize> = solid_edges(&topo, region.solid)
            .unwrap()
            .into_iter()
            .map(remus_topology::EdgeId::index)
            .collect();
        assert_eq!(
            region
                .evolution
                .edges
                .iter()
                .map(|(e, _)| *e)
                .collect::<BTreeSet<_>>(),
            result_edges,
            "{region_label}: edge history is not total"
        );
        let owner_of_face = |face: usize| operand_faces.iter().position(|set| set.contains(&face));
        for (edge, event) in &region.evolution.edges {
            let (from, to) = edge_ends(*edge, &edges);
            match event {
                EdgeEvent::Preserved(source) => {
                    let (a, b) = edge_ends(*source, &edges);
                    let same = (from - a).length() <= eps && (to - b).length() <= eps;
                    let flipped = (from - b).length() <= eps && (to - a).length() <= eps;
                    assert!(
                        same || flipped,
                        "{region_label}: edge {edge} is not edge {source}"
                    );
                }
                EdgeEvent::Modified(source) => {
                    let (a, b) = edge_ends(*source, &edges);
                    assert!(
                        distance_to_segment(from, a, b) <= eps
                            && distance_to_segment(to, a, b) <= eps,
                        "{region_label}: edge {edge} leaves its parent {source}"
                    );
                }
                EdgeEvent::Generated { face_a, face_b } => {
                    let (face_a, face_b) = (face_a.unwrap(), face_b.unwrap());
                    let (owner_a, owner_b) = (
                        owner_of_face(face_a).unwrap(),
                        owner_of_face(face_b).unwrap(),
                    );
                    assert_ne!(
                        owner_a, owner_b,
                        "{region_label}: edge {edge} from one operand"
                    );
                    for (face, owner) in [(face_a, owner_a), (face_b, owner_b)] {
                        let id = solid_faces(&topo, solids[owner])
                            .unwrap()
                            .into_iter()
                            .find(|f| f.index() == face)
                            .unwrap();
                        let (polygon, normal, d) = face_polygon(&topo, id);
                        for point in [from, to] {
                            assert!(
                                on_convex_face(point, &polygon, normal, d, eps),
                                "{region_label}: section edge {edge} is off face {face}"
                            );
                        }
                    }
                }
                EdgeEvent::Unresolved => panic!("{region_label}: edge {edge} unresolved"),
            }
        }
        let result_vertices: BTreeMap<usize, Point3> = solid_vertices(&topo, region.solid)
            .unwrap()
            .into_iter()
            .map(|vertex| (vertex.index(), topo.vertex(vertex).unwrap().point()))
            .collect();
        assert_eq!(
            region
                .evolution
                .vertices
                .iter()
                .map(|(v, _)| *v)
                .collect::<BTreeSet<_>>(),
            result_vertices.keys().copied().collect::<BTreeSet<_>>(),
            "{region_label}: vertex history is not total"
        );
        let operand_vertices: BTreeMap<usize, Point3> = solids
            .iter()
            .flat_map(|&solid| solid_vertices(&topo, solid).unwrap())
            .map(|vertex| (vertex.index(), topo.vertex(vertex).unwrap().point()))
            .collect();
        for (vertex, event) in &region.evolution.vertices {
            let at = result_vertices[vertex];
            match event {
                VertexEvent::Preserved(source) => assert!(
                    (at - operand_vertices[source]).length() <= eps,
                    "{region_label}: vertex {vertex} is not vertex {source}"
                ),
                // `Created` says only that the vertex is no copy of an
                // operand vertex; a corner the splitter rebuilt in place
                // reports it too, so its position is not held to anything.
                VertexEvent::Created => {}
            }
        }
    }

    // Material: every cell of the arrangement is inside exactly the region
    // its boxes belong to, or outside all of them.
    let region_of_box: BTreeMap<usize, usize> = expect
        .regions
        .iter()
        .enumerate()
        .flat_map(|(position, members)| members.iter().map(move |&index| (index, position)))
        .collect();
    for (centre, _, holder) in cells(boxes) {
        let point = placement.world(centre);
        let wanted = holder.map(|index| region_of_box[&index]);
        for (position, region) in result.regions.iter().enumerate() {
            let class = classify_point(&topo, region.solid, point, 0.01 * scale, 1e-7).unwrap();
            let inside = wanted == Some(position);
            assert_eq!(
                class,
                if inside {
                    PointClassification::Inside
                } else {
                    PointClassification::Outside
                },
                "{label}: probe {centre:?} against region {position}"
            );
        }
    }

    // Dispositions agree with the lineage they summarise.
    for (index, disposition) in result.operands.iter().enumerate() {
        let with_faces: Vec<usize> = result
            .regions
            .iter()
            .enumerate()
            .filter(|(_, region)| region.operands.contains(&index))
            .map(|(position, _)| position)
            .collect();
        match disposition {
            OperandDisposition::Preserved { region } => {
                assert_eq!(with_faces, vec![*region], "{label}: operand {index}");
                assert_eq!(result.regions[*region].solid, solids[index]);
                assert_eq!(result.regions[*region].operands, vec![index]);
            }
            OperandDisposition::Contributes { regions } => {
                assert_eq!(&with_faces, regions, "{label}: operand {index}");
                assert!(!regions.is_empty());
            }
            OperandDisposition::Absorbed { regions } => {
                assert!(with_faces.is_empty(), "{label}: operand {index} has faces");
                assert_eq!(
                    regions,
                    &vec![region_of_box[&index]],
                    "{label}: operand {index}"
                );
            }
        }
    }
    result
}

/// An expected `Contributes`, as an [`Expect::operands`] slot.
#[allow(clippy::unnecessary_wraps)]
fn contributes(regions: &[usize]) -> Option<OperandDisposition> {
    Some(OperandDisposition::Contributes {
        regions: regions.to_vec(),
    })
}

// ── Witnesses ──────────────────────────────────────────────────────────────

#[test]
fn two_overlapping_boxes_fuse_into_one_region() {
    for placement in placements() {
        check(
            "two boxes",
            &[A, b()],
            &placement,
            &Expect {
                regions: vec![vec![0, 1]],
                operands: vec![contributes(&[0]), contributes(&[0])],
            },
        );
    }
}

/// W-a: three boxes with no coplanar faces. One region of volume 17.375,
/// every result face claimed once, all three operands contributing.
#[test]
fn three_mutually_intersecting_boxes_fuse_into_one_region() {
    for placement in placements() {
        let result = check(
            "W-a",
            &[A, b(), c()],
            &placement,
            &Expect {
                regions: vec![vec![0, 1, 2]],
                operands: vec![contributes(&[0]), contributes(&[0]), contributes(&[0])],
            },
        );
        assert_eq!(result.regions[0].operands, vec![0, 1, 2]);
    }
}

/// W-b: unit cubes at x = 0, 0.5, 1 share four planes. Volume 2.
#[test]
fn a_coplanar_row_of_overlapping_cubes_fuses_into_one_region() {
    for placement in placements() {
        check(
            "W-b",
            &[
                unit_cube([0.0, 0.0, 0.0]),
                unit_cube([0.5, 0.0, 0.0]),
                unit_cube([1.0, 0.0, 0.0]),
            ],
            &placement,
            &Expect {
                regions: vec![vec![0, 1, 2]],
                // The middle cube lies inside the other two together; which
                // of the coincident side faces survive is the arrangement's
                // choice, and the lineage oracle holds either answer.
                operands: vec![contributes(&[0]), None, contributes(&[0])],
            },
        );
    }
}

/// W-c: unit cubes at x = 0, 1, 2 meet face to face. Volume 3.
#[test]
fn abutting_cubes_fuse_into_one_region() {
    for placement in placements() {
        check(
            "W-c",
            &[
                unit_cube([0.0, 0.0, 0.0]),
                unit_cube([1.0, 0.0, 0.0]),
                unit_cube([2.0, 0.0, 0.0]),
            ],
            &placement,
            &Expect {
                regions: vec![vec![0, 1, 2]],
                operands: vec![contributes(&[0]), contributes(&[0]), contributes(&[0])],
            },
        );
    }
}

/// Two cubes meeting over part of a face: the contact is a rectangle strictly
/// inside both faces.
#[test]
fn cubes_meeting_over_part_of_a_face_fuse_into_one_region() {
    for placement in placements() {
        check(
            "partial face contact",
            &[unit_cube([0.0, 0.0, 0.0]), unit_cube([1.0, 0.5, 0.25])],
            &placement,
            &Expect {
                regions: vec![vec![0, 1]],
                operands: vec![contributes(&[0]), contributes(&[0])],
            },
        );
    }
}

const FAR: Box3 = ([40.0, 30.0, 20.0], [1.0, 2.0, 3.0]);
const SMALL: Box3 = ([0.25, 0.25, 0.25], [0.5, 0.5, 0.5]);

/// W-d: the three boxes and one far away. Two regions; the far one is the
/// operand itself with identity lineage.
#[test]
fn a_far_operand_is_its_own_region_with_identity_lineage() {
    for placement in placements() {
        let result = check(
            "W-d",
            &[A, b(), c(), FAR],
            &placement,
            &Expect {
                regions: vec![vec![0, 1, 2], vec![3]],
                operands: vec![
                    contributes(&[0]),
                    contributes(&[0]),
                    contributes(&[0]),
                    Some(OperandDisposition::Preserved { region: 1 }),
                ],
            },
        );
        let far = &result.regions[1];
        assert!(
            far.evolution
                .faces
                .iter()
                .all(|&(face, source)| source == Some(face))
        );
        assert!(
            far.evolution.edges.iter().all(
                |(edge, event)| matches!(event, EdgeEvent::Preserved(source) if source == edge)
            )
        );
        assert!(far.evolution.vertices.iter().all(
            |(vertex, event)| matches!(event, VertexEvent::Preserved(source) if source == vertex)
        ));
    }
}

/// W-e: a small box inside the first. It leaves no face and the volume is
/// that of the three boxes alone.
#[test]
fn an_enclosed_operand_is_absorbed_and_changes_nothing() {
    for placement in placements() {
        assert_eq!(union_volume(&[A, b(), c(), SMALL]), 17.375);
        check(
            "W-e",
            &[A, b(), c(), SMALL],
            &placement,
            &Expect {
                regions: vec![vec![0, 1, 2, 3]],
                operands: vec![
                    contributes(&[0]),
                    contributes(&[0]),
                    contributes(&[0]),
                    Some(OperandDisposition::Absorbed { regions: vec![0] }),
                ],
            },
        );
    }
}

/// Five operands: the three boxes, the far one and the enclosed one, in an
/// order that puts the far box first — so region order follows input order.
#[test]
fn five_operands_keep_input_region_order_and_every_disposition() {
    for placement in placements() {
        check(
            "five mixed",
            &[FAR, c(), SMALL, A, b()],
            &placement,
            &Expect {
                regions: vec![vec![0], vec![1, 2, 3, 4]],
                operands: vec![
                    Some(OperandDisposition::Preserved { region: 0 }),
                    contributes(&[1]),
                    Some(OperandDisposition::Absorbed { regions: vec![1] }),
                    contributes(&[1]),
                    contributes(&[1]),
                ],
            },
        );
    }
}

/// Five boxes overlapping in one connected body, no two faces coplanar.
#[test]
fn five_mutually_overlapping_boxes_fuse_into_one_region() {
    let boxes = [
        A,
        b(),
        c(),
        ([1.25, 1.75, 1.5], [2.0, 2.0, 2.0]),
        ([-1.0, -0.75, 0.4], [1.5, 2.0, 1.5]),
    ];
    for placement in placements() {
        let result = check(
            "five overlapping",
            &boxes,
            &placement,
            &Expect {
                regions: vec![vec![0, 1, 2, 3, 4]],
                operands: (0..5).map(|_| contributes(&[0])).collect(),
            },
        );
        assert_eq!(result.regions[0].operands, vec![0, 1, 2, 3, 4]);
    }
}

/// Eight boxes staggered along a diagonal, no two faces coplanar: each
/// overlaps its next two neighbours, so most edges are cut more than once.
/// This is the operand layout of the `fuse_regions` bench.
#[test]
fn a_staggered_chain_of_eight_boxes_fuses_into_one_region() {
    let boxes: Vec<Box3> = (0..8)
        .map(|index| {
            let step = f64::from(index);
            ([0.875 * step, 0.375 * step, 0.25 * step], [2.0, 2.0, 2.0])
        })
        .collect();
    for placement in placements() {
        let result = check(
            "staggered chain",
            &boxes,
            &placement,
            &Expect {
                regions: vec![(0..8).collect()],
                operands: (0..8).map(|_| contributes(&[0])).collect(),
            },
        );
        assert_eq!(result.regions[0].operands, (0..8).collect::<Vec<_>>());
    }
}

/// Two distinct solids occupying the same box, and a third overlapping both.
/// Every face of the twins is coincident and same-oriented, so one of them
/// supplies the faces and the other is absorbed: no face of it survives,
/// and the union is the union of one twin with the third box.
#[test]
fn a_geometric_twin_is_absorbed_by_its_double() {
    for placement in placements() {
        let result = check(
            "twins",
            &[A, A, b()],
            &placement,
            &Expect {
                regions: vec![vec![0, 1, 2]],
                operands: vec![None, None, contributes(&[0])],
            },
        );
        let twins = [&result.operands[0], &result.operands[1]];
        let absorbed = twins
            .iter()
            .filter(|d| ***d == OperandDisposition::Absorbed { regions: vec![0] })
            .count();
        let contributing = twins
            .iter()
            .filter(|d| ***d == OperandDisposition::Contributes { regions: vec![0] })
            .count();
        assert_eq!(
            (absorbed, contributing),
            (1, 1),
            "[{}]: {twins:?}",
            placement.name
        );
    }
}

/// Two cubes a quarter apart. Axis-aligned their boxes are clear and each is
/// preserved; turned, the boxes overlap, the pair is fused in one arrangement
/// and comes back as two regions that do not touch.
#[test]
fn near_but_separate_cubes_stay_two_regions() {
    for placement in placements() {
        let result = check(
            "near gap",
            &[unit_cube([0.0, 0.0, 0.0]), unit_cube([1.25, 0.0, 0.0])],
            &placement,
            &Expect {
                regions: vec![vec![0], vec![1]],
                operands: vec![None, None],
            },
        );
        for (index, disposition) in result.operands.iter().enumerate() {
            assert!(
                matches!(
                    disposition,
                    OperandDisposition::Preserved { region } if *region == index
                ) || *disposition
                    == OperandDisposition::Contributes {
                        regions: vec![index]
                    },
                "{}: operand {index} is {disposition:?}",
                placement.name
            );
        }
    }
}

#[test]
fn the_result_is_deterministic() {
    let run = || {
        let placement = placements()[2];
        let mut topo = Topology::new();
        let solids: Vec<SolidId> = [A, b(), c(), FAR, SMALL]
            .iter()
            .map(|&bx| place(&mut topo, bx, &placement))
            .collect();
        let result = fuse_regions(&mut topo, &solids).unwrap();
        (result.regions, result.operands)
    };
    assert_eq!(run(), run());
}

/// A curved operand that can touch nothing is outside every arrangement: it
/// is preserved, not refused.
#[test]
fn a_far_curved_operand_is_preserved() {
    let mut topo = Topology::new();
    let placement = placements()[0];
    let first = place(&mut topo, A, &placement);
    let second = place(&mut topo, b(), &placement);
    let cylinder = make_cylinder(&mut topo, 1.0, 3.0).unwrap();
    transform_solid(&mut topo, cylinder, &Mat4::translation(50.0, 0.0, 0.0)).unwrap();

    let result = fuse_regions(&mut topo, &[first, cylinder, second]).unwrap();
    assert_eq!(result.regions.len(), 2);
    assert_eq!(result.regions[1].solid, cylinder);
    assert_eq!(
        result.operands,
        vec![
            OperandDisposition::Contributes { regions: vec![0] },
            OperandDisposition::Preserved { region: 1 },
            OperandDisposition::Contributes { regions: vec![0] },
        ]
    );
    let volume = solid_volume(&topo, result.regions[0].solid, 0.01).unwrap();
    assert!((volume - 13.375).abs() <= 1e-9);
}

// ── Refusals ───────────────────────────────────────────────────────────────

/// Everything a rollback must restore: the slot high-water mark and the live
/// entity counts.
fn footprint(topo: &Topology) -> (usize, [usize; 6]) {
    (
        topo.allocated_slot_count(),
        [
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        ],
    )
}

fn refusal(error: &OperationsError) -> NaryFuseRefusal {
    match error {
        OperationsError::Algo(AlgoError::UnsupportedNaryFuse { cause, .. }) => *cause,
        other => panic!("expected an N-ary fuse refusal, got {other:?}"),
    }
}

/// `fuse_regions` must refuse with `cause` and leave `topo` as it was,
/// operands included.
fn assert_refused(topo: &mut Topology, solids: &[SolidId], cause: NaryFuseRefusal) {
    let before = footprint(topo);
    let image = format!("{:?}", topo.to_image());
    let next_handles = next_box_handles(topo);
    let volumes: Vec<f64> = solids
        .iter()
        .map(|&solid| solid_volume(topo, solid, 0.01).unwrap())
        .collect();
    let error = fuse_regions(topo, solids).unwrap_err();
    assert_eq!(refusal(&error), cause, "{error}");
    assert_eq!(footprint(topo), before, "refusal changed the topology");
    assert_eq!(
        next_box_handles(topo),
        next_handles,
        "refusal consumed handles"
    );
    assert_eq!(format!("{:?}", topo.to_image()), image);
    for (&solid, &volume) in solids.iter().zip(&volumes) {
        assert_eq!(solid_volume(topo, solid, 0.01).unwrap(), volume);
        assert!(validate_solid(topo, solid).unwrap().is_valid());
    }
}

/// Actually allocate another box on a copy, so every boundary arena's next
/// handle is checked without changing the refusal witness.
fn next_box_handles(topo: &Topology) -> Vec<usize> {
    let mut copy = topo.clone();
    let solid = make_box(&mut copy, 1.0, 1.0, 1.0).unwrap();
    let mut handles = vec![
        solid.index(),
        copy.solid(solid).unwrap().outer_shell().index(),
    ];
    handles.extend(
        solid_vertices(&copy, solid)
            .unwrap()
            .iter()
            .map(|id| id.index()),
    );
    handles.extend(
        solid_edges(&copy, solid)
            .unwrap()
            .iter()
            .map(|id| id.index()),
    );
    for face in solid_faces(&copy, solid).unwrap() {
        handles.extend([face.index(), copy.face(face).unwrap().outer_wire().index()]);
        for &loop_id in copy.loops_of_face(face).unwrap() {
            handles.push(loop_id.index());
            handles.extend(
                copy.face_loop(loop_id)
                    .unwrap()
                    .coedges()
                    .iter()
                    .map(|id| id.index()),
            );
        }
    }
    handles
}

#[test]
fn a_later_vertex_contact_refuses_without_consuming_slots_or_handles() {
    let mut topo = Topology::new();
    let solids: Vec<_> = [
        A,
        b(),
        unit_cube([50.0, 0.0, 0.0]),
        unit_cube([51.0, 1.0, 1.0]),
    ]
    .into_iter()
    .map(|bx| place(&mut topo, bx, &placements()[0]))
    .collect();
    assert_eq!(topo.allocated_slot_count(), 256);
    assert_refused(&mut topo, &solids, NaryFuseRefusal::NonManifoldContact);
}

#[test]
fn a_later_curved_group_refuses_without_consuming_slots_or_handles() {
    let mut topo = Topology::new();
    let first = place(&mut topo, A, &placements()[0]);
    let second = place(&mut topo, b(), &placements()[0]);
    let block = place(&mut topo, shifted(A, [50.0, 0.0, 0.0]), &placements()[0]);
    let cylinder = make_cylinder(&mut topo, 1.0, 3.0).unwrap();
    transform_solid(&mut topo, cylinder, &Mat4::translation(50.0, 0.0, 0.0)).unwrap();
    assert_eq!(topo.allocated_slot_count(), 214);
    assert_refused(
        &mut topo,
        &[first, second, block, cylinder],
        NaryFuseRefusal::CurvedCluster,
    );
}

/// Separate group calls reproduce the original eager export order. Hold the
/// combined call to their exact handles and entity evolution as well as the
/// closed-form volumes and boundary counts.
#[test]
fn multiple_accepted_groups_keep_geometry_lineage_and_input_order() {
    let mut topo = Topology::new();
    let boxes = [
        A,
        FAR,
        shifted(A, [50.0, 0.0, 0.0]),
        b(),
        shifted(b(), [50.0, 0.0, 0.0]),
        SMALL,
    ];
    let solids: Vec<_> = boxes
        .into_iter()
        .map(|bx| place(&mut topo, bx, &placements()[0]))
        .collect();
    let mut separate = topo.clone();
    let first = fuse_regions(&mut separate, &[solids[0], solids[3], solids[5]]).unwrap();
    let second = fuse_regions(&mut separate, &[solids[2], solids[4]]).unwrap();
    let result = fuse_regions(&mut topo, &solids).unwrap();
    assert_eq!(result.regions.len(), 3);
    for (position, expected) in [(0, &first.regions[0]), (2, &second.regions[0])] {
        let region = &result.regions[position];
        assert_eq!(region.solid, expected.solid);
        assert_eq!(region.evolution, expected.evolution);
        assert!((solid_volume(&topo, region.solid, 0.01).unwrap() - 13.375).abs() < 1e-9);
        assert_eq!(solid_faces(&topo, region.solid).unwrap().len(), 12);
        assert_eq!(solid_edges(&topo, region.solid).unwrap().len(), 30);
    }
    assert_eq!(result.regions[1].solid, solids[1]);
    assert!((solid_volume(&topo, solids[1], 0.01).unwrap() - 6.0).abs() < 1e-9);
    assert_eq!(solid_faces(&topo, solids[1]).unwrap().len(), 6);
    assert_eq!(solid_edges(&topo, solids[1]).unwrap().len(), 12);
    assert_eq!(result.regions[0].operands, vec![0, 3]);
    assert_eq!(result.regions[2].operands, vec![2, 4]);
    assert_eq!(
        result.regions[0].face_operands,
        first.regions[0]
            .face_operands
            .iter()
            .map(|&i| [0, 3, 5][i])
            .collect::<Vec<_>>()
    );
    assert_eq!(
        result.regions[2].face_operands,
        second.regions[0]
            .face_operands
            .iter()
            .map(|&i| [2, 4][i])
            .collect::<Vec<_>>()
    );
    assert_eq!(
        result.operands,
        vec![
            OperandDisposition::Contributes { regions: vec![0] },
            OperandDisposition::Preserved { region: 1 },
            OperandDisposition::Contributes { regions: vec![2] },
            OperandDisposition::Contributes { regions: vec![0] },
            OperandDisposition::Contributes { regions: vec![2] },
            OperandDisposition::Absorbed { regions: vec![0] },
        ]
    );
}

#[test]
fn an_overlapping_cylinder_is_refused_as_a_curved_cluster() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let cylinder = make_cylinder(&mut topo, 1.0, 3.0).unwrap();
    assert_refused(
        &mut topo,
        &[block, cylinder],
        NaryFuseRefusal::CurvedCluster,
    );
    // A third, planar operand elsewhere does not rescue the cluster.
    let far = place(&mut topo, FAR, &placements()[0]);
    assert_refused(
        &mut topo,
        &[far, block, cylinder],
        NaryFuseRefusal::CurvedCluster,
    );
}

#[test]
fn the_same_handle_twice_is_refused() {
    let mut topo = Topology::new();
    let placement = placements()[0];
    let first = place(&mut topo, A, &placement);
    let second = place(&mut topo, b(), &placement);
    assert_refused(
        &mut topo,
        &[first, second, first],
        NaryFuseRefusal::DuplicateOperand,
    );
    assert_refused(
        &mut topo,
        &[first, first],
        NaryFuseRefusal::DuplicateOperand,
    );
}

#[test]
fn fewer_than_two_operands_are_refused() {
    let mut topo = Topology::new();
    let only = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    assert_refused(&mut topo, &[only], NaryFuseRefusal::TooFewOperands);
    assert_refused(&mut topo, &[], NaryFuseRefusal::TooFewOperands);
}

#[test]
fn a_dead_handle_is_an_error_and_changes_nothing() {
    let mut topo = Topology::new();
    let kept = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let dead = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    topo.delete_solid(dead).unwrap();
    let before = footprint(&topo);
    let error = fuse_regions(&mut topo, &[kept, dead]).unwrap_err();
    assert!(matches!(error, OperationsError::Topology(_)), "{error:?}");
    assert_eq!(footprint(&topo), before);
}

/// Cubes sharing only an edge, and cubes sharing only a corner: the union is
/// one body pinched along a line or at a point, which is not a solid.
#[test]
fn edge_only_and_vertex_only_contact_are_refused() {
    for placement in placements() {
        for (label, second) in [
            ("edge", unit_cube([1.0, 1.0, 0.0])),
            ("vertex", unit_cube([1.0, 1.0, 1.0])),
        ] {
            let mut topo = Topology::new();
            let first = place(&mut topo, unit_cube([0.0, 0.0, 0.0]), &placement);
            let second = place(&mut topo, second, &placement);
            let before = footprint(&topo);
            let error = fuse_regions(&mut topo, &[first, second]).unwrap_err();
            assert_eq!(
                refusal(&error),
                NaryFuseRefusal::NonManifoldContact,
                "{label} contact [{}]: {error}",
                placement.name
            );
            assert_eq!(
                footprint(&topo),
                before,
                "{label} contact [{}]",
                placement.name
            );
        }
    }
}

/// The boundary of this slice's lineage. Nine slabs on a 3×3 unit grid, each
/// a hundredth wider than its cell, share their top and bottom planes, so
/// the union's top is a patchwork of coplanar pieces from different
/// operands. A seam between two such pieces that the arrangement cut more
/// than once carries no construction record, and two faces of different
/// operands share no input edge that could name its parent by incidence.
/// The fuse is refused whole rather than returned with guessed lineage.
/// (Turned, the same layout does not get that far: the arrangement fails to
/// assemble, which is refused as an unqualified result.)
#[test]
fn coplanar_seams_without_a_construction_record_are_refused() {
    let boxes: Vec<Box3> = (0..9)
        .map(|index| {
            let (row, column) = (index / 3, index % 3);
            ([f64::from(column), f64::from(row), 0.0], [1.01, 1.01, 1.0])
        })
        .collect();
    for placement in placements() {
        let mut topo = Topology::new();
        let solids: Vec<SolidId> = boxes
            .iter()
            .map(|&bx| place(&mut topo, bx, &placement))
            .collect();
        let before = footprint(&topo);
        let error = fuse_regions(&mut topo, &solids).unwrap_err();
        let cause = refusal(&error);
        if placement.name == "aligned" {
            assert_eq!(cause, NaryFuseRefusal::LineageUnresolved, "{error}");
        } else {
            assert!(
                matches!(
                    cause,
                    NaryFuseRefusal::LineageUnresolved | NaryFuseRefusal::ResultUnqualified
                ),
                "[{}]: {error}",
                placement.name
            );
        }
        assert_eq!(footprint(&topo), before, "[{}]", placement.name);
        // The single-solid fuse of the same slabs is unaffected. (At 1e-3
        // turned it declines these slabs with `ExactOnlyUnattainable`, as it
        // did before this entry point existed — measured on 24040dda — so
        // that placement has no single solid to compare.)
        if placement.name == "1e-3 turned" {
            continue;
        }
        let fused = remus_operations::compound_ops::fuse_solids(&mut topo, &solids).unwrap();
        let volume = solid_volume(&topo, fused, 0.01 * placement.scale).unwrap();
        let expected = 3.01 * 3.01 * placement.scale.powi(3);
        assert!(
            (volume - expected).abs() <= 1e-9 * expected,
            "{volume} vs {expected}"
        );
    }
}

/// Ninety-two boxes in one place have 4186 touching pairs, over the 4096 one
/// arrangement accepts. Refused before any arrangement is built.
#[test]
fn more_touching_pairs_than_the_budget_are_refused() {
    let mut topo = Topology::new();
    let solids: Vec<SolidId> = (0..92)
        .map(|_| make_box(&mut topo, 1.0, 1.0, 1.0).unwrap())
        .collect();
    assert_eq!(solids.len() * (solids.len() - 1) / 2, 4186);
    let before = footprint(&topo);
    let error = fuse_regions(&mut topo, &solids).unwrap_err();
    assert_eq!(refusal(&error), NaryFuseRefusal::PairBudgetExceeded);
    assert_eq!(footprint(&topo), before);
}

#[test]
fn a_sheet_or_wire_operand_is_refused_by_class() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let other = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(&mut topo, other, &Mat4::translation(1.0, 0.5, 0.25)).unwrap();
    let donor = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let donor_face = solid_faces(&topo, donor).unwrap()[0];
    let sheet = remus_operations::sew::make_sheet_body(&mut topo, &[donor_face]).unwrap();
    let wire = topo.face(donor_face).unwrap().outer_wire();
    let free_wire = {
        let copy = topo.wire(wire).unwrap().clone();
        let id = topo.add_wire(copy);
        topo.set_wire_body_class(id, remus_topology::BodyClass::Wire)
            .unwrap();
        id
    };

    for (bodies, class) in [
        (vec![BodyId::Solid(block), BodyId::Shell(sheet)], "sheet"),
        (vec![BodyId::Solid(block), BodyId::Wire(free_wire)], "wire"),
        (
            vec![
                BodyId::Solid(block),
                BodyId::Solid(other),
                BodyId::Wire(free_wire),
                BodyId::Shell(sheet),
            ],
            "wire",
        ),
    ] {
        let before = footprint(&topo);
        let error = fuse_body_regions(&mut topo, &bodies).unwrap_err();
        assert!(
            matches!(
                error,
                OperationsError::BodyClassOperationUnsupported {
                    operation: "fuse_regions",
                    actual,
                } if actual == class
            ),
            "{class}: {error:?}"
        );
        assert_eq!(footprint(&topo), before, "{class}");
    }

    // An all-solid body list is the solid entry point.
    let through_bodies =
        fuse_body_regions(&mut topo, &[BodyId::Solid(block), BodyId::Solid(other)]).unwrap();
    assert_eq!(through_bodies.regions.len(), 1);
    let volume = solid_volume(&topo, through_bodies.regions[0].solid, 0.01).unwrap();
    assert!((volume - 13.375).abs() <= 1e-9);

    // A shell that bounds a solid is not a body at all.
    let owned_shell = topo.solid(block).unwrap().outer_shell();
    let before = footprint(&topo);
    let error = fuse_body_regions(
        &mut topo,
        &[BodyId::Solid(other), BodyId::Shell(owned_shell)],
    )
    .unwrap_err();
    assert!(
        matches!(error, OperationsError::InvalidInput { .. }),
        "{error:?}"
    );
    assert_eq!(footprint(&topo), before);
}

/// `fuse_solids` is unchanged by this entry point: it still returns one solid
/// for the same operands, of the same volume.
#[test]
fn fuse_solids_still_returns_one_solid_for_the_same_operands() {
    let placement = placements()[0];
    let mut topo = Topology::new();
    let solids: Vec<SolidId> = [A, b(), c()]
        .iter()
        .map(|&bx| place(&mut topo, bx, &placement))
        .collect();
    let fused = remus_operations::compound_ops::fuse_solids(&mut topo, &solids).unwrap();
    let volume = solid_volume(&topo, fused, 0.01).unwrap();
    assert!((volume - 17.375).abs() <= 1e-9, "{volume}");
}

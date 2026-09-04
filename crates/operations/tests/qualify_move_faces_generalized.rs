//! P-Class 6.2 qualification for exact generalized face movement.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used
)]

use std::collections::{BTreeSet, HashMap};

use remus_algo::PlaneFrame;
use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::evolution::EvolutionMap;
use remus_operations::journal_ops::{begin_scoped, move_faces_journaled, record_face_evolution};
use remus_operations::measure::{face_area, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::move_faces_with_evolution;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{
    edge_to_face_map, face_edges, face_vertices, solid_edges, solid_entity_counts, solid_faces,
};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::{EntityKey, EntityKind};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};
use remus_topology::solid::SolidId;

fn placed_box(topo: &mut Topology, size: [f64; 3], translation: [f64; 3]) -> SolidId {
    let solid = make_box(topo, size[0], size[1], size[2]).expect("box");
    transform_solid(
        topo,
        solid,
        &Mat4::translation(translation[0], translation[1], translation[2]),
    )
    .expect("place box");
    solid
}

fn line_edge_at(topo: &Topology, solid: SolidId, y: f64, z: f64, scale: f64) -> EdgeId {
    let tolerance = scale.mul_add(1e-8, 1e-11);
    solid_edges(topo, solid)
        .expect("solid edges")
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).expect("edge");
            if !matches!(edge.curve(), EdgeCurve::Line) {
                return false;
            }
            let start = topo.vertex(edge.start()).expect("start").point();
            let end = topo.vertex(edge.end()).expect("end").point();
            (start.y() - y).abs() <= tolerance
                && (end.y() - y).abs() <= tolerance
                && (start.z() - z).abs() <= tolerance
                && (end.z() - z).abs() <= tolerance
        })
        .expect("boss top edge")
}

fn top_holed_face(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|&face_id| {
            let face = topo.face(face_id).expect("face");
            face.effective_plane_normal().is_some_and(|normal| {
                normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - Tolerance::new().angular
            }) && face.inner_wires().len() == 1
        })
        .max_by(|&first, &second| {
            let max_z = |face| {
                face_vertices(topo, face)
                    .expect("face vertices")
                    .into_iter()
                    .map(|vertex| topo.vertex(vertex).expect("vertex").point().z())
                    .fold(f64::NEG_INFINITY, f64::max)
            };
            max_z(first).total_cmp(&max_z(second))
        })
        .expect("holed boss cap")
}

fn boss_on_plate(topo: &mut Topology, scale: f64, offset: Vec3) -> (SolidId, FaceId) {
    let plate = placed_box(
        topo,
        [40.0 * scale, 40.0 * scale, 5.0 * scale],
        [offset.x(), offset.y(), offset.z()],
    );
    let boss = placed_box(
        topo,
        [16.0 * scale, 16.0 * scale, 10.0 * scale],
        [
            offset.x() + 12.0 * scale,
            offset.y() + 12.0 * scale,
            offset.z() + 5.0 * scale,
        ],
    );
    let sharp = boolean(topo, BooleanOp::Fuse, plate, boss).expect("fuse boss to plate");

    let drill = make_cylinder(topo, 3.0 * scale, 17.0 * scale).expect("drill");
    transform_solid(
        topo,
        drill,
        &Mat4::translation(
            offset.x() + 20.0 * scale,
            offset.y() + 20.0 * scale,
            offset.z() - scale,
        ),
    )
    .expect("place drill");
    let bored = boolean(topo, BooleanOp::Cut, sharp, drill).expect("drill boss and plate");

    let edge = line_edge_at(
        topo,
        bored,
        offset.y() + 12.0 * scale,
        offset.z() + 15.0 * scale,
        scale,
    );
    let filleted = fillet_v2(topo, bored, &[edge], scale)
        .expect("fillet boss edge")
        .solid;
    let cap = top_holed_face(topo, filleted);
    (filleted, cap)
}

fn mesh_volume(positions: &[Point3], indices: &[u32]) -> f64 {
    indices
        .chunks_exact(3)
        .map(|triangle| {
            let a = positions[triangle[0] as usize] - Point3::new(0.0, 0.0, 0.0);
            let b = positions[triangle[1] as usize] - Point3::new(0.0, 0.0, 0.0);
            let c = positions[triangle[2] as usize] - Point3::new(0.0, 0.0, 0.0);
            a.dot(b.cross(c)) / 6.0
        })
        .sum::<f64>()
        .abs()
}

fn positional_edge_health(positions: &[Point3], indices: &[u32], scale: f64) -> (usize, usize) {
    let quantization = 1e7 / scale;
    let mut canonical = HashMap::new();
    let mut remap = vec![0_u32; positions.len()];
    for (index, point) in positions.iter().enumerate() {
        let key = (
            (point.x() * quantization).round() as i64,
            (point.y() * quantization).round() as i64,
            (point.z() * quantization).round() as i64,
        );
        let next = canonical.len() as u32;
        remap[index] = *canonical.entry(key).or_insert(next);
    }
    let mut uses = HashMap::new();
    for triangle in indices.chunks_exact(3) {
        let vertices = [
            remap[triangle[0] as usize],
            remap[triangle[1] as usize],
            remap[triangle[2] as usize],
        ];
        for (first, second) in [
            (vertices[0], vertices[1]),
            (vertices[1], vertices[2]),
            (vertices[2], vertices[0]),
        ] {
            *uses
                .entry((first.min(second), first.max(second)))
                .or_insert(0_usize) += 1;
        }
    }
    (
        uses.values().filter(|&&count| count == 1).count(),
        uses.values().filter(|&&count| count > 2).count(),
    )
}

fn assert_close(actual: f64, expected: f64, relative: f64, label: &str) {
    let slack = expected.abs().mul_add(relative, 1e-15);
    assert!(
        (actual - expected).abs() <= slack,
        "{label}: {actual} != {expected} within {slack}"
    );
}

fn assert_total_one_to_one(
    evolution: &EvolutionMap,
    source_faces: &[FaceId],
    result_faces: &[FaceId],
) {
    assert!(
        evolution.origin.is_exact(),
        "construction provenance required"
    );
    assert!(
        evolution.is_complete(),
        "unresolved: {:?}",
        evolution.unresolved
    );
    assert!(evolution.generated.is_empty());
    assert!(evolution.deleted.is_empty());
    assert_eq!(evolution.modified.len(), source_faces.len());
    let inputs: BTreeSet<_> = evolution.modified.keys().copied().collect();
    assert_eq!(
        inputs,
        source_faces.iter().map(|face| face.index()).collect()
    );
    let outputs: Vec<_> = evolution
        .modified
        .values()
        .flat_map(|faces| faces.iter().copied())
        .collect();
    assert!(
        evolution.modified.values().all(|faces| faces.len() == 1),
        "each source face must have one result"
    );
    assert_eq!(outputs.len(), outputs.iter().collect::<BTreeSet<_>>().len());
    assert_eq!(
        outputs.into_iter().collect::<BTreeSet<_>>(),
        result_faces.iter().map(|face| face.index()).collect()
    );
}

fn closest_on_edge(edge: &EdgeCurve, start: Point3, end: Point3, point: Point3, t: f64) -> Point3 {
    match edge {
        EdgeCurve::Line => {
            let direction = end - start;
            let fraction = (point - start).dot(direction) / direction.length_squared();
            start + direction * fraction
        }
        EdgeCurve::Circle(circle) => circle.evaluate(circle.project(point)),
        EdgeCurve::Ellipse(ellipse) => ellipse.evaluate(ellipse.project(point)),
        EdgeCurve::Hyperbola(hyperbola) => hyperbola.evaluate(hyperbola.project(point)),
        EdgeCurve::Parabola(parabola) => parabola.evaluate(parabola.project(point)),
        EdgeCurve::NurbsCurve(nurbs) => nurbs.evaluate(t),
    }
}

fn assert_trim_and_pcurve_authority(
    topo: &Topology,
    faces: &[FaceId],
    scale: f64,
) -> (usize, usize) {
    let mut uses = 0;
    let mut present = 0;
    for &face_id in faces {
        let face = topo.face(face_id).expect("pcurve face");
        let plane_frame = match face.surface() {
            FaceSurface::Plane { normal, .. } => {
                let points = topo
                    .wire(face.outer_wire())
                    .expect("plane outer wire")
                    .edges()
                    .iter()
                    .map(|oriented| {
                        let edge = topo.edge(oriented.edge()).expect("plane edge");
                        topo.vertex(edge.start()).expect("plane vertex").point()
                    })
                    .collect::<Vec<_>>();
                Some(PlaneFrame::from_plane_face(*normal, &points))
            }
            _ => None,
        };
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id).expect("pcurve wire").edges() {
                uses += 1;
                let edge = topo.edge(oriented.edge()).expect("pcurve edge");
                let start = topo.vertex(edge.start()).expect("edge start").point();
                let end = topo.vertex(edge.end()).expect("edge end").point();
                let (edge_start, edge_end) = edge.strict_domain().expect("edge trim authority");
                assert!(edge_start.is_finite() && edge_end.is_finite());
                assert!((edge_end - edge_start).abs() > f64::EPSILON);
                let Some(pcurve) =
                    topo.pcurve_oriented(oriented.edge(), face_id, oriented.is_forward())
                else {
                    continue;
                };
                present += 1;
                assert!(pcurve.t_start().is_finite() && pcurve.t_end().is_finite());
                assert!((pcurve.t_end() - pcurve.t_start()).abs() > f64::EPSILON);
                for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
                    let t = (pcurve.t_end() - pcurve.t_start()).mul_add(fraction, pcurve.t_start());
                    let uv = pcurve.evaluate(t);
                    let on_surface = plane_frame.as_ref().map_or_else(
                        || {
                            face.surface()
                                .evaluate(uv.x(), uv.y())
                                .expect("surface evaluation")
                        },
                        |frame| frame.evaluate(uv.x(), uv.y()),
                    );
                    let edge_t = (edge_end - edge_start).mul_add(fraction, edge_start);
                    let on_edge = closest_on_edge(edge.curve(), start, end, on_surface, edge_t);
                    let residual = (on_surface - on_edge).length();
                    assert!(
                        residual <= scale.mul_add(2e-5, 1e-10),
                        "face {} ({}) edge {} ({}) pcurve residual {residual} at {fraction}",
                        face_id.index(),
                        face.surface().type_tag(),
                        oriented.edge().index(),
                        edge.curve().type_tag()
                    );
                }
            }
        }
    }
    assert!(uses > 0, "trim-authority gate must be non-vacuous");
    (uses, present)
}

fn face_neighborhood(topo: &Topology, solid: SolidId, faces: &[FaceId]) -> Vec<FaceId> {
    let adjacency = topo.build_adjacency(solid).expect("face adjacency");
    let mut neighborhood = faces.to_vec();
    for &face in faces {
        for edge in face_edges(topo, face).expect("face edges") {
            neighborhood.extend(adjacency.faces_for_edge(edge).iter().copied());
        }
    }
    neighborhood.sort_unstable_by_key(|face| face.index());
    neighborhood.dedup();
    neighborhood
}

fn assert_result_geometry(
    topo: &Topology,
    source_counts: (usize, usize, usize),
    solid: SolidId,
    expected_volume: f64,
    expected_top: f64,
    scale: f64,
    expected_pcurve_coverage: (usize, usize),
) {
    let report = validate_solid(topo, solid).expect("strict validation");
    assert!(report.is_valid(), "validation: {:?}", report.issues);
    assert_eq!(
        solid_entity_counts(topo, solid).expect("counts"),
        source_counts
    );
    assert!(
        edge_to_face_map(topo, solid)
            .expect("adjacency")
            .values()
            .all(|faces| faces.len() == 2),
        "every B-Rep edge must have exactly two face uses"
    );

    let cap = top_holed_face(topo, solid);
    assert_eq!(topo.face(cap).expect("cap").inner_wires().len(), 1);
    for vertex in face_vertices(topo, cap).expect("cap vertices") {
        assert_close(
            topo.vertex(vertex).expect("vertex").point().z(),
            expected_top,
            1e-10,
            "moved cap elevation",
        );
    }
    let mut has_bore = false;
    let mut has_fillet = false;
    for face in solid_faces(topo, solid).expect("faces") {
        if let FaceSurface::Cylinder(cylinder) = topo.face(face).expect("face").surface() {
            has_bore |= Tolerance::new().approx_eq(cylinder.radius(), 3.0 * scale);
            has_fillet |= Tolerance::new().approx_eq(cylinder.radius(), scale);
        }
    }
    assert!(has_bore, "through-bore wall must survive and re-limit");
    assert!(has_fillet, "radius-one fillet band must be rebuilt");

    let exact = solid_volume(topo, solid, 0.001 * scale).expect("exact B-Rep volume");
    assert_close(exact, expected_volume, 4e-4, "exact volume");
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.003 * scale, 0.08).expect("mesh");
    assert!(is_watertight(&mesh), "index-welded mesh must be watertight");
    assert_eq!(
        positional_edge_health(&mesh.positions, &mesh.indices, scale),
        (0, 0),
        "position-welded mesh must be closed and manifold"
    );
    assert_close(
        mesh_volume(&mesh.positions, &mesh.indices),
        expected_volume,
        8e-4,
        "independent mesh volume",
    );

    let relimited_faces = face_neighborhood(topo, solid, &[cap]);
    assert!(relimited_faces.len() > 1, "re-limit gate needs neighbors");
    assert_eq!(
        assert_trim_and_pcurve_authority(topo, &relimited_faces, scale),
        expected_pcurve_coverage,
        "the move must preserve all pre-existing pcurve authority in its re-limited neighborhood"
    );
}

fn anchor_face_references(topo: &mut Topology, solid: SolidId) -> Vec<(usize, PersistentRef)> {
    let faces = solid_faces(topo, solid).expect("anchor faces");
    let pending = begin_scoped(topo, "fixture_anchor", &[solid]).expect("anchor scope");
    let mut identity = EvolutionMap::exact();
    for face in &faces {
        identity.add_modified(face.index(), face.index());
    }
    let op = record_face_evolution(topo, pending, &identity, &[solid]).expect("anchor entry");
    (0..faces.len())
        .map(|index| {
            let reference = PersistentRef::operation_output(op, EntityKind::Face, index);
            let Resolution::Bound { entity, provenance } = resolve(topo, &reference) else {
                panic!("anchor {index} did not bind")
            };
            assert_eq!(provenance, Provenance::Construction);
            (entity.index, reference)
        })
        .collect()
}

#[test]
fn bored_boss_cap_moves_through_fillet_with_bound_persistent_refs() {
    let mut topo = Topology::new();
    let (source, cap) = boss_on_plate(&mut topo, 1.0, Vec3::new(0.0, 0.0, 0.0));
    let pcurve_faces = face_neighborhood(&topo, source, &[cap]);
    let source_faces = solid_faces(&topo, source).expect("source faces");
    let source_counts = solid_entity_counts(&topo, source).expect("source counts");
    let source_volume = solid_volume(&topo, source, 0.001).expect("source volume");
    let _moved_area = face_area(&topo, cap, 0.001).expect("holed cap area");
    let pcurve_coverage = assert_trim_and_pcurve_authority(&topo, &pcurve_faces, 1.0);
    let anchored = anchor_face_references(&mut topo, source);

    let moved = move_faces_journaled(&mut topo, source, &[cap], 2.0).expect("move boss cap");
    let result_faces = solid_faces(&topo, moved.solid).expect("result faces");
    assert_total_one_to_one(&moved.map, &source_faces, &result_faces);
    assert_eq!(
        moved.map.modified[&cap.index()],
        vec![top_holed_face(&topo, moved.solid).index()],
        "the selected cap must retain its construction identity"
    );
    assert_result_geometry(
        &topo,
        source_counts,
        moved.solid,
        source_volume + 2.0 * (16.0 * 16.0 - std::f64::consts::PI * 3.0 * 3.0),
        17.0,
        1.0,
        pcurve_coverage,
    );

    for (source_face, reference) in anchored {
        let expected = moved.map.modified[&source_face][0];
        assert_eq!(
            resolve(&topo, &reference),
            Resolution::Bound {
                entity: EntityKey::face(expected),
                provenance: Provenance::Construction,
            },
            "source face {source_face} must stay bound"
        );
    }
}

#[test]
fn generalized_move_is_scale_and_translation_invariant() {
    for scale in [1e-3, 1.0, 1e3] {
        let offset = Vec3::new(1234.0 * scale, -987.0 * scale, 321.0 * scale);
        let mut topo = Topology::new();
        let (source, cap) = boss_on_plate(&mut topo, scale, offset);
        let pcurve_faces = face_neighborhood(&topo, source, &[cap]);
        let source_faces = solid_faces(&topo, source).expect("source faces");
        let source_counts = solid_entity_counts(&topo, source).expect("source counts");
        let before = solid_volume(&topo, source, 0.001 * scale).expect("source volume");
        let _area = face_area(&topo, cap, 0.001 * scale).expect("cap area");
        let pcurve_coverage = assert_trim_and_pcurve_authority(&topo, &pcurve_faces, scale);

        let moved = move_faces_with_evolution(&mut topo, source, &[cap], 2.0 * scale)
            .unwrap_or_else(|error| panic!("scale {scale}: {error}"));
        let result_faces = solid_faces(&topo, moved.solid).expect("result faces");
        assert_total_one_to_one(&moved.evolution, &source_faces, &result_faces);
        assert_result_geometry(
            &topo,
            source_counts,
            moved.solid,
            before + 2.0 * scale.powi(3) * (16.0 * 16.0 - std::f64::consts::PI * 3.0 * 3.0),
            offset.z() + 17.0 * scale,
            scale,
            pcurve_coverage,
        );
    }
}

#[test]
fn colliding_move_refuses_without_topology_or_journal_changes() {
    let mut topo = Topology::new();
    let (source, cap) = boss_on_plate(&mut topo, 1.0, Vec3::new(0.0, 0.0, 0.0));
    let _ = anchor_face_references(&mut topo, source);
    let before_counts = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.num_pcurves(),
    );
    let before_journal = topo.journal().snapshot();
    let before_volume = solid_volume(&topo, source, 0.001).expect("source volume");

    let error = move_faces_journaled(&mut topo, source, &[cap], -20.0)
        .expect_err("cap crossing the base must refuse");

    assert!(error.to_string().contains("move") || error.to_string().contains("topology"));
    assert_eq!(
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.num_pcurves(),
        ),
        before_counts,
        "refusal must restore live topology"
    );
    assert_eq!(topo.journal().snapshot(), before_journal);
    assert_close(
        solid_volume(&topo, source, 0.001).expect("restored source volume"),
        before_volume,
        1e-12,
        "restored source volume",
    );
}

#[test]
fn inward_bore_move_reuses_replace_surface_and_reports_total_evolution() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 40.0, 40.0, 10.0).expect("plate");
    let drill = make_cylinder(&mut topo, 3.0, 10.0).expect("drill");
    transform_solid(&mut topo, drill, &Mat4::translation(20.0, 20.0, 0.0)).expect("place drill");
    let source = boolean(&mut topo, BooleanOp::Cut, plate, drill).expect("drill plate");
    let source_faces = solid_faces(&topo, source).expect("source faces");
    let bore = source_faces
        .iter()
        .copied()
        .find(|face| {
            topo.face(*face).is_ok_and(|face| {
                face.is_reversed() && matches!(face.surface(), FaceSurface::Cylinder(_))
            })
        })
        .expect("bore wall");

    let moved = move_faces_with_evolution(&mut topo, source, &[bore], 1.0).expect("narrow bore");
    let result_faces = solid_faces(&topo, moved.solid).expect("result faces");
    assert_total_one_to_one(&moved.evolution, &source_faces, &result_faces);
    let radius = result_faces
        .iter()
        .find_map(|face| match topo.face(*face).expect("face").surface() {
            FaceSurface::Cylinder(cylinder) => Some(cylinder.radius()),
            _ => None,
        })
        .expect("result bore");
    assert!(Tolerance::new().approx_eq(radius, 2.0));
    let coverage = assert_trim_and_pcurve_authority(&topo, &result_faces, 1.0);
    assert_eq!(
        coverage.0, coverage.1,
        "replacement must write every pcurve"
    );
}

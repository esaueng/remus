//! P-Class 6.1 qualification for exact support-surface re-limitation.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used
)]

use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

use remus_algo::PlaneFrame;
use remus_math::mat::Mat4;
use remus_math::surfaces::{CylindricalSurface, SphericalSurface};
use remus_math::vec::{Point3, Vec3};
use remus_offset::OffsetError;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::replace_surface::replace_surface;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{edge_to_face_map, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const DX: f64 = 10.0;
const DY: f64 = 8.0;
const DZ: f64 = 4.0;
const BORE_X: f64 = 5.0;
const BORE_Y: f64 = 4.0;
const BORE_RADIUS: f64 = 1.0;

fn bored_block(topo: &mut Topology) -> SolidId {
    bored_block_at(topo, 1.0, Vec3::new(0.0, 0.0, 0.0))
}

fn bored_block_at(topo: &mut Topology, scale: f64, offset: Vec3) -> SolidId {
    let block = make_box(topo, DX * scale, DY * scale, DZ * scale).expect("block");
    transform_solid(
        topo,
        block,
        &Mat4::translation(offset.x(), offset.y(), offset.z()),
    )
    .expect("place block");
    let drill = make_cylinder(topo, BORE_RADIUS * scale, 2.0 * DZ * scale).expect("drill");
    transform_solid(
        topo,
        drill,
        &Mat4::translation(
            offset.x() + BORE_X * scale,
            offset.y() + BORE_Y * scale,
            offset.z() - 0.5 * DZ * scale,
        ),
    )
    .expect("place drill");
    boolean(topo, BooleanOp::Cut, block, drill).expect("bore block")
}

fn assert_scaled_result(topo: &Topology, solid: SolidId, expected_volume: f64, scale: f64) {
    let report = validate_solid(topo, solid).expect("scaled validation");
    assert!(report.is_valid(), "scaled validation: {:?}", report.issues);
    assert!(
        edge_to_face_map(topo, solid)
            .expect("scaled adjacency")
            .values()
            .all(|faces| faces.len() == 2)
    );
    let measured = solid_volume(topo, solid, 0.001 * scale).expect("scaled volume");
    assert!(
        (measured - expected_volume).abs() <= expected_volume.abs().mul_add(3e-4, 1e-18),
        "scaled volume {measured} != {expected_volume} at {scale}"
    );
}

fn top_face(topo: &Topology, solid: SolidId) -> FaceId {
    let candidates: Vec<_> = solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|&face_id| {
            let face = topo.face(face_id).expect("face");
            face.effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-9)
                && !face.inner_wires().is_empty()
        })
        .collect();
    assert_eq!(candidates.len(), 1, "one bored top face");
    candidates[0]
}

fn bore_face(topo: &Topology, solid: SolidId) -> FaceId {
    let candidates: Vec<_> = solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|&face_id| {
            matches!(
                topo.face(face_id).expect("face").surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .collect();
    assert_eq!(candidates.len(), 1, "one bore wall");
    candidates[0]
}

fn live_counts(topo: &Topology) -> (usize, usize, usize, usize, usize, usize, usize) {
    (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.num_pcurves(),
    )
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

fn positional_edge_health(positions: &[Point3], indices: &[u32]) -> (usize, usize) {
    let mut canonical = HashMap::new();
    let mut remap = vec![0_u32; positions.len()];
    for (index, point) in positions.iter().enumerate() {
        let key = (
            (point.x() * 1e6).round() as i64,
            (point.y() * 1e6).round() as i64,
            (point.z() * 1e6).round() as i64,
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

fn assert_qualified_result(topo: &Topology, solid: SolidId, expected_volume: f64) {
    let report = validate_solid(topo, solid).expect("strict validation");
    assert!(report.is_valid(), "validation: {:?}", report.issues);
    let edge_faces = edge_to_face_map(topo, solid).expect("edge adjacency");
    assert!(
        edge_faces.values().all(|faces| faces.len() == 2),
        "every result edge must have exactly two face uses: {edge_faces:?}"
    );

    let exact = solid_volume(topo, solid, 0.001).expect("exact volume");
    assert!(
        (exact - expected_volume).abs() <= expected_volume.abs().mul_add(2e-4, 1e-8),
        "volume {exact} != {expected_volume}"
    );
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.003, 0.08).expect("mesh");
    assert!(is_watertight(&mesh), "index-welded mesh must be watertight");
    assert_eq!(
        positional_edge_health(&mesh.positions, &mesh.indices),
        (0, 0),
        "position-welded mesh must be closed and manifold"
    );
    let meshed = mesh_volume(&mesh.positions, &mesh.indices);
    assert!(
        (meshed - expected_volume).abs() <= expected_volume.abs().mul_add(4e-4, 1e-7),
        "mesh volume {meshed} != {expected_volume}"
    );

    let use_count = solid_faces(topo, solid)
        .expect("faces")
        .into_iter()
        .map(|face_id| {
            let face = topo.face(face_id).expect("face");
            std::iter::once(face.outer_wire())
                .chain(face.inner_wires().iter().copied())
                .map(|wire| {
                    let uses = topo.wire(wire).expect("wire").edges();
                    for oriented in uses {
                        let pcurve = topo
                            .pcurve_oriented(oriented.edge(), face_id, oriented.is_forward())
                            .expect("fresh p-curve");
                        assert!(
                            pcurve.t_start().is_finite()
                                && pcurve.t_end().is_finite()
                                && (pcurve.t_end() - pcurve.t_start()).abs() > f64::EPSILON,
                            "p-curve use must have finite non-zero authority"
                        );
                    }
                    uses.len()
                })
                .sum::<usize>()
        })
        .sum::<usize>();
    assert_eq!(
        topo.num_pcurves(),
        use_count,
        "every result coedge must carry a freshly derived p-curve"
    );
}

#[test]
fn tilted_bored_cap_relimits_lines_and_circle_to_an_ellipse() {
    let mut topo = Topology::new();
    let source = bored_block(&mut topo);
    let source_counts = solid_entity_counts(&topo, source).expect("source counts");
    let selected = top_face(&topo, source);
    let slope = 0.1;
    let normal = Vec3::new(-slope, 0.0, 1.0).normalize().expect("normal");
    let pivot = Point3::new(BORE_X, BORE_Y, DZ);
    let d = normal.dot(pivot - Point3::new(0.0, 0.0, 0.0));

    let result = replace_surface(
        &mut topo,
        source,
        selected,
        FaceSurface::Plane { normal, d },
    )
    .expect("tilt bored top face");

    assert_eq!(
        solid_entity_counts(&topo, result.solid).expect("result counts"),
        source_counts
    );
    assert_qualified_result(&topo, result.solid, DX * DY * DZ - PI * DZ);
    let result_top = result.face_map[&selected.index()];
    let inner = topo.face(result_top).expect("top").inner_wires();
    assert_eq!(inner.len(), 1, "bore opening stays an inner trim");
    let rim_use = topo.wire(inner[0]).expect("rim wire").edges()[0];
    let rim = topo.edge(rim_use.edge()).expect("rim edge");
    assert!(matches!(rim.curve(), EdgeCurve::Ellipse(_)));
    let range = rim.strict_domain().expect("ellipse trim");
    assert!(((range.1 - range.0).abs() - TAU).abs() < 1e-9);

    let top = topo.face(result_top).expect("result top");
    let FaceSurface::Plane { normal, .. } = top.surface() else {
        unreachable!();
    };
    let frame_points: Vec<_> = topo
        .wire(top.outer_wire())
        .expect("outer wire")
        .edges()
        .iter()
        .map(|oriented| {
            let edge = topo.edge(oriented.edge()).expect("outer edge");
            topo.vertex(edge.start()).expect("outer vertex").point()
        })
        .collect();
    let frame = PlaneFrame::from_plane_face(*normal, &frame_points);
    let top_pcurve = topo
        .pcurve_oriented(rim_use.edge(), result_top, rim_use.is_forward())
        .expect("ellipse p-curve on tilted cap");
    for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let t3 = (range.1 - range.0).mul_add(fraction, range.0);
        let t2 =
            (top_pcurve.t_end() - top_pcurve.t_start()).mul_add(fraction, top_pcurve.t_start());
        let from_pcurve = top_pcurve.evaluate(t2);
        let from_edge = rim.curve().evaluate_with_endpoints(
            t3,
            topo.vertex(rim.start()).expect("start").point(),
            topo.vertex(rim.end()).expect("end").point(),
        );
        assert!(
            (frame.evaluate(from_pcurve.x(), from_pcurve.y()) - from_edge).length() < 2e-5,
            "tilted-cap p-curve misses ellipse at {fraction}"
        );
    }

    let cylinder_use = topo
        .pcurves_for_edge(rim_use.edge())
        .into_iter()
        .find(|(face_id, _, _)| {
            matches!(
                topo.face(*face_id).expect("adjacent face").surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .expect("ellipse p-curve on bore wall");
    let FaceSurface::Cylinder(cylinder) = topo.face(cylinder_use.0).expect("bore wall").surface()
    else {
        unreachable!();
    };
    for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let t3 = (range.1 - range.0).mul_add(fraction, range.0);
        let t2 = (cylinder_use.2.t_end() - cylinder_use.2.t_start())
            .mul_add(fraction, cylinder_use.2.t_start());
        let uv = cylinder_use.2.evaluate(t2);
        let from_surface = cylinder.evaluate(uv.x(), uv.y());
        let from_edge = rim.curve().evaluate_with_endpoints(
            t3,
            topo.vertex(rim.start()).expect("start").point(),
            topo.vertex(rim.end()).expect("end").point(),
        );
        assert!(
            (from_surface - from_edge).length() < 2e-5,
            "re-derived p-curve misses ellipse at {fraction}"
        );
    }
}

#[test]
fn coaxial_bore_radius_replacement_relimits_both_rims() {
    let mut topo = Topology::new();
    let source = bored_block(&mut topo);
    let selected = bore_face(&topo, source);
    let FaceSurface::Cylinder(cylinder) = topo.face(selected).expect("bore").surface() else {
        unreachable!();
    };
    let replacement = CylindricalSurface::with_ref_dir(
        cylinder.origin(),
        cylinder.axis(),
        2.0,
        cylinder.x_axis(),
    )
    .expect("larger cylinder");

    let result = replace_surface(
        &mut topo,
        source,
        selected,
        FaceSurface::Cylinder(replacement),
    )
    .expect("replace bore radius");

    assert_qualified_result(&topo, result.solid, DX * DY * DZ - PI * 4.0 * DZ);
    let result_bore = result.face_map[&selected.index()];
    let FaceSurface::Cylinder(cylinder) = topo.face(result_bore).expect("result bore").surface()
    else {
        unreachable!();
    };
    assert!((cylinder.radius() - 2.0).abs() < 1e-12);
    let adjacency = edge_to_face_map(&topo, result.solid).expect("result adjacency");
    let rims: Vec<_> = adjacency
        .iter()
        .filter(|(_, faces)| faces.contains(&result_bore))
        .map(|(&edge_index, _)| topo.edge_id_from_index(edge_index).expect("result edge id"))
        .filter(|&edge_id| {
            matches!(
                topo.edge(edge_id).expect("edge").curve(),
                EdgeCurve::Circle(_)
            )
        })
        .collect();
    assert_eq!(rims.len(), 2, "both bore rims must be re-limited");
    for rim in rims {
        let EdgeCurve::Circle(circle) = topo.edge(rim).expect("rim").curve() else {
            unreachable!();
        };
        assert!((circle.radius() - 2.0).abs() < 1e-12);
    }
}

#[test]
fn unsupported_surface_change_is_typed_and_transactional() {
    let mut topo = Topology::new();
    let source = bored_block(&mut topo);
    let selected = top_face(&topo, source);
    let before = live_counts(&topo);
    let sphere = SphericalSurface::new(Point3::new(BORE_X, BORE_Y, DZ), 20.0).expect("sphere");

    let error = replace_surface(&mut topo, source, selected, FaceSurface::Sphere(sphere))
        .expect_err("plane-to-sphere is outside the qualified cell");

    assert!(matches!(
        error,
        OperationsError::Offset(OffsetError::UnsupportedMoveFace { face, .. }) if face == selected
    ));
    assert_eq!(live_counts(&topo), before, "refusal must allocate nothing");
    let report = validate_solid(&topo, source).expect("source validation");
    assert!(report.is_valid(), "source changed after refusal");
    let volume = solid_volume(&topo, source, 0.001).expect("source volume");
    let expected = DX * DY * DZ - PI * DZ;
    assert!((volume - expected).abs() <= expected * 2e-4);
}

#[test]
fn replacement_that_crosses_the_opposite_face_names_the_failed_edit_and_rolls_back() {
    let mut topo = Topology::new();
    let source = bored_block(&mut topo);
    let selected = top_face(&topo, source);
    let before = live_counts(&topo);

    let error = replace_surface(
        &mut topo,
        source,
        selected,
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: -1.0,
        },
    )
    .expect_err("replacement must not cross the opposite face");

    assert!(matches!(
        error,
        OperationsError::Offset(OffsetError::TopologyChange {
            face: Some(face),
            ..
        }) if face == selected
    ));
    assert_eq!(live_counts(&topo), before, "failed edit leaked topology");
}

#[test]
fn enlarged_bore_that_reaches_an_outer_wall_names_the_crossed_edge_and_rolls_back() {
    let mut topo = Topology::new();
    let source = bored_block(&mut topo);
    let selected = bore_face(&topo, source);
    let before = live_counts(&topo);
    let FaceSurface::Cylinder(cylinder) = topo.face(selected).expect("bore").surface() else {
        unreachable!();
    };
    let replacement = CylindricalSurface::with_ref_dir(
        cylinder.origin(),
        cylinder.axis(),
        5.0,
        cylinder.x_axis(),
    )
    .expect("oversized cylinder");

    let error = replace_surface(
        &mut topo,
        source,
        selected,
        FaceSurface::Cylinder(replacement),
    )
    .expect_err("bore may not cross an outer wall");

    assert!(matches!(
        error,
        OperationsError::Offset(OffsetError::TopologyChange {
            face: Some(face),
            edge: Some(_),
            ..
        }) if face == selected
    ));
    assert_eq!(
        live_counts(&topo),
        before,
        "failed bore edit leaked topology"
    );
}

#[test]
fn replacement_cells_are_scale_and_translation_stable() {
    for scale in [1e-3, 1e3] {
        let offset = Vec3::new(7.0 * scale, -11.0 * scale, 3.0 * scale);
        let base_volume = (DX * DY * DZ - PI * DZ) * scale.powi(3);

        let mut plane_topology = Topology::new();
        let plane_source = bored_block_at(&mut plane_topology, scale, offset);
        let plane_face = top_face(&plane_topology, plane_source);
        let normal = Vec3::new(-0.1, 0.0, 1.0).normalize().expect("normal");
        let pivot = Point3::new(
            offset.x() + BORE_X * scale,
            offset.y() + BORE_Y * scale,
            offset.z() + DZ * scale,
        );
        let d = normal.dot(pivot - Point3::new(0.0, 0.0, 0.0));
        let plane_result = replace_surface(
            &mut plane_topology,
            plane_source,
            plane_face,
            FaceSurface::Plane { normal, d },
        )
        .expect("scaled plane replacement");
        assert_scaled_result(&plane_topology, plane_result.solid, base_volume, scale);

        let mut cylinder_topology = Topology::new();
        let cylinder_source = bored_block_at(&mut cylinder_topology, scale, offset);
        let cylinder_face = bore_face(&cylinder_topology, cylinder_source);
        let FaceSurface::Cylinder(cylinder) = cylinder_topology
            .face(cylinder_face)
            .expect("bore")
            .surface()
        else {
            unreachable!();
        };
        let replacement = CylindricalSurface::with_ref_dir(
            cylinder.origin(),
            cylinder.axis(),
            2.0 * scale,
            cylinder.x_axis(),
        )
        .expect("scaled cylinder");
        let cylinder_result = replace_surface(
            &mut cylinder_topology,
            cylinder_source,
            cylinder_face,
            FaceSurface::Cylinder(replacement),
        )
        .expect("scaled cylinder replacement");
        let cylinder_volume = (DX * DY * DZ - PI * 4.0 * DZ) * scale.powi(3);
        assert_scaled_result(
            &cylinder_topology,
            cylinder_result.solid,
            cylinder_volume,
            scale,
        );
    }
}

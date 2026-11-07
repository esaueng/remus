//! Exact removal of a recognized blend group back to its sharp parent.
//!
//! The oracle is the Gentle-Gecko pair: removing the whole fillet group of
//! `gentle_gecko_filleted.step` (seven members, two of them sampled corner
//! patches) must return `gentle_gecko_sharp.step` face for face: the same
//! eight carriers, the same nineteen edges and thirteen vertices, the same
//! volume. The native `shoulder_at` bodies (R4 and R5 seams, shoulder and top
//! edges over an R1.8 cap rim) must return their sharp blank, and removing
//! only the rim must leave the other seven blends valid and recognized.
//! Partial selections a dependency forbids, and supports that cannot be
//! extended exactly, are typed refusals that leave the caller's arena
//! byte-identical.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::PointClassification::{Inside, Outside};
use remus_math::compact_sort::CompactSort;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_cluster::{
    BlendCluster, BlendClusterError, BlendMemberKind, Convexity, SharpRestoration,
    recognize_blend_cluster, remove_blend_cluster, remove_blend_members,
};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder, make_sphere};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{
    face_edges, face_vertices, solid_edges, solid_entity_counts, solid_faces, solid_vertices,
};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

// ---------------------------------------------------------------------------
// Fixtures.

fn import(text: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid = remus_io::step::reader::read_step(text, &mut topo).unwrap()[0];
    (topo, solid)
}

fn gecko() -> (Topology, SolidId) {
    import(include_str!("data/gentle_gecko_filleted.step"))
}

fn gecko_sharp() -> (Topology, SolidId) {
    import(include_str!("data/gentle_gecko_sharp.step"))
}

fn boss_of(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c) if (c.radius() - 6.0).abs() < 1e-9)
        })
        .unwrap()
}

fn top_at(topo: &Topology, solid: SolidId, height: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-9)
                && face_vertices(topo, face)
                    .unwrap()
                    .into_iter()
                    .all(|vertex| (topo.vertex(vertex).unwrap().point().z() - height).abs() < 1e-7)
        })
        .unwrap()
}

struct Shoulder {
    topo: Topology,
    blank: SolidId,
    rounded: SolidId,
    solid: SolidId,
}

/// `regress_move_face_shoulder_clearance.rs`'s `shoulder_at` with a free
/// radius: an R1.8 cap rim, then the five boss/block edges at `radius`.
fn shoulder_at(height: f64, cap_height: f64, radius: f64) -> Shoulder {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 30.0, 18.0, height).unwrap();
    let boss = make_cylinder(&mut topo, 6.0, cap_height).unwrap();
    let blank = boolean(&mut topo, BooleanOp::Fuse, boss, block).unwrap();
    let cap = top_at(&topo, blank, cap_height);
    let rim = face_edges(&topo, cap).unwrap();
    let rounded = fillet_cascade(&mut topo, blank, &rim, 1.8).unwrap().solid;
    let perimeter = perimeter_edges(&topo, rounded, height);
    assert_eq!(perimeter.len(), 5);
    let solid = fillet_cascade(&mut topo, rounded, &perimeter, radius)
        .unwrap()
        .solid;
    Shoulder {
        topo,
        blank,
        rounded,
        solid,
    }
}

/// The two boss seams, the shoulder arc and the two top block edges at the
/// boss of a sharp (or rim-rounded) shoulder body.
fn perimeter_edges(topo: &Topology, solid: SolidId, height: f64) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            let at = |a: f64, b: f64| (a - b).abs() < 1e-7;
            let top = at(a.z(), height) && at(b.z(), height);
            match edge.curve() {
                EdgeCurve::Circle(_) => top,
                EdgeCurve::Line => {
                    (top && ((at(a.x(), 0.0) && at(b.x(), 0.0))
                        || (at(a.y(), 0.0) && at(b.y(), 0.0))))
                        || ((a.z() - b.z()).abs() > height - 0.01
                            && ((at(a.x(), 6.0)
                                && at(b.x(), 6.0)
                                && at(a.y(), 0.0)
                                && at(b.y(), 0.0))
                                || (at(a.y(), 6.0)
                                    && at(b.y(), 6.0)
                                    && at(a.x(), 0.0)
                                    && at(b.x(), 0.0))))
                }
                _ => false,
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Verification.

fn bbox_diagonal(topo: &Topology, solid: SolidId) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    let mesh = tessellate_solid(topo, solid, 0.1).unwrap();
    for p in &mesh.positions {
        for (axis, value) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
            lo[axis] = lo[axis].min(value);
            hi[axis] = hi[axis].max(value);
        }
    }
    ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt()
}

/// Signed mesh volume summed about a point of the mesh.
fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let origin = mesh.positions[0];
    mesh.indices
        .chunks(3)
        .map(|tri| {
            let a = mesh.positions[tri[0] as usize] - origin;
            let b = mesh.positions[tri[1] as usize] - origin;
            let c = mesh.positions[tri[2] as usize] - origin;
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// PLAN.md's bar: no free edges, a watertight manifold mesh at two
/// deflections at or below `bbox_diag * 5e-5`, and the API volume within
/// 1e-4 of the independent mesh volume. Returns the API volume.
fn verify(topo: &Topology, solid: SolidId, label: &str) -> f64 {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    let strict = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(strict.is_valid(), "{label}: {:?}", strict.issues);
    let diagonal = bbox_diagonal(topo, solid);
    let volume = solid_volume(topo, solid, diagonal * 1e-5).unwrap();
    for deflection in [diagonal * 5e-5, diagonal * 2e-5] {
        let mesh = tessellate_solid(topo, solid, deflection).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{label} at {deflection}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "{label} at {deflection}");
        let independent = mesh_volume(&mesh);
        assert!(
            (independent - volume).abs() <= 1e-4 * volume,
            "{label}: mesh volume {independent} vs {volume} at {deflection}"
        );
    }
    volume
}

/// Every plane, cylinder and torus of the body is still exact.
fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize, usize) {
    let mut counts = (0, 0, 0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => counts.0 += 1,
            FaceSurface::Cylinder(_) => counts.1 += 1,
            FaceSurface::Torus(_) => counts.2 += 1,
            _ => counts.3 += 1,
        }
    }
    counts
}

fn same_surface(a: &FaceSurface, b: &FaceSurface) -> bool {
    match (a, b) {
        (FaceSurface::Plane { normal: n1, d: d1 }, FaceSurface::Plane { normal: n2, d: d2 }) => {
            (*n1 - *n2).length() < 1e-9 && (d1 - d2).abs() < 1e-9
        }
        (FaceSurface::Cylinder(c1), FaceSurface::Cylinder(c2)) => {
            let offset = c2.origin() - c1.origin();
            let radial = offset - c1.axis() * offset.dot(c1.axis());
            (c1.radius() - c2.radius()).abs() < 1e-9
                && c1.axis().cross(c2.axis()).length() < 1e-9
                && radial.length() < 1e-9
        }
        // Retained blends of a partial removal are copied verbatim.
        (FaceSurface::Torus(_), FaceSurface::Torus(_))
        | (FaceSurface::Nurbs(_), FaceSurface::Nurbs(_)) => format!("{a:?}") == format!("{b:?}"),
        _ => false,
    }
}

fn edge_point(topo: &Topology, edge: EdgeId, fraction: f64) -> Point3 {
    let data = topo.edge(edge).unwrap();
    let a = topo.vertex(data.start()).unwrap().point();
    let b = topo.vertex(data.end()).unwrap().point();
    let (lo, hi) = data.strict_domain().expect("edge parameter authority");
    data.curve()
        .evaluate_with_endpoints(lo + (hi - lo) * fraction, a, b)
}

/// Geometric signature of an edge: its curve type, unordered ends and
/// midpoint.
fn edge_signature(topo: &Topology, edge: EdgeId) -> (&'static str, [Point3; 2], Point3) {
    let data = topo.edge(edge).unwrap();
    let a = topo.vertex(data.start()).unwrap().point();
    let b = topo.vertex(data.end()).unwrap().point();
    let ends = if (a.x(), a.y(), a.z()) <= (b.x(), b.y(), b.z()) {
        [a, b]
    } else {
        [b, a]
    };
    (data.curve().type_tag(), ends, edge_point(topo, edge, 0.5))
}

fn close(a: Point3, b: Point3) -> bool {
    (a - b).length() < 1e-6
}

/// Face-for-face equality of two bodies: one-to-one faces on the same
/// carrier with the same orientation, each bounded by matching edges (same
/// curve type, ends and midpoint) in loops of the same sizes, and one-to-one
/// vertices.
fn assert_face_for_face(topo: &Topology, solid: SolidId, other: &Topology, oracle: SolidId) {
    assert_eq!(
        solid_entity_counts(topo, solid).unwrap(),
        solid_entity_counts(other, oracle).unwrap()
    );
    let points = |t: &Topology, s: SolidId| -> Vec<Point3> {
        solid_vertices(t, s)
            .unwrap()
            .into_iter()
            .map(|v| t.vertex(v).unwrap().point())
            .collect()
    };
    let (ours, theirs) = (points(topo, solid), points(other, oracle));
    for p in &ours {
        assert_eq!(theirs.iter().filter(|q| close(*p, **q)).count(), 1, "{p:?}");
    }
    let mut used = Vec::new();
    for face in solid_faces(topo, solid).unwrap() {
        let data = topo.face(face).unwrap();
        let matches: Vec<FaceId> = solid_faces(other, oracle)
            .unwrap()
            .into_iter()
            .filter(|&candidate| {
                let theirs = other.face(candidate).unwrap();
                same_surface(data.surface(), theirs.surface())
                    && data.is_reversed() == theirs.is_reversed()
            })
            .collect();
        assert_eq!(matches.len(), 1, "face {} has no unique twin", face.index());
        let twin = matches[0];
        assert!(!used.contains(&twin));
        used.push(twin);
        let loops = |t: &Topology, f: FaceId| -> Vec<usize> {
            let data = t.face(f).unwrap();
            let mut sizes: Vec<usize> = std::iter::once(data.outer_wire())
                .chain(data.inner_wires().iter().copied())
                .map(|w| t.wire(w).unwrap().edges().len())
                .collect();
            sizes.compact_sort_unstable();
            sizes
        };
        assert_eq!(
            loops(topo, face),
            loops(other, twin),
            "face {}",
            face.index()
        );
        let theirs: Vec<_> = face_edges(other, twin)
            .unwrap()
            .into_iter()
            .map(|e| edge_signature(other, e))
            .collect();
        for edge in face_edges(topo, face).unwrap() {
            let (kind, ends, middle) = edge_signature(topo, edge);
            assert!(
                theirs.iter().any(|(k, e, m)| *k == kind
                    && close(e[0], ends[0])
                    && close(e[1], ends[1])
                    && close(*m, middle)),
                "edge {} ({kind} {ends:?} via {middle:?}) of face {} is not the oracle's",
                edge.index(),
                face.index()
            );
        }
    }
}

/// The STEP text of a solid: a byte-level fingerprint of everything a
/// refusal must leave alone.
fn fingerprint(topo: &Topology, solid: SolidId) -> (String, usize) {
    (
        remus_io::step::writer::write_step(topo, &[solid]).unwrap(),
        topo.allocated_slot_count(),
    )
}

fn refusal(result: Result<SharpRestoration, OperationsError>) -> BlendClusterError {
    match result {
        Err(OperationsError::BlendCluster(error)) => error,
        Err(other) => panic!("expected a typed blend-cluster refusal, got {other}"),
        Ok(restoration) => panic!(
            "expected a refusal, got a sharp body with {} faces",
            restoration.faces.len()
        ),
    }
}

/// Every source edge and vertex has exactly one history record, and every
/// result entity named by the restoration exists.
fn assert_history(
    topo: &Topology,
    source: SolidId,
    cluster: &BlendCluster,
    restoration: &SharpRestoration,
) {
    let edges = solid_edges(topo, source).unwrap();
    let vertices = solid_vertices(topo, source).unwrap();
    assert_eq!(
        restoration.boundary_history.len(),
        edges.len() + vertices.len()
    );
    let result_edges = solid_edges(topo, restoration.solid).unwrap();
    let result_vertices = solid_vertices(topo, restoration.solid).unwrap();
    for (index, pieces) in restoration.sharp_edges.iter().enumerate() {
        let restored = restoration
            .removed
            .contains(&cluster.sharp_edges[index].member);
        assert_eq!(!pieces.is_empty(), restored, "sharp edge {index}");
        for piece in pieces {
            assert!(result_edges.contains(piece));
        }
    }
    for vertex in restoration.sharp_vertices.iter().flatten() {
        assert!(result_vertices.contains(vertex));
    }
    let result_faces = solid_faces(topo, restoration.solid).unwrap();
    assert_eq!(restoration.faces.len(), result_faces.len());
    for (source_face, result) in &restoration.faces {
        assert!(
            cluster
                .member_of(*source_face)
                .is_none_or(|member| !restoration.removed.contains(&member))
        );
        assert!(result_faces.contains(result));
        assert!(same_surface(
            topo.face(*source_face).unwrap().surface(),
            topo.face(*result).unwrap().surface()
        ));
    }
    for &member in &restoration.removed {
        for face in &cluster.members[member].faces {
            assert!(restoration.evolution.deleted.contains(&face.index()));
        }
    }
    for (source_edge, result) in &restoration.retrimmed_edges {
        assert!(edges.contains(source_edge));
        assert!(result_edges.contains(result));
    }
}

// ---------------------------------------------------------------------------
// The oracle.

#[test]
fn gecko_group_removes_to_the_sharp_parent_face_for_face() {
    let (mut topo, solid) = gecko();
    let boss = boss_of(&topo, solid);
    let cluster = recognize_blend_cluster(&topo, solid, boss).unwrap();
    assert_eq!(cluster.members.len(), 7);
    let before = fingerprint(&topo, solid);
    // The existing removal paths cannot take this group: both refuse the
    // sampled corner patches.
    let faces = cluster.faces();
    assert!(remus_operations::remove_blends::remove_blends(&mut topo, solid, &faces).is_err());
    assert!(remus_operations::defeature::defeature(&mut topo, solid, &faces).is_err());
    assert_eq!(fingerprint(&topo, solid), before);

    let restoration = remove_blend_cluster(&mut topo, solid, &cluster).unwrap();
    let sharp = restoration.solid;

    // The input is a separate, untouched solid.
    assert_eq!(
        remus_io::step::writer::write_step(&topo, &[solid]).unwrap(),
        before.0
    );
    assert_eq!(solid_entity_counts(&topo, sharp).unwrap(), (8, 19, 13));
    assert_eq!(census(&topo, sharp), (7, 1, 0, 0));
    let (oracle_topo, oracle) = gecko_sharp();
    assert_face_for_face(&topo, sharp, &oracle_topo, oracle);

    let volume = verify(&topo, sharp, "gecko sharp");
    let expected = solid_volume(&oracle_topo, oracle, 1e-3).unwrap();
    assert!(
        (expected - 16352.920).abs() < 1e-3,
        "oracle volume {expected}"
    );
    assert!(
        (volume - expected).abs() <= 1e-9 * expected,
        "sharp volume {volume} vs oracle {expected}"
    );
    // Removal adds the fillet material back.
    assert!(volume > solid_volume(&topo, solid, 1e-3).unwrap());
    // Material is back under the convex top rounds and gone from the
    // concave seams and shoulder.
    for (probe, expected) in [
        (Point3::new(6.3, 0.3, 23.7), Inside),
        (Point3::new(0.3, 6.3, 23.7), Inside),
        (Point3::new(15.0, 0.3, 23.7), Inside),
        (Point3::new(0.3, 12.0, 23.7), Inside),
        (Point3::new(29.7, 0.3, 23.7), Inside),
        (Point3::new(4.3, 4.3, 24.3), Outside),
        (Point3::new(6.5, -0.3, 5.0), Outside),
        (Point3::new(-0.3, 6.5, 5.0), Outside),
        (Point3::new(5.7, 0.3, 30.0), Inside),
    ] {
        assert_eq!(
            remus_check::classify::classify_point(
                &topo,
                sharp,
                probe,
                &remus_check::classify::ClassifyOptions::default()
            )
            .unwrap(),
            expected,
            "probe {probe:?}"
        );
    }

    // History: five sharp edges, six sharp vertices; every retained face
    // keeps its carrier.
    assert_history(&topo, solid, &cluster, &restoration);
    assert_eq!(restoration.faces.len(), 8);
    assert!(
        restoration
            .sharp_edges
            .iter()
            .all(|pieces| pieces.len() == 1)
    );
    assert!(restoration.sharp_vertices.iter().all(Option::is_some));
    // The re-trimmed retained edges: both bottom lines into the seams, the
    // three seam-band walls' vertical edges, the end caps' top and side
    // edges, the boss's bottom arcs and its seam.
    assert_eq!(restoration.retrimmed_edges.len(), 9);
    let x30 = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { normal, d }
                if (normal.x() - 1.0).abs() < 1e-12 && (d - 30.0).abs() < 1e-12)
        })
        .unwrap();
    let x30_retrimmed = restoration
        .retrimmed_edges
        .iter()
        .filter(|(source, _)| face_edges(&topo, x30).unwrap().contains(source))
        .count();
    assert_eq!(
        x30_retrimmed, 2,
        "the x = 30 cap re-trims its top and front edges"
    );

    // The re-fillet stages map onto the restored edges.
    let concave = restoration.reblend_edges(&cluster, 0);
    let convex = restoration.reblend_edges(&cluster, 1);
    assert_eq!((concave.len(), convex.len()), (3, 2));
    let result_edges = solid_edges(&topo, sharp).unwrap();
    assert!(
        concave
            .iter()
            .chain(&convex)
            .all(|e| result_edges.contains(e))
    );
}

#[test]
fn gecko_removal_is_seed_independent_and_refillets_to_the_original() {
    let (mut topo, solid) = gecko();
    let original = solid_volume(&topo, solid, 1e-3).unwrap();
    let mut volumes = Vec::new();
    for seed in solid_faces(&topo, solid).unwrap() {
        let Ok(cluster) = recognize_blend_cluster(&topo, solid, seed) else {
            continue;
        };
        let restoration = remove_blend_cluster(&mut topo, solid, &cluster).unwrap();
        assert_eq!(
            solid_entity_counts(&topo, restoration.solid).unwrap(),
            (8, 19, 13)
        );
        volumes.push(solid_volume(&topo, restoration.solid, 1e-3).unwrap());
    }
    assert_eq!(volumes.len(), 11);
    assert!(volumes.iter().all(|v| (v - volumes[0]).abs() < 1e-9 * v));

    // Re-fillet the restored edges at the recognized radius: the result is
    // the filleted part again.
    let boss = boss_of(&topo, solid);
    let cluster = recognize_blend_cluster(&topo, solid, boss).unwrap();
    let restoration = remove_blend_cluster(&mut topo, solid, &cluster).unwrap();
    let edges: Vec<EdgeId> = (0..cluster.reblend_order.len())
        .flat_map(|step| restoration.reblend_edges(&cluster, step))
        .collect();
    let refilleted = fillet_cascade(&mut topo, restoration.solid, &edges, 4.0)
        .unwrap()
        .solid;
    assert_eq!(solid_faces(&topo, refilleted).unwrap().len(), 15);
    let volume = verify(&topo, refilleted, "gecko re-filleted");
    assert!(
        (volume - original).abs() <= 1e-4 * original,
        "re-filleted {volume} vs original {original}"
    );
    let again = recognize_blend_cluster(&topo, refilleted, boss_of(&topo, refilleted)).unwrap();
    assert_eq!(again.members.len(), 7);
}

// ---------------------------------------------------------------------------
// Native shoulder bodies.

#[test]
fn shoulder_groups_at_r4_and_r5_remove_to_the_sharp_blank() {
    for (height, cap, radius) in [
        (26.0, 35.0, 4.0),
        (26.0, 35.0, 5.0),
        (19.1, 28.5, 4.0),
        (19.1, 28.5, 5.0),
    ] {
        let label = format!("h{height} cap{cap} r{radius}");
        let mut shoulder = shoulder_at(height, cap, radius);
        let solid = shoulder.solid;
        let boss = boss_of(&shoulder.topo, solid);
        let cluster = recognize_blend_cluster(&shoulder.topo, solid, boss).unwrap();
        assert_eq!(cluster.members.len(), 8, "{label}");
        let restoration = remove_blend_cluster(&mut shoulder.topo, solid, &cluster).unwrap();
        let topo = &shoulder.topo;
        let sharp = restoration.solid;
        assert_history(topo, solid, &cluster, &restoration);

        // The blank's carriers, all exact; the rim is one closed circle.
        assert_eq!(census(topo, sharp), (7, 1, 0, 0), "{label}");
        let volume = verify(topo, sharp, &label);
        let blank = solid_volume(topo, shoulder.blank, 1e-3).unwrap();
        assert!(
            (volume - blank).abs() <= 1e-9 * blank,
            "{label}: sharp {volume} vs blank {blank}"
        );
        let closed = cluster
            .sharp_edges
            .iter()
            .position(|edge| edge.ends.is_none())
            .unwrap();
        let rim = &restoration.sharp_edges[closed];
        assert_eq!(rim.len(), 1, "{label}: the boss seam lands on the rim once");
        assert!(topo.edge(rim[0]).unwrap().is_closed());
        // Every face of the blank survives on its own carrier: the blank's
        // bottom is split into three coplanar faces, so match carriers.
        for face in solid_faces(topo, shoulder.blank).unwrap() {
            let surface = topo.face(face).unwrap().surface();
            assert!(
                solid_faces(topo, sharp)
                    .unwrap()
                    .iter()
                    .any(|f| same_surface(topo.face(*f).unwrap().surface(), surface)),
                "{label}: blank face {} has no carrier in the sharp body",
                face.index()
            );
        }
        // Material is back under every removed convex round and gone from
        // every removed concave one.
        for (probe, expected) in [
            (Point3::new(6.3, 0.3, height - 0.3), Inside),
            (Point3::new(0.3, 6.3, height - 0.3), Inside),
            (Point3::new(15.0, 0.3, height - 0.3), Inside),
            (Point3::new(0.3, 12.0, height - 0.3), Inside),
            (Point3::new(5.7, 0.3, cap - 0.3), Inside),
            (Point3::new(4.3, 4.3, height + 0.3), Outside),
            (Point3::new(6.5, -0.3, 5.0), Outside),
            (Point3::new(-0.3, 6.5, 5.0), Outside),
        ] {
            assert_eq!(
                remus_check::classify::classify_point(
                    topo,
                    sharp,
                    probe,
                    &remus_check::classify::ClassifyOptions::default()
                )
                .unwrap(),
                expected,
                "{label}: probe {probe:?}"
            );
        }
        let _ = shoulder.rounded;
    }
}

#[test]
fn shoulder_rim_alone_removes_and_leaves_the_other_blends_valid() {
    for radius in [4.0, 5.0] {
        let label = format!("r{radius}");
        let (height, cap) = (26.0, 35.0);
        let mut shoulder = shoulder_at(height, cap, radius);
        let solid = shoulder.solid;
        let boss = boss_of(&shoulder.topo, solid);
        let cluster = recognize_blend_cluster(&shoulder.topo, solid, boss).unwrap();
        let rim = cluster
            .members
            .iter()
            .position(|m| (m.radius - 1.8).abs() < 1e-9)
            .unwrap();
        let restoration =
            remove_blend_members(&mut shoulder.topo, solid, &cluster, &[rim]).unwrap();
        let topo = &shoulder.topo;
        let result = restoration.solid;
        assert_history(topo, solid, &cluster, &restoration);
        assert_eq!(
            solid_faces(topo, result).unwrap().len(),
            solid_faces(topo, solid).unwrap().len() - 1
        );
        let volume = verify(topo, result, &label);
        // The shoulder torus and both sampled corners survive untouched.
        let (planes, cylinders, tori, other) = census(topo, solid);
        assert_eq!(census(topo, result), (planes, cylinders, tori - 1, other));
        // The rim's spandrel ring, by Pappus.
        let (r, big) = (1.8_f64, 6.0_f64);
        let ring = std::f64::consts::TAU
            * (r * r * (1.0 - std::f64::consts::FRAC_PI_4) * (big - r) + r.powi(3) / 6.0);
        let before = solid_volume(topo, solid, 1e-3).unwrap();
        assert!(
            (volume - before - ring).abs() <= 1e-6 * volume,
            "{label}: rim removal added {} vs {ring}",
            volume - before
        );
        // The seven remaining blends are still one recognized group.
        let after = recognize_blend_cluster(topo, result, boss_of(topo, result)).unwrap();
        assert_eq!(after.members.len(), 7, "{label}");
        assert!(
            after
                .members
                .iter()
                .all(|m| (m.radius - radius).abs() < 1e-9)
        );
        assert_eq!(
            after
                .members
                .iter()
                .filter(|m| m.kind == BlendMemberKind::CornerPatch)
                .count(),
            2
        );
        // The cap is a sharp disc again: its group is empty.
        let cap_face = top_at(topo, result, cap);
        assert!(matches!(
            recognize_blend_cluster(topo, result, cap_face),
            Err(OperationsError::BlendCluster(
                BlendClusterError::NoDependentBlends { .. }
            ))
        ));
        // And the rest of the group then removes to the blank.
        let rest = remove_blend_cluster(&mut shoulder.topo, result, &after).unwrap();
        let blank = solid_volume(&shoulder.topo, shoulder.blank, 1e-3).unwrap();
        let sharp = verify(&shoulder.topo, rest.solid, &label);
        assert!((sharp - blank).abs() <= 1e-9 * blank, "{label}");
    }
}

// ---------------------------------------------------------------------------
// Refusals.

#[test]
fn gecko_top_bands_alone_are_refused_by_dependency_and_leave_the_body_alone() {
    let (mut topo, solid) = gecko();
    let boss = boss_of(&topo, solid);
    let cluster = recognize_blend_cluster(&topo, solid, boss).unwrap();
    let convex: Vec<usize> = cluster
        .members
        .iter()
        .enumerate()
        .filter(|(_, m)| m.kind == BlendMemberKind::Band && m.convexity == Convexity::Convex)
        .map(|(index, _)| index)
        .collect();
    assert_eq!(convex.len(), 2);
    let before = fingerprint(&topo, solid);
    match refusal(remove_blend_members(&mut topo, solid, &cluster, &convex)) {
        BlendClusterError::DependentMembers {
            requested,
            required,
        } => {
            assert_eq!(requested, convex);
            // The corner patches close the top bands into the seams and the
            // shoulder: the whole group.
            assert_eq!(required, (0..7).collect::<Vec<_>>());
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(fingerprint(&topo, solid), before);

    // A corner patch alone, the seams alone: the same closure.
    let corner = cluster
        .members
        .iter()
        .position(|m| m.kind == BlendMemberKind::CornerPatch)
        .unwrap();
    assert!(matches!(
        refusal(remove_blend_members(&mut topo, solid, &cluster, &[corner])),
        BlendClusterError::DependentMembers { .. }
    ));
    assert_eq!(fingerprint(&topo, solid), before);
}

#[test]
fn malformed_and_stale_requests_are_refused_without_touching_the_arena() {
    let (mut topo, solid) = gecko();
    let cluster = recognize_blend_cluster(&topo, solid, boss_of(&topo, solid)).unwrap();
    let before = fingerprint(&topo, solid);
    for members in [&[][..], &[7][..]] {
        assert!(matches!(
            refusal(remove_blend_members(&mut topo, solid, &cluster, members)),
            BlendClusterError::InvalidRemoval { .. }
        ));
        assert_eq!(fingerprint(&topo, solid), before);
    }
    // A cluster recognized on another solid does not describe this one.
    let (other_topo, other) = {
        let shoulder = shoulder_at(26.0, 35.0, 4.0);
        (shoulder.topo, shoulder.solid)
    };
    let foreign = recognize_blend_cluster(&other_topo, other, boss_of(&other_topo, other)).unwrap();
    let mut forged = cluster.clone();
    forged.members[0].radius = 4.5;
    for stale in [&forged, &foreign] {
        assert!(matches!(
            remove_blend_cluster(&mut topo, solid, stale),
            Err(OperationsError::BlendCluster(_))
        ));
        assert_eq!(fingerprint(&topo, solid), before);
    }
}

/// `reblend_cylinder_sphere_seam.rs`'s coaxial body, smaller: an R6 boss
/// fused into an R20 ball, its seam rounded at R2. The band rolls on the
/// boss and the sphere. The recognizer refuses the group (it reads the ball's
/// second hemisphere, tangent across the equator, as an unclosed corner), so
/// no removal can start; the sphere-support guard of the removal itself is
/// covered by the crate's unit tests with a hand-built cluster.
#[test]
fn a_sphere_supported_seam_is_refused_before_removal() {
    let mut topo = Topology::new();
    let ball = make_sphere(&mut topo, 20.0, 16).unwrap();
    let boss = make_cylinder(&mut topo, 6.0, 20.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        boss,
        &remus_math::mat::Mat4::translation(0.0, 0.0, 10.0),
    )
    .unwrap();
    let body = boolean(&mut topo, BooleanOp::Fuse, ball, boss).unwrap();
    let wall = boss_of(&topo, body);
    let adjacency = topo.build_adjacency(body).unwrap();
    let seam: Vec<EdgeId> = face_edges(&topo, wall)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            adjacency
                .faces_for_edge(edge)
                .iter()
                .any(|f| topo.face(*f).unwrap().surface().type_tag() == "sphere")
        })
        .collect();
    assert!(!seam.is_empty());
    let rounded = fillet_cascade(&mut topo, body, &seam, 2.0).unwrap().solid;
    let before = fingerprint(&topo, rounded);
    for seed in solid_faces(&topo, rounded).unwrap() {
        assert!(matches!(
            recognize_blend_cluster(&topo, rounded, seed),
            Err(OperationsError::BlendCluster(
                BlendClusterError::NoSharpForm { .. } | BlendClusterError::NoDependentBlends { .. }
            ))
        ));
    }
    assert_eq!(fingerprint(&topo, rounded), before);
}

#[test]
fn bored_plate_rim_removes_to_the_plate() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 40.0, 30.0, 10.0).unwrap();
    let tool = make_cylinder(&mut topo, 5.0, 30.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        tool,
        &remus_math::mat::Mat4::translation(20.0, 15.0, -10.0),
    )
    .unwrap();
    let plate = boolean(&mut topo, BooleanOp::Cut, block, tool).unwrap();
    let top = top_at(&topo, plate, 10.0);
    let rim: Vec<EdgeId> = face_edges(&topo, top)
        .unwrap()
        .into_iter()
        .filter(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(_)))
        .collect();
    let rounded = fillet_cascade(&mut topo, plate, &rim, 1.0).unwrap().solid;
    let bore = solid_faces(&topo, rounded)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let cluster = recognize_blend_cluster(&topo, rounded, bore).unwrap();
    let restoration = remove_blend_cluster(&mut topo, rounded, &cluster).unwrap();
    assert_history(&topo, rounded, &cluster, &restoration);
    let volume = verify(&topo, restoration.solid, "bored plate");
    let expected = solid_volume(&topo, plate, 1e-3).unwrap();
    assert!(
        (volume - expected).abs() <= 1e-9 * expected,
        "{volume} vs {expected}"
    );
    assert_eq!(census(&topo, restoration.solid), census(&topo, plate));
}
